#!/usr/bin/env python3
"""Validated single-client UDP control with a robot-side safety watchdog.

protocol 1: 기존 노트북 v1 (x/yaw_rate 실제 속도 + timestamp).
protocol 2: Quest v2 (forward/turn 정규화 입력, 중립 확인, 상태 ACK).
한 서버 실행은 설정한 한 가지 protocol만 해석하며 두 형식을 섞지 않습니다.
"""

import json
import logging
import math
import socket
import threading
import time
import uuid
from collections import OrderedDict
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple


LOGGER = logging.getLogger(__name__)
MAX_PACKET_BYTES = 2048
STOP = (0.0, 0.0, 0.0)


class PacketError(ValueError):
    """Raised when a control datagram is not safe to use."""


@dataclass(frozen=True)
class ControlPacket:
    action: str
    client_id: str
    sequence: int
    timestamp: float
    x: float = 0.0
    yaw_rate: float = 0.0


def _reject_json_constant(value):
    raise PacketError("JSON 상수 %s는 허용되지 않습니다" % value)


def parse_control_packet(data: bytes, protocol_version: int = 1) -> ControlPacket:
    """Parse one strict JSON packet. NaN, Inf, booleans-as-numbers and extras fail."""
    if not data or len(data) > MAX_PACKET_BYTES:
        raise PacketError("패킷 크기가 올바르지 않습니다")
    try:
        payload = json.loads(
            data.decode("utf-8"),
            parse_constant=_reject_json_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError, PacketError) as exc:
        raise PacketError("JSON 디코딩 실패: %s" % exc)

    if not isinstance(payload, dict):
        raise PacketError("JSON 객체만 허용됩니다")

    common = {"protocol", "type", "client_id", "sequence", "timestamp"}
    action = payload.get("type")
    if action == "command":
        allowed = common | {"x", "yaw_rate"}
        required = allowed
    elif action in {"emergency_stop", "clear_emergency", "disconnect"}:
        allowed = common
        required = common
    else:
        raise PacketError("알 수 없는 패킷 type")

    if set(payload) != required or set(payload) - allowed:
        raise PacketError("필수 필드 누락 또는 허용되지 않은 필드")
    if type(payload["protocol"]) is not int or payload["protocol"] != protocol_version:
        raise PacketError("프로토콜 버전 불일치")

    client_id = payload["client_id"]
    if not isinstance(client_id, str) or len(client_id) > 64:
        raise PacketError("client_id 형식 오류")
    try:
        parsed_uuid = uuid.UUID(client_id)
    except (ValueError, AttributeError):
        raise PacketError("client_id는 UUID여야 합니다")
    if str(parsed_uuid) != client_id.lower():
        raise PacketError("client_id UUID 표준 형식 오류")

    sequence = payload["sequence"]
    if type(sequence) is not int or sequence < 0 or sequence > (2**63 - 1):
        raise PacketError("sequence 범위 오류")

    timestamp = payload["timestamp"]
    if type(timestamp) not in (int, float) or not math.isfinite(float(timestamp)):
        raise PacketError("timestamp는 유한한 수여야 합니다")

    x = 0.0
    yaw_rate = 0.0
    if action == "command":
        x = payload["x"]
        yaw_rate = payload["yaw_rate"]
        if type(x) not in (int, float) or not math.isfinite(float(x)):
            raise PacketError("x는 유한한 수여야 합니다")
        if type(yaw_rate) not in (int, float) or not math.isfinite(float(yaw_rate)):
            raise PacketError("yaw_rate는 유한한 수여야 합니다")

    return ControlPacket(
        action=action,
        client_id=client_id.lower(),
        sequence=sequence,
        timestamp=float(timestamp),
        x=float(x),
        yaw_rate=float(yaw_rate),
    )


def clamp_velocity(
    x: float,
    yaw_rate: float,
    max_forward: float,
    max_reverse: float,
    max_yaw_rate: float,
) -> Tuple[float, float, float]:
    """Apply the final robot-side limits. y is deliberately always zero."""
    if not math.isfinite(x) or not math.isfinite(yaw_rate):
        raise ValueError("속도 값은 유한해야 합니다")
    limited_x = max(-max_reverse, min(max_forward, x))
    limited_yaw = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate))
    return float(limited_x), 0.0, float(limited_yaw)


# ───────────────────────────── protocol 2 (Quest v2) ─────────────────────────────

PROTOCOL_V2 = 2
V2_MAX_SEQUENCE = 9007199254740991  # 2**53 - 1, JavaScript/C# double 안전 정수
V2_COMMON_FIELDS = frozenset({"protocol", "type", "client_id", "sequence"})
V2_COMMAND_FIELDS = V2_COMMON_FIELDS | {"control_active", "forward", "turn"}
V2_EVENT_TYPES = frozenset({"emergency_stop", "clear_emergency", "disconnect"})
V2_STATES = (
    "idle",
    "active",
    "timeout",
    "emergency_stop",
    "awaiting_neutral",
    "fault",
)
V2_REASONS = (
    "ok",
    "invalid_packet",
    "unsupported_protocol",
    "out_of_order",
    "not_owner",
    "emergency_stop",
    "neutral_required",
    # 규격 §10의 후보 목록에 없는 로봇 측 추가 코드입니다. 출력 오류나 모터
    # 토픽 동시 발행자 감지로 state가 fault일 때만 사용합니다.
    "fault",
)
_NIL_UUID = "00000000-0000-0000-0000-000000000000"


class V2PacketError(PacketError):
    """v2 거부 사유 코드와, 해석 가능했던 경우 ACK에 되돌릴 식별자를 담습니다."""

    def __init__(self, reason: str, message: str, client_id=None, sequence=-1):
        super().__init__(message)
        self.reason = reason
        self.client_id = client_id
        self.sequence = sequence


@dataclass(frozen=True)
class V2Packet:
    action: str
    client_id: str  # ACK에 그대로 되돌리는 수신 문자열
    session_id: str  # 비교용 소문자 UUID
    sequence: int
    control_active: bool = False
    forward: float = 0.0
    turn: float = 0.0

    @property
    def is_neutral(self) -> bool:
        """control_active=false이거나 두 축이 0이면 이동 목표가 0인 중립입니다."""
        return not self.control_active or (self.forward == 0.0 and self.turn == 0.0)


def _reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise PacketError("중복 JSON 키: %s" % key)
        result[key] = value
    return result


def _decode_strict_json(data: bytes):
    if not data or len(data) > MAX_PACKET_BYTES:
        raise PacketError("패킷 크기가 올바르지 않습니다")
    try:
        return json.loads(
            data.decode("utf-8"),
            parse_constant=_reject_json_constant,
            object_pairs_hook=_reject_duplicate_keys,
        )
    except (UnicodeDecodeError, ValueError, RecursionError) as exc:
        # JSONDecodeError, PacketError, 과도한 정수 자릿수는 모두 ValueError입니다.
        raise PacketError("JSON 디코딩 실패: %s" % exc)


def _canonical_uuid(value) -> Optional[str]:
    """하이픈 포함 36자 표준 UUID만 허용하고 nil UUID는 거부합니다."""
    if type(value) is not str or len(value) != 36:
        return None
    try:
        text = str(uuid.UUID(value))
    except ValueError:
        return None
    if text != value.lower() or text == _NIL_UUID:
        return None
    return text


def _is_v2_sequence(value) -> bool:
    return type(value) is int and 0 <= value <= V2_MAX_SEQUENCE


def _unit_axis(value) -> Optional[float]:
    """-1~1 유한한 JSON 숫자만 허용합니다. bool·문자열·NaN·Inf는 거부합니다."""
    if type(value) is int:
        # 큰 정수를 float로 바꾸면 OverflowError가 나므로 범위를 먼저 비교합니다.
        return float(value) if -1 <= value <= 1 else None
    if type(value) is float and math.isfinite(value) and -1.0 <= value <= 1.0:
        return value
    return None


def parse_v2_packet(data: bytes) -> V2Packet:
    """Parse one strict v2 datagram. Missing, extra, duplicate or mistyped fields fail."""
    payload = None
    try:
        payload = _decode_strict_json(data)
    except PacketError as exc:
        raise V2PacketError("invalid_packet", str(exc))
    if not isinstance(payload, dict):
        raise V2PacketError("invalid_packet", "JSON 객체만 허용됩니다")

    session_id = _canonical_uuid(payload.get("client_id"))
    echo_id = payload["client_id"] if session_id is not None else None
    raw_sequence = payload.get("sequence")
    echo_sequence = raw_sequence if _is_v2_sequence(raw_sequence) else -1

    def reject(reason, message):
        return V2PacketError(reason, message, echo_id, echo_sequence)

    protocol = payload.get("protocol")
    if type(protocol) is not int:
        raise reject("invalid_packet", "protocol은 정수여야 합니다")
    if protocol != PROTOCOL_V2:
        raise reject("unsupported_protocol", "protocol %d은 지원하지 않습니다" % protocol)

    action = payload.get("type")
    if type(action) is not str:
        raise reject("invalid_packet", "type은 문자열이어야 합니다")
    if action == "command":
        required = V2_COMMAND_FIELDS
    elif action in V2_EVENT_TYPES:
        required = V2_COMMON_FIELDS
    else:
        raise reject("invalid_packet", "알 수 없는 type")

    fields = set(payload)
    if fields != required:
        raise reject(
            "invalid_packet",
            "필드 불일치: 누락=%s 추가=%s"
            % (sorted(required - fields), sorted(fields - required)),
        )
    if session_id is None:
        raise reject("invalid_packet", "client_id는 표준 UUID 문자열이어야 합니다")
    if echo_sequence == -1:
        raise reject("invalid_packet", "sequence는 0~%d 정수여야 합니다" % V2_MAX_SEQUENCE)

    if action != "command":
        return V2Packet(action, echo_id, session_id, raw_sequence)

    control_active = payload["control_active"]
    if type(control_active) is not bool:
        raise reject("invalid_packet", "control_active는 불리언이어야 합니다")
    forward = _unit_axis(payload["forward"])
    turn = _unit_axis(payload["turn"])
    if forward is None or turn is None:
        raise reject("invalid_packet", "forward/turn은 -1~1 유한한 숫자여야 합니다")
    return V2Packet(
        action,
        echo_id,
        session_id,
        raw_sequence,
        control_active,
        forward,
        turn,
    )


def normalized_to_velocity(
    forward: float,
    turn: float,
    max_forward: float,
    max_reverse: float,
    max_yaw_rate: float,
) -> Tuple[float, float, float]:
    """규격 §6 변환 뒤 기존 최종 속도 제한을 한 번 더 적용합니다. y는 항상 0입니다."""
    scale = max_forward if forward >= 0.0 else max_reverse
    return clamp_velocity(
        forward * scale,
        turn * max_yaw_rate,
        max_forward,
        max_reverse,
        max_yaw_rate,
    )


@dataclass(frozen=True)
class V2Result:
    ack: dict
    velocity: Optional[Tuple[float, float, float]]  # 지금 발행할 목표. None이면 발행 없음
    event: Optional[str] = None  # 상태 전이 로그


class V2Session:
    """Pure v2 controller state. The caller passes monotonic time and does all I/O.

    소유권(단일 조종자), sequence, 중립 확인, 비상정지 latch, watchdog, lease를
    관리합니다. 소켓·ROS·스레드에 의존하지 않으므로 가짜 시계로 시험합니다.
    """

    def __init__(
        self,
        max_forward: float,
        max_reverse: float,
        max_yaw_rate: float,
        command_timeout: float,
        lease_timeout: float,
        retired_limit: int = 1024,
    ):
        self._max_forward = max_forward
        self._max_reverse = max_reverse
        self._max_yaw = max_yaw_rate
        self._command_timeout = command_timeout
        self._lease_timeout = lease_timeout
        self._retired_limit = retired_limit

        self._owner_id = None  # type: Optional[str]
        self._owner_address = None
        self._last_sequence = -1
        self._last_owner_activity = 0.0
        self._last_command_rx = None  # type: Optional[float]
        self._neutral_confirmed = False
        self._timed_out = False
        self._active = False
        self._emergency = False
        self._fault = None  # type: Optional[str]
        self._target = STOP
        self._last_reason = "ok"
        # lease 만료로 풀린 직전 조종자. 다른 UUID가 인수하기 전까지만 이어서
        # 재획득할 수 있고, 이때도 마지막 sequence보다 큰 번호가 필요합니다.
        self._previous_owner = None  # type: Optional[Tuple[str, int]]
        # disconnect했거나 다른 UUID에게 소유권이 넘어간 세션. 지연 패킷으로
        # 소유권이 되돌아가지 않도록 서버 실행 동안 거부합니다(잠정 정책).
        self._retired = OrderedDict()  # type: OrderedDict

    # ── 조회 ──

    @property
    def state(self) -> str:
        if self._fault is not None:
            return "fault"
        if self._emergency:
            return "emergency_stop"
        if self._timed_out:
            return "timeout"
        if not self._neutral_confirmed:
            return "awaiting_neutral"
        return "active" if self._active else "idle"

    @property
    def emergency_stop(self) -> bool:
        return self._emergency

    @property
    def target(self) -> Tuple[float, float, float]:
        return self._target

    def snapshot(self) -> dict:
        return {
            "state": self.state,
            "emergency_stop": self._emergency,
            "owner": self._owner_id,
            "last_sequence": self._last_sequence,
            "last_reason": self._last_reason,
            "fault": self._fault,
            "x_cm_s": self._target[0],
            "y_cm_s": self._target[1],
            "yaw_rate_rad_s": self._target[2],
        }

    # ── 패킷 처리 ──

    def handle_datagram(self, data: bytes, address, now: float) -> V2Result:
        try:
            packet = parse_v2_packet(data)
        except V2PacketError as exc:
            return self._reject(exc.client_id, exc.sequence, exc.reason, None, str(exc))
        return self.handle_packet(packet, address, now)

    def handle_packet(self, packet: V2Packet, address, now: float) -> V2Result:
        sid = packet.session_id
        if sid in self._retired:
            return self._reject(packet.client_id, packet.sequence, "not_owner", None,
                                "종료되었거나 교체된 세션")

        claimed = None
        if self._owner_id is None:
            if packet.action == "disconnect":
                self._retire(sid)
                if self._previous_owner is not None and self._previous_owner[0] == sid:
                    self._previous_owner = None
                return self._accept(packet, None, None)
            baseline = -1
            if self._previous_owner is not None and self._previous_owner[0] == sid:
                baseline = self._previous_owner[1]
            if packet.sequence <= baseline:
                return self._reject(packet.client_id, packet.sequence, "out_of_order",
                                    None, "sequence 역전 또는 중복")
            if self._previous_owner is not None and self._previous_owner[0] != sid:
                self._retire(self._previous_owner[0])
            self._previous_owner = None
            self._owner_id = sid
            self._owner_address = address
            self._last_sequence = baseline
            self._reset_motion()
            claimed = "조종자 획득: %s %s:%d" % (sid, address[0], address[1])
        elif sid != self._owner_id or address != self._owner_address:
            return self._reject(packet.client_id, packet.sequence, "not_owner", None,
                                "다른 조종자가 제어 중")

        if packet.sequence <= self._last_sequence:
            return self._reject(packet.client_id, packet.sequence, "out_of_order", None,
                                "sequence 역전 또는 중복")
        self._last_sequence = packet.sequence
        self._last_owner_activity = now

        result = self._apply(packet, now)
        if claimed is not None:
            event = claimed if result.event is None else "%s; %s" % (claimed, result.event)
            result = V2Result(result.ack, result.velocity, event)
        return result

    def _apply(self, packet: V2Packet, now: float) -> V2Result:
        if packet.action == "emergency_stop":
            was_latched = self._emergency
            self._emergency = True
            self._reset_motion()
            return self._accept(packet, STOP, None if was_latched else "비상정지 latch")
        if packet.action == "clear_emergency":
            was_latched = self._emergency
            self._emergency = False
            self._reset_motion()
            return self._accept(
                packet,
                STOP,
                "비상정지 해제; 새 중립 대기" if was_latched else None,
            )
        if packet.action == "disconnect":
            self._retire(self._owner_id)
            self._release_owner()
            return self._accept(packet, STOP, "조종자 disconnect; 소유권 해제")

        if packet.is_neutral:
            event = None
            if self._fault is None and not self._emergency:
                if not self._neutral_confirmed:
                    event = "중립 확인"
                self._neutral_confirmed = True
                self._timed_out = False
                self._active = packet.control_active
                self._last_command_rx = now
            self._target = STOP
            return self._accept(packet, STOP, event)

        if self._fault is not None:
            return self._reject(packet.client_id, packet.sequence, "fault", STOP, None)
        if self._emergency:
            return self._reject(packet.client_id, packet.sequence, "emergency_stop",
                                STOP, None)
        if not self._neutral_confirmed:
            return self._reject(packet.client_id, packet.sequence, "neutral_required",
                                STOP, None)
        self._active = True
        self._target = normalized_to_velocity(
            packet.forward,
            packet.turn,
            self._max_forward,
            self._max_reverse,
            self._max_yaw,
        )
        self._last_command_rx = now
        return self._accept(packet, self._target, None)

    # ── 시간 기반 정지 ──

    def check_timeouts(self, now: float) -> List[str]:
        """watchdog/lease 만료 시 정지해야 할 이유 목록을 돌려줍니다."""
        events = []
        if (
            self._last_command_rx is not None
            and now - self._last_command_rx >= self._command_timeout
        ):
            self._reset_motion()
            self._timed_out = True
            events.append("%.3f초 command watchdog 만료" % self._command_timeout)
        if (
            self._owner_id is not None
            and now - self._last_owner_activity >= self._lease_timeout
        ):
            self._previous_owner = (self._owner_id, self._last_sequence)
            self._release_owner()
            events.append("%.3f초 조종자 lease 만료" % self._lease_timeout)
        return events

    def force_stop(self):
        """외부 정지(종료·영상 변경 등). 소유권은 유지하고 새 중립을 요구합니다."""
        self._reset_motion()

    def enter_fault(self, reason: str):
        """재시작 전까지 이동 명령을 거부하는 fault latch."""
        self._fault = reason
        self._reset_motion()

    # ── 내부 ──

    def _reset_motion(self):
        self._target = STOP
        self._last_command_rx = None
        self._neutral_confirmed = False
        self._timed_out = False
        self._active = False

    def _release_owner(self):
        self._owner_id = None
        self._owner_address = None
        self._last_sequence = -1
        self._last_owner_activity = 0.0
        self._reset_motion()

    def _retire(self, session_id: str):
        self._retired[session_id] = True
        self._retired.move_to_end(session_id)
        while len(self._retired) > self._retired_limit:
            self._retired.popitem(last=False)

    def _ack(self, client_id, sequence, accepted, reason) -> dict:
        self._last_reason = reason
        return {
            "protocol": PROTOCOL_V2,
            "type": "ack",
            "client_id": client_id,
            "sequence": sequence,
            "accepted": accepted,
            "reason": reason,
            "emergency_stop": self._emergency,
            "state": self.state,
        }

    def _accept(self, packet, velocity, event) -> V2Result:
        return V2Result(self._ack(packet.client_id, packet.sequence, True, "ok"),
                        velocity, event)

    def _reject(self, client_id, sequence, reason, velocity, detail) -> V2Result:
        return V2Result(self._ack(client_id, sequence, False, reason), velocity, detail)


class RemoteControlServer:
    """Owns the UDP socket, client lease, emergency latch and watchdog."""

    def __init__(
        self,
        config: dict,
        publish_velocity: Callable[[float, float, float], None],
        on_state_change: Optional[Callable[[dict], None]] = None,
    ):
        self._bind_address = str(config.get("bind_address", "0.0.0.0"))
        self._port = int(config["control_port"])
        self._protocol_version = int(config.get("protocol_version", 1))
        self._command_timeout = float(config.get("command_timeout_seconds", 0.3))
        self._client_lease_timeout = float(
            config.get("client_lease_timeout_seconds", 1.0)
        )
        self._max_packet_age = float(config.get("max_packet_age_seconds", 0.3))
        self._max_future = float(config.get("max_future_seconds", 1.0))
        self._stop_repetitions = int(config.get("stop_repetitions", 3))

        limits = config["velocity_limits"]
        self._max_forward = float(limits["max_forward_cm_s"])
        self._max_reverse = float(limits["max_reverse_cm_s"])
        self._max_yaw = float(limits["max_yaw_rate_rad_s"])
        self._validate_config()

        self._publish_velocity = publish_velocity
        self._on_state_change = on_state_change
        self._lock = threading.Lock()
        # 모든 속도 발행을 직렬화합니다. fault 이후에는 0이 아닌 값을 내보내지 않습니다.
        self._output_lock = threading.RLock()
        self._fault = None  # type: Optional[str]
        self._socket = None
        self._thread = None
        self._running = threading.Event()

        self._video_client_ip = None
        self._owner_id = None
        self._owner_address = None
        self._last_sequence = -1
        self._last_client_activity = 0.0
        self._last_command_rx = 0.0
        self._watchdog_stopped = True
        self._emergency_stop = False
        self._last_invalid_log = 0.0

        self._v2 = None  # type: Optional[V2Session]
        self._last_state_key = None
        if self._protocol_version == PROTOCOL_V2:
            self._v2 = V2Session(
                self._max_forward,
                self._max_reverse,
                self._max_yaw,
                self._command_timeout,
                self._client_lease_timeout,
            )

    @property
    def protocol_version(self) -> int:
        return self._protocol_version

    @property
    def emergency_stop(self) -> bool:
        with self._lock:
            if self._v2 is not None:
                return self._v2.emergency_stop
            return self._emergency_stop

    def state_snapshot(self) -> dict:
        """v2 제어기 상태(ROS 상태 토픽·시험용). v1은 최소 정보만 돌려줍니다."""
        with self._lock:
            if self._v2 is not None:
                return self._v2.snapshot()
            return {
                "state": "emergency_stop" if self._emergency_stop else (
                    "fault" if self._fault is not None else "v1"
                ),
                "emergency_stop": self._emergency_stop,
                "owner": self._owner_id,
                "last_sequence": self._last_sequence,
                "fault": self._fault,
            }

    def _validate_config(self):
        if self._protocol_version not in (1, PROTOCOL_V2):
            raise ValueError("protocol_version은 1 또는 2여야 합니다")
        positive = {
            "command_timeout_seconds": self._command_timeout,
            "client_lease_timeout_seconds": self._client_lease_timeout,
            "max_packet_age_seconds": self._max_packet_age,
            "max_future_seconds": self._max_future,
            "max_forward_cm_s": self._max_forward,
            "max_reverse_cm_s": self._max_reverse,
            "max_yaw_rate_rad_s": self._max_yaw,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError("%s 설정은 0보다 큰 유한한 값이어야 합니다" % name)
        if self._command_timeout > 0.3:
            raise ValueError("command_timeout_seconds는 0.3 이하여야 합니다")
        if self._client_lease_timeout <= self._command_timeout:
            raise ValueError(
                "client_lease_timeout_seconds는 command_timeout_seconds보다 커야 합니다"
            )
        if not 1 <= self._port <= 65535:
            raise ValueError("control_port 범위 오류")
        if self._stop_repetitions < 1:
            raise ValueError("stop_repetitions는 1 이상이어야 합니다")

    def start(self):
        if self._running.is_set():
            return
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # SO_REUSEADDR를 켜면 Linux에서 두 서버가 같은 UDP 포트에 동시에 bind되어
        # 명령이 어느 쪽으로 갈지 알 수 없습니다. 중복 실행은 bind 오류로 드러냅니다.
        try:
            udp_socket.bind((self._bind_address, self._port))
        except OSError:
            udp_socket.close()
            raise
        udp_socket.settimeout(min(0.05, self._command_timeout / 3.0))
        self._socket = udp_socket
        self._running.set()
        self.publish_stop_repeated("프로그램 시작")
        self._thread = threading.Thread(
            target=self._receive_loop,
            name="control-udp",
            daemon=True,
        )
        self._thread.start()
        LOGGER.info(
            "조종 UDP 수신: %s:%d (protocol %d)",
            self._bind_address,
            self.bound_port,
            self._protocol_version,
        )
        self._notify_state()

    @property
    def bound_port(self) -> int:
        udp_socket = self._socket
        if udp_socket is None:
            return self._port
        return int(udp_socket.getsockname()[1])

    def set_video_client(self, client_ip: str):
        """Prefer the streaming laptop IP without coupling both connections."""
        if self._v2 is not None:
            self.force_stop("v2에서는 TCP 영상 연결과 조종을 결합하지 않습니다")
            return
        with self._lock:
            self._video_client_ip = client_ip
            if (
                self._owner_address is not None
                and self._owner_address[0] != client_ip
            ):
                self._clear_owner_locked()
        self._publish_stop()
        LOGGER.warning("안전 정지: 영상 클라이언트 연결 변경")
        LOGGER.info("제어 허용 노트북 IP: %s", client_ip)

    def clear_video_client(self, client_ip: str):
        if self._v2 is not None:
            self.force_stop("영상 연결 종료")
            return
        with self._lock:
            if self._video_client_ip == client_ip:
                self._video_client_ip = None
                self._last_command_rx = 0.0
                self._watchdog_stopped = True
        self._publish_stop()
        LOGGER.warning("영상 연결 종료: %s; 로봇 정지", client_ip)

    def force_stop(self, reason: str):
        with self._lock:
            self._last_command_rx = 0.0
            self._watchdog_stopped = True
            if self._v2 is not None:
                self._v2.force_stop()
        self._publish_stop()
        LOGGER.warning("안전 정지: %s", reason)
        self._notify_state()

    def enter_fault(self, reason: str):
        """출력 계층 이상. 재시작 전까지 0이 아닌 속도를 발행하지 않습니다."""
        with self._lock:
            first = self._fault is None
            self._fault = reason
            self._last_command_rx = 0.0
            self._watchdog_stopped = True
            if self._v2 is not None:
                self._v2.enter_fault(reason)
        self._publish_stop()
        if first:
            LOGGER.error("fault latch: %s; 재시작 전까지 이동 명령을 거부합니다", reason)
        self._notify_state()

    def publish_stop_repeated(self, reason: str):
        LOGGER.warning("반복 안전 정지: %s", reason)
        for _ in range(self._stop_repetitions):
            self._publish_stop()
            time.sleep(0.02)

    def shutdown(self):
        if not self._running.is_set():
            self.publish_stop_repeated("프로그램 종료")
            return
        self._running.clear()
        udp_socket = self._socket
        self._socket = None
        if udp_socket is not None:
            try:
                udp_socket.close()
            except OSError:
                pass
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        with self._lock:
            if self._v2 is not None:
                self._v2.force_stop()
        self.publish_stop_repeated("프로그램 종료")

    def _receive_loop(self):
        # 한 패킷의 예외가 이 스레드를 끝내면 watchdog 검사도 함께 멈춥니다.
        # 그래서 패킷 처리와 timeout 검사를 각각 보호하고 루프는 계속 돕니다.
        while self._running.is_set():
            udp_socket = self._socket
            if udp_socket is None:
                break
            try:
                data, address = udp_socket.recvfrom(MAX_PACKET_BYTES + 1)
            except socket.timeout:
                self._guarded_check_timeouts()
                continue
            except OSError as exc:
                if not self._running.is_set():
                    break
                LOGGER.warning("조종 UDP 수신 오류: %s", exc)
                self._guarded_check_timeouts()
                time.sleep(0.01)
                continue
            try:
                self._handle_datagram(data, address)
            except Exception:
                LOGGER.exception("조종 패킷 처리 중 예기치 않은 오류")
                self.force_stop("조종 패킷 처리 오류")
            self._guarded_check_timeouts()

    def _guarded_check_timeouts(self):
        try:
            self._check_timeouts()
        except Exception:
            LOGGER.exception("watchdog 검사 오류")
            self.force_stop("watchdog 검사 오류")

    def _handle_datagram(self, data: bytes, address: Tuple[str, int]):
        if self._v2 is not None:
            self._handle_datagram_v2(data, address)
        else:
            self._handle_datagram_v1(data, address)

    def _handle_datagram_v2(self, data: bytes, address: Tuple[str, int]):
        now = time.monotonic()
        output_error = None
        with self._lock:
            result = self._v2.handle_datagram(data, address, now)
            ack = result.ack
            if result.velocity is not None:
                try:
                    self._emit(*result.velocity)
                except Exception as exc:
                    output_error = "속도 발행 실패: %s" % exc
                    self._v2.enter_fault(output_error)
                    self._fault = output_error
                    ack = dict(
                        ack,
                        accepted=False,
                        reason="fault",
                        emergency_stop=self._v2.emergency_stop,
                        state=self._v2.state,
                    )
        if output_error is not None:
            LOGGER.error("fault latch: %s", output_error)
            self._publish_stop()
        self._send_payload(address, ack)
        if not ack["accepted"]:
            self._log_invalid(address, "%s (%s)" % (ack["reason"], result.event or "-"))
        elif result.event is not None:
            LOGGER.warning("%s", result.event)
        self._notify_state()

    def _handle_datagram_v1(self, data: bytes, address: Tuple[str, int]):
        try:
            packet = parse_control_packet(data, self._protocol_version)
            now_wall = time.time()
            age = now_wall - packet.timestamp
            if age > self._max_packet_age:
                raise PacketError("오래된 timestamp")
            if age < -self._max_future:
                raise PacketError("미래 timestamp")

            publish = None
            stop_reason = None
            accepted_reason = "ok"
            now_mono = time.monotonic()
            with self._lock:
                if (
                    self._video_client_ip is not None
                    and address[0] != self._video_client_ip
                ):
                    raise PacketError("현재 영상 클라이언트 IP가 아님")

                if self._owner_id is None:
                    self._owner_id = packet.client_id
                    self._owner_address = address
                    self._last_sequence = -1
                elif (
                    packet.client_id != self._owner_id
                    or address != self._owner_address
                ):
                    raise PacketError("다른 노트북이 이미 제어 중")

                if packet.sequence <= self._last_sequence:
                    raise PacketError("sequence 역전 또는 중복")

                self._last_sequence = packet.sequence
                self._last_client_activity = now_mono

                if packet.action == "emergency_stop":
                    self._emergency_stop = True
                    self._last_command_rx = 0.0
                    self._watchdog_stopped = True
                    stop_reason = "비상정지 수신"
                elif packet.action == "clear_emergency":
                    self._emergency_stop = False
                    self._last_command_rx = 0.0
                    self._watchdog_stopped = True
                    stop_reason = "비상정지 해제; 새 명령 대기"
                elif packet.action == "disconnect":
                    self._last_command_rx = 0.0
                    self._watchdog_stopped = True
                    self._clear_owner_locked()
                    stop_reason = "클라이언트 정상 종료"
                elif self._emergency_stop:
                    accepted_reason = "비상정지 상태이므로 이동 명령 무시"
                    self._last_command_rx = 0.0
                    self._watchdog_stopped = True
                    stop_reason = accepted_reason
                elif self._fault is not None:
                    accepted_reason = "fault 상태이므로 이동 명령 무시: %s" % self._fault
                    self._last_command_rx = 0.0
                    self._watchdog_stopped = True
                    stop_reason = accepted_reason
                else:
                    publish = clamp_velocity(
                        packet.x,
                        packet.yaw_rate,
                        self._max_forward,
                        self._max_reverse,
                        self._max_yaw,
                    )
                    self._last_command_rx = now_mono
                    self._watchdog_stopped = False

            if publish is not None:
                self._emit(*publish)
            if stop_reason is not None:
                self._publish_stop()
            self._send_ack(address, packet.sequence, True, accepted_reason)
        except (PacketError, ValueError) as exc:
            self._log_invalid(address, str(exc))
            self._send_ack(address, -1, False, str(exc))

    def _check_timeouts(self):
        if self._v2 is not None:
            with self._lock:
                events = self._v2.check_timeouts(time.monotonic())
                if events:
                    self._emit(*STOP)
            for event in events:
                LOGGER.warning("%s; 로봇 정지", event)
            if events:
                self._notify_state()
            return

        now = time.monotonic()
        stop_for_watchdog = False
        release_owner = False
        with self._lock:
            if (
                not self._watchdog_stopped
                and self._last_command_rx > 0
                and now - self._last_command_rx >= self._command_timeout
            ):
                self._watchdog_stopped = True
                self._last_command_rx = 0.0
                stop_for_watchdog = True

            if (
                self._owner_id is not None
                and self._last_client_activity > 0
                and now - self._last_client_activity >= self._client_lease_timeout
            ):
                self._clear_owner_locked()
                release_owner = True

        if stop_for_watchdog:
            self._publish_stop()
            LOGGER.warning(
                "%.3f초 명령 watchdog 만료; 로봇 정지",
                self._command_timeout,
            )
        if release_owner:
            self._publish_stop()
            LOGGER.warning("제어 클라이언트 lease 만료")

    def _clear_owner_locked(self):
        self._owner_id = None
        self._owner_address = None
        self._last_sequence = -1
        self._last_client_activity = 0.0
        self._last_command_rx = 0.0
        self._watchdog_stopped = True

    def _emit(self, x: float, y: float, yaw_rate: float):
        with self._output_lock:
            if self._fault is not None:
                x, y, yaw_rate = STOP
            self._publish_velocity(x, y, yaw_rate)

    def _publish_stop(self):
        try:
            self._emit(*STOP)
        except Exception:
            LOGGER.exception("정지 명령 발행 실패")

    def _notify_state(self):
        callback = self._on_state_change
        if callback is None:
            return
        snapshot = self.state_snapshot()
        key = (
            snapshot.get("state"),
            snapshot.get("emergency_stop"),
            snapshot.get("owner"),
            snapshot.get("fault"),
        )
        if key == self._last_state_key:
            return
        self._last_state_key = key
        try:
            callback(snapshot)
        except Exception:
            LOGGER.exception("상태 알림 실패")

    def _send_payload(self, address: Tuple[str, int], payload: dict):
        udp_socket = self._socket
        if udp_socket is None:
            return
        response = json.dumps(
            payload,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        try:
            udp_socket.sendto(response, address)
        except OSError:
            pass

    def _send_ack(
        self,
        address: Tuple[str, int],
        sequence: int,
        accepted: bool,
        reason: str,
    ):
        udp_socket = self._socket
        if udp_socket is None:
            return
        with self._lock:
            emergency = self._emergency_stop
        response = json.dumps(
            {
                "protocol": self._protocol_version,
                "type": "ack",
                "sequence": sequence,
                "accepted": accepted,
                "reason": reason,
                "emergency_stop": emergency,
                "robot_timestamp": time.time(),
            },
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        try:
            udp_socket.sendto(response, address)
        except OSError:
            pass

    def _log_invalid(self, address: Tuple[str, int], reason: str):
        now = time.monotonic()
        if now - self._last_invalid_log >= 1.0:
            self._last_invalid_log = now
            LOGGER.warning("조종 패킷 거부 %s:%d: %s", address[0], address[1], reason)
