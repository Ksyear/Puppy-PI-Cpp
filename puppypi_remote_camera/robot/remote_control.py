#!/usr/bin/env python3
"""Validated single-client UDP control with a robot-side safety watchdog."""

import json
import logging
import math
import socket
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Callable, Tuple


LOGGER = logging.getLogger(__name__)
MAX_PACKET_BYTES = 2048


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


class RemoteControlServer:
    """Owns the UDP socket, client lease, emergency latch and watchdog."""

    def __init__(
        self,
        config: dict,
        publish_velocity: Callable[[float, float, float], None],
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
        self._lock = threading.Lock()
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

    @property
    def emergency_stop(self) -> bool:
        with self._lock:
            return self._emergency_stop

    def _validate_config(self):
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
        if not 1 <= self._port <= 65535:
            raise ValueError("control_port 범위 오류")
        if self._stop_repetitions < 1:
            raise ValueError("stop_repetitions는 1 이상이어야 합니다")

    def start(self):
        if self._running.is_set():
            return
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_socket.bind((self._bind_address, self._port))
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
        LOGGER.info("조종 UDP 수신: %s:%d", self._bind_address, self._port)

    def set_video_client(self, client_ip: str):
        """Authorize control packets only from the currently streaming laptop."""
        with self._lock:
            self._video_client_ip = client_ip
            self._clear_owner_locked()
        self._publish_stop()
        LOGGER.warning("안전 정지: 영상 클라이언트 연결 변경")
        LOGGER.info("제어 허용 노트북 IP: %s", client_ip)

    def clear_video_client(self, client_ip: str):
        with self._lock:
            if self._video_client_ip == client_ip:
                self._video_client_ip = None
                self._clear_owner_locked()
        self._publish_stop()
        LOGGER.warning("영상 연결 종료: %s; 로봇 정지", client_ip)

    def force_stop(self, reason: str):
        with self._lock:
            self._last_command_rx = 0.0
            self._watchdog_stopped = True
        self._publish_stop()
        LOGGER.warning("안전 정지: %s", reason)

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
        self.publish_stop_repeated("프로그램 종료")

    def _receive_loop(self):
        while self._running.is_set():
            try:
                data, address = self._socket.recvfrom(MAX_PACKET_BYTES + 1)
            except socket.timeout:
                self._check_timeouts()
                continue
            except OSError:
                break
            self._handle_datagram(data, address)
            self._check_timeouts()

    def _handle_datagram(self, data: bytes, address: Tuple[str, int]):
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
                if self._video_client_ip is None or address[0] != self._video_client_ip:
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
                self._publish_velocity(*publish)
            if stop_reason is not None:
                self._publish_stop()
            self._send_ack(address, packet.sequence, True, accepted_reason)
        except (PacketError, ValueError) as exc:
            self._log_invalid(address, str(exc))
            self._send_ack(address, -1, False, str(exc))

    def _check_timeouts(self):
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

    def _publish_stop(self):
        try:
            self._publish_velocity(0.0, 0.0, 0.0)
        except Exception:
            LOGGER.exception("정지 명령 발행 실패")

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
