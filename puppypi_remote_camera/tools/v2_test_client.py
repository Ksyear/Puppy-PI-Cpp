#!/usr/bin/env python3
"""Quest 없이 UDP 5001 JSON v2를 보내고 ACK를 판정하는 시험 클라이언트.

ROS·Unity 없이 표준 라이브러리만 사용합니다. 서버(시험 모드)를 띄운 Pi에서
127.0.0.1로, 또는 같은 네트워크의 노트북에서 Pi IP로 실행합니다.

  python3 v2_test_client.py --robot 127.0.0.1 --scenario all
  python3 v2_test_client.py --robot <PI_IP> --scenario drive --forward 0.6 --turn -0.3

시나리오
  drive     중립 -> 활성(forward, turn) -> 중립 -> disconnect
  watchdog  활성 송신을 끊은 뒤 timeout 상태와 neutral_required를 확인
  estop     비상정지 latch, 해제 후 중립 요구를 확인
  lease     1초 넘게 끊은 뒤 소유권 해제와 재획득을 확인
  all       위 네 가지를 차례로 실행 (각각 새 UUID)
"""

import argparse
import json
import socket
import sys
import time
import uuid

SEND_PERIOD = 0.05  # 20Hz


class V2Client:
    def __init__(self, host: str, port: int, ack_timeout: float = 0.2):
        self.client_id = str(uuid.uuid4())
        self.sequence = 0
        self.address = (host, port)
        self.ack_timeout = ack_timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", 0))

    def close(self):
        self.sock.close()

    def send(self, payload_type: str, **fields):
        payload = {
            "protocol": 2,
            "type": payload_type,
            "client_id": self.client_id,
            "sequence": self.sequence,
        }
        payload.update(fields)
        self.sequence += 1
        self.sock.sendto(json.dumps(payload).encode("utf-8"), self.address)
        return self._wait_ack(payload["sequence"])

    def command(self, control_active: bool, forward: float, turn: float):
        return self.send(
            "command",
            control_active=control_active,
            forward=float(forward),
            turn=float(turn),
        )

    def neutral(self):
        return self.command(False, 0.0, 0.0)

    def _wait_ack(self, sequence: int):
        """현재 UUID와 방금 보낸 번호에 대응하는 새 ACK만 인정합니다."""
        deadline = time.monotonic() + self.ack_timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self.sock.settimeout(remaining)
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                return None
            try:
                ack = json.loads(data.decode("utf-8"))
            except ValueError:
                continue
            if (
                isinstance(ack, dict)
                and ack.get("type") == "ack"
                and ack.get("client_id") == self.client_id
                and ack.get("sequence") == sequence
            ):
                return ack


class Checker:
    def __init__(self, name: str):
        self.name = name
        self.failures = 0

    def expect(self, description: str, ack, **expected):
        ok = ack is not None and all(ack.get(k) == v for k, v in expected.items())
        if not ok:
            self.failures += 1
        shown = "ACK 없음" if ack is None else "accepted=%s reason=%s state=%s" % (
            ack.get("accepted"), ack.get("reason"), ack.get("state"))
        print("[%s] %s %s -> %s" % (self.name, "PASS" if ok else "FAIL", description, shown))
        return ok


def stream(client: V2Client, seconds: float, control_active: bool, forward: float,
           turn: float):
    acks = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        acks.append(client.command(control_active, forward, turn))
        time.sleep(SEND_PERIOD)
    return acks


def summarize(checker: Checker, description: str, acks, **expected):
    received = [ack for ack in acks if ack is not None]
    ok = len(received) == len(acks) and all(
        all(ack.get(k) == v for k, v in expected.items()) for ack in received
    )
    if not ok:
        checker.failures += 1
    states = sorted({ack.get("state") for ack in received})
    reasons = sorted({ack.get("reason") for ack in received})
    print("[%s] %s %s: 송신 %d, ACK %d, state=%s reason=%s" % (
        checker.name, "PASS" if ok else "FAIL", description, len(acks), len(received),
        states, reasons))


def scenario_drive(host, port, forward, turn, seconds):
    checker = Checker("drive")
    client = V2Client(host, port)
    try:
        checker.expect("시작 직후 활성 명령은 거부", client.command(True, forward, turn),
                       accepted=False, reason="neutral_required")
        summarize(checker, "중립", stream(client, 0.3, False, 0.0, 0.0),
                  accepted=True, state="idle")
        summarize(checker, "활성 forward=%.2f turn=%.2f" % (forward, turn),
                  stream(client, seconds, True, forward, turn),
                  accepted=True, state="active")
        summarize(checker, "중립 복귀", stream(client, 0.3, False, 0.0, 0.0),
                  accepted=True, state="idle")
        checker.expect("disconnect", client.send("disconnect"),
                       accepted=True, state="awaiting_neutral")
    finally:
        client.close()
    return checker.failures


def scenario_watchdog(host, port, forward, turn, seconds):
    checker = Checker("watchdog")
    client = V2Client(host, port)
    try:
        stream(client, 0.2, False, 0.0, 0.0)
        summarize(checker, "활성", stream(client, min(seconds, 1.0), True, forward, turn),
                  accepted=True, state="active")
        print("[watchdog] 0.5초 동안 송신 중단 (서버는 0.25초 뒤 정지해야 함)")
        time.sleep(0.5)
        checker.expect("중단 뒤 활성 명령", client.command(True, forward, turn),
                       accepted=False, reason="neutral_required", state="timeout")
        checker.expect("새 중립", client.neutral(), accepted=True, state="idle")
        checker.expect("중립 뒤 활성", client.command(True, forward, turn),
                       accepted=True, state="active")
        client.neutral()
        client.send("disconnect")
    finally:
        client.close()
    return checker.failures


def scenario_estop(host, port, forward, turn, seconds):
    checker = Checker("estop")
    client = V2Client(host, port)
    try:
        stream(client, 0.2, False, 0.0, 0.0)
        stream(client, 0.3, True, forward, turn)
        checker.expect("emergency_stop", client.send("emergency_stop"),
                       accepted=True, emergency_stop=True, state="emergency_stop")
        checker.expect("비상정지 중 활성", client.command(True, forward, turn),
                       accepted=False, reason="emergency_stop")
        checker.expect("비상정지 중 중립", client.neutral(),
                       accepted=True, state="emergency_stop")
        checker.expect("clear_emergency", client.send("clear_emergency"),
                       accepted=True, emergency_stop=False, state="awaiting_neutral")
        checker.expect("해제 직후 활성", client.command(True, forward, turn),
                       accepted=False, reason="neutral_required")
        checker.expect("새 중립", client.neutral(), accepted=True, state="idle")
        checker.expect("중립 뒤 활성", client.command(True, forward, turn),
                       accepted=True, state="active")
        client.neutral()
        client.send("disconnect")
    finally:
        client.close()
    return checker.failures


def scenario_lease(host, port, forward, turn, seconds):
    checker = Checker("lease")
    client = V2Client(host, port)
    other = V2Client(host, port)
    try:
        stream(client, 0.2, False, 0.0, 0.0)
        checker.expect("소유 중 다른 UUID", other.neutral(),
                       accepted=False, reason="not_owner")
        print("[lease] 1.2초 동안 송신 중단 (서버 lease 1초)")
        time.sleep(1.2)
        checker.expect("lease 만료 뒤 같은 UUID 활성", client.command(True, forward, turn),
                       accepted=False, reason="neutral_required", state="awaiting_neutral")
        checker.expect("같은 UUID 중립으로 재획득", client.neutral(),
                       accepted=True, state="idle")
        client.send("disconnect")
        checker.expect("disconnect한 UUID 재사용", client.neutral(),
                       accepted=False, reason="not_owner")
    finally:
        client.close()
        other.close()
    return checker.failures


SCENARIOS = {
    "drive": scenario_drive,
    "watchdog": scenario_watchdog,
    "estop": scenario_estop,
    "lease": scenario_lease,
}


def run(host, port, scenario, forward, turn, seconds) -> int:
    names = list(SCENARIOS) if scenario == "all" else [scenario]
    failures = 0
    for name in names:
        failures += SCENARIOS[name](host, port, forward, turn, seconds)
        time.sleep(0.1)
    print("결과: %s (실패 %d)" % ("PASS" if failures == 0 else "FAIL", failures))
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot", default="127.0.0.1", help="Pi IP (기본 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--scenario", default="all", choices=sorted(SCENARIOS) + ["all"])
    parser.add_argument("--forward", type=float, default=0.6)
    parser.add_argument("--turn", type=float, default=-0.3)
    parser.add_argument("--seconds", type=float, default=2.0, help="활성 명령 송신 시간")
    args = parser.parse_args()
    if not (-1.0 <= args.forward <= 1.0 and -1.0 <= args.turn <= 1.0):
        parser.error("--forward/--turn은 -1~1")
    if args.forward == 0.0 and args.turn == 0.0:
        parser.error("활성 시나리오에는 0이 아닌 --forward 또는 --turn이 필요합니다")
    return 1 if run(args.robot, args.port, args.scenario, args.forward, args.turn,
                    args.seconds) else 0


if __name__ == "__main__":
    sys.exit(main())
