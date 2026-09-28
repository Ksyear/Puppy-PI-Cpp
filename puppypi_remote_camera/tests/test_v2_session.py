"""v2 controller state machine with an explicit clock (ROS·소켓 불필요)."""

import json
import unittest

from _paths import CLIENT_ID, OTHER_ID
from remote_control import STOP, V2_REASONS, V2_STATES, V2Session

ADDRESS = ("192.168.0.50", 40000)
OTHER_ADDRESS = ("192.168.0.51", 40000)
ACK_KEYS = [
    "protocol",
    "type",
    "client_id",
    "sequence",
    "accepted",
    "reason",
    "emergency_stop",
    "state",
]


class Harness:
    def __init__(self):
        self.session = V2Session(5.0, 3.0, 0.20, 0.25, 1.0)
        # 0에서 시작해 경계값(0.25s, 1.0s) 비교가 부동소수 오차 없이 정확합니다.
        self.now = 0.0
        self.sequences = {}

    def _send(self, payload, client_id, address, sequence):
        if sequence is None:
            sequence = self.sequences.get(client_id, -1) + 1
        self.sequences[client_id] = max(self.sequences.get(client_id, -1), sequence)
        payload.update({"protocol": 2, "client_id": client_id, "sequence": sequence})
        data = json.dumps(payload).encode("utf-8")
        return self.session.handle_datagram(data, address, self.now)

    def command(self, active=True, forward=0.6, turn=-0.3, client_id=CLIENT_ID,
                address=ADDRESS, sequence=None):
        payload = {
            "type": "command",
            "control_active": active,
            "forward": forward,
            "turn": turn,
        }
        return self._send(payload, client_id, address, sequence)

    def neutral(self, **kwargs):
        return self.command(active=False, forward=0.0, turn=0.0, **kwargs)

    def event(self, action, client_id=CLIENT_ID, address=ADDRESS, sequence=None):
        return self._send({"type": action}, client_id, address, sequence)

    def advance(self, seconds):
        self.now += seconds
        return self.session.check_timeouts(self.now)


class V2SessionTests(unittest.TestCase):
    def setUp(self):
        self.h = Harness()

    def assertAck(self, result, accepted, reason, state=None):
        ack = result.ack
        self.assertEqual(list(ack), ACK_KEYS)
        self.assertEqual((ack["protocol"], ack["type"]), (2, "ack"))
        self.assertIn(ack["reason"], V2_REASONS)
        self.assertIn(ack["state"], V2_STATES)
        self.assertEqual((ack["accepted"], ack["reason"]), (accepted, reason), ack)
        if state is not None:
            self.assertEqual(ack["state"], state, ack)

    def test_starts_stopped_and_awaiting_neutral(self):
        self.assertEqual(self.h.session.state, "awaiting_neutral")
        self.assertEqual(self.h.session.target, STOP)

    def test_active_before_neutral_is_rejected(self):
        result = self.h.command()
        self.assertAck(result, False, "neutral_required", "awaiting_neutral")
        self.assertEqual(result.velocity, STOP)

    def test_neutral_then_active_is_converted(self):
        self.assertAck(self.h.neutral(), True, "ok", "idle")
        result = self.h.command(forward=0.6, turn=-0.3)
        self.assertAck(result, True, "ok", "active")
        x, y, yaw = result.velocity
        self.assertAlmostEqual(x, 3.0)
        self.assertEqual(y, 0.0)
        self.assertAlmostEqual(yaw, -0.06)

    def test_ack_echoes_uuid_and_sequence(self):
        result = self.h.neutral(sequence=41)
        self.assertEqual((result.ack["client_id"], result.ack["sequence"]), (CLIENT_ID, 41))

    def test_gripped_center_counts_as_neutral(self):
        result = self.h.command(active=True, forward=0.0, turn=0.0)
        self.assertAck(result, True, "ok", "active")
        self.assertEqual(result.velocity, STOP)
        self.assertAck(self.h.command(), True, "ok", "active")

    def test_control_active_false_overrides_axes(self):
        self.h.neutral()
        result = self.h.command(active=False, forward=0.9, turn=0.9)
        self.assertAck(result, True, "ok", "idle")
        self.assertEqual(result.velocity, STOP)

    def test_duplicate_and_reordered_packets_are_not_applied(self):
        self.h.neutral(sequence=10)
        self.h.command(sequence=11, forward=0.2)
        self.h.now += 0.1
        duplicate = self.h.command(sequence=11, forward=1.0)
        self.assertAck(duplicate, False, "out_of_order")
        self.assertIsNone(duplicate.velocity)
        older = self.h.command(sequence=10, forward=1.0)
        self.assertAck(older, False, "out_of_order")
        self.assertAlmostEqual(self.h.session.target[0], 1.0)  # 0.2 * 5cm/s 유지
        # 중복·역순 패킷은 watchdog을 갱신하지 않습니다.
        self.assertTrue(self.h.advance(0.16))
        self.assertEqual(self.h.session.target, STOP)

    def test_watchdog_stops_after_250ms_and_requires_neutral(self):
        self.h.neutral()
        self.h.command()
        self.assertEqual(self.h.advance(0.249), [])
        self.assertEqual(self.h.session.state, "active")
        events = self.h.advance(0.001)
        self.assertEqual(len(events), 1)
        self.assertEqual(self.h.session.target, STOP)
        self.assertEqual(self.h.session.state, "timeout")
        self.assertAck(self.h.command(), False, "neutral_required", "timeout")
        self.assertAck(self.h.neutral(), True, "ok", "idle")
        self.assertAck(self.h.command(), True, "ok", "active")

    def test_neutral_commands_refresh_watchdog(self):
        self.h.neutral()
        for _ in range(20):
            self.assertEqual(self.h.advance(0.05), [])
            self.h.neutral()
        self.assertEqual(self.h.session.state, "idle")

    def test_idle_watchdog_also_requires_new_neutral(self):
        self.h.neutral()
        self.assertTrue(self.h.advance(0.25))
        self.assertEqual(self.h.session.state, "timeout")
        self.assertAck(self.h.command(), False, "neutral_required")

    def test_invalid_packets_refresh_neither_watchdog_nor_lease(self):
        self.h.neutral()
        self.h.command()
        for _ in range(4):
            self.h.now += 0.05
            result = self.h.session.handle_datagram(b"{broken", ADDRESS, self.h.now)
            self.assertAck(result, False, "invalid_packet")
            self.assertEqual((result.ack["client_id"], result.ack["sequence"]), (None, -1))
        self.assertTrue(self.h.advance(0.06))  # 마지막 유효 command 뒤 0.26초
        for _ in range(15):
            self.h.now += 0.05
            self.h.session.handle_datagram(b"{broken", ADDRESS, self.h.now)
        events = self.h.advance(0.05)
        self.assertTrue(any("lease" in event for event in events), events)

    def test_emergency_stop_latch(self):
        self.h.neutral()
        self.h.command()
        result = self.h.event("emergency_stop")
        self.assertAck(result, True, "ok", "emergency_stop")
        self.assertIs(result.ack["emergency_stop"], True)
        self.assertEqual(result.velocity, STOP)
        self.assertAck(self.h.command(), False, "emergency_stop", "emergency_stop")
        self.assertAck(self.h.neutral(), True, "ok", "emergency_stop")
        self.assertEqual(self.h.advance(0.5), [])  # 비상정지 중에는 이동 watchdog 없음
        cleared = self.h.event("clear_emergency")
        self.assertAck(cleared, True, "ok", "awaiting_neutral")
        self.assertIs(cleared.ack["emergency_stop"], False)
        self.assertAck(self.h.command(), False, "neutral_required")
        self.assertAck(self.h.neutral(), True, "ok", "idle")
        self.assertAck(self.h.command(), True, "ok", "active")

    def test_emergency_latch_survives_owner_change(self):
        self.h.neutral()
        self.h.event("emergency_stop")
        self.h.advance(1.0)  # lease 만료
        self.assertAck(self.h.command(client_id=OTHER_ID), False, "emergency_stop")
        self.assertAck(self.h.event("clear_emergency", client_id=OTHER_ID), True, "ok")
        self.assertAck(self.h.neutral(client_id=OTHER_ID), True, "ok", "idle")

    def test_single_owner_by_uuid_and_address(self):
        self.h.neutral()
        other = self.h.neutral(client_id=OTHER_ID)
        self.assertAck(other, False, "not_owner")
        self.assertIsNone(other.velocity)
        same_uuid_other_port = self.h.neutral(address=(ADDRESS[0], ADDRESS[1] + 1))
        self.assertAck(same_uuid_other_port, False, "not_owner")
        other_host = self.h.neutral(address=OTHER_ADDRESS)
        self.assertAck(other_host, False, "not_owner")

    def test_lease_expiry_releases_owner(self):
        self.h.neutral()
        self.h.command()
        events = self.h.advance(1.0)
        self.assertTrue(any("lease" in event for event in events), events)
        self.assertEqual(self.h.session.snapshot()["owner"], None)
        self.assertAck(self.h.neutral(client_id=OTHER_ID), True, "ok", "idle")
        # 다른 UUID가 인수한 뒤에는 예전 세션이 되돌아올 수 없습니다.
        self.assertAck(self.h.neutral(), False, "not_owner")

    def test_same_uuid_reclaim_needs_higher_sequence(self):
        self.h.neutral(sequence=5)
        self.h.advance(1.0)
        self.assertAck(self.h.neutral(sequence=5), False, "out_of_order")
        self.assertAck(self.h.neutral(sequence=4), False, "out_of_order")
        self.assertIsNone(self.h.session.snapshot()["owner"])
        self.assertAck(self.h.command(sequence=6), False, "neutral_required",
                       "awaiting_neutral")
        self.assertAck(self.h.neutral(sequence=7), True, "ok", "idle")

    def test_old_session_cannot_reclaim_after_replacement(self):
        self.h.neutral(sequence=1)
        self.h.advance(1.0)
        self.h.neutral(client_id=OTHER_ID)
        self.h.event("disconnect", client_id=OTHER_ID)
        delayed = self.h.command(sequence=2)  # 번호는 증가했지만 교체된 세션
        self.assertAck(delayed, False, "not_owner")

    def test_disconnect_stops_releases_and_retires(self):
        self.h.neutral()
        self.h.command()
        result = self.h.event("disconnect")
        self.assertAck(result, True, "ok", "awaiting_neutral")
        self.assertEqual(result.velocity, STOP)
        self.assertIsNone(self.h.session.snapshot()["owner"])
        self.assertAck(self.h.neutral(), False, "not_owner")
        self.assertAck(self.h.neutral(client_id=OTHER_ID), True, "ok", "idle")

    def test_non_owner_cannot_emergency_stop_or_disconnect(self):
        self.h.neutral()
        self.assertAck(self.h.event("emergency_stop", client_id=OTHER_ID), False,
                       "not_owner")
        self.assertAck(self.h.event("disconnect", client_id=OTHER_ID), False, "not_owner")
        self.assertIs(self.h.session.emergency_stop, False)

    def test_fault_rejects_motion_until_restart(self):
        self.h.neutral()
        self.h.session.enter_fault("test")
        self.assertAck(self.h.command(), False, "fault", "fault")
        self.assertAck(self.h.neutral(), True, "ok", "fault")
        self.assertAck(self.h.event("clear_emergency"), True, "ok", "fault")
        self.assertAck(self.h.command(), False, "fault")

    def test_force_stop_requires_new_neutral(self):
        self.h.neutral()
        self.h.command()
        self.h.session.force_stop()
        self.assertEqual(self.h.session.target, STOP)
        self.assertAck(self.h.command(), False, "neutral_required")

    def test_known_gap_delayed_increasing_sequence_is_accepted(self):
        """미합의 사항 기록: 번호만 증가하면 오래 지연된 패킷도 구별하지 못합니다.

        v2 기본 패킷에는 송신 시각이 없으므로 이 서버는 지연 패킷의 신선도를
        판단하지 않습니다. 정책이 정해지면 이 시험을 새 정책으로 바꿉니다.
        """
        self.h.neutral(sequence=1)
        self.h.command(sequence=2, forward=0.1)
        self.h.now += 0.2
        delayed = self.h.command(sequence=3, forward=1.0)
        self.assertAck(delayed, True, "ok", "active")


if __name__ == "__main__":
    unittest.main()
