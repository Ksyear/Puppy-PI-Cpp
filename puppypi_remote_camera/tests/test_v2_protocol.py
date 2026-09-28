"""v2 strict JSON parser and forward/turn -> velocity conversion (ROS 불필요)."""

import json
import unittest

from _paths import CLIENT_ID
from remote_control import (
    MAX_PACKET_BYTES,
    V2_MAX_SEQUENCE,
    V2PacketError,
    normalized_to_velocity,
    parse_v2_packet,
)

LIMITS = (5.0, 3.0, 0.20)


def command(**overrides) -> bytes:
    payload = {
        "protocol": 2,
        "type": "command",
        "client_id": CLIENT_ID,
        "sequence": 123,
        "control_active": True,
        "forward": 0.6,
        "turn": -0.3,
    }
    payload.update(overrides)
    return json.dumps(payload).encode("utf-8")


def event(action: str, **overrides) -> bytes:
    payload = {"protocol": 2, "type": action, "client_id": CLIENT_ID, "sequence": 125}
    payload.update(overrides)
    return json.dumps(payload).encode("utf-8")


class ParseV2Tests(unittest.TestCase):
    def assertRejected(self, data: bytes, reason: str = "invalid_packet"):
        with self.assertRaises(V2PacketError) as context:
            parse_v2_packet(data)
        self.assertEqual(context.exception.reason, reason, str(context.exception))
        return context.exception

    def test_spec_command_example(self):
        packet = parse_v2_packet(command())
        self.assertEqual(packet.action, "command")
        self.assertEqual(packet.client_id, CLIENT_ID)
        self.assertEqual(packet.sequence, 123)
        self.assertIs(packet.control_active, True)
        self.assertEqual((packet.forward, packet.turn), (0.6, -0.3))
        self.assertFalse(packet.is_neutral)

    def test_spec_neutral_example(self):
        packet = parse_v2_packet(
            command(sequence=124, control_active=False, forward=0.0, turn=0.0)
        )
        self.assertTrue(packet.is_neutral)

    def test_events_have_only_common_fields(self):
        for action in ("emergency_stop", "clear_emergency", "disconnect"):
            with self.subTest(action=action):
                self.assertEqual(parse_v2_packet(event(action)).action, action)
                self.assertRejected(event(action, control_active=False))

    def test_every_command_field_is_required(self):
        base = json.loads(command().decode())
        for field in base:
            with self.subTest(field=field):
                payload = dict(base)
                del payload[field]
                self.assertRejected(json.dumps(payload).encode())

    def test_unknown_fields_are_rejected(self):
        self.assertRejected(command(timestamp=1.0))  # v1 필드
        self.assertRejected(command(grip="ball"))  # 그립 종류는 전송하지 않음

    def test_duplicate_keys_are_rejected(self):
        text = command().decode()[:-1] + ',"forward":0.1}'
        self.assertRejected(text.encode())

    def test_numbers_must_be_json_numbers(self):
        for bad in ("0.5", True, False, None, [0.5], {"v": 0.5}):
            with self.subTest(value=bad):
                self.assertRejected(command(forward=bad))
                self.assertRejected(command(turn=bad))

    def test_control_active_must_be_boolean(self):
        for bad in (1, 0, "true", None):
            with self.subTest(value=bad):
                self.assertRejected(command(control_active=bad))

    def test_nan_and_infinity_are_rejected(self):
        for literal in ("NaN", "Infinity", "-Infinity"):
            with self.subTest(literal=literal):
                text = command().decode().replace("0.6", literal)
                self.assertRejected(text.encode())

    def test_overflowing_numbers_are_rejected_without_crash(self):
        self.assertRejected(command().decode().replace("0.6", "1e400").encode())
        self.assertRejected(command().decode().replace("0.6", "1" + "0" * 400).encode())

    def test_axis_range(self):
        for good in (-1, 1, -1.0, 1.0, 0, -0.0, 0.999):
            with self.subTest(value=good):
                self.assertEqual(parse_v2_packet(command(forward=good)).forward, float(good))
        for bad in (1.0001, -1.0001, 2, -2):
            with self.subTest(value=bad):
                self.assertRejected(command(forward=bad))
                self.assertRejected(command(turn=bad))

    def test_protocol_version(self):
        self.assertRejected(command(protocol=1), "unsupported_protocol")
        self.assertRejected(command(protocol=3), "unsupported_protocol")
        for bad in ("2", 2.0, True, None):
            with self.subTest(value=bad):
                self.assertRejected(command(protocol=bad), "invalid_packet")

    def test_v1_packet_is_not_mixed_into_v2(self):
        v1 = {
            "protocol": 1,
            "type": "command",
            "client_id": CLIENT_ID,
            "sequence": 7,
            "timestamp": 1.0,
            "x": 5.0,
            "yaw_rate": 0.0,
        }
        error = self.assertRejected(json.dumps(v1).encode(), "unsupported_protocol")
        self.assertEqual((error.client_id, error.sequence), (CLIENT_ID, 7))

    def test_uuid_format(self):
        upper = CLIENT_ID.upper()
        packet = parse_v2_packet(command(client_id=upper))
        self.assertEqual(packet.client_id, upper)  # ACK에는 받은 문자열 그대로
        self.assertEqual(packet.session_id, CLIENT_ID)
        for bad in (
            "{%s}" % CLIENT_ID,
            "urn:uuid:" + CLIENT_ID,
            CLIENT_ID.replace("-", ""),
            "00000000-0000-0000-0000-000000000000",
            "d8b27f1-009a8-4c21-b649-1e7058d46b72",
            "not-a-uuid",
            123,
            None,
        ):
            with self.subTest(value=bad):
                error = self.assertRejected(command(client_id=bad))
                self.assertIsNone(error.client_id)

    def test_sequence_range(self):
        self.assertEqual(parse_v2_packet(command(sequence=0)).sequence, 0)
        self.assertEqual(
            parse_v2_packet(command(sequence=V2_MAX_SEQUENCE)).sequence,
            V2_MAX_SEQUENCE,
        )
        for bad in (V2_MAX_SEQUENCE + 1, -1, 1.0, True, "1", None):
            with self.subTest(value=bad):
                error = self.assertRejected(command(sequence=bad))
                self.assertEqual(error.sequence, -1)

    def test_rejection_echoes_identifiers_when_known(self):
        error = self.assertRejected(command(forward=5))
        self.assertEqual((error.client_id, error.sequence), (CLIENT_ID, 123))

    def test_non_object_and_malformed_datagrams(self):
        for data in (
            b"",
            b"[]",
            b'"command"',
            b"1",
            b"null",
            b"{",
            b"X:0.0,Z:30.0",  # 기존 5005 문자열 형식
            b"\xff\xfe",
            b"[" * 1500,  # 깊은 중첩: RecursionError가 새지 않아야 함
            b" " * (MAX_PACKET_BYTES + 1),
        ):
            with self.subTest(data=data[:20]):
                error = self.assertRejected(data)
                self.assertEqual((error.client_id, error.sequence), (None, -1))

    def test_type_must_be_known_string(self):
        self.assertRejected(command(type=["command"]))
        self.assertRejected(command(type="move"))


class ConversionTests(unittest.TestCase):
    def convert(self, forward, turn):
        return normalized_to_velocity(forward, turn, *LIMITS)

    def test_spec_examples(self):
        x, y, yaw = self.convert(0.6, -0.3)
        self.assertAlmostEqual(x, 3.0)
        self.assertEqual(y, 0.0)
        self.assertAlmostEqual(yaw, -0.06)
        self.assertAlmostEqual(self.convert(-0.5, 0.0)[0], -1.5)

    def test_initial_limits(self):
        self.assertEqual(self.convert(1.0, 1.0), (5.0, 0.0, 0.2))
        self.assertEqual(self.convert(-1.0, -1.0), (-3.0, 0.0, -0.2))
        self.assertEqual(self.convert(0.0, 0.0), (0.0, 0.0, 0.0))

    def test_turn_sign_kept_while_reversing(self):
        self.assertGreater(self.convert(-0.5, 0.5)[2], 0.0)

    def test_final_clamp_still_applies(self):
        self.assertEqual(self.convert(2.0, -3.0), (5.0, 0.0, -0.2))


if __name__ == "__main__":
    unittest.main()
