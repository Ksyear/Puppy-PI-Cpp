"""RemoteControlServer v2 over real UDP sockets on 127.0.0.1 (ROS·모터 불필요)."""

import json
import socket
import threading
import time
import unittest

from _paths import CLIENT_ID
from remote_control import RemoteControlServer
import v2_test_client
from v2_test_client import V2Client


def free_udp_port() -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def v2_config(port: int) -> dict:
    return {
        "bind_address": "127.0.0.1",
        "control_port": port,
        "protocol_version": 2,
        "command_timeout_seconds": 0.25,
        "client_lease_timeout_seconds": 1.0,
        "stop_repetitions": 3,
        "velocity_limits": {
            "max_forward_cm_s": 5.0,
            "max_reverse_cm_s": 3.0,
            "max_yaw_rate_rad_s": 0.20,
        },
    }


class Recorder:
    def __init__(self):
        self.lock = threading.Lock()
        self.velocities = []  # (monotonic, x, y, yaw)
        self.states = []

    def publish(self, x, y, yaw):
        with self.lock:
            self.velocities.append((time.monotonic(), x, y, yaw))

    def on_state(self, snapshot):
        with self.lock:
            self.states.append(snapshot)

    def snapshot(self):
        with self.lock:
            return list(self.velocities)


class LoopbackTests(unittest.TestCase):
    def setUp(self):
        self.port = free_udp_port()
        self.recorder = Recorder()
        self.server = RemoteControlServer(
            v2_config(self.port), self.recorder.publish, self.recorder.on_state
        )
        self.server.start()
        self.clients = []

    def tearDown(self):
        for client in self.clients:
            client.close()
        self.server.shutdown()

    def client(self) -> V2Client:
        client = V2Client("127.0.0.1", self.port)
        self.clients.append(client)
        return client

    def raw_ack(self, data: bytes) -> dict:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.5)
        try:
            sock.sendto(data, ("127.0.0.1", self.port))
            return json.loads(sock.recvfrom(4096)[0].decode("utf-8"))
        finally:
            sock.close()

    def stream(self, client, seconds, active, forward, turn):
        acks = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            acks.append(client.command(active, forward, turn))
            time.sleep(0.05)
        return acks

    def test_startup_publishes_repeated_stop(self):
        zeros = [v for v in self.recorder.snapshot() if v[1:] == (0.0, 0.0, 0.0)]
        self.assertGreaterEqual(len(zeros), 3)
        self.assertEqual(self.recorder.states[0]["state"], "awaiting_neutral")

    def test_ack_format_and_conversion_over_udp(self):
        client = self.client()
        ack = client.neutral()
        self.assertEqual(
            list(ack),
            ["protocol", "type", "client_id", "sequence", "accepted", "reason",
             "emergency_stop", "state"],
        )
        self.assertEqual((ack["client_id"], ack["sequence"]), (client.client_id, 0))
        ack = client.command(True, 0.6, -0.3)
        self.assertEqual((ack["accepted"], ack["state"]), (True, "active"))
        _, x, y, yaw = self.recorder.snapshot()[-1]
        self.assertAlmostEqual(x, 3.0)
        self.assertEqual(y, 0.0)
        self.assertAlmostEqual(yaw, -0.06)

    def test_watchdog_stops_within_300ms_of_last_command(self):
        client = self.client()
        client.neutral()
        self.stream(client, 0.3, True, 1.0, 0.0)
        time.sleep(0.6)
        history = self.recorder.snapshot()
        last_active = max(i for i, v in enumerate(history) if v[1] != 0.0)
        stops = [v for v in history[last_active + 1:] if v[1:] == (0.0, 0.0, 0.0)]
        self.assertTrue(stops, "watchdog stop이 발행되지 않음")
        delay = stops[0][0] - history[last_active][0]
        self.assertGreaterEqual(delay, 0.24)
        self.assertLess(delay, 0.30, "정지 지연 %.3fs" % delay)
        ack = client.command(True, 1.0, 0.0)
        self.assertEqual((ack["reason"], ack["state"]), ("neutral_required", "timeout"))

    def test_invalid_flood_does_not_refresh_watchdog(self):
        client = self.client()
        client.neutral()
        client.command(True, 1.0, 0.0)
        started = time.monotonic()
        flooder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            while time.monotonic() - started < 0.5:
                flooder.sendto(b"{not json", ("127.0.0.1", self.port))
                client.sock.sendto(b'{"protocol":2}', ("127.0.0.1", self.port))
                time.sleep(0.01)
        finally:
            flooder.close()
        self.assertEqual(self.server.state_snapshot()["state"], "timeout")
        self.assertEqual(self.recorder.snapshot()[-1][1:], (0.0, 0.0, 0.0))

    def test_malicious_packets_do_not_kill_receive_thread(self):
        client = self.client()
        client.neutral()
        client.command(True, 1.0, 0.0)
        huge = json.dumps({
            "protocol": 2, "type": "command", "client_id": CLIENT_ID, "sequence": 1,
            "control_active": True, "forward": 0.5, "turn": 0.0,
        }).replace("0.5", "1" + "0" * 400).encode()
        for data in (b"[" * 1500, huge, b"\xff" * 10):
            ack = self.raw_ack(data)
            self.assertEqual((ack["accepted"], ack["reason"]), (False, "invalid_packet"))
        time.sleep(0.4)  # 수신 스레드가 살아 있어야 watchdog이 정지시킵니다.
        self.assertEqual(self.server.state_snapshot()["state"], "timeout")
        self.assertEqual(client.neutral()["state"], "idle")

    def test_v1_and_legacy_5005_formats_are_rejected(self):
        v1 = json.dumps({
            "protocol": 1, "type": "command", "client_id": CLIENT_ID, "sequence": 3,
            "timestamp": time.time(), "x": 5.0, "yaw_rate": 0.0,
        }).encode()
        ack = self.raw_ack(v1)
        self.assertEqual((ack["protocol"], ack["reason"]), (2, "unsupported_protocol"))
        ack = self.raw_ack(b"X:0.0,Z:30.0")
        self.assertEqual((ack["client_id"], ack["sequence"]), (None, -1))
        self.assertEqual(ack["reason"], "invalid_packet")

    def test_second_server_on_same_port_fails_to_bind(self):
        duplicate = RemoteControlServer(v2_config(self.port), lambda *a: None)
        with self.assertRaises(OSError):
            duplicate.start()

    def test_shutdown_publishes_stop(self):
        client = self.client()
        client.neutral()
        client.command(True, 1.0, 0.0)
        self.server.shutdown()
        tail = self.recorder.snapshot()[-3:]
        self.assertTrue(all(v[1:] == (0.0, 0.0, 0.0) for v in tail))

    def test_state_callback_reports_transitions(self):
        client = self.client()
        client.command(True, 1.0, 0.0)
        client.neutral()
        client.command(True, 1.0, 0.0)
        client.send("emergency_stop")
        states = [s["state"] for s in self.recorder.states]
        for expected in ("awaiting_neutral", "idle", "active", "emergency_stop"):
            self.assertIn(expected, states)

    def test_output_failure_latches_fault(self):
        calls = {"n": 0}

        def failing(x, y, yaw):
            calls["n"] += 1
            if x != 0.0:
                raise RuntimeError("publisher broken")

        self.server.shutdown()
        self.server = RemoteControlServer(v2_config(self.port), failing)
        self.server.start()
        client = self.client()
        client.neutral()
        ack = client.command(True, 1.0, 0.0)
        self.assertEqual((ack["accepted"], ack["reason"], ack["state"]),
                         (False, "fault", "fault"))
        self.assertEqual(client.command(True, 1.0, 0.0)["reason"], "fault")

    def test_all_client_scenarios_pass(self):
        failures = v2_test_client.run("127.0.0.1", self.port, "all", 0.6, -0.3, 0.5)
        self.assertEqual(failures, 0)


if __name__ == "__main__":
    unittest.main()
