"""robot_server wiring with fake rospy modules (실제 ROS·제조사 패키지 불필요).

가짜 rospy가 만들어진 Publisher와 서비스 호출을 기록하므로, 시험 모드가
모터 토픽에 발행하지 않고 go_home을 호출하지 않는지 확인할 수 있습니다.
"""

import copy
import json
import socket
import sys
import time
import types
import unittest

from _paths import CONFIG_PATH
import robot_server
from v2_test_client import V2Client


class FakePublisher:
    def __init__(self, registry, topic, message_class, queue_size=None, latch=False):
        self.topic = topic
        self.message_class = message_class
        self.latch = latch
        self.messages = []
        registry.append(self)

    def publish(self, message):
        self.messages.append(message)


class _Obj:
    pass


class FakeTwistStamped:
    def __init__(self):
        self.header = _Obj()
        self.header.stamp = None
        self.header.frame_id = ""
        self.twist = _Obj()
        self.twist.linear = _Obj()
        self.twist.angular = _Obj()
        for vector in (self.twist.linear, self.twist.angular):
            vector.x = vector.y = vector.z = 0.0


class FakeString:
    def __init__(self, data=""):
        self.data = data


def install_fake_ros(test):
    publishers = []
    service_calls = []
    fake_rospy = types.ModuleType("rospy")
    fake_rospy.Publisher = lambda topic, cls, **kw: FakePublisher(publishers, topic, cls, **kw)
    fake_rospy.Time = types.SimpleNamespace(now=lambda: time.time())
    fake_rospy.loginfo_throttle = lambda *a, **k: None
    fake_rospy.loginfo = fake_rospy.logwarn = lambda *a, **k: None
    fake_rospy.get_name = lambda: "/puppypi_v2_control"
    fake_rospy.ROSException = type("ROSException", (Exception,), {})
    fake_rospy.ServiceException = type("ServiceException", (Exception,), {})

    def forbidden_service(*args, **kwargs):
        service_calls.append(args)
        raise AssertionError("시험 모드에서 서비스 호출: %r" % (args,))

    fake_rospy.wait_for_service = forbidden_service
    fake_rospy.ServiceProxy = forbidden_service

    geometry = types.ModuleType("geometry_msgs")
    geometry_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msg.TwistStamped = FakeTwistStamped
    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = FakeString

    modules = {
        "rospy": fake_rospy,
        "geometry_msgs": geometry,
        "geometry_msgs.msg": geometry_msg,
        "std_msgs": std,
        "std_msgs.msg": std_msg,
        # 일반 Pi 4에는 제조사 패키지가 없습니다. None이면 import가 실패합니다.
        "puppy_control": None,
        "puppy_control.msg": None,
    }
    saved = {name: sys.modules.get(name, KeyError) for name in modules}
    sys.modules.update(modules)

    def restore():
        for name, module in saved.items():
            if module is KeyError:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    test.addCleanup(restore)
    return publishers, service_calls


def free_udp_port() -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


class ConfigTests(unittest.TestCase):
    def setUp(self):
        self.config = robot_server.load_config(CONFIG_PATH)

    def test_repository_default_is_motorless_quest_v2(self):
        self.assertIs(self.config["enable_motor_control"], False)
        self.assertEqual(self.config["protocol_version"], 2)
        self.assertIs(self.config["tcp_video_enabled"], False)
        self.assertEqual(self.config["control_port"], 5001)
        self.assertEqual(self.config["command_timeout_seconds"], 0.25)
        self.assertEqual(self.config["client_lease_timeout_seconds"], 1.0)
        self.assertEqual(
            self.config["velocity_limits"],
            {"max_forward_cm_s": 5.0, "max_reverse_cm_s": 3.0, "max_yaw_rate_rad_s": 0.2},
        )

    def invalid(self, **changes):
        config = copy.deepcopy(self.config)
        config.update(changes)
        with self.assertRaises(ValueError):
            robot_server.validate_profile(config)

    def test_v2_forbids_tcp_camera(self):
        self.invalid(tcp_video_enabled=True)

    def test_protocol_values(self):
        self.invalid(protocol_version=3)
        self.invalid(protocol_version=True)
        self.invalid(protocol_version="2")

    def test_topics_are_separated(self):
        self.invalid(test_topic=self.config["motor_topic"])
        self.invalid(test_topic="/puppy_control/test")
        self.invalid(state_topic="/puppy_control/state")
        self.invalid(state_topic=self.config["test_topic"])
        self.invalid(test_topic="relative_topic")

    def test_v1_laptop_profile_is_still_valid(self):
        config = copy.deepcopy(self.config)
        config.update(protocol_version=1, tcp_video_enabled=True)
        robot_server.validate_profile(config)

    def test_motor_mode_needs_explicit_confirmation_when_non_interactive(self):
        config = copy.deepcopy(self.config)
        config["enable_motor_control"] = True
        with self.assertRaises(RuntimeError):
            robot_server.confirm_motor_control(config, False, interactive=False)
        robot_server.confirm_motor_control(config, True, interactive=False)

    def test_strip_ros_arguments(self):
        self.assertEqual(
            robot_server.strip_ros_arguments(
                ["--config", "a.yaml", "__name:=x", "__log:=/tmp/y.log", "--non-interactive"]
            ),
            ["--config", "a.yaml", "--non-interactive"],
        )

    def test_foreign_publishers(self):
        publishers = [
            ["/puppy_control/velocity/autogait", ["/vr_udp_teleop", "/puppypi_v2_control"]],
            ["/rosout", ["/puppypi_v2_control"]],
        ]
        topic = "/puppy_control/velocity/autogait"
        self.assertEqual(
            robot_server.foreign_publishers(publishers, topic, "/puppypi_v2_control"),
            ["/vr_udp_teleop"],
        )
        self.assertEqual(
            robot_server.foreign_publishers(publishers[1:], topic, "/puppypi_v2_control"),
            [],
        )


class MotorlessModeTests(unittest.TestCase):
    def setUp(self):
        self.publishers, self.service_calls = install_fake_ros(self)
        self.config = robot_server.load_config(CONFIG_PATH)
        self.config["bind_address"] = "127.0.0.1"
        self.config["control_port"] = free_udp_port()

    def test_starts_without_vendor_package_and_never_touches_motor_topic(self):
        server = robot_server.RobotServer(self.config)
        self.addCleanup(server.shutdown)
        server.start()
        topics = {p.topic: p for p in self.publishers}
        self.assertEqual(
            sorted(topics),
            ["/puppypi_remote_camera/control_state", "/puppypi_remote_camera/test_velocity"],
        )
        self.assertNotIn(self.config["motor_topic"], topics)
        self.assertIs(topics["/puppypi_remote_camera/control_state"].latch, True)
        self.assertEqual(server.output_type, "geometry_msgs/TwistStamped")
        self.assertIsNone(server._camera)  # v2: 이 프로세스는 카메라를 열지 않음

        client = V2Client("127.0.0.1", self.config["control_port"])
        self.addCleanup(client.close)
        self.assertEqual(client.neutral()["state"], "idle")
        self.assertEqual(client.command(True, 0.6, -0.3)["state"], "active")
        velocity = topics["/puppypi_remote_camera/test_velocity"].messages[-1]
        self.assertAlmostEqual(velocity.twist.linear.x, 0.03)  # 3cm/s -> m/s
        self.assertEqual(velocity.twist.linear.y, 0.0)
        self.assertAlmostEqual(velocity.twist.angular.z, -0.06)
        self.assertEqual(velocity.header.frame_id, "base_link")

        states = [json.loads(m.data)["state"]
                  for m in topics["/puppypi_remote_camera/control_state"].messages]
        self.assertEqual(states[-1], "active")
        self.assertIn("idle", states)

        server.shutdown()
        self.assertEqual(velocity.__class__, FakeTwistStamped)
        tail = topics["/puppypi_remote_camera/test_velocity"].messages[-3:]
        self.assertTrue(all(m.twist.linear.x == 0.0 and m.twist.angular.z == 0.0
                            for m in tail))
        self.assertEqual(self.service_calls, [])  # go_home 등 서비스 호출 없음

    def test_real_mode_without_vendor_package_fails_clearly(self):
        self.config["enable_motor_control"] = True
        with self.assertRaises(RuntimeError) as context:
            robot_server.RobotServer(self.config)
        self.assertIn("puppy_control", str(context.exception))
        self.assertEqual(self.publishers, [])
        self.assertEqual(self.service_calls, [])


if __name__ == "__main__":
    unittest.main()
