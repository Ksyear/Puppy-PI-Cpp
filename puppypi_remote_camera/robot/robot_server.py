#!/usr/bin/env python3
"""PuppyPi teleoperation server for ROS1 Noetic.

기본값은 모터 없는 시험 모드입니다. 제조사 puppy_control 패키지가 없어도
시작하며, 변환된 속도를 표준 메시지 시험 토픽에만 발행합니다.

- 시험 모드: geometry_msgs/TwistStamped, SI 단위(m/s, rad/s) -> test_topic
- 실제 모드: puppy_control/Velocity, x·y cm/s, yaw_rate rad/s -> motor_topic
"""

import argparse
import json
import logging
import os
import sys
import threading
import time

import yaml

from remote_control import PROTOCOL_V2, RemoteControlServer


LOGGER = logging.getLogger(__name__)
TEST_VELOCITY_TYPE = "geometry_msgs/TwistStamped"
REAL_VELOCITY_TYPE = "puppy_control/Velocity"
STATE_TYPE = "std_msgs/String"
DEFAULT_STATE_TOPIC = "/puppypi_remote_camera/control_state"
VENDOR_TOPIC_PREFIX = "/puppy_control/"

V2_REAL_MODE_NOTICE = (
    "v2 미합의 정책: 오래 지연됐지만 sequence가 증가한 패킷의 신선도 검증 방식이 "
    "아직 정해지지 않았습니다(README '미합의 정책'). 가감속·최소 출발 속도도 "
    "실측 전입니다. 낮은 속도 상한과 들어 올린 상태에서만 시험하십시오."
)


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, dict):
        raise ValueError("robot_config.yaml 최상위 값은 객체여야 합니다")
    required = {
        "enable_motor_control",
        "motor_topic",
        "test_topic",
        "video_port",
        "control_port",
        "camera",
        "velocity_limits",
    }
    missing = required - set(config)
    if missing:
        raise ValueError("robot_config.yaml 필수 항목 누락: %s" % sorted(missing))
    if type(config["enable_motor_control"]) is not bool:
        raise ValueError("enable_motor_control은 true 또는 false여야 합니다")
    if type(config.get("stand_on_start", True)) is not bool:
        raise ValueError("stand_on_start는 true 또는 false여야 합니다")
    validate_profile(config)
    return config


def validate_profile(config: dict):
    """protocol·영상 경로·토픽 조합을 검사합니다."""
    protocol = config.get("protocol_version", 1)
    if type(protocol) is not int or protocol not in (1, PROTOCOL_V2):
        raise ValueError("protocol_version은 1 또는 2여야 합니다")
    tcp_video = config.get("tcp_video_enabled", True)
    if type(tcp_video) is not bool:
        raise ValueError("tcp_video_enabled는 true 또는 false여야 합니다")
    if protocol == PROTOCOL_V2 and tcp_video:
        raise ValueError(
            "protocol_version 2(Quest)에서는 영상이 UDP 5006(usb_cam + "
            "camera_udp_sender) 경로이므로 tcp_video_enabled는 false여야 합니다. "
            "같은 카메라를 두 프로세스가 동시에 열지 않도록 막는 설정입니다."
        )

    topics = {
        "motor_topic": config["motor_topic"],
        "test_topic": config["test_topic"],
        "state_topic": config.get("state_topic", DEFAULT_STATE_TOPIC),
    }
    for name, value in topics.items():
        if type(value) is not str or not value.startswith("/"):
            raise ValueError("%s는 '/'로 시작하는 절대 토픽 이름이어야 합니다" % name)
    if len(set(topics.values())) != len(topics):
        raise ValueError("motor_topic, test_topic, state_topic은 서로 달라야 합니다")
    for name in ("test_topic", "state_topic"):
        if topics[name].startswith(VENDOR_TOPIC_PREFIX):
            raise ValueError(
                "%s는 제조사 제어 네임스페이스(%s) 밖이어야 합니다"
                % (name, VENDOR_TOPIC_PREFIX)
            )


def confirm_motor_control(
    config: dict,
    command_line_confirmed: bool,
    interactive: bool = True,
):
    if not config["enable_motor_control"]:
        return
    warning = (
        "\n실제 모터 제어가 활성화되어 있습니다.\n"
        "로봇을 바닥에서 들어 올렸거나 넓고 안전한 시험 공간에 두고, "
        "주변 사람과 장애물을 제거했는지 확인해야 합니다."
    )
    if config.get("protocol_version", 1) == PROTOCOL_V2:
        warning += "\n" + V2_REAL_MODE_NOTICE
    print(warning, file=sys.stderr)
    if command_line_confirmed:
        return
    if not interactive or not sys.stdin.isatty():
        raise RuntimeError(
            "비대화형 실행에서는 --confirm-motor-control이 필요합니다 "
            "(roslaunch: confirm_motor_control:=true)"
        )
    answer = input("안전을 확인했으면 정확히 ENABLE MOTORS 를 입력하십시오: ")
    if answer.strip() != "ENABLE MOTORS":
        raise RuntimeError("사용자 안전 확인이 없어 모터 제어를 시작하지 않습니다")


class MotorlessTwistOutput:
    """모터 없는 시험 출력. 제조사 패키지 없이 동작하며 모터 토픽을 쓰지 않습니다."""

    message_type = TEST_VELOCITY_TYPE
    motor_enabled = False

    def __init__(self, config: dict):
        import rospy

        try:
            from geometry_msgs.msg import TwistStamped
        except ImportError as exc:
            raise RuntimeError(
                "geometry_msgs를 가져올 수 없습니다. ros-noetic-ros-base 설치와 "
                "/opt/ros/noetic/setup.bash source를 확인하십시오 (%s)" % exc
            )
        self._rospy = rospy
        self._message_class = TwistStamped
        self.topic = str(config["test_topic"])
        if self.topic == str(config["motor_topic"]):
            raise ValueError("시험 토픽이 실제 모터 토픽과 같습니다")
        self._publisher = rospy.Publisher(self.topic, TwistStamped, queue_size=1)

    def publish(self, x: float, y: float, yaw_rate: float):
        message = self._message_class()
        message.header.stamp = self._rospy.Time.now()
        message.header.frame_id = "base_link"
        # 시험 토픽은 REP-103 SI 단위입니다. 실제 Velocity의 cm/s와 다릅니다.
        message.twist.linear.x = x / 100.0
        message.twist.linear.y = y / 100.0
        message.twist.angular.z = yaw_rate
        self._publisher.publish(message)
        self._rospy.loginfo_throttle(
            0.5,
            "[TEST VELOCITY] x=%+.2fcm/s y=%+.2fcm/s yaw_rate=%+.3frad/s"
            % (x, y, yaw_rate),
        )


class PuppyVelocityOutput:
    """실제 모드 출력. 제조사 puppy_control/Velocity (x·y cm/s, yaw_rate rad/s)."""

    message_type = REAL_VELOCITY_TYPE
    motor_enabled = True

    def __init__(self, config: dict):
        import rospy

        try:
            from puppy_control.msg import Velocity
        except ImportError as exc:
            raise RuntimeError(
                "실제 모터 모드에는 제조사 puppy_control 패키지가 필요합니다. "
                "PUPPYPI_WORKSPACE_SETUP=/실제/catkin_ws/devel/setup.bash 를 "
                "지정하거나 enable_motor_control: false로 시험 모드를 사용하십시오 "
                "(%s)" % exc
            )
        self._message_class = Velocity
        self.topic = str(config["motor_topic"])
        self._publisher = rospy.Publisher(self.topic, Velocity, queue_size=1)

    def publish(self, x: float, y: float, yaw_rate: float):
        self._publisher.publish(self._message_class(x=x, y=y, yaw_rate=yaw_rate))


def build_velocity_output(config: dict):
    if config["enable_motor_control"]:
        return PuppyVelocityOutput(config)
    return MotorlessTwistOutput(config)


class RosStatePublisher:
    """제어기 상태를 JSON 문자열로 latch 발행합니다(상태가 바뀔 때만)."""

    message_type = STATE_TYPE

    def __init__(self, topic: str):
        import rospy
        from std_msgs.msg import String

        self._message_class = String
        self.topic = topic
        self._publisher = rospy.Publisher(topic, String, queue_size=10, latch=True)

    def publish(self, snapshot: dict):
        text = json.dumps(snapshot, sort_keys=True, allow_nan=False)
        self._publisher.publish(self._message_class(data=text))


def foreign_publishers(publishers, topic: str, own_node: str):
    """rosgraph getSystemState()의 publisher 목록에서 자신 외 발행 노드를 찾습니다."""
    for name, nodes in publishers:
        if name == topic:
            return sorted(node for node in nodes if node != own_node)
    return []


class MotorTopicGuard:
    """실제 모드 전용: 모터 토픽에 다른 발행자(예: 5005 vr_udp_teleop)가 생기면 fault."""

    def __init__(self, topic: str, own_node: str, on_conflict, period: float = 1.0):
        self._topic = topic
        self._own_node = own_node
        self._on_conflict = on_conflict
        self._period = period
        self._running = threading.Event()
        self._thread = None

    def check_once(self):
        import rosgraph

        publishers, _, _ = rosgraph.Master(self._own_node).getSystemState()
        return foreign_publishers(publishers, self._topic, self._own_node)

    def start(self):
        self._running.set()
        self._thread = threading.Thread(
            target=self._loop, name="motor-topic-guard", daemon=True
        )
        self._thread.start()

    def shutdown(self):
        self._running.clear()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)

    def _loop(self):
        failures = 0
        while self._running.is_set():
            try:
                others = self.check_once()
                failures = 0
            except Exception as exc:
                failures += 1
                LOGGER.warning("모터 토픽 발행자 조회 실패 (%d회): %s", failures, exc)
                others = []
                if failures >= 3:
                    self._on_conflict("모터 토픽 발행자를 3회 연속 확인하지 못함")
                    return
            if others:
                self._on_conflict(
                    "%s에 다른 발행자가 있습니다: %s" % (self._topic, ", ".join(others))
                )
                return
            time.sleep(self._period)


class RobotServer:
    def __init__(self, config: dict):
        self._config = config
        self._shutdown_lock = threading.Lock()
        self._shut_down = False
        self._output = build_velocity_output(config)
        self._state = RosStatePublisher(
            str(config.get("state_topic", DEFAULT_STATE_TOPIC))
        )
        self._controller = RemoteControlServer(
            config,
            self._output.publish,
            self._state.publish,
        )
        self._guard = None
        self._camera = None
        if config.get("tcp_video_enabled", True):
            # 노트북 v1 프로파일에서만 이 프로세스가 카메라를 직접 엽니다.
            from camera_stream import CameraStreamer

            self._camera = CameraStreamer(
                config,
                self._controller.set_video_client,
                self._controller.clear_video_client,
            )

    @property
    def output_topic(self) -> str:
        return self._output.topic

    @property
    def output_type(self) -> str:
        return self._output.message_type

    @property
    def controller(self) -> RemoteControlServer:
        return self._controller

    def start(self):
        if self._output.motor_enabled:
            import rospy

            self._guard = MotorTopicGuard(
                self._output.topic,
                rospy.get_name(),
                self._controller.enter_fault,
            )
            others = self._guard.check_once()
            if others:
                raise RuntimeError(
                    "%s에 이미 다른 발행자가 있어 시작하지 않습니다: %s "
                    "(5005 vr_udp_teleop 등 다른 조종기를 먼저 종료하십시오)"
                    % (self._output.topic, ", ".join(others))
                )
        # 서기 자세를 먼저 끝낸 뒤 조종 패킷을 받습니다.
        self._stand_up_if_enabled()
        self._controller.start()
        try:
            if self._guard is not None:
                self._guard.start()
            if self._camera is not None:
                self._camera.start()
        except Exception:
            self._controller.shutdown()
            raise

    def _stand_up_if_enabled(self):
        if (
            not self._config["enable_motor_control"]
            or not self._output.motor_enabled
            or not self._config.get("stand_on_start", True)
        ):
            return

        import rospy
        from std_srvs.srv import Empty

        service_name = str(
            self._config.get("stand_service", "/puppy_control/go_home")
        )
        timeout = float(self._config.get("stand_service_timeout_seconds", 15.0))
        if not service_name or timeout <= 0:
            raise ValueError("서기 자세 서비스 설정이 올바르지 않습니다")

        rospy.logwarn("서기 자세 서비스 대기: %s", service_name)
        try:
            rospy.wait_for_service(service_name, timeout=timeout)
            rospy.ServiceProxy(service_name, Empty)()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise RuntimeError(
                "서기 자세 서비스 호출 실패; 조종 서버를 시작하지 않습니다: %s"
                % exc
            )
        rospy.loginfo("서기 자세 완료: %s", service_name)

    def shutdown(self):
        with self._shutdown_lock:
            if self._shut_down:
                return
            self._shut_down = True
        if self._guard is not None:
            self._guard.shutdown()
        self._controller.shutdown()
        if self._camera is not None:
            self._camera.shutdown()


def strip_ros_arguments(argv):
    """roslaunch가 붙이는 __name:=, __log:= 같은 remapping 인자를 제거합니다."""
    return [argument for argument in argv if ":=" not in argument]


def build_argument_parser() -> argparse.ArgumentParser:
    default_config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "robot_config.yaml")
    )
    parser = argparse.ArgumentParser(
        description="PuppyPi 안전 원격 조종 서버 (ROS1 Noetic, UDP 5001)"
    )
    parser.add_argument(
        "--config",
        default=default_config,
        help="robot_config.yaml 경로",
    )
    parser.add_argument(
        "--confirm-motor-control",
        action="store_true",
        help="비대화형 실행에서 실제 모터 시험 안전 확인을 명시",
    )
    parser.add_argument(
        "--non-interactive",
        action="store_true",
        help="안전 확인 프롬프트를 띄우지 않음(roslaunch용)",
    )
    return parser


def main(argv=None) -> int:
    arguments = strip_ros_arguments(sys.argv[1:] if argv is None else argv)
    args = build_argument_parser().parse_args(arguments)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        config = load_config(os.path.abspath(os.path.expanduser(args.config)))
        confirm_motor_control(
            config,
            args.confirm_motor_control,
            interactive=not args.non_interactive,
        )
    except (OSError, ValueError, RuntimeError) as exc:
        print("설정 오류: %s" % exc, file=sys.stderr)
        return 2

    try:
        import rospy
    except ImportError:
        print(
            "rospy를 가져올 수 없습니다. 먼저 source /opt/ros/noetic/setup.bash 를 "
            "실행하십시오.",
            file=sys.stderr,
        )
        return 3

    rospy.init_node("puppypi_remote_camera_server", disable_signals=False)
    try:
        server = RobotServer(config)
        rospy.on_shutdown(server.shutdown)
        server.start()
        rospy.loginfo(
            "조종 서버 시작: UDP %d protocol %d; 속도 출력 %s (%s)",
            server.controller.bound_port,
            server.controller.protocol_version,
            server.output_topic,
            server.output_type,
        )
        if not config["enable_motor_control"]:
            rospy.logwarn(
                "모터 없는 시험 모드: 실제 모터 토픽과 go_home을 사용하지 않고 "
                "%s (m/s, rad/s)에만 발행합니다",
                server.output_topic,
            )
        rospy.spin()
        return 0
    except Exception as exc:
        LOGGER.exception("서버 실행 실패: %s", exc)
        return 1
    finally:
        if "server" in locals():
            server.shutdown()


if __name__ == "__main__":
    sys.exit(main())
