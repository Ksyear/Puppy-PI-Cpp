#!/usr/bin/env python3
"""PuppyPi camera and teleoperation server."""

import argparse
import logging
import os
import sys
import threading

import yaml

from camera_stream import CameraStreamer
from remote_control import RemoteControlServer


LOGGER = logging.getLogger(__name__)


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
    return config


def confirm_motor_control(config: dict, command_line_confirmed: bool):
    if not config["enable_motor_control"]:
        return
    warning = (
        "\n실제 모터 제어가 활성화되어 있습니다.\n"
        "로봇을 바닥에서 들어 올렸거나 넓고 안전한 시험 공간에 두고, "
        "주변 사람과 장애물을 제거했는지 확인해야 합니다."
    )
    print(warning, file=sys.stderr)
    if command_line_confirmed:
        return
    if not sys.stdin.isatty():
        raise RuntimeError(
            "비대화형 실행에서는 --confirm-motor-control이 필요합니다"
        )
    answer = input("안전을 확인했으면 정확히 ENABLE MOTORS 를 입력하십시오: ")
    if answer.strip() != "ENABLE MOTORS":
        raise RuntimeError("사용자 안전 확인이 없어 모터 제어를 시작하지 않습니다")


class RosVelocityOutput:
    def __init__(self, config: dict):
        import rospy
        from puppy_control.msg import Velocity

        self._rospy = rospy
        self._message_type = Velocity
        self._motor_enabled = bool(config["enable_motor_control"])
        self.topic = (
            str(config["motor_topic"])
            if self._motor_enabled
            else str(config["test_topic"])
        )
        self._publisher = rospy.Publisher(self.topic, Velocity, queue_size=1)

    def publish(self, x: float, y: float, yaw_rate: float):
        message = self._message_type(x=x, y=y, yaw_rate=yaw_rate)
        self._publisher.publish(message)
        if not self._motor_enabled:
            self._rospy.loginfo_throttle(
                0.5,
                "[TEST VELOCITY] x=%+.2f y=%+.2f yaw_rate=%+.3f"
                % (x, y, yaw_rate),
            )


class RobotServer:
    def __init__(self, config: dict):
        self._shutdown_lock = threading.Lock()
        self._shut_down = False
        self._output = RosVelocityOutput(config)
        self._controller = RemoteControlServer(config, self._output.publish)
        self._camera = CameraStreamer(
            config,
            self._controller.set_video_client,
            self._controller.clear_video_client,
        )

    @property
    def output_topic(self) -> str:
        return self._output.topic

    def start(self):
        self._controller.start()
        try:
            self._camera.start()
        except Exception:
            self._controller.shutdown()
            raise

    def shutdown(self):
        with self._shutdown_lock:
            if self._shut_down:
                return
            self._shut_down = True
        self._controller.shutdown()
        self._camera.shutdown()


def build_argument_parser() -> argparse.ArgumentParser:
    default_config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "robot_config.yaml")
    )
    parser = argparse.ArgumentParser(
        description="PuppyPi 카메라 스트리밍 및 안전 원격 조종 서버"
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
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        config = load_config(os.path.abspath(os.path.expanduser(args.config)))
        confirm_motor_control(config, args.confirm_motor_control)
    except (OSError, ValueError, RuntimeError) as exc:
        print("설정 오류: %s" % exc, file=sys.stderr)
        return 2

    try:
        import rospy
    except ImportError:
        print(
            "rospy를 가져올 수 없습니다. ROS Noetic과 PuppyPi workspace 환경을 "
            "먼저 source해야 합니다.",
            file=sys.stderr,
        )
        return 3

    rospy.init_node("puppypi_remote_camera_server", disable_signals=False)
    try:
        server = RobotServer(config)
        rospy.on_shutdown(server.shutdown)
        server.start()
        rospy.loginfo(
            "PuppyPi 원격 카메라 서버 시작; 속도 출력 토픽: %s",
            server.output_topic,
        )
        if not config["enable_motor_control"]:
            rospy.logwarn(
                "모터 제어 비활성화: 실제 토픽 대신 %s 에 시험 명령을 발행합니다",
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
