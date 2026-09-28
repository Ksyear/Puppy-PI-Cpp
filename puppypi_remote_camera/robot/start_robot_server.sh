#!/usr/bin/env bash
# 조종 서버 단독 실행. Quest v2는 start_quest_v2.sh(roslaunch)를 권장합니다.
# 이 스크립트는 roscore가 이미 실행 중이어야 하며, 노트북 v1 프로파일
# (protocol_version: 1, tcp_video_enabled: true)에서도 사용합니다.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "오류: /opt/ros/noetic/setup.bash가 없습니다." >&2
  exit 2
fi
set +u
source /opt/ros/noetic/setup.bash

# 제조사 puppy_control은 enable_motor_control: true인 실제 모드에서만 필요합니다.
# 시험 모드에서는 없어도 됩니다.
if [[ -n "${PUPPYPI_WORKSPACE_SETUP:-}" ]]; then
  if [[ ! -f "${PUPPYPI_WORKSPACE_SETUP}" ]]; then
    echo "오류: PUPPYPI_WORKSPACE_SETUP 파일이 없습니다: ${PUPPYPI_WORKSPACE_SETUP}" >&2
    exit 2
  fi
  # shellcheck source=/dev/null
  source "${PUPPYPI_WORKSPACE_SETUP}"
elif [[ -f /home/pi/PuppyPi/devel/setup.bash ]]; then
  source /home/pi/PuppyPi/devel/setup.bash
elif [[ -f /home/pi/puppy_pi/devel/setup.bash ]]; then
  source /home/pi/puppy_pi/devel/setup.bash
elif [[ -f /home/pi/Puppy-PI-Cpp/devel/setup.bash ]]; then
  source /home/pi/Puppy-PI-Cpp/devel/setup.bash
fi
set -u

if pgrep -f 'vr_udp_teleop' >/dev/null 2>&1; then
  echo "오류: 기존 5005 문자열 조종기(vr_udp_teleop)가 실행 중입니다. 먼저 종료하십시오." >&2
  exit 4
fi
if ! python3 -c 'import yaml, rospy, rosgraph, geometry_msgs.msg, std_msgs.msg' >/dev/null 2>&1; then
  echo "오류: Python 필수 모듈을 가져올 수 없습니다." >&2
  echo "최초 1회: sudo apt install python3-yaml ros-noetic-ros-base" >&2
  exit 3
fi
if ! rostopic list >/dev/null 2>&1; then
  echo "오류: ROS master에 연결할 수 없습니다. 다른 터미널에서 roscore를 실행하거나" >&2
  echo "  Quest v2는 ./start_quest_v2.sh 를 사용하십시오 (roscore 자동 실행)." >&2
  exit 3
fi
# puppy_control·v4l2-ctl·OpenCV는 설정(실제 모드, tcp_video_enabled)에 따라
# robot_server.py가 필요한 경우에만 검사하고 구체적인 설치 방법을 출력합니다.

cd "${SCRIPT_DIR}"
exec python3 robot_server.py "$@"
