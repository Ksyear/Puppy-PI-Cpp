#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "오류: /opt/ros/noetic/setup.bash가 없습니다." >&2
  exit 2
fi
source /opt/ros/noetic/setup.bash

if [[ -n "${PUPPYPI_WORKSPACE_SETUP:-}" ]]; then
  if [[ ! -f "${PUPPYPI_WORKSPACE_SETUP}" ]]; then
    echo "오류: PUPPYPI_WORKSPACE_SETUP 파일이 없습니다: ${PUPPYPI_WORKSPACE_SETUP}" >&2
    exit 2
  fi
  source "${PUPPYPI_WORKSPACE_SETUP}"
elif [[ -f /home/pi/PuppyPi/devel/setup.bash ]]; then
  source /home/pi/PuppyPi/devel/setup.bash
elif [[ -f /home/pi/puppy_pi/devel/setup.bash ]]; then
  source /home/pi/puppy_pi/devel/setup.bash
elif [[ -f /home/pi/Puppy-PI-Cpp/devel/setup.bash ]]; then
  source /home/pi/Puppy-PI-Cpp/devel/setup.bash
fi

if ! command -v v4l2-ctl >/dev/null 2>&1; then
  echo "오류: v4l2-ctl이 없습니다. 최초 1회: sudo apt install v4l-utils" >&2
  exit 3
fi
if ! rospack find puppy_control >/dev/null 2>&1; then
  echo "오류: puppy_control ROS 패키지를 찾을 수 없습니다." >&2
  echo "PUPPYPI_WORKSPACE_SETUP=/실제/catkin_ws/devel/setup.bash 를 지정하십시오." >&2
  exit 3
fi
if ! python3 -c 'import cv2, yaml, rospy; from puppy_control.msg import Velocity' >/dev/null 2>&1; then
  echo "오류: Python 필수 모듈을 가져올 수 없습니다." >&2
  echo "최초 1회: sudo apt install python3-opencv python3-yaml" >&2
  exit 3
fi

cd "${SCRIPT_DIR}"
exec python3 robot_server.py "$@"
