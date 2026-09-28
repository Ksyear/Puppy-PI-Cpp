#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# Quest v2 스택 실행 (ROS1 Noetic, 일반 Raspberry Pi 4 / Ubuntu Server 20.04)
#
#   ./start_quest_v2.sh                          # 시험 모드 + 영상 UDP 5006
#   ./start_quest_v2.sh use_camera:=false        # 조종 서버만 (카메라 없이)
#   ./start_quest_v2.sh quest_ip:=192.168.0.50   # 영상 목적지 고정 (hello 대신)
#   ./start_quest_v2.sh image_width:=1280 image_height:=720
#
# 나머지 인자는 roslaunch 인자로 그대로 전달한다. roscore 는 roslaunch 가
# 필요하면 자동으로 띄운다. 종료는 이 터미널에서 Ctrl+C.
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REPO_DIR="$(cd -- "${PKG_DIR}/.." && pwd)"
CONFIG="${PKG_DIR}/config/robot_config.yaml"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
    echo "오류: /opt/ros/noetic/setup.bash가 없습니다. README 2절 ROS Noetic 설치를 먼저 하십시오." >&2
    exit 2
  fi
  set +u
  source /opt/ros/noetic/setup.bash
  set -u
fi
if [[ "${ROS_DISTRO}" != "noetic" ]]; then
  echo "오류: ROS_DISTRO=${ROS_DISTRO} 입니다. ROS1 Noetic 환경의 새 터미널에서 실행하십시오." >&2
  exit 2
fi
# 실제 모드에서만 필요한 제조사 workspace (puppy_control) overlay.
if [[ -n "${PUPPYPI_WORKSPACE_SETUP:-}" ]]; then
  if [[ ! -f "${PUPPYPI_WORKSPACE_SETUP}" ]]; then
    echo "오류: PUPPYPI_WORKSPACE_SETUP 파일이 없습니다: ${PUPPYPI_WORKSPACE_SETUP}" >&2
    exit 2
  fi
  set +u
  # shellcheck source=/dev/null
  source "${PUPPYPI_WORKSPACE_SETUP}"
  set -u
fi

# 같은 로봇을 두 조종기가 동시에 제어하지 않도록 기존 5005 경로와 배타 실행한다.
if pgrep -f 'vr_udp_teleop' >/dev/null 2>&1; then
  echo "오류: 기존 5005 문자열 조종기(vr_udp_teleop)가 실행 중입니다." >&2
  echo "  run_vr.sh / vr_control.launch 터미널에서 Ctrl+C로 먼저 종료하십시오." >&2
  exit 4
fi
if pgrep -f 'robot_server.py' >/dev/null 2>&1; then
  echo "오류: robot_server.py가 이미 실행 중입니다 (pgrep -af robot_server.py)." >&2
  exit 4
fi
if pgrep -f 'camera_udp_sender' >/dev/null 2>&1; then
  echo "오류: camera_udp_sender(UDP 5006)가 이미 실행 중입니다 (pgrep -af camera_udp_sender)." >&2
  exit 4
fi

use_camera=true
image_width=640
image_height=480
pixel_format=mjpeg
framerate=30
video_device=""
confirm_motor=false
for argument in "$@"; do
  case "${argument}" in
    use_camera:=false) use_camera=false ;;
    image_width:=*) image_width="${argument#*:=}" ;;
    image_height:=*) image_height="${argument#*:=}" ;;
    pixel_format:=*) pixel_format="${argument#*:=}" ;;
    framerate:=*) framerate="${argument#*:=}" ;;
    video_device:=*) video_device="${argument#*:=}" ;;
    config:=*) CONFIG="${argument#*:=}" ;;
    confirm_motor_control:=true) confirm_motor=true ;;
  esac
done

if ! python3 -c 'import yaml, rospy, rosgraph, geometry_msgs.msg, std_msgs.msg' >/dev/null 2>&1; then
  echo "오류: Python ROS 모듈을 가져올 수 없습니다." >&2
  echo "  sudo apt install python3-yaml ros-noetic-ros-base" >&2
  exit 3
fi

motor_enabled="$(python3 - "${CONFIG}" <<'PY'
import sys, yaml
with open(sys.argv[1], encoding="utf-8") as f:
    config = yaml.safe_load(f)
print("true" if isinstance(config, dict) and config.get("enable_motor_control") is True else "false")
PY
)"
if [[ "${motor_enabled}" == "true" && "${confirm_motor}" != "true" ]]; then
  echo "오류: ${CONFIG} 에서 enable_motor_control: true 입니다." >&2
  echo "  제조사 puppy_control·보행 엔진·하드웨어를 확인한 경우에만" >&2
  echo "  confirm_motor_control:=true 를 명시해 실행하십시오. 시험 모드는 false로 되돌리십시오." >&2
  exit 2
fi

extra_arguments=()
if [[ "${use_camera}" == "true" ]]; then
  if ! command -v v4l2-ctl >/dev/null 2>&1; then
    echo "오류: v4l2-ctl이 없습니다. sudo apt install v4l-utils" >&2
    exit 3
  fi
  if ! rospack find usb_cam >/dev/null 2>&1; then
    echo "오류: usb_cam 패키지가 없습니다. sudo apt install ros-noetic-usb-cam" >&2
    exit 3
  fi
  if ! rospack find compressed_image_transport >/dev/null 2>&1; then
    echo "오류: JPEG 압축 토픽 플러그인이 없습니다. sudo apt install ros-noetic-image-transport-plugins" >&2
    exit 3
  fi
  if [[ -z "${video_device}" ]]; then
    video_device="$(cd "${SCRIPT_DIR}" && python3 find_camera.py \
      --config "${CONFIG}" --width "${image_width}" --height "${image_height}" \
      --pixel-format "${pixel_format}" --fps "${framerate}")" || exit 5
    extra_arguments+=("video_device:=${video_device}")
  fi
  device_real="$(readlink -f "${video_device}")"
  # 한 카메라는 한 프로세스(usb_cam)만 연다. 같은 사용자 프로세스까지 확인된다.
  if command -v fuser >/dev/null 2>&1 && fuser -s "${device_real}" 2>/dev/null; then
    echo "오류: 카메라 ${device_real} 를 이미 다른 프로세스가 사용 중입니다:" >&2
    fuser -v "${device_real}" >&2 || true
    exit 4
  fi
fi

export ROS_PACKAGE_PATH="${PKG_DIR}:${REPO_DIR}/noetic_fallback${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
chmod +x "${SCRIPT_DIR}/robot_server.py" \
  "${REPO_DIR}/noetic_fallback/puppy_vr_control_noetic/scripts/camera_udp_sender.py" \
  "${REPO_DIR}/noetic_fallback/puppy_vr_control_noetic/scripts/robot_status_sender.py"

# 빈 배열도 set -u에서 안전하게 펼칩니다(bash 4.4 미만 호환).
exec roslaunch puppypi_remote_camera quest_v2.launch ${extra_arguments[@]+"${extra_arguments[@]}"} "$@"
