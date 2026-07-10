#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# [Noetic 로봇 전용] VR 조종 한 번에 실행: export + chmod + roslaunch
#
#   ./run_vr.sh                              # 기본 실행 (서기 + 조종 + 영상 + 상태)
#   ./run_vr.sh max_speed_x:=8               # 더 천천히
#   ./run_vr.sh debug:=true use_camera:=false
#
# 이전에 떠 있던 같은 노드는 자동으로 정리하고 시작한다 (중복 실행 방지).
# ─────────────────────────────────────────────────────────────────────────────
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS 환경: 이미 잡혀 있으면 절대 다시 source 하지 않는다!
# (기본 setup.bash 를 다시 source 하면 로봇 워크스페이스 오버레이가 지워져
#  puppy_control 메시지 import 가 실패한다 — 실제로 겪은 버그)
if [ -z "${ROS_DISTRO:-}" ]; then
  source /opt/ros/noetic/setup.bash
  # 로봇 워크스페이스(devel/setup.bash)가 있으면 오버레이
  for ws in "$HOME"/*/devel/setup.bash; do
    [ -f "$ws" ] && source "$ws" && break
  done
fi

export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$DIR"
chmod +x "$DIR/puppy_vr_control_noetic/scripts/"*.py

# 이전 인스턴스 정리 (roslaunch 든 직접 실행이든)
pkill -f 'puppy_vr_control_noetic' 2>/dev/null || true
sleep 0.5

exec roslaunch puppy_vr_control_noetic vr_control.launch "$@"
