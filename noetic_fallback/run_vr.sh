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

# ROS1 환경 (이미 .bashrc 에서 source 됐어도 무해)
source /opt/ros/noetic/setup.bash 2>/dev/null || true

export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$DIR"
chmod +x "$DIR/puppy_vr_control_noetic/scripts/"*.py

# 이전 인스턴스 정리 (roslaunch 든 직접 실행이든)
pkill -f 'puppy_vr_control_noetic' 2>/dev/null || true
sleep 0.5

exec roslaunch puppy_vr_control_noetic vr_control.launch "$@"
