#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# 오프라인 자동 테스트 실행 (하드웨어 불필요, 빌드 후 아무 때나)
#
#   ./test_all.sh
#
# 무엇을 검증하나:
#   - test_board_protocol : 시리얼 프레임 21종이 파이썬 SDK 기준 바이트와 일치,
#                           CRC8, 수신 파서의 CRC 오염 프레임 거부
#   - test_mapping_core   : 가상 8x8m 방 시뮬레이션 — 스캔매칭 위치오차/지도 정확도
#   - test_vr_logic       : VR 패킷 파싱/데드존/정규화/양자화/영상 청크 산술
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

bash "$WS/tools/check_public_tree.sh"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

colcon test \
  --packages-select ros_robot_controller_cpp lidar_mapping_cpp puppy_vr_control \
  --event-handlers console_cohesion+

echo
colcon test-result --verbose
echo
echo "[안내] 실기기(하드웨어) 테스트 순서는 공부자료/16_빌드_실행_테스트_가이드.md 참고"
