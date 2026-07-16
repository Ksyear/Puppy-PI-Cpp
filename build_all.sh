#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# PuppyPi C++/파이썬 신규 패키지 전체를 한 번에 빌드 (로봇 라즈베리파이에서 실행)
#
#   ./build_all.sh          # 전체 빌드 (메시지 패키지 등 의존성은 자동 포함)
#   ./build_all.sh clean    # build/install 지우고 처음부터
#
# 사용법: 이 저장소를 로봇의 워크스페이스로 쓰거나(권장),
#         src/ 내용을 ~/ros2_ws/src/ 로 복사했다면 이 스크립트도 함께 복사.
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[오류] ROS2 Humble 이 없습니다."
  echo "  - Hiwonder 순정 이미지가 아닌 맨 우분투라면 먼저: ./setup_from_scratch.sh robot"
  echo "  - Mac/Windows 에서는 실행 불가 — 로봇(라즈베리파이)에서 실행하세요."
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if [ "${1:-}" = "clean" ]; then
  echo "[정리] build/ install/ log/ 삭제"
  rm -rf build install log
fi

# 빌드 대상: 이 프로젝트에서 만든 패키지 전부.
# --packages-up-to 라서 puppy_control_msgs 등 의존 패키지는 자동으로 먼저 빌드됨.
PKGS=(
  ldlidar_stl_ros2          # LD19 official C++ driver (fetched by robot setup)
  sdk                        # Hiwonder public Python board SDK
  ros_robot_controller       # Hiwonder public Python board node (hybrid runtime)
  puppy_control              # Hiwonder public Python gait wrapper (hybrid runtime)
  peripherals                # 카메라/LiDAR 설정 (자동실행 launch 가 share 를 참조)
  ros_robot_controller_cpp   # 확장보드 시리얼 드라이버 (C++)
  puppy_control_cpp          # 제어 래퍼 (C++, 보행엔진은 스텁 상태)
  peripherals_cpp            # 게임패드/키보드/IMU TF (C++)
  app_cpp                    # LiDAR 앱 (C++)
  lidar_mapping_cpp          # 자체 SLAM 지도작성 (C++)
  obstacle_climb_cpp         # OpenCV 장애물 넘기 (C++)
  puppy_vr_control           # VR 조종/영상/상태 전송 (C++)
  puppy_vr_control_py        # 위와 동일 기능의 파이썬판
)

if [ ! -f "$WS/src/ldlidar_stl_ros2/package.xml" ]; then
  echo "[오류] LD19 드라이버가 없습니다."
  echo "  CMake 전체 설치: cmake --build cmake-build --target robot"
  echo "  의존성만 설치:  cmake --build cmake-build --target robot_deps"
  exit 1
fi

echo "[빌드] ${PKGS[*]}"
# Pi 4 는 RAM 이 작아 병렬 2개로 제한 (스왑 폭주 방지)
colcon build \
  --packages-up-to "${PKGS[@]}" \
  --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "[완료] 이제 셸마다 아래를 실행하세요 (또는 ~/.bashrc 에 추가):"
echo "  source $WS/install/setup.bash"
echo
echo "다음 단계:"
echo "  ./test_all.sh                                  # 오프라인 자동 테스트"
echo "  공부자료/16_빌드_실행_테스트_가이드.md          # 무엇을 실행할지 / 실기기 테스트 순서"
