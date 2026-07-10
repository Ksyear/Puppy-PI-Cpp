#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# 완전 처음(맨 Ubuntu 22.04)에서 필요한 것을 전부 설치하는 스크립트
#
#   ./setup_from_scratch.sh robot   # 로봇(라즈베리파이) — ros-base + 드라이버류
#   ./setup_from_scratch.sh pc      # 모니터링/개발 PC — desktop(rviz2 포함)
#
# 주의: Hiwonder 순정 이미지에는 ROS2 Humble 이 이미 설치되어 있으므로
#       이 스크립트가 필요 없다 (git clone + ./build_all.sh 만 하면 됨).
#       이 스크립트는 맨 우분투에 다시 구축할 때 사용.
#       (보행 엔진 /home/ubuntu/software/puppypi_control 과 서보 캘리브레이션은
#        Hiwonder 이미지에만 있으므로, 맨 우분투만으로는 로봇 구동이 안 된다 —
#        공부자료/16 의 0절 참고)
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

ROLE="${1:-robot}"   # robot | pc
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! grep -q 'VERSION_ID="22.04"' /etc/os-release 2>/dev/null; then
  echo "[경고] Ubuntu 22.04 가 아닙니다. ROS2 Humble 은 22.04 전용입니다. 계속하려면 Enter, 중단은 Ctrl+C"
  read -r
fi

echo "[1/5] 기본 도구 + APT 저장소"
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release git locales
sudo locale-gen en_US en_US.UTF-8
sudo add-apt-repository -y universe

# ROS2 Humble APT 저장소 등록 (이미 있으면 건너뜀)
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update
fi

echo "[2/5] ROS2 Humble 설치 (role=$ROLE)"
if [ "$ROLE" = "pc" ]; then
  sudo apt install -y ros-humble-desktop          # rviz2, rqt 포함
else
  sudo apt install -y ros-humble-ros-base         # GUI 없는 로봇용
fi

echo "[3/5] 빌드 도구 + 이 프로젝트가 쓰는 ROS 패키지"
sudo apt install -y \
  ros-dev-tools python3-colcon-common-extensions python3-rosdep python3-pip \
  build-essential cmake \
  ros-humble-cv-bridge libopencv-dev \
  ros-humble-usb-cam \
  ros-humble-image-transport-plugins \
  ros-humble-slam-toolbox
# web_video_server: Humble 은 apt 로 제공. 만약 없다는 오류가 나면 소스 빌드:
#   git clone -b ros2 https://github.com/RobotWebTools/web_video_server "$WS/src/web_video_server"
sudo apt install -y ros-humble-web-video-server || \
  echo "[안내] web_video_server apt 미제공 환경 — 위 주석의 소스 빌드 방법 사용"

echo "[4/5] 파이썬 원본 노드용 라이브러리 (드라이버/조이스틱)"
pip3 install --user pyserial pygame

echo "[5/5] rosdep 으로 남은 의존성 자동 해결"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update || true
if [ -d "$WS/src" ]; then
  rosdep install --from-paths "$WS/src" --ignore-src -r -y || true
fi

if [ "$ROLE" = "robot" ]; then
  echo
  echo "[로봇 전용 추가 설정]"
  # 장치 이름 고정 (확장보드 /dev/rrc 등)
  if [ -d "$WS/src/peripherals/scripts" ]; then
    sudo cp "$WS"/src/peripherals/scripts/*.rules /etc/udev/rules.d/ 2>/dev/null || true
    sudo udevadm control --reload && sudo udevadm trigger
    echo "  - udev 규칙 설치 완료 (/dev/rrc 등)"
  fi
  # LD19 LiDAR 드라이버 (Hiwonder 이미지 외에는 없음) — 필요 시:
  #   git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2 "$WS/src/ldlidar_stl_ros2"
fi

echo
echo "[완료] 다음 순서:"
echo "  1) 셸 재시작 또는: source /opt/ros/humble/setup.bash"
echo "  2) ./build_all.sh"
echo "  3) ./test_all.sh"
echo "  자세한 경로별 안내: 공부자료/16_빌드_실행_테스트_가이드.md 0절"
