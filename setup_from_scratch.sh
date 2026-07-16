#!/usr/bin/env bash
# Install the public PuppyPi ROS2 stack on a fresh Ubuntu 22.04 system.
# The private Hiwonder gait/IK/servo engine is intentionally not downloaded.
set -euo pipefail

ROLE="${1:-robot}" # robot | pc
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LDLIDAR_DIR="$WS/src/ldlidar_stl_ros2"
LDLIDAR_VERSION="v3.0.3"

if ! grep -q 'VERSION_ID="22.04"' /etc/os-release 2>/dev/null; then
  echo "[오류] Ubuntu 22.04에서만 실행할 수 있습니다."
  echo "       기존 Noetic 저장장치는 보존하고 새 Ubuntu 22.04 USB로 부팅하세요."
  exit 1
fi

if [ "$ROLE" != "robot" ] && [ "$ROLE" != "pc" ]; then
  echo "[오류] role은 robot 또는 pc만 가능합니다: $ROLE"
  exit 1
fi

if [ "$ROLE" = "robot" ] && [ "$(dpkg --print-architecture)" != "arm64" ]; then
  echo "[오류] Raspberry Pi 로봇은 Ubuntu 22.04 64-bit(arm64)여야 합니다."
  exit 1
fi

ROOT_SOURCE="$(findmnt -n -o SOURCE / 2>/dev/null || echo unknown)"
echo "[환경] root filesystem: $ROOT_SOURCE"
if [ "$ROLE" = "robot" ] && [[ "$ROOT_SOURCE" == /dev/mmcblk* ]] && \
   [ "${PUPPYPI_ALLOW_MMC:-0}" != "1" ]; then
  echo "[오류] 현재 root filesystem이 SD/eMMC($ROOT_SOURCE)에 있습니다."
  echo "       Noetic 저장장치를 보호하기 위해 중단합니다. 새 Ubuntu 22.04 USB로 부팅하세요."
  exit 1
fi

echo "[1/5] 기본 도구와 ROS2 APT 저장소"
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release git locales
sudo locale-gen en_US en_US.UTF-8
sudo add-apt-repository -y universe

if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ] || \
   [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi
sudo apt update

echo "[2/5] ROS2 Humble 설치 (role=$ROLE)"
if [ "$ROLE" = "pc" ]; then
  sudo apt install -y ros-humble-desktop
else
  sudo apt install -y ros-humble-ros-base
fi

echo "[3/5] 빌드 도구와 프로젝트 의존 패키지"
sudo apt install -y \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  build-essential \
  cmake \
  libopencv-dev \
  ros-humble-cv-bridge \
  ros-humble-image-transport-plugins \
  ros-humble-laser-filters \
  ros-humble-slam-toolbox \
  ros-humble-usb-cam \
  ros-humble-web-video-server

echo "[4/5] Python 런타임과 LD19 공식 드라이버"
sudo apt install -y \
  python3-matplotlib \
  python3-numpy \
  python3-pygame \
  python3-serial \
  python3-transforms3d \
  python3-yaml

if [ "$ROLE" = "robot" ]; then
  if [ ! -e "$LDLIDAR_DIR" ]; then
    git clone --branch "$LDLIDAR_VERSION" --depth 1 \
      https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git \
      "$LDLIDAR_DIR"
  elif [ ! -f "$LDLIDAR_DIR/package.xml" ]; then
    echo "[오류] $LDLIDAR_DIR 가 존재하지만 유효한 LD19 패키지가 아닙니다."
    exit 1
  else
    echo "  - 기존 LD19 드라이버 사용: $LDLIDAR_DIR"
  fi
fi

echo "[5/5] rosdep 의존성 해결"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths "$WS/src" --ignore-src -r -y

if [ "$ROLE" = "robot" ]; then
  echo "[로봇] udev 규칙과 장치 그룹 설정"
  sudo cp "$WS"/src/peripherals/scripts/*.rules /etc/udev/rules.d/
  sudo usermod -aG dialout,video "$USER"
  sudo udevadm control --reload-rules
  sudo udevadm trigger
fi

echo
echo "[완료] 공개 의존성과 LD19 드라이버 설치가 끝났습니다."
echo "  다음 단계: ./build_all.sh"
echo "  새 그룹 권한 적용: 설치 완료 후 한 번 재부팅"
echo "  주의: Hiwonder 보행 엔진과 ActionGroups는 설치하거나 Git에 넣지 않습니다."
if [ "$ROLE" = "robot" ] && [ ! -d /home/ubuntu/software/puppypi_control ]; then
  echo "  경고: /home/ubuntu/software/puppypi_control 이 없습니다."
  echo "        빌드는 가능하지만 실제 보행 전 기존 Noetic 장치에서 로컬 복원해야 합니다."
fi
