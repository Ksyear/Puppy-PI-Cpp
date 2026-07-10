#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# 부팅 자동실행 설정 (로봇에서 실행) — 껐다 켜면 VR 조종이 바로 되도록.
#
#   ./install_autostart.sh        # systemd 서비스 등록 + 즉시 시작
#   ./install_autostart.sh off    # 자동실행 해제
#   ./install_autostart.sh status # 상태/로그 확인
#
# 등록되는 것: puppy-vr.service
#   → ros2 launch puppy_vr_control vr_bringup.launch.py
#   → 카메라(usb_cam) + VR 조종(vr_udp_teleop) + 영상(camera_udp_sender)
#     + 상태(robot_status_sender), 전부 죽으면 자동 재시작
#
# 전제: 로봇을 실제로 움직이는 puppy_control / ros_robot_controller 는
#       Hiwonder 순정 이미지의 기본 서비스가 부팅 시 이미 띄운다.
#       (확인: systemctl list-units | grep -iE 'ros|puppy' )
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE=/etc/systemd/system/puppy-vr.service
CMD="${1:-on}"

if [ "$CMD" = "status" ]; then
  systemctl status puppy-vr --no-pager || true
  echo; echo "실시간 로그: journalctl -u puppy-vr -f"
  exit 0
fi

if [ "$CMD" = "off" ]; then
  sudo systemctl disable --now puppy-vr 2>/dev/null || true
  sudo rm -f "$SERVICE"
  sudo systemctl daemon-reload
  echo "[해제] puppy-vr 자동실행 제거 완료"
  exit 0
fi

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "[오류] 빌드가 안 되어 있습니다. 먼저: ./build_all.sh"
  exit 1
fi

echo "[등록] $SERVICE (워크스페이스: $WS, 사용자: $USER)"
sudo tee "$SERVICE" > /dev/null <<EOF
[Unit]
Description=PuppyPi VR bringup (camera + VR teleop + video + status)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER
# Hiwonder launch 들이 참조하는 환경변수 (설치된 share 디렉터리 사용)
Environment=need_compile=True
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && exec ros2 launch puppy_vr_control vr_bringup.launch.py'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now puppy-vr

echo
echo "[완료] 이제 로봇을 껐다 켜면 VR 조종이 자동으로 준비됩니다."
echo "  상태 확인 : ./install_autostart.sh status"
echo "  로그 보기 : journalctl -u puppy-vr -f"
echo "  일시 중지 : sudo systemctl stop puppy-vr   (재부팅하면 다시 켜짐)"
echo "  완전 해제 : ./install_autostart.sh off"
