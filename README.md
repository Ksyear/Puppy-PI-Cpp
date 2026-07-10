# PuppyPi VR 텔레오퍼레이션 프로젝트

Hiwonder **PuppyPi**(4족 보행 로봇, Raspberry Pi 4, ROS2 Humble)를
**Meta Quest(VR)로 조종**하고, 카메라 영상(FPV)·배터리/연결 상태를 헤드셋으로
받아보는 다학제간 팀 프로젝트입니다.

> 이 저장소는 [Hiwonder/PuppyPi](https://github.com/Hiwonder/PuppyPi) (`ros2` 브랜치)를
> 기반으로 한 **교육 목적의 확장 포크**입니다. 원본 로봇 소프트웨어의 저작권은
> Hiwonder에 있으며, 공식 문서는 [docs.hiwonder.com](https://docs.hiwonder.com/projects/PuppyPi/en/latest/) 을 참고하세요.
> VR 컨트롤러(Unity) 프로토콜은 [inrjin/PuppyPi_MR_Controller](https://github.com/inrjin/PuppyPi_MR_Controller) 를 따릅니다.

```
┌─ Meta Quest (Unity/OpenXR) ──────┐        ┌─ PuppyPi (Raspberry Pi 4, ROS2 Humble) ────────────┐
│ 가상 조이스틱  "X:12.3,Z:-5.0" ──┼─:5005─▶│ vr_udp_teleop → /puppy_control/velocity/autogait    │
│ 긴급정지  "ESTOP"/"RESUME" ──────┼─:5005─▶│   (1초 무신호 자동정지·속도 클램프)   → 보행 엔진   │
│ FPV 화면   ◀── JPEG 청크 ────────┼─:5006──┤ camera_udp_sender ◀── /image_raw ◀── usb_cam        │
│ 배터리/연결 HUD ◀── 상태 문자열 ─┼─:5007──┤ robot_status_sender ◀── battery/RSSI                │
└──────────────────────────────────┘        └─────────────────────────────────────────────────────┘
```

## 이 포크에서 추가된 것

| 패키지 | 내용 |
|---|---|
| `src/puppy_vr_control` (+`_py`) | VR UDP 조종·영상·상태 전송 노드 (C++/파이썬 동일 기능 쌍) |
| `src/lidar_mapping_cpp` | 자체 C++ SLAM (스캔매칭+점유격자, 오도메트리 불필요, 시뮬레이션 검증) |
| `src/obstacle_climb_cpp` | OpenCV 장애물 인식 → 정면 정렬 → 다리 동작으로 넘어가기 |
| `src/driver/ros_robot_controller_cpp` | 확장보드 시리얼 드라이버의 C++ 이식 (프로토콜 바이트 검증 완료) |
| `src/driver/puppy_control_cpp` | 제어 노드 C++ 이식 (ROS 래퍼 완료, 보행엔진은 로봇 내장 코드 필요) |
| `src/peripherals_cpp`, `src/app_cpp` | 조이스틱/키보드/IMU TF/LiDAR 앱의 C++ 이식 |
| `공부자료/` | 한국어 학습 문서 16편 (아키텍처·프로토콜·Safety·실험 기록 등) |
| `*.sh` | 원클릭 설치/빌드/테스트/부팅자동실행 스크립트 |
| `noetic_fallback/` | **비상용**: 로봇이 ROS1(Noetic) 이미지일 때 쓰는 rospy 버전 VR 노드 (UDP 프로토콜 동일) |
| `tools/vr_test_dashboard.py` | **VR 없이 테스트하는 PC용 대시보드** (조종+영상+배터리+속도, Quest와 동일 프로토콜) |

## 빠른 시작 (로봇에서)

> **ROS 버전 주의**: 이 저장소는 **ROS2 Humble 전용**입니다. 로봇 SD카드가
> ROS1(Noetic) 이미지라면 Hiwonder의 ROS2(Humble) 이미지로 다시 구우세요
> (둘 다 공식 제공 — support@hiwonder.com 에 주문번호로 요청). 확인: `printenv ROS_DISTRO`

```bash
git clone <이 저장소 주소> puppypi_ws && cd puppypi_ws
./setup_from_scratch.sh robot   # 맨 Ubuntu 22.04일 때만 (Hiwonder 순정 이미지는 생략)
./build_all.sh && source install/setup.bash
./test_all.sh                   # 하드웨어 없이 자동 테스트 3종
./install_autostart.sh          # 부팅 자동실행 등록 → 이후 전원만 켜면 VR 조종 준비 완료
```

Unity/Quest 쪽 준비와 상세 절차: [공부자료/16_빌드_실행_테스트_가이드.md](./공부자료/16_빌드_실행_테스트_가이드.md)

## VR 없이 한 창으로 테스트 — 대시보드 (권장)

Quest 없이도 **조종·영상·배터리·속도·연결상태를 한 창에서** 확인할 수 있습니다.
Quest 앱과 완전히 동일한 UDP 프로토콜로 동작하므로, 이 창에서 되면 VR에서도 됩니다.

```bash
# 노트북(Mac/Windows/Ubuntu)에서 — ROS 불필요, pygame 만 있으면 됨
pip3 install pygame
python3 tools/vr_test_dashboard.py --robot <로봇IP>
```

| 화면 요소 | 내용 |
|---|---|
| 영상 (좌측) | 로봇 카메라 FPV + fps 표시 |
| 가상 조이스틱 | 마우스 드래그 또는 W/A/S/D — 놓으면 자동 중립(정지) |
| 상태 패널 | 배터리 전압(7.0V 미만 경고색), Wi-Fi RSSI, 로봇 연결 상태(3초 무수신=LOST) |
| 속도 미리보기 | 전송 각도(X/Z)와 로봇이 실행할 예상 명령(cm/s, rad/s) |
| SPACE / R | 긴급정지(ESTOP) / 해제(RESUME) |
| P | 송신 일시정지 — **통신 끊김 시뮬레이션** (로봇이 1초 내 자동 정지해야 정상) |

로봇 쪽 사전 조건: ROS2 는 `ros2 launch puppy_vr_control vr_control.launch.py`,
ROS1(Noetic)은 `roslaunch puppy_vr_control_noetic vr_control.launch` (+카메라 노드).
내부 로직 검증: `python3 tools/vr_test_dashboard.py --selftest`

## ros2 topic 으로 직접 테스트하기

VR 없이 터미널에서 로봇을 시험하는 명령 모음입니다.
(ROS2 이므로 ROS1 의 `rostopic` 대신 `ros2 topic` 을 씁니다.
⚠️ 처음에는 **로봇을 들어올려 다리가 땅에 닿지 않게** 한 뒤 시험하세요.)

```bash
# 전진 (x: cm/s 단위, 허용 범위 |x|<=35 — 초과하면 조용히 무시됨)
ros2 topic pub -1 /puppy_control/velocity/autogait puppy_control_msgs/msg/Velocity \
  "{x: 10.0, y: 0.0, yaw_rate: 0.0}"

# 후진
ros2 topic pub -1 /puppy_control/velocity/autogait puppy_control_msgs/msg/Velocity \
  "{x: -10.0, y: 0.0, yaw_rate: 0.0}"

# 제자리 좌회전 (yaw_rate: rad/s, +가 좌회전, 허용 |yaw|<=0.89)
ros2 topic pub -1 /puppy_control/velocity/autogait puppy_control_msgs/msg/Velocity \
  "{x: 0.0, y: 0.0, yaw_rate: 0.35}"

# ★ 정지 (모두 0) — 시험이 끝나면 반드시 보낼 것
ros2 topic pub -1 /puppy_control/velocity/autogait puppy_control_msgs/msg/Velocity \
  "{x: 0.0, y: 0.0, yaw_rate: 0.0}"

# 자세 바꾸기 — 고개 숙이기 (pitch: rad, height: -15~-5 cm)
ros2 topic pub -1 /puppy_control/pose puppy_control_msgs/msg/Pose \
  "{roll: 0.0, pitch: 0.26, yaw: 0.0, height: -10.0, x_shift: -0.5, stance_x: 0.0, stance_y: 0.0, run_time: 500}"

# 기본 서기 자세로 복귀 (서비스)
ros2 service call /puppy_control/go_home std_srvs/srv/Empty

# 하드웨어 살아있나 확인 — 부저 삑
ros2 topic pub -1 /ros_robot_controller/set_buzzer ros_robot_controller_msgs/msg/BuzzerState \
  "{freq: 1900, on_time: 0.1, off_time: 0.9, repeat: 1}"

# 배터리 전압(mV) / 카메라 프레임레이트 / 조종 값 모니터링
ros2 topic echo /ros_robot_controller/battery --once
ros2 topic hz /image_raw
ros2 topic echo /puppy_control/velocity/autogait
```

VR 경로(UDP) 자체를 시험하려면 (Unity 없이):

```bash
# 조이스틱 패킷 흉내 → 전진 (1초 안에 반복해서 보내지 않으면 자동 정지됨)
python3 -c "import socket;socket.socket(socket.AF_INET,socket.SOCK_DGRAM).sendto(b'X:0.0,Z:30.0',('<로봇IP>',5005))"
# 긴급정지 / 해제
python3 -c "import socket;socket.socket(socket.AF_INET,socket.SOCK_DGRAM).sendto(b'ESTOP',('<로봇IP>',5005))"
python3 -c "import socket;socket.socket(socket.AF_INET,socket.SOCK_DGRAM).sendto(b'RESUME',('<로봇IP>',5005))"
# 상태 받아보기 (배터리/신호세기)
python3 -c "import socket;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.sendto(b'hello',('<로봇IP>',5007));print(s.recvfrom(256)[0].decode())"
# 카메라는 브라우저로: http://<로봇IP>:8080/stream_viewer?topic=/image_raw  (web_video_server 실행 시)
```

전체 실기기 테스트 순서(10단계)와 합격 기준: [공부자료/16](./공부자료/16_빌드_실행_테스트_가이드.md) 3절.

## 안전 주의

- 조종 노드에는 1초 무신호 자동정지·속도 제한·ESTOP 이 구현되어 있지만,
  **첫 시험은 반드시 로봇을 들어올린 상태**에서 하세요.
- UDP 명령에는 인증이 없어 같은 네트워크의 누구나 보낼 수 있습니다 —
  공용 네트워크에서는 사용하지 마세요. 설계 근거: [공부자료/14](./공부자료/14_Safety_안전설계.md), [13](./공부자료/13_네트워크_UDP_WebSocket.md)

## 문서

한국어 학습/설계 문서는 [`공부자료/`](./공부자료/00_목차.md) 폴더에 있습니다
(ROS2 기초, 제어 토픽과 단위계, 카메라 스트리밍, VR 프로토콜, C++ 이식 현황,
SLAM, Safety, 실험·문제해결 기록 등 16편).

## 크레딧 / 라이선스

- 원본 로봇 소프트웨어: © [Hiwonder](https://www.hiwonder.com/) —
  [Hiwonder/PuppyPi](https://github.com/Hiwonder/PuppyPi)
  (원본 저장소에 별도 LICENSE 파일이 없으므로 해당 코드의 모든 권리는
  원저작자에게 있습니다. 원본 안내는 위 저장소의 README 와 공식 문서 참고)
- VR 컨트롤러 프로토콜: [inrjin/PuppyPi_MR_Controller](https://github.com/inrjin/PuppyPi_MR_Controller)
- 이 포크에서 새로 작성한 패키지(`puppy_vr_control*`, `lidar_mapping_cpp`,
  `obstacle_climb_cpp`, `*_cpp` 이식판, 스크립트, `공부자료/`)는 **Apache-2.0** 입니다.
- 본 저장소는 대학 수업(다학제간 프로젝트) 목적이며 상업적 사용을 의도하지 않습니다.
