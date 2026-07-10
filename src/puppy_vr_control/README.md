# puppy_vr_control

Meta Quest(Unity) VR 컨트롤러로 PuppyPi를 조종하기 위한 **C++ ROS2 패키지**.

[PuppyPi_MR_Controller](https://github.com/inrjin/PuppyPi_MR_Controller) 저장소의
`raspberry_pi/udp_joystick_receiver.py`(파이썬, 로봇 제어부는 미구현 상태)를
C++/ROS2 노드로 이식하고, 실제 PuppyPi 제어 연결까지 완성한 것이다.
카메라 영상을 Quest로 UDP 전송하는 노드도 포함한다.

```
[Quest/Unity]                         [PuppyPi (Raspberry Pi 5)]
UDP_Joystick_Sender.cs ──"X:12.3,Z:-5.0"──▶ vr_udp_teleop ──Velocity──▶ puppy_control ──▶ 서보
(C# 영상 수신)         ◀──JPEG 청크(UDP)── camera_udp_sender ◀──/image_raw/compressed── usb_cam
```

## 노드 구성

### 1. `vr_udp_teleop` — VR 조이스틱 수신 → 로봇 제어
- UDP 포트 **5005** 에서 `"X:12.3,Z:-5.0"` 형식(Unity가 20Hz 전송) 수신
- 데드존(±5°) / 정규화(45°에서 최대) 후
  `puppy_control_msgs/Velocity` 를 `/puppy_control/velocity/autogait` 로 발행
  (공식 조이스틱 노드 `remote_control_joystick.py` 와 같은 토픽·단위)
- **1초 무신호 시 자동 안전 정지** (Wi-Fi 끊김, Quest 앱 종료 대비)
- 속도 양자화(1 cm/s, 2 deg/s 단위)로 gait 파라미터가 매 패킷마다 재계산되는 것 방지

### 2. `camera_udp_sender` — 카메라 영상 → Quest
- `usb_cam` 이 발행하는 `/image_raw/compressed`(JPEG) 구독
- JPEG 프레임을 1400바이트 청크로 쪼개 12바이트 헤더와 함께 UDP 전송
  (헤더: `uint32 frame_id | uint16 chunk_index | uint16 chunk_count | uint32 frame_size`, 네트워크 바이트순서)
- Quest가 5006 포트로 아무 패킷("hello")을 보내면 그 주소로 자동 스트리밍 시작
- Unity 쪽 수신 코드 예시는 `공부자료/05_VR_Unity_UDP_통신.md` 참고

> 카메라를 Unity에서 더 간단히 받고 싶다면 `web_video_server`(MJPEG, HTTP 8080)를
> 써도 된다 — 두 방식 비교는 `공부자료/04_카메라_영상_스트리밍.md` 참고.

## 빌드 (로봇의 라즈베리파이에서)

```bash
# 이 패키지를 로봇 워크스페이스로 복사한 뒤
cd /home/ubuntu/ros2_ws
colcon build --packages-select puppy_vr_control
source install/setup.bash
```

## 실행

```bash
# 1) 로봇 기본 제어 노드가 떠 있어야 함 (보통 부팅 시 자동 실행)
#    puppy_control, ros_robot_controller

# 2) 카메라 (영상 전송을 쓸 경우)
ros2 launch peripherals usb_cam.launch.py

# 3) VR 제어 + 카메라 전송
ros2 launch puppy_vr_control vr_control.launch.py

# 옵션: 카메라 전송 없이 제어만 / 디버그 출력
ros2 launch puppy_vr_control vr_control.launch.py use_camera:=false debug:=true
```

## 로봇 없이 수신 테스트

PC나 라즈베리파이에서 노드를 띄워 두고, MR 저장소의 가짜 송신기로 확인:

```bash
# 터미널 A
ros2 run puppy_vr_control vr_udp_teleop --ros-args -p debug:=true

# 터미널 B (PuppyPi_MR_Controller/raspberry_pi/)
python3 mock_joystick_sender.py --host <노드 실행 중인 IP>

# 발행 확인
ros2 topic echo /puppy_control/velocity/autogait
```

## 주요 파라미터 (`config/vr_control_params.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `port` | 5005 | Unity 송신 포트와 일치해야 함 |
| `deadzone_deg` | 5.0 | 중립 처리 각도 (±) |
| `max_angle_deg` | 45.0 | 최대 속도가 되는 기울기 |
| `recv_timeout_sec` | 1.0 | 무신호 안전 정지 시간 |
| `max_speed_x` | 15.0 | 최대 전진 속도 (cm/s, 한계 35) |
| `max_yaw_rate_deg` | 20.0 | 최대 회전 속도 (deg/s, 한계 51) |
| `invert_forward` / `invert_turn` | false / true | 실기기 방향이 반대면 뒤집기 |
| `client_ip` | "" | 비우면 hello 패킷으로 자동 발견 |
| `max_fps` | 15.0 | 영상 전송 프레임 제한 |

## 방향이 반대로 움직일 때

Unity 씬의 조이스틱 모델 축 방향에 따라 전진/회전 부호가 다를 수 있다.
실기기를 띄운 상태에서 조이스틱을 **앞으로만** 기울여 보고:
- 뒤로 가면 → `invert_forward: true`
- 좌우가 반대면 → `invert_turn` 뒤집기

## 체크리스트 (움직이지 않을 때)

1. Unity `UDP_Joystick_Sender.raspberryPi_IP` = 라즈베리파이 실제 IP (`hostname -I`)
2. Quest와 라즈베리파이가 **같은 Wi-Fi**
3. `ros2 node list` 에 `puppy_control` 존재 확인
4. `ros2 run puppy_vr_control vr_udp_teleop --ros-args -p debug:=true` 로 수신 로그 확인
5. 방화벽: `sudo ufw allow 5005/udp && sudo ufw allow 5006/udp`
