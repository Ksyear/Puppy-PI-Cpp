# PuppyPi Remote Camera — ROS1 Noetic 조종 서버

PuppyPi를 **Meta Quest(Unity)** 또는 기존 **macOS 노트북**으로 원격 조종하는
ROS1 Noetic 프로그램입니다. 이 문서의 기본 대상은 다음과 같습니다.

- **일반 Raspberry Pi 4** + **Ubuntu Server 20.04 ARM64** + **ROS1 Noetic(APT)**
- Quest ↔ Pi 제어: **UDP 5001 JSON v2** (`로보틱스_담당자_전달용_제어규격_v2.md`)
- Quest ← Pi 영상: **UDP 5006 JPEG 청크** (`usb_cam` + `camera_udp_sender`)
- 기본값은 **모터 없는 시험 모드**입니다. 제조사 `puppy_control` 패키지가 없어도
  시작하며, `go_home`을 호출하지 않고 실제 모터 토픽에 발행하지 않습니다.

> 이 경로에서는 Docker, ROS2 Humble/Jazzy, 저장소 루트의 `setup_from_scratch.sh`,
> `build_all.sh`, CMake `robot` target을 **사용하지 않습니다**(모두 ROS2용).
> 이 디렉터리는 순수 rospy 패키지라 `catkin_make`도 필요 없습니다.

## 0. 구성 한눈에 보기

```text
Meta Quest (Unity)                          Raspberry Pi 4 (ROS1 Noetic)
그립 -> forward/turn(-1~1) 20Hz ──UDP 5001──> robot_server.py (v2 파서·상태기)
             ACK(state/reason) <─UDP 5001──┘   ├ 시험: /puppypi_remote_camera/test_velocity
                                               │       geometry_msgs/TwistStamped (m/s)
                                               └ 실제: /puppy_control/velocity/autogait
                                                       puppy_control/Velocity (cm/s)
"hello"(1초마다) ─────────────────UDP 5006──> camera_udp_sender
JPEG 청크 <───────────────────────UDP 5006──┘  <- /usb_cam/image_raw/compressed <- usb_cam <- 카메라
```

| 포트 | 방향 | 형식 | 이 경로에서 |
|---|---|---|---|
| 5001/udp | Quest → Pi, ACK는 보낸 주소로 | UTF-8 JSON 객체 1개 = datagram 1개 (v2) | **사용** |
| 5006/udp | Quest hello → Pi, 청크 Pi → Quest | `!IHHI` 헤더 + JPEG 조각 | **사용** |
| 5007/udp | 선택 | `RSSI:-52;UP:123` 문자열 | `use_status:=true`일 때만 |
| 5005/udp | — | 기존 `X:..,Z:..` 문자열 조종 | **사용 안 함** (동시 실행 차단) |
| 5000/tcp | — | 노트북 v1 영상(`PRC1` 헤더 + JPEG) | 노트북 v1 프로파일 전용 |

### 영상 경로를 UDP 5006 하나로 정한 이유

- 저장소에 두 영상 경로가 따로 있습니다. TCP 5000(`camera_stream.py`)은 이
  서버가 카메라를 직접 열어 노트북 GUI로 보내고, UDP 5006
  (`noetic_fallback/.../camera_udp_sender.py`)은 ROS 이미지 토픽을 Quest용
  청크로 보냅니다.
- 이 저장소가 Quest용으로 정의한 형식은 UDP 5006이며, Quest와 같은 수신
  방식의 참조 구현(`tools/vr_test_dashboard.py`의 `FrameAssembler`)이 있습니다.
  TCP 5000에는 Quest 쪽 수신 구현이 없습니다.
- **확인된 공백:** 공개 Unity 저장소
  [inrjin/PuppyPi_MR_Controller](https://github.com/inrjin/PuppyPi_MR_Controller)
  (2026-09-28 확인)에는 5005 문자열 송신기 `UDP_Joystick_Sender.cs`만 있고
  **영상 수신 코드와 v2 JSON 송신 코드는 없습니다.** Unity 측에서 6절의 형식대로
  새로 구현해야 합니다.
- 카메라는 `usb_cam` **한 프로세스만** 엽니다. `protocol_version: 2`에서는
  `tcp_video_enabled: false`가 강제되어 조종 서버가 카메라를 열지 않고,
  `start_quest_v2.sh`는 카메라 장치를 다른 프로세스가 쓰고 있으면 시작하지
  않습니다.

## 1. SD 카드에 Ubuntu Server 20.04 ARM64 설치

1. 노트북에서 공식 이미지를 받고 체크섬을 확인합니다.

   ```bash
   curl -LO https://cdimage.ubuntu.com/releases/20.04/release/ubuntu-20.04.5-preinstalled-server-arm64+raspi.img.xz
   shasum -a 256 ubuntu-20.04.5-preinstalled-server-arm64+raspi.img.xz
   # 44b98acd3fd4379c6b194696520b6aecb2f596b601e43e9b6934c83f0aa61026 이어야 합니다.
   ```

2. [Raspberry Pi Imager](https://www.raspberrypi.com/software/)에서 `Use custom`으로
   위 `.img.xz`를 선택해 microSD에 씁니다.
3. Pi 4에 유선 LAN을 연결하고 부팅합니다. 로컬 콘솔 또는 `ssh ubuntu@<PI_IP>`로
   `ubuntu` / `ubuntu` 로그인 후 새 비밀번호를 정합니다(첫 로그인에서 강제).
4. Wi-Fi가 필요하면 별도 netplan 파일을 만듭니다.

   ```bash
   sudo tee /etc/netplan/60-wifi.yaml >/dev/null <<'EOF'
   network:
     version: 2
     wifis:
       wlan0:
         dhcp4: true
         optional: true
         access-points:
           "<SSID>":
             password: "<PASSWORD>"
   EOF
   sudo chmod 600 /etc/netplan/60-wifi.yaml
   sudo netplan apply
   hostname -I      # 이 IP를 Quest에 입력합니다
   ```

5. 첫 부팅 직후에는 자동 업데이트가 apt lock을 잡고 있을 수 있습니다.
   `Could not get lock`이 나오면 몇 분 기다린 뒤 다시 실행합니다.

   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

> **지원 상태:** Ubuntu 20.04 표준 지원과 ROS Noetic은 모두 2025-05-31에
> 종료되었습니다. 2026-09-28 확인 시 `packages.ros.org`의 focal arm64 저장소에는
> Noetic 최종 패키지(2025-05-29 동기화)가 남아 있어 설치는 가능하지만 보안
> 업데이트는 없습니다. 인터넷에서 분리된 시험 네트워크에서 사용하십시오.

## 2. ROS1 Noetic APT 설치

ROS 저장소 서명 키가 2025-06-01에 교체되었으므로 예전 `apt-key add` 방식 대신
공식 `ros-apt-source` 패키지를 사용합니다. 아래 1.3.0 focal 패키지는
`/etc/apt/sources.list.d/ros.sources`(`http://packages.ros.org/ros/ubuntu focal main`,
`Signed-By: /usr/share/keyrings/ros-archive-keyring.gpg`)를 설치합니다.

```bash
sudo apt install -y curl ca-certificates
ROS_APT_SOURCE_VERSION=1.3.0
curl -fL -o /tmp/ros-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros-apt-source_${ROS_APT_SOURCE_VERSION}.focal_all.deb"
sudo dpkg -i /tmp/ros-apt-source.deb
sudo apt update
sudo apt install -y ros-noetic-ros-base
```

예전 방식으로 만든 `/etc/apt/sources.list.d/ros-latest.list`가 있으면 `apt update`가
`Signed-By` 충돌을 보고합니다. 그 파일을 지운 뒤 다시 `sudo apt update` 하십시오.

확인:

```bash
source /opt/ros/noetic/setup.bash
printenv ROS_DISTRO        # noetic
python3 --version          # Python 3.8.x
```

## 3. 저장소 클론과 의존성

```bash
sudo apt install -y git python3-yaml python3-opencv v4l-utils psmisc \
  ros-noetic-usb-cam ros-noetic-image-transport-plugins ros-noetic-cv-bridge

cd ~
git clone https://github.com/Ksyear/Puppy-PI-Cpp.git
cd ~/Puppy-PI-Cpp/puppypi_remote_camera
```

| 패키지 | 용도 |
|---|---|
| `ros-noetic-ros-base` | `rospy`, `roslaunch`, `rostopic`, `geometry_msgs`, `std_msgs`, `std_srvs` |
| `python3-yaml` | 설정 파일 |
| `ros-noetic-usb-cam` (0.3.7) | 카메라 유일 점유 노드 |
| `ros-noetic-image-transport-plugins` | `/usb_cam/image_raw/compressed` JPEG 토픽 |
| `python3-opencv`, `v4l-utils` | 카메라 이름으로 장치·모드 확인(`find_camera.py`) |
| `ros-noetic-cv-bridge` | 압축 토픽이 없을 때 `camera_udp_sender`의 원본 인코딩 대체 경로 |
| `psmisc` | `fuser`로 카메라 동시 점유 검사 |

## 4. ROS·모터 없이 단위 테스트

Pi에서도, 노트북(Python 3.8 이상 + PyYAML)에서도 실행할 수 있습니다. ROS master,
카메라, 제조사 패키지가 필요 없습니다.

```bash
cd ~/Puppy-PI-Cpp/puppypi_remote_camera
python3 -m unittest discover -s tests -v
```

| 파일 | 검사 내용 |
|---|---|
| `tests/test_v2_protocol.py` | strict JSON(누락·추가·중복 키, bool/문자열 숫자, NaN·Inf·오버플로, 범위), UUID·sequence, v1·5005 형식 거부, 속도 변환 |
| `tests/test_v2_session.py` | 중립 확인 후 출발, 250ms watchdog, 1초 lease, 비상정지 latch, 단일 조종자, 중복·역순, 세션 교체, fault |
| `tests/test_udp_loopback.py` | 127.0.0.1 실제 UDP로 ACK 형식, watchdog 정지 지연, 오류 패킷 폭주, 수신 스레드 생존, 같은 포트 중복 실행 거부 |
| `tests/test_robot_server.py` | 가짜 `rospy`로 시험 모드가 시험·상태 토픽만 만들고 모터 토픽·`go_home`을 쓰지 않는지, 설정 검증 |

## 5. 시험 모드 실행 (기본)

1. `config/robot_config.yaml`이 다음 상태인지 확인합니다(저장소 기본값).

   ```yaml
   enable_motor_control: false
   protocol_version: 2
   tcp_video_enabled: false
   ```

2. 카메라를 USB에 연결하고 이름을 확인합니다.

   ```bash
   v4l2-ctl --list-devices
   ls -l /dev/v4l/by-id
   ```

   출력의 장치 이름 일부를 `camera.name_contains`에 적습니다(기본
   `NV50-HD220S`). 일치하는 장치가 없으면 다른 카메라로 대체하지 않고 종료합니다.

3. 실행합니다. roscore는 roslaunch가 자동으로 띄웁니다.

   ```bash
   cd ~/Puppy-PI-Cpp/puppypi_remote_camera/robot
   ./start_quest_v2.sh                       # 조종 + 영상(640x480 MJPG, 최대 15fps 전송)
   ./start_quest_v2.sh use_camera:=false     # 카메라 없이 조종 서버만
   ./start_quest_v2.sh image_width:=1280 image_height:=720
   ```

   스크립트는 ROS Noetic 환경, 5005 조종기(`vr_udp_teleop`)·중복 서버·기존
   `camera_udp_sender` 실행 여부, `usb_cam`·압축 플러그인 설치, 카메라 이름·요청
   모드 광고 여부, 카메라 동시 점유를 검사한 뒤
   `roslaunch puppypi_remote_camera quest_v2.launch`를 실행합니다.

   정상 로그 예:

   ```text
   조종 UDP 수신: 0.0.0.0:5001 (protocol 2)
   모터 없는 시험 모드: 실제 모터 토픽과 go_home을 사용하지 않고 /puppypi_remote_camera/test_velocity (m/s, rad/s)에만 발행합니다
   영상 전송 대기(ROS1): /usb_cam/image_raw/compressed -> UDP 5006
   ```

## 6. Quest(Unity) 연결 설정

### Pi 쪽

- Pi IP: `hostname -I`. Quest와 Pi는 같은 서브넷이어야 하며 AP의 client
  isolation이 꺼져 있어야 합니다.
- 조종 ACK는 패킷을 보낸 주소로 돌아가므로 Pi에 Quest IP를 설정하지 않습니다.
- 영상은 기본적으로 Quest가 5006으로 보내는 hello의 출발 주소로 보냅니다.
  목적지를 고정하려면 `./start_quest_v2.sh quest_ip:=<QUEST_IP>` (이때 Quest는
  UDP 5006에서 수신).
- 방화벽: Ubuntu Server 기본값은 `ufw` 비활성입니다. 켰다면
  `sudo ufw allow 5001/udp && sudo ufw allow 5006/udp`.

### Unity 쪽 계약

| 항목 | 값 |
|---|---|
| 조종 목적지 | `<PI_IP>:5001`, 20Hz, 같은 `UdpClient`로 ACK 수신 |
| 영상 | 한 `UdpClient`로 `<PI_IP>:5006`에 1초마다 임의 datagram(`hello`) 송신, 같은 소켓으로 청크 수신. 5초 동안 hello가 없으면 전송 중단 |
| 영상 청크 | 빅엔디언 `uint32 frame_id, uint16 chunk_index, uint16 chunk_count, uint32 frame_size` + JPEG 조각(≤1400B). 모든 조각이 모이면 JPEG 1장, frame_id가 바뀌면 미완성 프레임 폐기 |

v2 명령(모든 필드 필수, 추가 필드 거부):

```json
{"protocol":2,"type":"command","client_id":"<앱 실행마다 새 UUID>","sequence":123,
 "control_active":true,"forward":0.6,"turn":-0.3}
```

- `client_id`: 하이픈 포함 36자 표준 UUID(C# `Guid.NewGuid().ToString()`). nil UUID 거부.
- `sequence`: 0~9007199254740991 정수, 모든 type이 같은 번호 흐름으로 증가.
- `forward`, `turn`: -1~1 유한한 JSON 숫자. 문자열·bool·NaN·Infinity 거부.
- 이벤트: `emergency_stop`, `clear_emergency`, `disconnect` (공통 4필드만).
- Unity 담당: 선택된 그립 하나의 기울기 → 중립 구간·감도 곡선 → `forward/turn`,
  그립 놓기·추적 상실·일시정지 시 **즉시** 중립 송신, 중립도 20Hz로 계속 송신.
  로봇은 Unity의 X/Z 각도를 받지 않고 감도 곡선을 다시 적용하지 않습니다.

로봇 측 변환(변환 뒤 같은 상한으로 한 번 더 제한, `y`는 항상 0):

```text
forward >= 0: x = forward × 5 cm/s      forward < 0: x = forward × 3 cm/s
yaw_rate = turn × 0.20 rad/s            예) 0.6, -0.3 -> x=3.0cm/s, yaw_rate=-0.06rad/s
```

ACK (정확히 이 8개 필드):

```json
{"protocol":2,"type":"ack","client_id":"...","sequence":124,"accepted":true,
 "reason":"ok","emergency_stop":false,"state":"idle"}
```

| `state` | 의미 |
|---|---|
| `awaiting_neutral` | 시작·새 조종자·비상정지 해제·lease 만료 뒤. 중립 명령 대기 |
| `idle` | 중립 확인됨, `control_active:false` |
| `active` | `control_active:true` 명령 적용 중(두 축 0이면 목표 0) |
| `timeout` | 마지막 인정 command 후 250ms 경과로 정지. 새 중립 필요 |
| `emergency_stop` | latch. `clear_emergency` 전까지 활성 명령 거부 |
| `fault` | 속도 발행 실패 또는 실제 모드의 모터 토픽 동시 발행자 감지. 재시작 필요 |

| `reason` | 의미 |
|---|---|
| `ok` | 규칙에 따라 받아들임(실제 보행 성공을 뜻하지 않음) |
| `invalid_packet` | JSON·필드·자료형·범위 오류. 식별 가능하면 `client_id`/`sequence`를 되돌리고, 아니면 `null`/`-1` |
| `unsupported_protocol` | `protocol`이 2가 아님(v1 노트북 패킷 포함) |
| `out_of_order` | 이 세션의 이전 번호 이하 |
| `not_owner` | 다른 조종자가 소유 중, 또는 종료·교체된 세션 |
| `emergency_stop` | 비상정지 중 활성 명령 |
| `neutral_required` | 중립 확인 전 활성 명령 |
| `fault` | `fault` 상태의 활성 명령 (**규격 후보 목록에 없던 로봇 측 추가 코드**) |

- "중립 명령" = `control_active:false`, 또는 `control_active:true`이면서 두 축이 0.
  `control_active:false`이면 두 축 값과 관계없이 목표 0.
- watchdog은 **새로 인정한 command**(중립 포함)만 갱신합니다. 오류·중복·역순·다른
  조종자 패킷과 이벤트는 갱신하지 않습니다. 로봇 내부 monotonic 시계 기준입니다.
- 비상정지 latch는 조종자가 바뀌어도 유지됩니다. 비상정지 중에도 중립 command와
  이벤트는 `accepted:true`로 처리합니다.
- 단일 조종자는 UUID와 송신 주소(IP·포트)로 식별합니다. lease(1초) 동안 다른
  UUID나 다른 포트의 패킷은 `not_owner`입니다.

## 7. ROS 토픽 확인

새 터미널마다 `source /opt/ros/noetic/setup.bash` 후 실행합니다.

| 모드 | 토픽 | 타입 | 단위 |
|---|---|---|---|
| 시험(기본) | `/puppypi_remote_camera/test_velocity` | `geometry_msgs/TwistStamped` | `twist.linear.x` **m/s**(= x cm/s ÷ 100), `linear.y` m/s(항상 0), `angular.z` rad/s, `header.stamp` 발행 시각 |
| 실제 | `/puppy_control/velocity/autogait` | `puppy_control/Velocity` (`float32 x, y, yaw_rate`) | `x`, `y` **cm/s**, `yaw_rate` rad/s |
| 공통 | `/puppypi_remote_camera/control_state` | `std_msgs/String` (JSON, latch, 변화 시 발행) | `state`, `emergency_stop`, `owner`, `last_sequence`, `last_reason`, `fault`, 목표 속도(cm/s, rad/s) |

같은 입력 `forward=0.6, turn=-0.3`은 시험 토픽에서 `linear.x: 0.03`,
`angular.z: -0.06`, 실제 토픽에서 `x: 3.0`, `yaw_rate: -0.06`입니다.

```bash
rostopic info /puppypi_remote_camera/test_velocity   # Type: geometry_msgs/TwistStamped
rostopic echo /puppypi_remote_camera/control_state
rostopic echo /puppypi_remote_camera/test_velocity
rostopic hz /puppypi_remote_camera/test_velocity      # Quest 20Hz 송신 중이면 약 20Hz
rostopic hz /usb_cam/image_raw/compressed

# 시험 모드에서는 아래 두 명령이 아무것도 보여주지 않아야 합니다.
rostopic info /puppy_control/velocity/autogait        # ERROR: Unknown topic
rosservice list | grep go_home
```

Quest 없이 v2 경로 전체를 확인하려면 Pi의 다른 터미널에서 시험 클라이언트를
실행합니다(Quest 앱은 꺼 두십시오. 조종자는 한 명만 허용됩니다).

```bash
python3 ~/Puppy-PI-Cpp/puppypi_remote_camera/tools/v2_test_client.py --robot 127.0.0.1 --scenario all
# 마지막 줄: 결과: PASS (실패 0)
```

`drive`는 중립 → `forward=0.6, turn=-0.3` 2초 → 중립 → disconnect,
`watchdog`은 송신을 끊고 `timeout`·`neutral_required`, `estop`은 latch와 해제 후
중립 요구, `lease`는 1초 이상 끊은 뒤 소유권 해제·재획득·종료 세션 거부를
판정합니다. 노트북에서 `--robot <PI_IP>`로 실행하면 Wi-Fi 경로도 확인됩니다.

## 8. 종료 절차

1. Quest 앱에서 조종을 끝냅니다(앱이 `disconnect`를 보내면 즉시 정지·소유권 해제,
   보내지 못해도 250ms watchdog과 1초 lease가 정지·해제합니다).
2. `start_quest_v2.sh` 터미널에서 `Ctrl+C`. roslaunch가 노드에 SIGINT를 보내고
   조종 서버는 정지 명령을 `stop_repetitions`(5)회 발행한 뒤 끝납니다.
   `required="true"`라 조종 서버가 먼저 죽어도 전체 launch가 종료됩니다.
3. 남은 프로세스와 카메라 점유가 없는지 확인합니다.

   ```bash
   pgrep -af 'robot_server.py|usb_cam|camera_udp_sender'   # 출력 없음
   fuser -v /dev/video*                                    # 출력 없음
   ```

4. 전원을 끌 때는 `sudo poweroff` 후 초록 LED가 멈추면 분리합니다.

## 9. 실제 PuppyPi 모터 모드 (명시적으로만)

일반 Pi 4에는 PuppyPi 하드웨어·서보 보드·제조사 보행 엔진이 없으므로 이 절은
해당하지 않습니다. 실제 PuppyPi에서 아래를 모두 확인한 뒤에만 켭니다.

1. 제조사 ROS1 workspace가 있고 메시지·서비스가 보입니다. 제조사 패키지는 APT로
   설치되지 않습니다.

   ```bash
   export PUPPYPI_WORKSPACE_SETUP=/실제/catkin_ws/devel/setup.bash
   source /opt/ros/noetic/setup.bash && source "$PUPPYPI_WORKSPACE_SETUP"
   rosmsg show puppy_control/Velocity        # float32 x, float32 y, float32 yaw_rate
   rosservice list | grep /puppy_control/go_home
   rostopic info /puppy_control/velocity/autogait   # 다른 Publishers가 없어야 함
   ```

2. 로봇을 들어 올린 상태에서 제조사 경로 자체를 먼저 확인합니다
   (`noetic_fallback/README.md`의 `rostopic pub` 절차).
3. 10절의 미합의 정책, 특히 **지연 패킷 신선도 정책**을 VR 측과 결정합니다.
4. `config/robot_config.yaml`에서 `enable_motor_control: true`로 바꾸고 실행합니다.

   ```bash
   ./start_quest_v2.sh confirm_motor_control:=true
   ```

   두 조건 중 하나라도 없으면 시작하지 않습니다. 실제 모드 서버는 모터 토픽에 다른
   발행자가 있으면 시작을 거부하고, 실행 중 다른 발행자가 생기거나 발행자 조회가
   3회 연속 실패하면 `fault`로 멈춥니다. 조종 패킷은 `go_home` 서기 자세가 끝난
   뒤에 받습니다.

제조사 ROS1 공개 소스(`Hiwonder/PuppyPi` ros1 브랜치 `puppy.py`) 기준 autogait
콜백은 `|x| ≤ 35`, `y == 0`, `|yaw_rate| ≤ 51°/s`인 메시지만 적용하고
**범위를 벗어난 메시지는 조용히 무시해 직전 보행을 유지**합니다. 모두 0이면
`move_stop`입니다. 현재 상한(5cm/s, 0.20rad/s ≈ 11.5°/s)은 이 범위 안이지만,
콜백이 메시지마다 gait·stance를 다시 설정하므로 20Hz 매 명령 발행이 보행에
미치는 영향은 실기로 확인해야 합니다.

## 10. 미합의 정책과 확인이 필요한 항목

아래는 v2 wire 필드를 추가하지 않은 채 **실제 모터 적용 전에 VR 측과 결정해야 할**
항목입니다.

| 항목 | 현재 구현 | 결정 필요 |
|---|---|---|
| 지연됐지만 sequence가 증가한 패킷 | **판별하지 않음.** v2 기본 패킷에 송신 시각이 없어 수신 순서·번호만 검사합니다(`test_known_gap_delayed_increasing_sequence_is_accepted`가 이 공백을 기록) | 연결 시 시계 차 추정, 로봇 발급 단기 토큰 등. 선택에 따라 v2 필드 추가 가능 |
| 세션 교체 후 예전 세션 패킷 | 잠정: `disconnect`한 UUID와 다른 UUID에 소유권을 넘긴 UUID는 서버 실행 동안 `not_owner`. lease 만료만 된 같은 UUID는 마지막 번호보다 큰 번호로 재획득 가능 | 정책 확정 |
| 비소유자의 `emergency_stop` | 거부(`not_owner`) | 안전상 누구의 비상정지든 받을지 |
| 영상 단절 시 조종 차단 | v2에서는 로봇이 영상과 조종을 결합하지 않음 | Unity·로봇 공통 정책 |
| 가감속·복합 입력 제한·최소 출발 속도 | 미구현. 최종 상한 clamp만 적용 | 보행 엔진 실측 후 적용 계층 결정 |
| 제자리 회전 후진 드리프트 | 미보정(레거시 5005 경로는 `turn_forward_bias` 1.5cm/s) | 로봇 측 보정 여부 |
| 실제 모드 발행 주기 | 인정한 command마다 발행(20Hz) | 변경 시 발행 + heartbeat가 나은지 실측 |
| 250ms watchdog과 Wi-Fi 지연 | 인정한 command 간격이 250ms를 넘으면 `timeout` → 새 중립·재조작 필요 | 실제 Wi-Fi에서 20Hz 송신 간격·유실을 측정해 값 확정 |
| ACK 상실 판정 시간 | VR 측 결정 사항 | 통합 시험 전 확정 |
| `reason: fault` | 로봇 측 추가 코드 | 합의 |

## 11. 검증 범위

| 항목 | 상태 |
|---|---|
| 4절 단위·UDP loopback 테스트 65개 | macOS에서 Python 3.8.20·3.14로 실행, 모두 통과 (2026-09-28) |
| watchdog 정지 지연(loopback, 30회) | 253.7~257.7ms (macOS, Pi 수치 아님) |
| 시작 스크립트 분기(가짜 명령) | 5005 조종기·중복 서버·카메라 점유·실제 모드 확인 누락 시 거부, 인자 전달 확인 |
| Raspberry Pi 4 / Ubuntu 20.04 / ROS Noetic 실행 | **미실행** |
| roslaunch, `usb_cam`, 압축 토픽, UDP 5006 영상 | **미실행** |
| Unity/Quest 송수신 | **미실행** (공개 Unity 저장소에 v2 송신·영상 수신 코드 없음) |
| 실제 PuppyPi 모터·보행 엔진·정지 지연 | **미실행** |

---

## 노트북 v1 프로파일 (기존 macOS 키보드 조종·녹화)

NV50-HD220S의 영상을 PuppyPi에서 macOS 노트북으로 전송하고, 노트북
키보드로 PuppyPi를 수동 조종하며, 사용자가 지정한 구간만 노트북에
녹화하는 기존 기능입니다. Quest v2와 **동시에 쓰지 않습니다**. 한 서버 실행은
한 protocol만 해석합니다.

이 기능에는 LiDAR, SLAM, `move_base`, `explore_lite`, 미로 탐색, AI,
지도 작성 및 자율주행 코드가 없습니다. 로봇은 JPEG 프레임을 메모리에서
TCP로 보낼 뿐 영상 파일을 만들지 않습니다. 녹화 파일 생성 코드는
`laptop/video_recorder.py`에만 있습니다.

### 구성

```text
NV50-HD220S --USB--> PuppyPi
                         ├── TCP 5000 영상 ──> macOS GUI/노트북 녹화
                         └<─ UDP 5001 제어 ── macOS 키보드
```

`config/robot_config.yaml`에서 다음 두 값을 바꿉니다.

```yaml
protocol_version: 1
tcp_video_enabled: true
```

영상 프레임은 TCP, 조종 패킷은 UDP를 사용합니다. 조종 JSON에는 UUID
`client_id`, 증가하는 `sequence`, Unix `timestamp`가 들어갑니다. 로봇은
영상 클라이언트가 연결되어 있으면 같은 IP에서 온 한 개의 UUID/UDP 주소만
받아들입니다. 영상 재연결 중에는 기존 UDP 소유권과 watchdog이 독립적으로
유지됩니다.
중복·역전 sequence, 0.3초보다 오래된 timestamp, NaN, Inf, 누락·추가 필드,
잘못된 JSON은 거부합니다.

로봇 쪽 최종 속도 제한은 다음과 같습니다(실제 모드 `puppy_control/Velocity` 값).

| 입력 | `puppy_control/Velocity` |
|---|---|
| W | `x=+5.0`, `y=0.0`, `yaw_rate=0.0` |
| S | `x=-3.0`, `y=0.0`, `yaw_rate=0.0` |
| A | `x=0.0`, `y=0.0`, `yaw_rate=+0.20` |
| D | `x=0.0`, `y=0.0`, `yaw_rate=-0.20` |

W/A처럼 전진과 회전을 함께 누르면 두 성분이 함께 전송됩니다. 서버가 다시
범위를 제한하며 `y`는 패킷과 관계없이 항상 0입니다.

### v1-1. PuppyPi 설치

1~3절의 설치를 따르거나, 이미 ROS Noetic이 있는 PuppyPi에 이 디렉터리를
복사합니다. 아래 명령은 macOS에서 실행하는 예입니다.

```bash
cd "/path/to/PuppyPi C++"
rsync -av puppypi_remote_camera/ \
  pi@<PUPPYPI_IP>:/home/pi/Puppy-PI-Cpp/puppypi_remote_camera/
```

PuppyPi에서 OS 패키지를 한 번 설치합니다.

```bash
sudo apt update
sudo apt install v4l-utils python3-opencv python3-yaml
chmod +x /home/pi/Puppy-PI-Cpp/puppypi_remote_camera/robot/start_robot_server.sh
```

실제 모터 모드에서만 제조사 `puppy_control` 패키지가 필요합니다. 다음 두 명령이
실패하면 실제 모드를 켜기 전에 PuppyPi의 실제 catkin workspace를 먼저 복구해야
합니다. 시험 모드는 없어도 시작합니다.

```bash
source /opt/ros/noetic/setup.bash
rospack find puppy_control
rosmsg show puppy_control/Velocity
```

제조사 workspace 경로가 자동 탐색 경로와 다르면 실행 시 해당 setup 파일을
지정합니다.

```bash
export PUPPYPI_WORKSPACE_SETUP=/실제/catkin_ws/devel/setup.bash
```

### v1-2. 카메라 식별과 실제 모드 확인

카메라를 PuppyPi의 USB 포트에 연결한 뒤 다음을 실행합니다.

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id
```

출력에서 NV50-HD220S 장치 이름을 확인합니다. 기본 식별 문자열은
`config/robot_config.yaml`의 `camera.name_contains: NV50-HD220S`입니다.
실제 출력 문자열이 다르면 확인된 고유 문자열로만 바꿉니다. 일치하는 장치가
없을 때 프로그램은 `/dev/video0`이나 다른 웹캠으로 대체하지 않고 종료하며,
탐지된 장치 목록을 출력합니다. 일치한 장치의 `/dev/v4l/by-id` 별칭이 있으면
그 영구 경로를 우선 사용합니다.

장치가 광고하는 실제 모드는 다음과 같이 독립적으로 확인할 수 있습니다.

```bash
v4l2-ctl --device /dev/v4l/by-id/<확인한-카메라-별칭> --list-formats-ext
```

서버도 시작할 때 지원 pixel format, 해상도, FPS를 모두 출력합니다.
`MJPG 1920x1080 30fps`가 실제 목록에 있으면 그것을 우선 선택합니다. 목록에
없으면 프로그램이 가장 가까운 유효 모드를 선택하므로, 이 경우 1080p/30fps가
된다고 간주하면 안 됩니다. GUI의 해상도와 수신 FPS가 실제 결과입니다.

### v1-3. PuppyPi 실행

이 프로파일에서는 조종 서버가 카메라를 직접 엽니다. `usb_cam`이나
`start_quest_v2.sh`가 실행 중이면 카메라를 열 수 없으므로 먼저 종료합니다.
단독 실행 스크립트는 ROS master가 필요하므로 다른 터미널에서 `roscore`를 먼저
실행합니다.

```bash
roscore                       # 터미널 1
cd /home/pi/Puppy-PI-Cpp/puppypi_remote_camera/robot
./start_robot_server.sh       # 터미널 2
```

스크립트는 ROS Noetic과 찾을 수 있는 PuppyPi workspace를 source하고,
PyYAML·`rospy`·표준 메시지와 ROS master 연결을 검사한 뒤 서버를 실행합니다.
`v4l2-ctl`·OpenCV·`puppy_control`은 설정상 필요할 때 서버가 검사합니다.

PuppyPi IP는 `hostname -I`로 확인합니다.

노트북과 PuppyPi의 시스템 시간이 0.3초 이상 어긋나면 정상 명령도 오래된
패킷으로 거부될 수 있습니다. 두 장치에서 `date`를 확인하고 자동 시간 동기화를
켜 두십시오. watchdog 자체는 PuppyPi의 monotonic clock을 사용합니다.

### v1-4. macOS 실행

Python 3와 Tkinter가 동작하는지 먼저 확인합니다.

```bash
python3 -c "import tkinter; print(tkinter.TkVersion)"
```

실패하면 Tk를 포함한 Python 3 배포판을 설치해야 합니다. 그 뒤 노트북에서는
다음 한 명령으로 실행합니다.

```bash
cd puppypi_remote_camera/laptop
chmod +x start_laptop_client.sh
./start_laptop_client.sh --robot-ip <PUPPYPI_IP>
```

스크립트는 첫 실행에 `laptop/.venv`를 만들고 필요한 Python 패키지를
설치합니다. 이후에는 `requirements.txt`가 바뀐 경우에만 다시 설치합니다.
스크립트는 Tk 8.6 이상이 있는 Python을 선택합니다. Apple 시스템 Tk 8.5는
폐기 예정이며 영상 표시 문제가 발생할 수 있어 사용하지 않습니다. Homebrew
Python에서 Tk가 없으면 `brew install python-tk@3.14`로 설치합니다.
자동 선택이 맞지 않으면
`PUPPYPI_PYTHON=/경로/python3`으로 명시할 수 있습니다.
`--robot-ip`를 생략하면 `config/laptop_config.yaml`의 `robot_ip`를
사용하고, 그것도 비어 있으면 GUI가 IP를 묻습니다.

수동 설치 방식이 필요한 경우에는 다음 명령도 사용할 수 있습니다.

```bash
cd puppypi_remote_camera/laptop
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 laptop_client.py --robot-ip <PUPPYPI_IP>
```

macOS 방화벽이 수신 연결 허용 여부를 물으면 Python을 허용해야 합니다.
공유기/AP의 client isolation이 활성화되어 있으면 두 장치가 같은 Wi-Fi
이름을 사용해도 서로 연결되지 않을 수 있습니다.

### v1-5. 조종과 종료

| 키 | 동작 |
|---|---|
| W | 누르는 동안 전진 |
| S | 누르는 동안 후진 |
| A | 누르는 동안 좌회전 |
| D | 누르는 동안 우회전 |
| Space | 즉시 정지 |
| E | 비상정지 latch 활성화 |
| R | 노트북 녹화 시작/종료 |
| Q | 정지 패킷 반복 전송 후 종료 |

W/S/A/D는 키를 누른 상태에서만 유효합니다. 키를 놓으면 새 속도 또는 정지를
즉시 전송합니다. 창이 포커스를 잃거나 영상 연결이 끊기거나 프로그램이
종료되면 노트북이 정지 패킷을 반복 전송합니다. 패킷이 하나도 도착하지 않는
경우에도 PuppyPi의 독립 watchdog이 정지 명령을 발행합니다. 설정 상한은
0.3초이고 기본값은 scheduling 여유를 둔 0.25초입니다.

macOS에서 한글 입력 상태여도 W/A/S/D/E/R/Q/Space의 물리 키 위치를
인식합니다. 키가 반응하지 않으면 영상 영역을 한 번 클릭한 뒤 GUI의
`키보드 입력`을 확인하십시오. `현재 이동 명령`에 `차단(...)`이 표시되면
괄호 안의 `창 포커스 없음`, `영상 연결 없음`, `제어 ACK 없음`, `비상정지`
중 표시된 조건을 먼저 해결해야 합니다. 안전 조건을 우회해 키를 전역
수집하지는 않습니다.

E는 toggle이 아니라 latch입니다. E를 누른 뒤에는 GUI의 `비상정지 해제`
버튼으로 명시적으로 해제하고, 새 이동 키를 다시 눌러야 움직입니다.

### v1-6. 노트북 녹화

R 또는 `녹화 시작` 버튼을 누르기 전에는 영상 파일을 만들지 않습니다.
기본 폴더는 다음과 같으며 프로그램이 없으면 생성합니다.

```text
~/PuppyPiRecordings
```

기본 파일명은 실제 수신 크기를 사용합니다.

```text
puppypi_YYYYMMDD_HHMMSS_1920x1080.mp4
```

같은 초에 이름이 겹치면 `_01` 같은 번호를 붙여 기존 파일을 덮어쓰지
않습니다. 수신한 decoded BGR 프레임을 크기 변경 없이 writer에 전달합니다.
프레임 간격이 벌어지면 직전 프레임을 제한적으로 반복해 고정 FPS 컨테이너의
시간축을 유지합니다. GUI 표시 영상만 축소됩니다.

프로그램은 실제 첫 프레임으로 MP4 writer를 미리 열고 읽어 보는 검사를
수행합니다. 사용할 수 없으면 AVI, 그 다음 MKV writer를 시도하고 GUI와
터미널에 이유를 표시합니다. 녹화 종료 시 파일 크기, 첫 프레임 디코딩,
해상도를 다시 검사합니다. 빈 파일이나 OpenCV로 첫 프레임을 읽지 못하는
파일은 성공으로 표시하지 않습니다. 성공 여부와 무관하게 경로, 해상도, FPS,
녹화 시간 및 검사 결과를 터미널에 출력합니다.

### v1-7. 모터를 움직이지 않는 검증

`enable_motor_control: false`를 유지합니다. 이 상태에서 변환된 속도는 실제
제어 토픽이 아니라 `/puppypi_remote_camera/test_velocity`에
`geometry_msgs/TwistStamped`로 발행됩니다. **단위가 m/s이므로** 위 표의
W(`x=+5.0` cm/s)는 `twist.linear.x: 0.05`로 보입니다(이전 버전은 이 토픽에
`puppy_control/Velocity` cm/s를 발행했습니다). PuppyPi의 별도 터미널에서
확인합니다.

```bash
source /opt/ros/noetic/setup.bash
rostopic echo /puppypi_remote_camera/test_velocity
```

다음 항목을 순서대로 확인하십시오.

1. 서버 로그의 선택 장치 이름과 `/dev/v4l/by-id` 경로가 NV50-HD220S인지
   확인합니다.
2. 서버의 지원 모드 목록에서 `MJPG 1920x1080 30fps` 존재 여부를 확인합니다.
3. 노트북 GUI 영상과 표시된 실제 해상도·수신 FPS를 확인합니다.
4. R로 녹화를 시작·종료하고 터미널에 `녹화 성공`이 출력되는지 확인합니다.
5. macOS에서 출력 경로를 QuickTime Player 등으로 열어 전체 구간을
   재생합니다.
6. PuppyPi에서 다음 명령의 출력이 비어 있는지 확인합니다.

   ```bash
   find /home/pi/Puppy-PI-Cpp/puppypi_remote_camera \
     -type f \( -name '*.mp4' -o -name '*.avi' -o -name '*.mkv' \) -print
   ```

7. W/S/A/D를 각각 누르고 위 표의 값(÷100, m/s)과 `rostopic echo` 결과가 같은지
   확인합니다.
8. Space와 키 release에서 즉시 0 명령이 나오는지 확인합니다.
9. 이동 키를 누른 상태에서 노트북 프로그램 또는 Wi-Fi를 끊고, 마지막
   non-zero 메시지 뒤 0.3초 이내에 0 명령이 발행되는지 `header.stamp`로
   확인합니다.
10. Q와 창 닫기에서 0 명령이 여러 번 발행되는지 확인합니다.

이 테스트는 실제 하드웨어에서 수행해야 합니다. 소스 코드 정적 검사만으로
NV50-HD220S의 펌웨어가 제공하는 모드, Wi-Fi 처리량, ROS master 연결 또는
모터의 실제 정지를 증명할 수 없습니다.

### v1-8. 실제 모터 제어 전 안전 확인

위 시험 토픽 검증이 모두 성공하기 전에는 `enable_motor_control: true`로 바꾸면
안 됩니다. 변경 후 서버를 실행하면 실제 모터 제어가 활성화되었다는 경고와 함께
안전 확인을 요청합니다. 로봇을 먼저 바닥에서 들어 올리거나 충분히 넓은 시험
공간에 두고, 주변 사람·동물·장애물을 제거한 뒤 프롬프트에 정확히
`ENABLE MOTORS`를 입력해야 시작됩니다. 비대화형 실행은 명시적인
`--confirm-motor-control` 인자가 없으면 거부됩니다.

안전 확인이 끝나고 실제 모터 제어가 활성화된 경우에만 서버는 기본적으로
`/puppy_control/go_home` (`std_srvs/Empty`) 서비스를 호출해 다리를 먼저
폅니다. 서비스 호출이 끝난 뒤 조종 패킷을 받으며, 호출이 실패하면 서버 실행을
중단합니다. 시험 모드(`enable_motor_control: false`)에서는 이 서비스를 호출하지
않으므로 실제 서보가 움직이지 않습니다.

실제 모드 서버는 `/puppy_control/velocity/autogait`에 다른 발행자(다른 원격
조종기나 자율주행 노드)가 있으면 시작하지 않고, 실행 중 생기면 `fault`로
정지합니다. 다른 발행자를 자동으로 종료하지는 않습니다.

```bash
rostopic info /puppy_control/velocity/autogait
```

### v1 설정

포트는 두 설정 파일에서 같은 값이어야 합니다.

- 로봇: `config/robot_config.yaml`의 `video_port`, `control_port`
- 노트북: `config/laptop_config.yaml`의 `network.video_port`,
  `network.control_port`, `network.protocol_version`(1)

기본값은 영상 TCP 5000, 조종 UDP 5001입니다. 변경한 포트는 PuppyPi 방화벽과
네트워크에서도 허용해야 합니다. 통신은 암호화·인증되지 않으므로 신뢰할 수
있는 로컬 네트워크에서만 사용하십시오(v2도 같습니다).

1080p JPEG의 전송량은 장면에 따라 달라집니다. 기본 JPEG 품질은 85이며
프로그램은 품질이나 녹화 해상도를 자동으로 낮추지 않습니다. 대신 송수신
TCP 버퍼를 제한하고 전송이 끝날 때마다 가장 최신 프레임을 선택해 오래된
프레임 누적을 제한합니다. Wi-Fi 실효 처리량이 실제 JPEG 전송량보다 낮으면
품질·해상도·FPS를 모두 유지한 채 지연까지 없앨 수는 없습니다. 단순 UDP
변경은 큰 JPEG를 여러 datagram으로 분할해야 하고 유실 프레임이 생기므로
녹화 무결성을 보장하지 않습니다.
