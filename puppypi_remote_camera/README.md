# PuppyPi Remote Camera

NV50-HD220S의 영상을 PuppyPi에서 macOS 노트북으로 전송하고, 노트북
키보드로 PuppyPi를 수동 조종하며, 사용자가 지정한 구간만 노트북에
녹화하는 프로그램입니다.

이 프로젝트에는 LiDAR, SLAM, `move_base`, `explore_lite`, 미로 탐색, AI,
지도 작성 및 자율주행 코드가 없습니다. 로봇은 JPEG 프레임을 메모리에서
TCP로 보낼 뿐 영상 파일을 만들지 않습니다. 녹화 파일 생성 코드는
`laptop/video_recorder.py`에만 있습니다.

## 구성

```text
NV50-HD220S --USB--> PuppyPi
                         ├── TCP 5000 영상 ──> macOS GUI/노트북 녹화
                         └<─ UDP 5001 제어 ── macOS 키보드
```

영상 프레임은 TCP, 조종 패킷은 UDP를 사용합니다. 조종 JSON에는 UUID
`client_id`, 증가하는 `sequence`, Unix `timestamp`가 들어갑니다. 로봇은
현재 영상 클라이언트와 같은 IP에서 온 한 개의 UUID/UDP 주소만 받아들입니다.
중복·역전 sequence, 0.3초보다 오래된 timestamp, NaN, Inf, 누락·추가 필드,
잘못된 JSON은 거부합니다.

로봇 쪽 최종 속도 제한은 다음과 같습니다.

| 입력 | `puppy_control/Velocity` |
|---|---|
| W | `x=+5.0`, `y=0.0`, `yaw_rate=0.0` |
| S | `x=-3.0`, `y=0.0`, `yaw_rate=0.0` |
| A | `x=0.0`, `y=0.0`, `yaw_rate=+0.20` |
| D | `x=0.0`, `y=0.0`, `yaw_rate=-0.20` |

W/A처럼 전진과 회전을 함께 누르면 두 성분이 함께 전송됩니다. 서버가 다시
범위를 제한하며 `y`는 패킷과 관계없이 항상 0입니다.

## 1. PuppyPi 최초 1회 설치

이 저장소에서 만든 `puppypi_remote_camera` 디렉터리를 PuppyPi의 요구 경로로
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

ROS Noetic과 제조사 `puppy_control` 패키지가 기존에 정상 설치되어 있어야
합니다. 다음 두 명령이 실패하면 이 프로그램을 실행하기 전에 PuppyPi의 실제
catkin workspace를 먼저 복구해야 합니다.

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

## 2. 카메라 식별과 실제 모드 확인

카메라를 PuppyPi의 USB 포트에 연결한 뒤 다음을 실행합니다.

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id
```

출력에서 NV50-HD220S 장치 이름을 확인합니다. 기본 식별 문자열은
`config/robot_config.yaml`의 다음 항목입니다.

```yaml
camera:
  name_contains: NV50-HD220S
```

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

## 3. PuppyPi 한 번에 실행

기본 설정은 실제 모터 토픽을 사용하지 않습니다.

```bash
cd /home/pi/Puppy-PI-Cpp/puppypi_remote_camera/robot
./start_robot_server.sh
```

스크립트는 ROS Noetic과 찾을 수 있는 PuppyPi workspace를 source하고,
`v4l2-ctl`, OpenCV, PyYAML, `rospy`, `puppy_control/Velocity`를 검사한 뒤
서버를 실행합니다. OS 패키지 설치는 `sudo`가 필요하므로 앞의 최초 1회
단계에서만 수행합니다.

PuppyPi IP는 다음으로 확인합니다.

```bash
hostname -I
```

노트북과 PuppyPi의 시스템 시간이 0.3초 이상 어긋나면 정상 명령도 오래된
패킷으로 거부될 수 있습니다. 두 장치에서 `date`를 확인하고 자동 시간 동기화를
켜 두십시오. watchdog 자체는 PuppyPi의 monotonic clock을 사용합니다.

## 4. macOS 한 번에 실행

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

## 5. 조종과 종료

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
괄호 안의 `창 포커스 없음`, `최신 영상 없음`, `제어 ACK 없음`, `비상정지`
중 표시된 조건을 먼저 해결해야 합니다. 안전 조건을 우회해 키를 전역
수집하지는 않습니다.

E는 toggle이 아니라 latch입니다. E를 누른 뒤에는 GUI의 `비상정지 해제`
버튼으로 명시적으로 해제하고, 새 이동 키를 다시 눌러야 움직입니다.

## 6. 노트북 녹화

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

## 7. 모터를 움직이지 않는 검증

`config/robot_config.yaml`을 다음 상태로 유지합니다.

```yaml
enable_motor_control: false
```

이 상태에서 `puppy_control/Velocity` 메시지는 실제 제어 토픽이 아니라
`/puppypi_remote_camera/test_velocity`에 발행됩니다. PuppyPi의 별도
터미널에서 확인합니다.

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

7. W/S/A/D를 각각 누르고 위 표의 값과 `rostopic echo` 결과가 같은지
   확인합니다.
8. Space와 키 release에서 즉시 0 명령이 나오는지 확인합니다.
9. 이동 키를 누른 상태에서 노트북 프로그램 또는 Wi-Fi를 끊고, 마지막
   non-zero 메시지 뒤 0.3초 이내에 0 명령이 발행되는지 timestamp를 포함해
   확인합니다.
10. Q와 창 닫기에서 0 명령이 여러 번 발행되는지 확인합니다.

이 테스트는 실제 하드웨어에서 수행해야 합니다. 소스 코드 정적 검사만으로
NV50-HD220S의 펌웨어가 제공하는 모드, Wi-Fi 처리량, ROS master 연결 또는
모터의 실제 정지를 증명할 수 없습니다.

## 8. 실제 모터 제어 전 안전 확인

위 시험 토픽 검증이 모두 성공하기 전에는 다음 값을 변경하면 안 됩니다.

```yaml
enable_motor_control: true
```

변경 후 서버를 실행하면 실제 모터 제어가 활성화되었다는 경고와 함께 안전
확인을 요청합니다. 로봇을 먼저 바닥에서 들어 올리거나 충분히 넓은 시험
공간에 두고, 주변 사람·동물·장애물을 제거한 뒤 프롬프트에 정확히
`ENABLE MOTORS`를 입력해야 시작됩니다. 비대화형 실행은 명시적인
`--confirm-motor-control` 인자가 없으면 거부됩니다.

안전 확인이 끝나고 실제 모터 제어가 활성화된 경우에만 서버는 기본적으로
`/puppy_control/go_home` (`std_srvs/Empty`) 서비스를 호출해 다리를 먼저
폅니다. 서비스 호출이 실패하면 이동 명령을 받기 전에 서버 실행을 중단합니다.
시험 모드(`enable_motor_control: false`)에서는 이 서비스를 호출하지 않으므로
실제 서보가 움직이지 않습니다.

실제 시험 직전에는 다음도 확인하십시오.

```bash
rostopic info /puppy_control/velocity/autogait
```

다른 원격 조종기나 자율주행 노드가 같은 토픽을 동시에 발행 중이면 충돌하므로
모두 종료해야 합니다. 이 프로젝트는 다른 발행자를 자동으로 종료하지 않습니다.

## 설정

포트는 두 설정 파일에서 같은 값이어야 합니다.

- 로봇: `config/robot_config.yaml`의 `video_port`, `control_port`
- 노트북: `config/laptop_config.yaml`의 `network.video_port`,
  `network.control_port`

기본값은 영상 TCP 5000, 조종 UDP 5001입니다. 변경한 포트는 PuppyPi 방화벽과
네트워크에서도 허용해야 합니다. 통신은 암호화·인증되지 않으므로 신뢰할 수
있는 로컬 네트워크에서만 사용하십시오.

1080p JPEG의 전송량은 장면에 따라 달라집니다. 기본 JPEG 품질은 85이며
프로그램은 품질이나 녹화 해상도를 자동으로 낮추지 않습니다. 대신 송수신
TCP 버퍼를 제한하고 전송이 끝날 때마다 가장 최신 프레임을 선택해 오래된
프레임 누적을 제한합니다. Wi-Fi 실효 처리량이 실제 JPEG 전송량보다 낮으면
품질·해상도·FPS를 모두 유지한 채 지연까지 없앨 수는 없습니다. 단순 UDP
변경은 큰 JPEG를 여러 datagram으로 분할해야 하고 유실 프레임이 생기므로
녹화 무결성을 보장하지 않습니다.
