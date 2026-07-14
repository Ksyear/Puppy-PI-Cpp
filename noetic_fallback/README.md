# noetic_fallback — [비상용 3안] ROS1 Noetic 용 VR 조종

**이 폴더는 비상용입니다.** 기본 계획(2안: ROS2 Humble 환경 구축)이 준비되기 전,
지금 로봇에 꽂혀 있는 **ROS1(Noetic) 이미지 그대로** VR 조종을 돌려야 할 때만 씁니다.
ROS2 환경이 완성되면 이 폴더는 사용하지 않습니다.

- UDP 프로토콜(조종 5005 / 영상 5006 / 상태 5007, ESTOP/RESUME 포함)은
  ROS2판과 **완전히 동일** — Unity(Quest) 앱은 아무것도 바꿀 필요 없음
- ROS1 스택에서도 제어 토픽이 같음을 확인함
  (`/puppy_control/velocity/autogait`, `puppy_control/Velocity` — ros1 브랜치 원본으로 검증)
- 카메라 토픽만 다름: ROS1 은 `/usb_cam/image_raw/compressed` (기본값 반영됨)

## 실제 로봇 접속 (Hiwonder 이미지의 네트워크 모드)

부팅하면 로봇이 자체 핫스팟(AP)을 켠다: SSID `HW-XXXXXXXX`, 비밀번호 기본 `hiwonder`,
이 모드에서 로봇 IP 는 **192.168.149.1 고정**. 노트북을 그 Wi-Fi 에 붙이고:

```bash
ssh pi@192.168.149.1     # 안 되면 ubuntu@ / 비밀번호는 raspberrypi 또는 hiwonder 시도
```
(WonderPi 앱으로 공유기 Wi-Fi 에 붙이는 LAN 모드도 가능 — 시연은 휴대폰 핫스팟 권장)

## 설치 전 확인 (순서 중요)

```bash
printenv ROS_DISTRO                      # noetic 확인
rostopic list | grep puppy               # /puppy_control/velocity/autogait 존재 확인
rostopic list | grep -iE 'image|bat'     # 카메라/배터리 토픽 이름 메모
echo $ROS_PACKAGE_PATH                   # catkin 워크스페이스 경로 메모
# 로봇 들어올리고 자체 동작 확인:
rostopic pub -1 /puppy_control/velocity/autogait puppy_control/Velocity "{x: 10.0, y: 0.0, yaw_rate: 0.0}"
rostopic pub -1 /puppy_control/velocity/autogait puppy_control/Velocity "{x: 0.0, y: 0.0, yaw_rate: 0.0}"
```

카메라 노드는 이미지가 부팅 시 이미 띄우는 경우가 많다 — `/usb_cam/image_raw` 가
이미 보이면 카메라 launch 를 또 실행하지 말 것. WonderPi 앱과 동시 조종 금지.

## 설치 (Noetic 로봇에서) — 빌드 불필요

(주의) **저장소 루트에서 `catkin_make` 를 실행하지 말 것** — 저장소가 catkin
워크스페이스로 오인되어 `src/CMakeLists.txt` 심링크 등 부산물이 생기고
(ROS2 빌드 방해), 패키지도 못 찾는다. 실수로 실행했다면 정리:
```bash
rm -rf <저장소>/build <저장소>/devel <저장소>/src/CMakeLists.txt <저장소>/.catkin_workspace
```

**가장 간단한 실행 — 한 방 스크립트** (export+chmod+roslaunch+이전 노드 정리 포함):
```bash
~/Puppy-PI-Cpp/noetic_fallback/run_vr.sh   # 실기기 튜닝값이 기본 (16cm/s, 회전보정 1.5)
~/Puppy-PI-Cpp/noetic_fallback/run_vr.sh max_speed_x:=10   # 더 천천히 하고 싶을 때
```

수동으로 하고 싶다면 — rospy 순수 파이썬 패키지라 **catkin 빌드 없이** ROS_PACKAGE_PATH 등록만으로 동작한다:

```bash
# 로봇에서 (저장소를 clone 한 상태 기준)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:<저장소>/noetic_fallback
chmod +x <저장소>/noetic_fallback/puppy_vr_control_noetic/scripts/*.py
roslaunch puppy_vr_control_noetic vr_control.launch
# 매번 export 가 귀찮으면: 위 export 줄을 ~/.bashrc 에 추가

# 또는 스크립트 직접 실행 (등록조차 불필요):
cd <저장소>/noetic_fallback/puppy_vr_control_noetic/scripts
python3 vr_udp_teleop.py     # "수신 대기 0.0.0.0:5005" 뜨면 성공
```

(정석대로 catkin 에 넣고 싶다면: `echo $ROS_PACKAGE_PATH` 로 로봇의 **기존**
워크스페이스를 찾아 그 안의 `src/` 로 이 패키지를 복사한 뒤 거기서
`catkin_make --only-pkg-with-deps puppy_vr_control_noetic`)

## 실행

```bash
# 카메라 (ROS1 방식)
roslaunch puppy_bringup usb_cam.launch
# VR 조종 + 영상 + 상태
roslaunch puppy_vr_control_noetic vr_control.launch
# 조종만 / 디버그
roslaunch puppy_vr_control_noetic vr_control.launch use_camera:=false debug:=true
```

## 테스트 (ROS1 이므로 rostopic 사용)

```bash
# 전진 → 정지
rostopic pub -1 /puppy_control/velocity/autogait puppy_control/Velocity "{x: 10.0, y: 0.0, yaw_rate: 0.0}"
rostopic pub -1 /puppy_control/velocity/autogait puppy_control/Velocity "{x: 0.0, y: 0.0, yaw_rate: 0.0}"
# VR 경로 (UDP — ROS2판과 동일)
python3 -c "import socket;socket.socket(socket.AF_INET,socket.SOCK_DGRAM).sendto(b'X:0.0,Z:30.0',('<로봇IP>',5005))"
python3 -c "import socket;socket.socket(socket.AF_INET,socket.SOCK_DGRAM).sendto(b'ESTOP',('<로봇IP>',5005))"
```

## 배터리 표시에 대한 결론 (실기기 확인 완료)

이 로봇(ROS1 이미지 + RasAdapter 보드)은 **배터리 전압을 소프트웨어로 읽을 수 없다**:
토픽/서비스 없음, I2C 스캔 결과 0x68(MPU6050 IMU)만 존재 — 전압 ADC는 뒷면
FND(숫자 표시) 전용 MCU 에만 연결. `sensor_control.py` 의 getBattery()(0x7A)는
다른 제품용 SDK 잔재로 이 보드에서는 Errno 121.
→ 대시보드의 BATTERY 는 `--` 로 표시되며, 배터리 확인은 **뒷면 FND 육안**,
저전압 경고는 **보드 자체 부저**(6.8V 미만)가 담당한다. 노드는 30초 탐지 후
조용히 포기하므로 로그를 오염시키지 않는다.
(ROS2 이미지/신형 보드로 전환하면 `/ros_robot_controller/battery` 토픽으로 자동 연결됨)

## 확인이 필요한 것 (이미지 버전에 따라 다를 수 있음)

1. **배터리 토픽**: `rostopic list | grep -i bat` 로 이름 확인 →
   `rostopic info <토픽>` 으로 타입 확인 → launch 에 파라미터 추가:
   `<param name="battery_topic" value="..."/>`, `<param name="battery_type" value="float32"/>`
   (UInt16=mV, Float32=V 지원)
2. **카메라 압축 토픽**: `rostopic list | grep compressed` — 없으면
   `rosrun image_transport republish raw in:=/usb_cam/image_raw compressed out:=/usb_cam/image_raw`

## 왜 이 폴더는 colcon 빌드에서 제외되나

상위에 `COLCON_IGNORE` 가 있어 ROS2 빌드(`./build_all.sh`)가 이 폴더를 건너뛴다.
ROS1(catkin)과 ROS2(colcon) 패키지가 섞이면 빌드가 깨지기 때문이다.
| `lidar_mapping` | LiDAR로 지도 작성 → `/map` + UDP 5008 이미지 전송 (대시보드 M 키로 보기, `use_mapping:=true` 로 활성) |
