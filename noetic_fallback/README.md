# noetic_fallback — [비상용 3안] ROS1 Noetic 용 VR 조종

**이 폴더는 비상용입니다.** 기본 계획(2안: ROS2 Humble 환경 구축)이 준비되기 전,
지금 로봇에 꽂혀 있는 **ROS1(Noetic) 이미지 그대로** VR 조종을 돌려야 할 때만 씁니다.
ROS2 환경이 완성되면 이 폴더는 사용하지 않습니다.

- UDP 프로토콜(조종 5005 / 영상 5006 / 상태 5007, ESTOP/RESUME 포함)은
  ROS2판과 **완전히 동일** — Unity(Quest) 앱은 아무것도 바꿀 필요 없음
- ROS1 스택에서도 제어 토픽이 같음을 확인함
  (`/puppy_control/velocity/autogait`, `puppy_control/Velocity` — ros1 브랜치 원본으로 검증)
- 카메라 토픽만 다름: ROS1 은 `/usb_cam/image_raw/compressed` (기본값 반영됨)

## 설치 (Noetic 로봇에서)

```bash
# 이 폴더의 패키지를 로봇의 catkin 워크스페이스로 복사
scp -r noetic_fallback/puppy_vr_control_noetic ubuntu@<로봇IP>:~/ros_ws/src/
ssh ubuntu@<로봇IP>
cd ~/ros_ws && catkin_make --pkg puppy_vr_control_noetic   # (워크스페이스 경로는 이미지에 따라 ~/puppypi 등일 수 있음)
source devel/setup.bash
```

> 워크스페이스 경로 확인: `echo $ROS_PACKAGE_PATH` 로 기존 src 위치를 찾으면 된다.

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
