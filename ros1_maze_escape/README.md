# ros1_maze_escape

PuppyPi가 LD19 계열 2D LiDAR, ROS Navigation, 프런티어 탐색을 이용해
미지의 실내 미로를 탐색하도록 구성한 ROS1 Noetic catkin 패키지입니다. 실행
대상은 Ubuntu 20.04 + ROS Noetic + Raspberry Pi 4입니다. 이 저장소에서는
소스와 초기 설정만 제공하며 macOS 노트북에서 ROS 노드나 하드웨어를 실행하지
않습니다.

현재 구현은 안전한 초기 골격입니다. `/move_base/make_plan`을 이용한 도달 가능성
검사, 규칙 기반 후보 점수, 반복 실패 blacklist, `move_base` 목표 실행까지
구현되어 있습니다. 미로의 실제 출구를 판정하는 로직과 ONNX 추론 백엔드는
`TODO`이며, 지금의 `FINISHED`는 실제 출구 검출이 아니라 도달 가능한 프런티어가
연속으로 사라졌음을 뜻합니다. 이 차이를 실제 출구 탈출 성공으로 해석하면
안 됩니다.

## 시스템 구조

```text
LD19 driver
  └─ /scan (sensor_msgs/LaserScan)
       ├─ RF2O ─ /odom + odom -> base_footprint
       ├─ slam_gmapping ─ /map + map -> odom
       └─ move_base costmaps

/map + map -> base_footprint
  ├─ explore_lite (mode:=baseline에서만 목표 발행)
  └─ maze_brain_node (mode:=custom에서만 목표 발행)
       ├─ 프런티어 free 셀 후보 생성
       ├─ /move_base/make_plan으로 실제 전역 경로 길이 검사
       └─ /move_base action에 선택한 목표 전송

move_base
  ├─ navfn/NavfnROS
  ├─ dwa_local_planner/DWAPlannerROS
  └─ /cmd_vel (geometry_msgs/Twist, m/s 및 rad/s)
       └─ cmd_vel_adapter_node
            └─ /puppy_control/velocity/autogait
               (puppy_control/Velocity, cm/s 및 rad/s)
```

기준 TF 트리는 다음과 같습니다.

```text
map -> odom -> base_footprint -> lidar_frame
```

`slam_gmapping`이 `map -> odom`, RF2O 또는 로봇의 기존 오도메트리가
`odom -> base_footprint`을 제공합니다. `base_footprint -> lidar_frame`은
실측 전에는 제공하지 않습니다.

## 노드별 책임

| 노드 | 입력 | 출력 및 책임 |
|---|---|---|
| LD19 드라이버(외부) | LiDAR 직렬 데이터 | `/scan`; 헤더 frame은 `lidar_frame`이어야 함 |
| `rf2o_laser_odometry`(선택) | `/scan` | `/odom`, `odom -> base_footprint`; 기존 오도메트리가 없을 때만 사용 |
| `slam_gmapping` | `/scan`, TF | `/map`, `map -> odom` |
| `move_base` | 지도, TF, `/scan`, 목표 | Navfn 전역 경로, DWA 지역 회피, `/cmd_vel` |
| `explore` | `/map`, TF | baseline 모드에서만 `move_base` 목표 생성 |
| `maze_brain_node` | `/map`, TF, `/move_base/make_plan`, `/emergency_stop` | custom 모드의 후보 검사·점수·목표 실행, `~state` 발행 |
| `cmd_vel_adapter_node` | `/cmd_vel`, `/emergency_stop` | 단위 변환, 제한, watchdog, 최종 정지 후 PuppyPi 속도 발행 |

`maze_brain_node`의 상태는 `IDLE`, `CHECK_SYSTEM`, `SELECT_FRONTIER`,
`REQUEST_GLOBAL_PLAN`, `NAVIGATING`, `CHECK_EXIT`, `RECOVERY`, `FINISHED`,
`ERROR`, `EMERGENCY_STOP`으로 정의되어 있습니다. 현재 상태는
`/maze_brain/state`에서 확인할 수 있습니다.

## 속도 어댑터와 안전 정책

`cmd_vel_adapter_node`만 m/s를 cm/s로 변환합니다.

```text
output.x        = clamp(input.linear.x * 100.0, -reverse_limit, forward_limit)
output.y        = 0.0
output.yaw_rate = clamp(input.angular.z, -yaw_limit, yaw_limit)
```

기본 제한은 전진 10 cm/s, 후진 10 cm/s, 회전 0.28 rad/s입니다. 모든
`Twist` 성분을 검사하여 NaN 또는 Inf가 하나라도 있으면 즉시 정지합니다.
`/cmd_vel` 수신이 0.5초 끊겨도 정지하고, 시작할 때 정지 명령을 먼저 내리며,
종료 콜백에서는 정지를 3회 발행합니다. `/emergency_stop`
(`std_msgs/Bool`)이 `true`이면 모든 출력을 0으로 만들고, 해제 뒤에는 오래된
명령을 재사용하지 않고 새 `/cmd_vel`을 기다립니다. ROS debug 로그의
`[RAW_TWIST]`와 `[PUPPY_COMMAND]`로 입력과 출력을 구분합니다.

토픽, 속도 제한, watchdog은 모두 private 파라미터입니다. 기본값은
`launch/navigation.launch`에서 확인하고 launch 인자로 덮어쓸 수 있습니다.
안전 정지와 마지막 속도 제한은 AI나 프런티어 점수 코드가 아니라 이 결정론적
노드가 담당합니다.

## 프런티어 선택

탐지기는 점유 격자에서 unknown 셀과 인접한 free 셀을 프런티어로 묶습니다.
목표는 unknown 셀이나 기하학적 중심점이 아니라 프런티어 군집 안의 free 셀로
둡니다. 그 뒤 `/move_base/make_plan`이 비어 있지 않은 경로를 반환한 후보만
점수 계산에 넣습니다. 실제 경로 길이는 응답의 연속 pose 간 거리 합으로
계산합니다.

규칙 기반 점수의 입력은 다음 7개입니다.

- `information_gain`
- `actual_path_length`
- `clearance`
- `heading_change`
- `visit_count`
- `failure_count`
- `exit_probability`

초기 가중치는 다음 식과 같고 모두 `config/maze_brain.yaml`에 있습니다.

```text
score =
  2.0 * information_gain
  - 1.5 * actual_path_length
  + 1.5 * clearance
  - 0.4 * heading_change
  - 3.0 * visit_count
  - 5.0 * failure_count
  + 3.0 * exit_probability
```

같은 반경 안에서 반복 실패한 목표는 `failure_count`가 증가하고
`failure_blacklist_threshold`에 도달하면 후보에서 빠집니다. 현재
`exit_probability`는 출구 검출기가 없으므로 정확히 `0.0`입니다.

## explore_lite를 기준선으로만 사용하는 이유

`explore_lite`는 알려진 프런티어 탐색 구현과 비교 가능한 1차 기준선을
제공합니다. custom 방식의 `/move_base/make_plan` 기반 실제 경로 길이,
방문·실패 이력, 향후 출구 확률 점수의 효과를 검증하려면 동일 조건에서
기준선이 필요합니다. 최종 의사결정 로직으로 동시에 실행하지는 않습니다.

`launch/maze_escape.launch`의 `mode`는 `custom` 또는 `baseline`이어야 합니다.
각 모드는 서로 다른 조건부 group에 있어 `maze_brain_node`와 `explore_lite`가
동시에 시작되지 않습니다. 다른 문자열을 주면 launch substitution 검증이
실패하여 노드가 시작되지 않습니다.

```bash
roslaunch ros1_maze_escape maze_escape.launch mode:=baseline
roslaunch ros1_maze_escape maze_escape.launch mode:=custom
```

두 명령을 동시에 실행하거나 별도 터미널에서 `explore`를 추가 실행하면 목표
충돌이 발생하므로 금지합니다.

## AI가 `/cmd_vel`을 직접 제어하지 않는 이유

학습 모델의 출력은 시간 초과, NaN, 분포 밖 입력, 모델 파일 손상에 의해
불안정해질 수 있습니다. 따라서 향후 AI의 권한은 이미 안전성과 도달 가능성이
검사된 프런티어 후보의 점수 계산으로만 제한합니다. `move_base`가 경로와
장애물 회피 속도를 만들고, `cmd_vel_adapter_node`가 최종 제한과 정지를
적용합니다.

`FrontierScorer`를 구현하는 ONNX scorer를 별도 소스와 라이브러리로 추가한 뒤
`MazeBrain::scoreCandidate()`에 연결하면 됩니다. 연결 시 다음 순서를
유지해야 합니다.

1. free 셀 후보와 non-empty `make_plan`을 먼저 확인합니다.
2. 고정 크기·단위가 명시된 7개 feature만 모델에 전달합니다.
3. `ai_timeout_ms` 안에 반환된 유한한 단일 점수만 채택합니다.
4. 예외, 시간 초과, NaN, Inf, 모델 미로딩은 `HeuristicFrontierScorer`로
   복귀합니다.
5. 선택 뒤에도 blacklist, 목표 유효성, emergency stop, 속도 제한을 다시
   적용합니다.

현재 `use_ai_scorer:=true`를 주어도 ONNX 백엔드가 없다는 경고 후 규칙 기반
scorer를 사용합니다. 모델이 구현된 것처럼 동작하지 않습니다.

## 노트북의 ROS Noetic Docker 검증

Docker Desktop이 실행 중인 macOS 또는 Linux 호스트에서 다음 명령 하나로
Ubuntu 20.04 + ROS Noetic catkin 빌드와 단위 테스트를 수행합니다.

```bash
cd /path/to/ros1_maze_escape
./scripts/validate_noetic.sh
```

스크립트는 `docker/Dockerfile.noetic`으로 검증 이미지를 만들고, 네트워크가
차단된 일회용 컨테이너의 `/tmp/noetic_validation_ws`에서 다음 작업을
순서대로 수행합니다.

1. 현재 패키지를 임시 catkin workspace로 복사합니다.
2. 검증 전용 `puppy_control` 메시지 패키지를 임시 workspace에만 만듭니다.
3. `catkin_make`, `catkin_make run_tests`,
   `catkin_test_results --verbose`를 실행합니다.
4. 모든 launch XML과 YAML을 정적으로 파싱합니다.
5. `roslaunch --files`로 launch include와 substitution을 검사합니다.
6. `mode:=invalid`가 반드시 roslaunch 오류를 반환하는지 확인합니다.

검증 전용 `puppy_control/Velocity.msg`에는 `float32 x`, `float32 y`,
`float32 yaw_rate`만 있습니다. 이것은 테스트 컴파일을 위한 최소 stub이며
PuppyPi vendor 패키지를 대체하거나 그 호환성을 보증하지 않습니다. 실제
로봇에서는 반드시 다음 결과를 다시 확인해야 합니다.

```bash
rosmsg show puppy_control/Velocity
```

검증 스크립트는 장치 파일을 mount하지 않고 `--network none`으로 컨테이너를
실행합니다. `roslaunch --files`는 노드를 시작하지 않으므로 실제 로봇 토픽에
접속하거나 속도 명령을 발행하지 않습니다.

등록된 단위 테스트:

- `velocity_adapter_test`: 단위 변환, 제한, y 차단, emergency stop,
  NaN/Inf 정지
- `frontier_scorer_test`: 가중치 계산, 경로·clearance·실패 영향,
  NaN/Inf 거부
- `frontier_detector_test`: 합성 지도 검출, 최소 크기, free 목표,
  잘못된 data 크기, 회전된 map origin

## Raspberry Pi의 ROS Noetic 환경에 설치

아래 명령은 Raspberry Pi의 Ubuntu 20.04 + ROS Noetic 환경에서 실행할
절차입니다. 현재 macOS 노트북에서는 실행하지 않습니다. ROS Noetic 설치
자체는 [ROS Wiki의 Ubuntu 설치 문서](https://wiki.ros.org/noetic/Installation/Ubuntu)를
따릅니다.

프로젝트가 `/home/pi/Puppy-PI-Cpp/ros1_maze_escape`에 있고 vendor ROS1
workspace가 `/home/pi/puppy_pi`에 있으면 다음 한 명령으로 별도 catkin
workspace 링크 생성, Release 빌드, 단위 테스트와 결과 검증을 수행합니다.
기존 링크나 디렉터리가 다른 대상을 가리키면 덮어쓰지 않고 실패합니다.

```bash
cd /home/pi/Puppy-PI-Cpp/ros1_maze_escape
./scripts/setup_and_build_puppypi.sh
```

성공하면 마지막에 `PUPPYPI_BUILD_AND_TEST_OK`가 출력됩니다. 스크립트가 끝난
뒤 현재 터미널에서 빌드 결과를 사용하려면 다음 환경을 적용합니다.
세 gtest 결과 파일이 생성되고 각 파일에 한 개 이상의 테스트가 기록되지
않으면 스크립트는 성공 메시지를 출력하지 않고 종료 코드 1로 실패합니다.

```bash
source /home/pi/puppypi_maze_ws/devel/setup.bash
```

이 스크립트는 ROS 노드를 실행하거나 실제 속도 토픽에 발행하지 않습니다.
또한 패키지를 복사하지 않고 현재 프로젝트 폴더를
`/home/pi/puppypi_maze_ws/src/ros1_maze_escape`에 심볼릭 링크합니다.

일반 ROS 의존성 apt 패키지:

```bash
sudo apt update
sudo apt install \
  ros-noetic-navigation \
  ros-noetic-slam-gmapping \
  ros-noetic-explore-lite \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  libeigen3-dev
```

다음 두 의존성은 별도 확인이 필요합니다.

- `puppy_control`: PuppyPi ROS1 이미지에 있는 vendor catkin 패키지이며
  `puppy_control/Velocity` 메시지를 제공해야 합니다. `rospack find
  puppy_control`과 `rosmsg show puppy_control/Velocity`로 확인합니다.
- RF2O: upstream의 [ROS1 브랜치](https://github.com/MAPIRlab/rf2o_laser_odometry/tree/ros1)를
  같은 workspace의 `src`에 고정 커밋으로 복제합니다. 다른 오도메트리가
  `odom -> base_footprint`을 이미 제공하면 RF2O를 설치하거나 실행하지 말고
  `use_rf2o:=false`를 사용합니다.

LD19 드라이버 패키지는 보드·펌웨어·연결 방식에 따라 달라지므로 이 문서에서
임의로 특정 패키지를 지정하지 않습니다. 실제 사용 중인 드라이버가
`sensor_msgs/LaserScan`을 `/scan`에 발행하도록 설정해야 합니다.

독립 catkin workspace에 복사하고 빌드하는 예:

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/puppypi_maze_ws/src
cp -R /path/to/ros1_maze_escape ~/puppypi_maze_ws/src/

# RF2O가 필요한 경우에만:
git clone --branch ros1 https://github.com/MAPIRlab/rf2o_laser_odometry.git \
  ~/puppypi_maze_ws/src/rf2o_laser_odometry

cd ~/puppypi_maze_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

`puppy_control`이 다른 vendor workspace에 있으면 그 workspace의
`devel/setup.bash`를 먼저 source해야 합니다. 이 저장소 루트는 ROS2 colcon
workspace일 수 있으므로 그 루트에서 `catkin_make`를 실행하지 않습니다.
이 패키지 루트의 `COLCON_IGNORE`는 colcon이 ROS1 패키지를 건너뛰게 합니다.

## 실기기에서 먼저 측정할 값

다음 값은 현재 정보가 부족하므로 최종값이 아닙니다.

- `base_footprint -> lidar_frame`의 x, y, z, yaw, pitch, roll
- 보행 중 다리 끝까지 포함한 최대 footprint
- LD19 실제 최소·최대 유효 거리, 스캔 주기, 각도 방향, `frame_id`
- 바닥 재질과 보행 진동에서 RF2O 드리프트
- 실제 정지거리와 가속 한계

`config/costmap_common.yaml`의 footprint는 `MEASURE_REQUIRED`로 표시한
SAMPLE 사각형입니다. 자와 기준 좌표계를 이용해 최대 다리 점유 영역을 측정한
뒤 바꿔야 합니다. `launch/mapping.launch`의 LiDAR static TF는 기본적으로
꺼져 있고 모든 수치 기본값이 `MEASURE_REQUIRED`입니다. 측정값을 모두 넣은
경우에만 다음처럼 활성화합니다.

```bash
roslaunch ros1_maze_escape maze_escape.launch \
  publish_lidar_tf:=true \
  lidar_x:=<measured_m> lidar_y:=<measured_m> lidar_z:=<measured_m> \
  lidar_yaw:=<measured_rad> lidar_pitch:=<measured_rad> \
  lidar_roll:=<measured_rad>
```

임의의 0 또는 추정치를 실제 변환처럼 사용하면 지도와 장애물 위치가
왜곡됩니다.

## 토픽과 TF 점검

실제 로봇에서 구동 전에 다음을 확인합니다.

```bash
rostopic type /scan
rostopic hz /scan
rostopic echo -n 1 /scan/header
rostopic type /odom
rostopic echo -n 1 /odom
rostopic type /cmd_vel
rosmsg show puppy_control/Velocity
rostopic info /puppy_control/velocity/autogait
rosservice call /move_base/make_plan \
  "start: {header: {frame_id: map}, pose: {orientation: {w: 1.0}}}
goal: {header: {frame_id: map}, pose: {orientation: {w: 1.0}}}
tolerance: 0.1"

rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo base_footprint lidar_frame
rosrun tf view_frames
```

`/puppy_control/velocity/autogait`에는 명령 발행자가 하나만 있어야 합니다.
`rostopic info`의 Publishers가 예상 노드만 포함하는지 확인합니다.

## 안전한 시험 순서

1. 전원을 끈 상태에서 footprint와 LiDAR TF를 측정하고 설정 파일에
   기록합니다.
2. 로봇 다리가 바닥에 닿지 않도록 안정적으로 들어 올린 뒤 vendor
   `puppy_control`의 직접 정지 명령부터 검증합니다.
3. LiDAR 드라이버만 실행하고 `/scan`의 단위, 방향, frame, 주기를
   확인합니다.
4. 정적 TF만 활성화하고 RViz에서 스캔이 로봇 회전에 맞게 고정되어 보이는지
   확인합니다.
5. 다른 오도메트리를 끈 상태에서만 RF2O를 켜고 TF 발행자가 중복되지 않는지
   확인합니다.
6. 모터 명령 없이 bag 또는 정지 상태에서 gmapping, costmap, make_plan을
   확인합니다.
7. 들어 올린 상태에서 `/cmd_vel`의 작은 유한값, NaN/Inf 차단,
   0.5초 watchdog, emergency stop, 종료 정지 3회를 각각 확인합니다.
8. 넓고 통제된 공간에서 낮은 속도의 단일 `move_base` 목표를 시험하고 실제
   정지거리와 footprint 여유를 조정합니다.
9. 보호자가 즉시 전원을 차단할 수 있는 상태에서 `mode:=baseline`을 먼저
   기록합니다.
10. 동일한 시작 위치·미로·속도 제한으로 `mode:=custom`을 별도 실행해
    비교합니다.

실기기 최초 시험에서 로봇을 미로 안에 바로 넣거나 두 goal source를 동시에
실행하지 않습니다.

## baseline과 custom 평가 지표

동일한 지도, 시작 pose, 속도 제한, 배터리 조건에서 여러 반복 실험을 수행하고
각 run의 원시 rosbag과 설정 파일을 함께 보관합니다.

- 실제 출구 도달 성공률
- 출구 도달 시간과 탐색 종료 시간
- `/odom` 기준 이동 경로 길이
- 선택한 목표 수, non-empty `make_plan` 비율
- `move_base` 성공·중단·거부·timeout 횟수
- recovery 횟수와 blacklist 지점 수
- 중복 방문 횟수와 지도 커버리지
- 최소 장애물 여유, 충돌·접촉·수동 비상정지 횟수
- 평균·최대 CPU 및 메모리 사용량
- 탐색 종료 시 미탐색 영역과 최종 지도 품질

성공한 run만 평균내면 비교가 왜곡됩니다. 실패 run을 포함해 성공률, 중앙값,
분산, 최악값을 함께 기록해야 합니다.

## 설정 파일 위치

- `config/rf2o.yaml`: LiDAR 오도메트리
- `config/gmapping.yaml`: 2D SLAM
- `config/costmap_common.yaml`: SAMPLE footprint, 장애물·inflation
- `config/global_costmap.yaml`: `map` 기준 정적 전역 costmap
- `config/local_costmap.yaml`: `odom` 기준 rolling local costmap
- `config/move_base.yaml`: Navfn, DWA 및 recovery 주기
- `config/dwa_local_planner.yaml`: 비홀로노믹 저속 초기값
- `config/explore_lite.yaml`: baseline 요청값
- `config/maze_brain.yaml`: 상태 머신, blacklist, 점수 가중치

## 알려진 TODO

- `TODO(EXIT_DETECTOR)`: 실제 미로 출구 판정 기준과 센서 근거 구현
- `TODO(EXIT_MODEL)`: 검증된 `exit_probability` feature 생성
- `TODO(ONNX_SCORER)`: 제한 시간·유한값 검사를 포함한 ONNX scorer 구현
- `TODO(MAINTAINER)`: `package.xml`의 maintainer 이름과 이메일 교체
- `MEASURE_REQUIRED`: 실제 footprint와 LiDAR 6-DoF TF
- 실기기에서 gmapping, RF2O, DWA 수치 튜닝
- watchdog ROS 통합 테스트와 rosbag 회귀 테스트 추가

## ROS Noetic EOL 주의

ROS Noetic은 2025년 5월 31일 EOL이 되었고 이후 공식 신규 기능, 보안 업데이트,
버그 수정, 갱신 바이너리를 제공하지 않습니다. 근거는
[ROS 공식 EOL 안내](https://www.ros.org/blog/noetic-eol/)입니다. 재현성을
위해 `results/`의 각 실험 폴더에 다음을 반드시 기록합니다.

- Ubuntu 이미지와 커널 버전
- `apt-cache policy`로 확인한 ROS 패키지 버전
- 이 패키지와 `puppy_control`의 Git 커밋
- [RF2O ROS1 브랜치](https://github.com/MAPIRlab/rf2o_laser_odometry/tree/ros1),
  [m-explore noetic-devel](https://github.com/hrnr/m-explore/tree/noetic-devel),
  [ROS Navigation noetic-devel](https://github.com/ros-planning/navigation/tree/noetic-devel)의
  정확한 커밋
- 모든 YAML과 launch 파일 사본

EOL 시스템을 인터넷에 직접 노출하지 말고, 장기 운영이 필요하면 지원되는
Ubuntu·ROS2로의 이전 계획을 별도로 세워야 합니다.
