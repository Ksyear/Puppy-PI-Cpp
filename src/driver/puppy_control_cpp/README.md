# puppy_control_cpp

`puppy_control`(파이썬, `puppy.py` 851줄)의 C++ 이식판.

## 현재 상태 — 반드시 읽을 것

| 구성요소 | 상태 |
|---|---|
| ROS 래퍼 (토픽/서비스/파라미터/명령 검증/autogait 타이밍 계산/IMU 상보필터/100Hz 발행) | **완전 이식** (`src/puppy_control_node.cpp`) |
| 보행·IK 엔진 (HiwonderPuppy) | **미이식** — `src/stub_engine.cpp` 는 스텁(서보 출력 없음) |

**공개된 이 패키지만으로는 로봇이 움직이지 않는다.** 원본이 사용하는 보행 엔진과
서보 출력 코드는 공식 GitHub가 아니라 사용자가 보유한 Hiwonder system image
내부에 있기 때문이다:

```
/home/ubuntu/software/puppypi_control/
├── puppy_kinematics.py      # HiwonderPuppy — 보행 궤적·역기구학의 실체
├── pwm_servo_control.py     # 서보 펄스 출력
├── servo_controller.py
└── action_group_control.py  # .d6a 동작 그룹 재생
```

### 로컬 엔진 사용 절차

1. 원본 Python 엔진은 로봇의 `/home/ubuntu/software/puppypi_control`에 그대로 두고
   workspace 안으로 복사하지 않는다.
2. 백업은 저장소 밖(예: `~/puppypi_private/robot_software_backup`)에 보관한다.
3. 비공개 C++ 구현은 이 패키지의 `private/src/*.cpp`와
   `private/include/`에 둔다. `PuppyEngine` factory는
   `std::unique_ptr<PuppyEngine> make_puppy_engine()`을 하나만 제공해야 한다.
4. CMake는 `private/src/*.cpp`가 있으면 이를 사용하고, 없으면 공개용
   `stub_engine.cpp`를 사용한다.

`private/`, 원본 Python 엔진, `ActionGroups`, `*.d6a`와 image 백업은 모두
`.gitignore` 대상이다. 라이선스가 확인되지 않은 Hiwonder 구성요소나 그 포팅본을
공개 저장소에 commit하지 않는다.

로컬 C++ 엔진이 완성되기 전에는 순정 이미지의 **파이썬 `puppy_control`을 그대로
사용하면 된다** — 이 패키지는
토픽/서비스 이름이 완전히 동일한 드롭인 교체이므로, 나머지 시스템(VR 노드 등)은
어느 쪽이 돌고 있는지 구분하지 못한다. 단, **둘을 동시에 실행하지 말 것**
(같은 토픽을 두 노드가 처리하게 됨).

## 빌드 / 실행 (로봇에서)

```bash
cd /home/ubuntu/ros2_ws
colcon build --packages-up-to puppy_control_cpp
source install/setup.bash

# 파이썬판이 돌고 있지 않은지 확인 후 (부팅 자동실행 서비스 확인!)
ros2 launch puppy_control_cpp puppy_control.launch.py
```

스텁 상태에서도 다음이 동작하므로 인터페이스 검증에 쓸 수 있다:
- 모든 토픽/서비스가 생성됨 (`ros2 node info /puppy`)
- 명령을 보내면 "엔진 호출 확인: move(...)" 로그로 파이프라인 확인
- `/puppy_control/legs_coord` 에 stance 목표값이 발행됨

## 원본과 의도적으로 다른 점

1. 로봇팔(ArmIK) 초기화 생략 — 팔 없는 구성(with_arm=0) 기준. 원본이 하던
   `setServoPulse(9, 1500, 300)` 호출은 유지.
2. `pub()` 안의 2ms 보정 sleep 생략 (rclpy 타이밍 핵).
3. `gait/pc` 콜백에 배열 길이 검사 추가 (원본은 짧은 배열이 오면 예외 발생).
4. 전역변수(PuppyPose/GaitConfig) → 멤버 변수화. 초기 현재자세는 파라미터의
   Stand 프리셋 (원본은 모듈 상수 — 기본값이 같아 실질 차이 없음).
