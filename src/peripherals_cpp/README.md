# peripherals_cpp

`peripherals` 파이썬 노드들의 C++ 이식판 (조종/유틸 계열).
카메라(usb_cam)·LiDAR·web_video_server 는 원래부터 C++ 패키지를 launch 로 띄우는
것이라 이식 대상이 아니다 — 기존 `peripherals` 패키지의 launch 를 그대로 쓰면 된다.

| 실행파일 | 원본 | 역할 |
|---|---|---|
| `remote_control_joystick` | remote_control_joystick.py | 무선 게임패드 → `/puppy_control/velocity/autogait`, `/puppy_control/pose` |
| `teleop_key_control` | teleop_key_control.py | 키보드 w/a/s/d → `controller/cmd_vel` |
| `tf_broadcaster_imu` | tf_broadcaster_imu.py | IMU orientation → TF (역회전 = 켤레 쿼터니언으로 단순화) |

원본 중 `joystick_control.py` 는 setup.py 에 entry point 가 없어(실행 불가 상태)
이식하지 않았다. `remote_control_joystick0.py` 는 구버전 사본이라 제외.

## 빌드/실행

```bash
cd /home/ubuntu/ros2_ws
colcon build --packages-up-to peripherals_cpp
source install/setup.bash

ros2 launch peripherals_cpp joystick_control.launch.py   # 게임패드 조종
ros2 run peripherals_cpp teleop_key_control              # 키보드 조종 (터미널에서 직접)
```

## 파이썬판과의 차이 / 실기기 검증 필요 항목

1. **pygame → 리눅스 조이스틱 API**(`/dev/input/js0`, `js_event`) 직접 읽기.
   조이스틱 연결 감시(0.2초 주기 재연결)는 원본과 동일.
2. **(중요) 축/버튼 번호 매핑은 장치 의존** — pygame 과 js API 가 대부분 같은 커널
   순서를 쓰지만, Hiwonder 게임패드에서 확인 필요:
   - 십자키(hat)가 축 4,5 가 아니면 `hat_x_axis`/`hat_y_axis` 파라미터 조정
   - 십자키 위/아래가 반대면 `invert_hat_y` 반전
   - 버튼 번호가 다르면 소스의 `enum Btn` 순서 조정
3. joy_node 불필요 (원본 launch 는 joy_node 도 띄웠지만 아무도 구독하지 않았음).
4. 버튼 판정 로직(xor 변화 감지, 3틱 HOLD, 속도 5~25 클램프, 자세 클램프)은
   원본과 동일하게 이식.
