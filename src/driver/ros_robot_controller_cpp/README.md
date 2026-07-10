# ros_robot_controller_cpp

`ros_robot_controller`(파이썬)의 C++ 이식판 — STM32 확장보드(서보·IMU·부저·LED·모터)
시리얼 드라이버. 토픽/서비스/메시지 스케일링이 원본과 동일한 **드롭인 교체**다.

## 구성

| 파일 | 내용 |
|---|---|
| `include/.../board_protocol.hpp` | 프레임 생성/파싱/CRC8 — **순수 로직** (ROS 없이 테스트 가능) |
| `include/.../board.hpp` + `src/board.cpp` | Board 클래스: `/dev/rrc` @1Mbps, 수신 스레드, 기능별 큐 |
| `src/ros_robot_controller_node.cpp` | ROS 노드 (50Hz 발행 루프, 서보 서비스 등) |

## 검증 상태

- 시리얼 **프레임 바이트가 파이썬 SDK 와 완전 일치**하는지 대조 테스트로 확인
  (set_buzzer / set_led / set_rgb / motor / pwm·bus servo / CRC8 / 수신 파싱 —
  검증 스크립트는 개발 세션에서 실행, 결과는 저장소 커밋 메시지/공부자료 참조)
- 실기기(보드 연결) 검증은 아직 안 됨 — 아래 절차로 확인할 것

## 빌드 / 실행 (로봇에서)

```bash
cd /home/ubuntu/ros2_ws
colcon build --packages-up-to ros_robot_controller_cpp
source install/setup.bash

# 파이썬판 노드가 돌고 있으면 먼저 중지할 것 (같은 시리얼 포트 사용)
ros2 launch ros_robot_controller_cpp ros_robot_controller.launch.py
```

### 실기기 확인 순서 (안전한 것부터)
```bash
ros2 topic echo /ros_robot_controller/battery --once     # 1) 전압 읽기
ros2 topic pub -1 /ros_robot_controller/set_buzzer \
  ros_robot_controller_msgs/msg/BuzzerState "{freq: 1900, on_time: 0.1, off_time: 0.9, repeat: 1}"  # 2) 부저
ros2 topic hz /ros_robot_controller/imu_raw               # 3) IMU ~50Hz
```

## 원본과 의도적으로 다른 점

1. **원본 버그 수정**: 파이썬 `get_bus_servo_state` 서비스는 존재하지 않는 SDK 메서드
   (`bus_servo_read_voltage`, `bus_servo_read_torque`)를 호출해 get_voltage /
   get_torque_state 요청 시 예외가 났음 → 올바른 메서드(read_vin / read_torque_state)로 연결.
2. 서보 읽기 응답 대기에 **1초 타임아웃** 추가 (원본은 무한 대기 → 보드 무응답 시
   서비스가 영원히 멈춤).
3. 파이썬의 `self.enable_reception` 이름 충돌(메서드를 bool 로 덮어씀)은
   별도 플래그 변수로 정리 — 동작은 동일 (초기값 true).
