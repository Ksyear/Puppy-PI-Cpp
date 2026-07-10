# obstacle_climb_cpp

OpenCV로 전방 장애물(계단/턱)을 감지하고 **PuppyPi 다리로 넘어가는** C++ 노드.
공식 `negotiate_stairs_demo.py`의 이식 + 개선판.

## 동작 흐름

```
[탐색/접근]                     [등반]                    [통과]
카메라 LAB 색인식으로 장애물   정지 후 runActionGroup    3초 전진해 몸 전체를
검출 → 중심 오차에 비례한     'up_stairs_2cm.d6ac'      넘긴 뒤 정지
yaw 보정으로 정면 정렬하며    (다리 궤적: 앞다리 걸치기
전진, 화면 하단 도달 시 정지   → 뒷다리 밀어올리기)
```

"다리를 이용해 넘어가는 계산"의 실체: 다리 관절의 상세 궤적은 로봇에 내장된
**동작 그룹(.d6ac)** — 시간축을 따라 8개 서보 각도를 기록한 시퀀스 — 이
수행하고, 이 노드는 (1) 장애물 인식, (2) **정면 정렬**(비스듬히 오르면 넘어지므로
중요), (3) 트리거 타이밍, (4) 통과 후 마무리를 계산한다.
다른 높이 장애물은 `climb_action` 파라미터로 다른 동작 그룹을 지정
(로봇의 `/home/ubuntu/software/puppypi_control/ActionGroups/` 목록 참고).

## 실행 (로봇에서)

```bash
colcon build --packages-up-to obstacle_climb_cpp && source install/setup.bash

# 사전 조건: puppy_control(자동 실행) + 카메라
ros2 launch peripherals usb_cam.launch.py
ros2 launch obstacle_climb_cpp obstacle_climb.launch.py

# 중단/재시작
ros2 service call /obstacle_climb/set_running std_srvs/srv/SetBool "{data: false}"
# 검출 상태 확인 (PC에서)
ros2 run rqt_image_view rqt_image_view /obstacle_climb/debug_image
```

## 색 캘리브레이션 (중요)

기본값은 빨간 장애물용 대략치다. 로봇에서 보정된 값을 쓸 것:
`/home/ubuntu/software/lab_tool/lab_config.yaml` 의 `red` 항목 min/max 를
launch 의 `lab_min`/`lab_max` 에 복사.

## 안전 장치

- 이동 중 영상 2초 끊김 → 즉시 정지
- 노드 종료 → 정지 명령 발행
- `~/set_running false` → 즉시 정지

## 원본 데모와 다른 점

1. **정면 정렬 추가**: 원본은 정렬 없이 직진 → 비스듬히 올라 실패하기 쉬움
2. 통과 후 정지: 원본은 전진 명령을 낸 채 끝나서 로봇이 계속 걸음
3. 원본이 보내던 무효 Pose 명령(height=0.3, 허용범위 -15~-5 밖이라 조용히
   무시되던 버그) 제거
4. `cv2.imshow` 대신 `~/debug_image` 토픽 (헤드리스 환경 대응)
