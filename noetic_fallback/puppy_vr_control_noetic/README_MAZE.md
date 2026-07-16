# ROS1 자율 미로 탐색 MVP

검증된 `lidar_mapping.py`가 이동 중 `/map`, `/lidar_mapping/pose`,
`map -> base_footprint` TF를 만들고, `maze_autonomy.py`가 frontier를 찾아 A* 경로로
이동한다. Qwen은 선택 사항이며 속도가 아닌 frontier ID만 선택한다.

## 안전 원칙

- 기본값은 `autostart:=false`다.
- VR, WonderPi, 수동 조종 노드와 동시에 실행하지 않는다.
- 첫 시험은 로봇을 들어 올리거나 넓은 평지에서 `forward_speed:=3.0`으로 한다.
- 전방 LiDAR가 `stop_distance` 이내를 감지하거나 센서 데이터가 끊기면 정지한다.
- 카메라 노드가 `/maze/exit_detected`에 `true`를 발행하면 즉시 종료한다.

## 실행

```bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:<저장소>/noetic_fallback
chmod +x <저장소>/noetic_fallback/puppy_vr_control_noetic/scripts/*.py

# 규칙 기반 frontier 선택, 정지 상태로 시작
roslaunch puppy_vr_control_noetic maze_autonomy.launch forward_speed:=3.0

# 센서와 경로 확인 후 시작
rosservice call /maze_autonomy/set_running "data: true"

# 즉시 정지
rosservice call /maze_autonomy/set_running "data: false"
```

Qwen2.5/llama.cpp를 이미 8081에서 실행했다면:

```bash
roslaunch puppy_vr_control_noetic maze_autonomy.launch use_llm:=true
```

## 확인 토픽

```text
/map                         실시간 점유격자
/lidar_mapping/pose          스캔매칭 위치
/maze_autonomy/goal          선택된 frontier
/maze_autonomy/path          A* 경로
/maze_autonomy/status        상태와 정지 사유
/puppy_control/velocity      최종 PuppyPi 이동 명령
```

오프라인 코어 테스트:

```bash
PYTHONPATH=noetic_fallback/puppy_vr_control_noetic/src \
python3 -m unittest discover noetic_fallback/puppy_vr_control_noetic/test
```
