# lidar_mapping_cpp

LiDAR(LD19)로 **2D 점유격자 지도를 만드는 자체 C++ SLAM 노드**.
오도메트리가 없는 PuppyPi에서 바로 동작하도록 스캔매칭만으로 자세(x, y, θ)를
추정한다 — 실내에서 GPS를 대신하는 위치 추정 수단이기도 하다.

## 알고리즘 (tinySLAM 계열, `include/.../mapping_core.hpp`)

1. **점유격자**: 셀마다 로그오즈(log-odds) 값 유지 — 광선 경로는 비움(−), 끝점은 점유(+)
2. **스캔매칭**: 새 스캔이 오면 현재 지도 위에서 언덕오르기 탐색(±x, ±y, ±θ,
   보폭 절반씩 감소)으로 "스캔 끝점들이 벽에 가장 잘 얹히는" 자세를 찾음
3. **통합**: 5cm/0.05rad 이상 움직였을 때만 지도에 반영 (제자리 노이즈 방지)

**검증**: 가상 8×8m 방을 도는 60스텝(3m 이동+108° 회전) 시뮬레이션에서
위치 오차 5cm, 각도 오차 0°, 벽/빈공간/미탐사 분류 정확 — Mac에서
`clang++` 단독 테스트로 확인 완료.

**한계 (정직하게)**: 루프 클로저가 없어 큰 공간을 한 바퀴 돌면 누적 오차가
남는다. 교실 한두 칸 규모까지 적합. 더 큰 지도는 공식 `slam_toolbox`
(`ros2 launch slam slam.launch.py`, 이것도 C++)를 사용할 것.
프레임(map/base_footprint)과 저장 형식이 같아 어느 쪽이든 Nav2에 쓸 수 있다.

## 사용법 (로봇에서)

```bash
colcon build --packages-select lidar_mapping_cpp && source install/setup.bash

# 1) LiDAR 드라이버
ros2 launch peripherals lidar.launch.py
# 2) 지도작성 노드
ros2 launch lidar_mapping_cpp lidar_mapping.launch.py
# 3) (VR 이나 조이스틱으로 로봇을 천천히 몰면서 방을 돌기)
# 4) PC 에서 rviz2 로 확인: Fixed Frame=map, Map 토픽=/map
# 5) 지도 저장 (slam_toolbox/Nav2 와 같은 pgm+yaml 형식)
ros2 service call /lidar_mapping/save_map std_srvs/srv/Trigger
```

저장 위치 기본값: `/home/ubuntu/ros2_ws/src/slam/maps/cpp_map.pgm(.yaml)`
→ 그대로 Nav2 `map_server` 로 불러 자율주행에 사용 가능.

## 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `resolution` | 0.05 | 격자 해상도 (m/셀) |
| `map_size_m` | 20.0 | 지도 한 변 크기 (시작 위치가 중앙) |
| `max_range` | 11.5 | 이 거리 이상 반사는 무시 (LD19 스펙 12m) |
| `min_travel_distance` | 0.05 | 지도 통합 최소 이동 거리 (m) |
| `laser_x/laser_y/laser_yaw` | 0 | LiDAR 장착 오프셋 (base 기준) |
| `map_save_path`, `map_name` | slam/maps, cpp_map | 저장 경로/이름 |
| `publish_tf` | true | map→base_footprint TF 방송 |

## 위치(GPS 대용)를 VR로 보내려면

이 노드가 추정한 자세는 TF(map→base_footprint)로 방송된다.
`/robot_pose` 같은 토픽이 필요하면 TF를 구독해 UDP로 쏘는 노드를
`puppy_vr_control` 의 상태 전송 노드에 합치면 된다 (공부자료 09 참고).

## 실기기 주의사항

- 지도가 어긋나면: 로봇을 **더 천천히** 몰 것 (스캔매칭 탐색 범위는
  `match_lin_step`/`match_ang_step` 로 확대 가능하나 CPU 사용 증가 — Pi 4 기준
  기본값으로 360빔 @10Hz 처리에 충분)
- LiDAR 가 거꾸로 장착된 경우 `laser_yaw: 3.14159` 로 보정
