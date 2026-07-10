# app_cpp

`app` 파이썬 패키지의 C++ 이식판.

| 실행파일 | 원본 | 역할 |
|---|---|---|
| `lidar` | app/lidar.py | LiDAR 응용 — 모드 1: 장애물 회피, 2: 추적, 3: 경비. `/scan` → `/cmd_vel_nav` |

원본 `app/setup.py` 에는 `line_following`, `object_tracking` 등 다른 entry point 도
선언돼 있지만 **해당 소스 파일이 저장소에 존재하지 않아**(선언만 남은 상태)
이식 대상은 lidar 하나다.

## 실행

```bash
colcon build --packages-up-to app_cpp && source install/setup.bash
ros2 run app_cpp lidar

# 사용 (원본과 동일한 서비스)
ros2 service call /lidar_app/enter std_srvs/srv/Trigger            # 시작
ros2 service call /lidar_app/set_running puppy_control_msgs/srv/SetInt64 "{data: 1}"  # 회피 모드
ros2 service call /lidar_app/exit std_srvs/srv/Trigger             # 종료
```

파이썬판과 달라진 점: heartbeat 타이머를 threading.Timer 대신 rclcpp 타이머로 구현
(5초 무-heartbeat 시 자동 종료 동작은 동일), `/scan` 콜백의 최솟값 탐색은
`np.nanargmin` 과 같은 NaN 무시 방식.
