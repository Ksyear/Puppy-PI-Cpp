# puppy_vr_control_py

**C++ 패키지 `puppy_vr_control`의 파이썬 버전.**
로봇을 파이썬 스택으로 돌릴 때 VR(Meta Quest)과 UDP 통신해 조종하기 위한 노드 모음이다.
프로토콜, 토픽, 파라미터, 기본값, 안전 정지 동작 모두 C++판과 동일하므로
어느 쪽을 실행해도 Unity 앱은 구분하지 못한다. **단, 둘을 동시에 실행하면
UDP 포트(5005/5006)가 충돌하므로 반드시 하나만 실행할 것.**

| 노드 | 역할 |
|---|---|
| `vr_udp_teleop` | UDP 5005 수신(`"X:..,Z:.."`) → `/puppy_control/velocity/autogait` 발행, 1초 무신호 시 안전 정지 |
| `camera_udp_sender` | `/image_raw/compressed`(JPEG) → UDP 5006 청크 전송 (hello 패킷 자동 발견) |

## 빌드/실행 (로봇에서)

```bash
cd /home/ubuntu/ros2_ws
colcon build --packages-select puppy_vr_control_py
source install/setup.bash

# 실행 (카메라 스트림 포함)
ros2 launch puppy_vr_control_py vr_control.launch.py
# 조종만 / 디버그 출력
ros2 launch puppy_vr_control_py vr_control.launch.py use_camera:=false debug:=true
```

파이썬 패키지라 컴파일이 없어서 수정→재실행이 빠르다.
**프로토콜을 실험/수정할 때는 이 파이썬판으로 먼저 맞춰 보고,
확정되면 C++판에 반영하는 흐름을 권장**한다.

세부 파라미터 설명·Unity 쪽 코드·트러블슈팅은 C++판 문서 참고:
- `src/puppy_vr_control/README.md`
- `공부자료/05_VR_Unity_UDP_통신.md`
