# ROS1 지도작성 검증

로봇에서 지도작성 실행 전체를 기록하고, Mac에서 ROS1 없이 분석하는 절차다.
사용자가 동작을 글로 설명하는 대신 rosbag, 지도, 위치, 로그와 시스템 상태를
하나의 검증 번들로 남긴다.

## 수집되는 자료

```text
run.bag                 /scan, /map, pose, TF, 제어 명령 원본
final_map.pgm/.yaml     마지막 OccupancyGrid
maps/*.pgm              5초 간격 지도 변화
pose.csv                시간별 스캔매칭 위치
scan_metrics.csv        LiDAR 유효 빔과 거리
map_metrics.csv         known/free/occupied 셀 변화
summary.json            PASS/WARN/FAIL 자동 판정
manifest.json           Git 커밋과 수집 완료 여부
rosparams.yaml          ROS 실행 파라미터
roslaunch.log           실행 오류와 경고
system-*.txt            CPU, 메모리, 온도, 저장공간
checksums.sha256        번들 내부 무결성
```

PGM 파일 존재만으로 성공을 판단하지 않는다. 유효 스캔이 없어도 미탐사 지도 파일은
만들어질 수 있으므로 `summary.json`의 메시지 수와 셀 통계를 함께 확인한다.

## 1. 로봇 준비

수동 조종 노드와 LiDAR 드라이버는 먼저 실행하되, 지도 노드는 중복 실행하지 않는다.
`run_vr.sh`를 이용한다면 수집기보다 먼저 실행하고 `use_mapping:=false`로 둔다.

```bash
cd <저장소>/noetic_fallback

# 필요할 때만: 수동 VR 조종을 먼저 실행
./run_vr.sh use_mapping:=false use_camera:=false max_speed_x:=6
```

다른 터미널에서 LaserScan 토픽을 확인한다.

```bash
rostopic type /scan
# sensor_msgs/LaserScan 이어야 함
```

## 2. 첫 2분 수집

```bash
cd <저장소>/noetic_fallback
./collect_mapping_run.sh --duration 120 --stationary 20 --label first_map
```

시험 순서:

1. 처음 20초는 로봇을 움직이지 않는다.
2. 천천히 직선으로 이동한다.
3. 90도 회전 후 복도를 이동한다.
4. 가능하면 시작 위치 근처로 돌아온다.

시작점 복귀 오차도 검사하려면 다음 옵션을 사용한다.

```bash
./collect_mapping_run.sh --duration 120 --stationary 20 \
  --expect-return --label return_test
```

LiDAR 토픽이나 장착 방향이 다르면 명시한다.

```bash
./collect_mapping_run.sh \
  --scan-topic /your_scan_topic \
  --laser-yaw 3.14159 \
  --label reversed_lidar
```

완료되면 로봇에 다음 파일이 생긴다.

```text
~/puppy_mapping_runs/<실행ID>/
~/puppy_mapping_runs/<실행ID>.tar.gz
~/puppy_mapping_runs/<실행ID>.tar.gz.sha256
```

수집이 완전하지 않아도 원인 분석용 번들을 보존한다. 전송과 체크섬 검증이 끝나기
전에는 로봇의 원본 파일을 삭제하지 않는다.

## 3. Mac으로 가져오기

로봇 AP 모드의 기본 주소를 사용하는 예:

```bash
cd <저장소>
./tools/fetch_mapping_run.sh pi@192.168.149.1 latest
```

SSH config에 `puppypi` 별칭을 등록했다면:

```bash
./tools/fetch_mapping_run.sh puppypi latest
```

이 명령은 다음 작업을 자동 수행한다.

```text
최신 tar.gz와 SHA-256 가져오기
→ 번들 체크섬 검증
→ 내부 파일 체크섬 검증
→ robot_runs/inbox에 안전하게 압축 해제
→ 오프라인 분석과 보고서 생성
```

## 4. 결과 확인

```text
robot_runs/inbox/<실행ID>/analysis/report.md
robot_runs/inbox/<실행ID>/analysis/final_map.png
robot_runs/inbox/<실행ID>/analysis/trajectory.svg
robot_runs/inbox/<실행ID>/analysis/timeline.html
```

판정 의미:

```text
PASS    지도작성 내부 지표가 초기 기준을 통과
WARN    지도는 생성됐지만 드리프트·누락·프레임 등을 확인해야 함
FAIL    scan/map/pose 또는 유효 지도 내용이 없음
```

자동 분석은 지도 내부의 일관성을 판정한다. 실제 벽과의 절대 위치 정확도는 같은 시간의
영상, 실측된 시험 코스 또는 별도 위치 센서와 비교해야 확정할 수 있다.

## 5. Codex와 반복 수정

번들을 가져온 뒤 사용자가 동작을 글로 재작성할 필요는 없다. Codex가 다음 파일을 읽어
실패 구간과 수정 대상을 결정한다.

```text
summary.json
analysis/report.md
trajectory.svg
timeline.html
roslaunch.log
run.bag
```

수정 후에는 새 Git 커밋으로 기록하고 같은 절차로 다음 실행 번들을 생성한다. 각 번들의
`manifest.json`에는 사용한 Git 커밋이 남으므로 코드와 실험 결과를 정확히 연결할 수 있다.
