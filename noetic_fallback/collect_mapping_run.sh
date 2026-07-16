#!/usr/bin/env bash
# PuppyPi ROS1 지도작성 검증 데이터를 한 번에 수집하고 번들로 만든다.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ORIGINAL_ARGS=("$@")

DURATION="120"
STATIONARY="20"
SNAPSHOT_PERIOD="5"
LABEL="mapping"
SCAN_TOPIC="/scan"
MAP_SIZE="12.0"
RESOLUTION="0.05"
LASER_YAW="0.0"
MIN_RANGE="0.15"
MAX_RANGE="8.0"
BASE_FRAME="base_footprint"
EXPECT_RETURN="false"
OUTPUT_ROOT="${PUPPY_MAPPING_RUNS:-$HOME/puppy_mapping_runs}"

usage() {
  cat <<'USAGE'

사용:
  ./collect_mapping_run.sh [옵션]

옵션:
  --duration SEC          전체 수집 시간 (기본 120)
  --stationary SEC        시작 후 정지 유지 시간 (기본 20)
  --snapshot-period SEC   지도 스냅샷 간격 (기본 5)
  --label NAME            실행 식별자 (기본 mapping)
  --scan-topic TOPIC      LaserScan 토픽 (기본 /scan)
  --map-size M            지도 한 변 크기 (기본 12.0)
  --resolution M          지도 해상도 (기본 0.05)
  --laser-yaw RAD         LiDAR 장착 yaw 보정 (기본 0.0)
  --min-range M           사용할 최소 거리 (기본 0.15)
  --max-range M           사용할 최대 거리 (기본 8.0)
  --base-frame FRAME      로봇 기준 프레임 (기본 base_footprint)
  --expect-return         시작 위치 근처 복귀 오차도 검사
  --output-root DIR       로봇 저장 위치 (기본 ~/puppy_mapping_runs)
  -h, --help              도움말

시험 절차:
  처음 stationary초 동안 로봇을 움직이지 않는다.
  이후 천천히 직선 이동, 90도 회전, 복도 이동을 수행한다.
  이 스크립트는 자율주행이나 수동 조종 노드를 시작하지 않는다.
USAGE
}

fail() {
  printf '[실패] %s\n' "$*" >&2
  exit 2
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --duration) DURATION="${2:?값 필요}"; shift 2 ;;
    --stationary) STATIONARY="${2:?값 필요}"; shift 2 ;;
    --snapshot-period) SNAPSHOT_PERIOD="${2:?값 필요}"; shift 2 ;;
    --label) LABEL="${2:?값 필요}"; shift 2 ;;
    --scan-topic) SCAN_TOPIC="${2:?값 필요}"; shift 2 ;;
    --map-size) MAP_SIZE="${2:?값 필요}"; shift 2 ;;
    --resolution) RESOLUTION="${2:?값 필요}"; shift 2 ;;
    --laser-yaw) LASER_YAW="${2:?값 필요}"; shift 2 ;;
    --min-range) MIN_RANGE="${2:?값 필요}"; shift 2 ;;
    --max-range) MAX_RANGE="${2:?값 필요}"; shift 2 ;;
    --base-frame) BASE_FRAME="${2:?값 필요}"; shift 2 ;;
    --expect-return) EXPECT_RETURN="true"; shift ;;
    --output-root) OUTPUT_ROOT="${2:?값 필요}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) fail "알 수 없는 옵션: $1" ;;
  esac
done

for value in "$DURATION" "$STATIONARY" "$SNAPSHOT_PERIOD" "$MAP_SIZE" \
             "$RESOLUTION" "$MIN_RANGE" "$MAX_RANGE"; do
  [[ "$value" =~ ^[0-9]+([.][0-9]+)?$ ]] || fail "숫자 형식이 아님: $value"
done
[[ "$LASER_YAW" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || fail "laser-yaw 숫자 형식 오류"

if [ -z "${ROS_DISTRO:-}" ]; then
  [ -f /opt/ros/noetic/setup.bash ] || fail "/opt/ros/noetic/setup.bash 없음"
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  for workspace_setup in "$HOME"/*/devel/setup.bash; do
    if [ -f "$workspace_setup" ]; then
      # shellcheck disable=SC1090
      source "$workspace_setup"
      break
    fi
  done
fi
[ "${ROS_DISTRO:-}" = "noetic" ] || fail "ROS1 Noetic 필요 (현재 ${ROS_DISTRO:-없음})"

if [ -n "${ROS_PACKAGE_PATH:-}" ]; then
  export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$SCRIPT_DIR"
else
  export ROS_PACKAGE_PATH="$SCRIPT_DIR"
fi
chmod +x "$SCRIPT_DIR/puppy_vr_control_noetic/scripts/"*.py

rostopic list >/dev/null 2>&1 || fail "ROS master에 연결할 수 없음"
SCAN_TYPE="$(rostopic type "$SCAN_TOPIC" 2>/dev/null || true)"
[ "$SCAN_TYPE" = "sensor_msgs/LaserScan" ] || \
  fail "$SCAN_TOPIC 타입이 sensor_msgs/LaserScan이 아님: ${SCAN_TYPE:-없음}"

if rosnode list 2>/dev/null | grep -Fxq '/lidar_mapping'; then
  fail "/lidar_mapping 노드가 이미 실행 중임. 중복 실행을 종료한 뒤 다시 시도"
fi

USE_SIM_TIME="$(rosparam get /use_sim_time 2>/dev/null || printf 'false')"
if [ "$USE_SIM_TIME" = "true" ] && ! rostopic list | grep -Fxq '/clock'; then
  fail "/use_sim_time=true지만 /clock이 없어 지도 발행 Timer가 멈춤"
fi

SAFE_LABEL="$(printf '%s' "$LABEL" | tr -c 'A-Za-z0-9._-' '_')"
[ -n "$SAFE_LABEL" ] || SAFE_LABEL="mapping"
RUN_ID="$(date +%Y%m%d_%H%M%S)_${SAFE_LABEL}"
OUTPUT_ROOT="$(python3 -c 'import os,sys; print(os.path.abspath(os.path.expanduser(sys.argv[1])))' "$OUTPUT_ROOT")"
RUN_DIR="$OUTPUT_ROOT/$RUN_ID"
BUNDLE="$OUTPUT_ROOT/$RUN_ID.tar.gz"
mkdir -p "$RUN_DIR"

AVAILABLE_KB="$(df -Pk "$OUTPUT_ROOT" | awk 'NR==2 {print $4}')"
if [ "${AVAILABLE_KB:-0}" -lt 524288 ]; then
  fail "저장공간 512MiB 미만: ${AVAILABLE_KB:-0}KiB"
fi

{
  printf 'run_id=%s\n' "$RUN_ID"
  printf 'started_at=%s\n' "$(date --iso-8601=seconds 2>/dev/null || date)"
  printf 'hostname=%s\n' "$(hostname)"
  printf 'ros_distro=%s\n' "${ROS_DISTRO:-}"
  printf 'scan_topic=%s\n' "$SCAN_TOPIC"
  printf 'duration_sec=%s\n' "$DURATION"
  printf 'stationary_sec=%s\n' "$STATIONARY"
  printf 'expect_return=%s\n' "$EXPECT_RETURN"
  printf 'map_size_m=%s\n' "$MAP_SIZE"
  printf 'resolution_m=%s\n' "$RESOLUTION"
  printf 'laser_yaw_rad=%s\n' "$LASER_YAW"
} > "$RUN_DIR/run.conf"

{
  uname -a
  python3 --version
  free -h 2>&1 || true
  df -h 2>&1 || true
  if command -v vcgencmd >/dev/null 2>&1; then
    vcgencmd measure_temp 2>&1 || true
  fi
} > "$RUN_DIR/system-before.txt"

{
  printf 'command:'
  printf ' %q' "$0" "${ORIGINAL_ARGS[@]}"
  printf '\n'
} > "$RUN_DIR/command.txt"

git -C "$REPO_ROOT" rev-parse HEAD > "$RUN_DIR/git-commit.txt" 2>/dev/null || true
git -C "$REPO_ROOT" status --short --branch > "$RUN_DIR/git-status.txt" 2>/dev/null || true
rostopic list -v > "$RUN_DIR/ros-topics-before.txt" 2>&1 || true
rostopic info "$SCAN_TOPIC" > "$RUN_DIR/scan-topic-info.txt" 2>&1 || true
rosnode list > "$RUN_DIR/ros-nodes-before.txt" 2>&1 || true
rosparam dump "$RUN_DIR/rosparams.yaml" >/dev/null 2>&1 || true

printf '[수집 시작] %s\n' "$RUN_ID"
printf '첫 %s초는 로봇을 움직이지 마세요. 이후 천천히 수동 조종하세요.\n' "$STATIONARY"

set +e
roslaunch puppy_vr_control_noetic mapping_validation.launch \
  output_dir:="$RUN_DIR" \
  duration_sec:="$DURATION" \
  stationary_sec:="$STATIONARY" \
  snapshot_period_sec:="$SNAPSHOT_PERIOD" \
  expect_return:="$EXPECT_RETURN" \
  scan_topic:="$SCAN_TOPIC" \
  map_size_m:="$MAP_SIZE" \
  resolution:="$RESOLUTION" \
  laser_yaw:="$LASER_YAW" \
  min_range:="$MIN_RANGE" \
  max_range:="$MAX_RANGE" \
  base_frame:="$BASE_FRAME" 2>&1 | tee "$RUN_DIR/roslaunch.log"
LAUNCH_STATUS="${PIPESTATUS[0]}"
set -e

{
  printf 'finished_at=%s\n' "$(date --iso-8601=seconds 2>/dev/null || date)"
  printf 'roslaunch_exit_code=%s\n' "$LAUNCH_STATUS"
} > "$RUN_DIR/collection-status.txt"

{
  free -h 2>&1 || true
  df -h 2>&1 || true
  if command -v vcgencmd >/dev/null 2>&1; then
    vcgencmd measure_temp 2>&1 || true
  fi
} > "$RUN_DIR/system-after.txt"

BAG_OK="false"
if [ -s "$RUN_DIR/run.bag" ] && ! compgen -G "$RUN_DIR/*.bag.active" >/dev/null; then
  rosbag info "$RUN_DIR/run.bag" > "$RUN_DIR/rosbag-info.txt" 2>&1 && BAG_OK="true"
  rosbag info --yaml "$RUN_DIR/run.bag" > "$RUN_DIR/rosbag-info.yaml" 2>&1 || true
fi

if [ -s "$RUN_DIR/summary.json" ] && [ "$BAG_OK" = "true" ]; then
  printf 'completed_at=%s\n' "$(date --iso-8601=seconds 2>/dev/null || date)" \
    > "$RUN_DIR/COLLECTION_COMPLETE"
fi

python3 - "$RUN_DIR" "$RUN_ID" "$LAUNCH_STATUS" "$BAG_OK" <<'PY'
import json
import os
import sys

root, run_id, launch_status, bag_ok = sys.argv[1:]
configuration = {}
with open(os.path.join(root, 'run.conf')) as stream:
    for line in stream:
        key, separator, value = line.rstrip('\n').partition('=')
        if separator:
            configuration[key] = value
summary = None
summary_path = os.path.join(root, 'summary.json')
if os.path.isfile(summary_path):
    with open(summary_path) as stream:
        summary = json.load(stream)
commit_path = os.path.join(root, 'git-commit.txt')
commit = open(commit_path).read().strip() if os.path.isfile(commit_path) else ''
status_path = os.path.join(root, 'git-status.txt')
git_status = open(status_path).read().splitlines() if os.path.isfile(status_path) else []
manifest = {
    'schema_version': 1,
    'run_id': run_id,
    'configuration': configuration,
    'git_commit': commit,
    'git_status': git_status,
    'roslaunch_exit_code': int(launch_status),
    'rosbag_complete': bag_ok == 'true',
    'recorder_verdict': summary.get('verdict') if summary else None,
    'collection_complete': os.path.isfile(os.path.join(root, 'COLLECTION_COMPLETE')),
}
with open(os.path.join(root, 'manifest.json'), 'w') as stream:
    json.dump(manifest, stream, ensure_ascii=False, indent=2, sort_keys=True)
    stream.write('\n')
PY

python3 - "$RUN_DIR" <<'PY'
import hashlib
import os
import sys

root = os.path.abspath(sys.argv[1])
rows = []
for directory, _, names in os.walk(root):
    for name in sorted(names):
        if name == 'checksums.sha256':
            continue
        path = os.path.join(directory, name)
        digest = hashlib.sha256()
        with open(path, 'rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
        rows.append((os.path.relpath(path, root), digest.hexdigest()))
with open(os.path.join(root, 'checksums.sha256'), 'w') as stream:
    for relative, value in sorted(rows):
        stream.write('%s  %s\n' % (value, relative))
PY

tar -C "$OUTPUT_ROOT" -czf "$BUNDLE" "$RUN_ID"
if command -v sha256sum >/dev/null 2>&1; then
  (cd "$OUTPUT_ROOT" && sha256sum "$RUN_ID.tar.gz" > "$RUN_ID.tar.gz.sha256")
else
  (cd "$OUTPUT_ROOT" && shasum -a 256 "$RUN_ID.tar.gz" > "$RUN_ID.tar.gz.sha256")
fi

printf '[번들 완료] %s\n' "$BUNDLE"
printf 'BUNDLE=%s\n' "$BUNDLE"

if [ ! -f "$RUN_DIR/COLLECTION_COMPLETE" ]; then
  printf '[경고] summary 또는 정상 종료된 rosbag이 없음. 번들은 원인 분석용으로 보존됨.\n' >&2
  exit 3
fi
exit "$LAUNCH_STATUS"
