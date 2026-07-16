#!/usr/bin/env bash
# PuppyPi에서 지도작성 번들을 가져와 체크섬 검증과 오프라인 분석을 실행한다.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCAL_ROOT="$REPO_ROOT/robot_runs/inbox"
REQUEST="latest"
ANALYZE="true"

usage() {
  cat <<'USAGE'
사용:
  ./tools/fetch_mapping_run.sh SSH_TARGET [latest|BUNDLE_NAME] [옵션]

예:
  ./tools/fetch_mapping_run.sh pi@192.168.149.1
  ./tools/fetch_mapping_run.sh puppypi latest
  ./tools/fetch_mapping_run.sh puppypi 20260716_120000_mapping.tar.gz

옵션:
  --local-root DIR   Mac 저장 위치 (기본 <repo>/robot_runs/inbox)
  --no-analyze       가져오기만 하고 자동 분석하지 않음
  -h, --help         도움말

로봇의 기본 원격 위치는 ~/puppy_mapping_runs다.
원격 환경변수 PUPPY_MAPPING_RUNS로 다른 위치를 지정할 수 있다.
USAGE
}

fail() {
  printf '[실패] %s\n' "$*" >&2
  exit 2
}

if [ "$#" -eq 0 ]; then
  usage
  exit 2
fi
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
  usage
  exit 0
fi

TARGET="$1"
shift
if [ "$#" -gt 0 ] && [[ "$1" != --* ]]; then
  REQUEST="$1"
  shift
fi
while [ "$#" -gt 0 ]; do
  case "$1" in
    --local-root) LOCAL_ROOT="${2:?값 필요}"; shift 2 ;;
    --no-analyze) ANALYZE="false"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "알 수 없는 옵션: $1" ;;
  esac
done

for command in ssh rsync python3; do
  command -v "$command" >/dev/null 2>&1 || fail "필수 명령 없음: $command"
done

if [ "$REQUEST" = "latest" ]; then
  REMOTE_FILE="$(ssh -o ConnectTimeout=10 "$TARGET" \
    'root="${PUPPY_MAPPING_RUNS:-$HOME/puppy_mapping_runs}";
     latest=$(ls -1t "$root"/*.tar.gz 2>/dev/null | head -n 1);
     [ -n "$latest" ] || exit 4;
     printf "%s\n" "$latest"')" || fail "원격 지도 번들을 찾을 수 없음"
else
  [[ "$REQUEST" =~ ^[A-Za-z0-9._-]+$ ]] || fail "안전하지 않은 번들 이름: $REQUEST"
  case "$REQUEST" in
    *.tar.gz) ;;
    *) REQUEST="$REQUEST.tar.gz" ;;
  esac
  REMOTE_FILE="$(ssh -o ConnectTimeout=10 "$TARGET" \
    "root=\"\${PUPPY_MAPPING_RUNS:-\$HOME/puppy_mapping_runs}\";
     file=\"\$root/$REQUEST\";
     [ -f \"\$file\" ] || exit 4;
     printf '%s\\n' \"\$file\"")" || fail "원격 번들을 찾을 수 없음: $REQUEST"
fi

REMOTE_FILE="${REMOTE_FILE//$'\r'/}"
[ -n "$REMOTE_FILE" ] || fail "원격 번들 경로가 비어 있음"
LOCAL_ROOT="$(python3 -c 'import os,sys; print(os.path.abspath(os.path.expanduser(sys.argv[1])))' "$LOCAL_ROOT")"
mkdir -p "$LOCAL_ROOT"

printf '[가져오기] %s:%s\n' "$TARGET" "$REMOTE_FILE"
rsync -avP -e "ssh -o ConnectTimeout=10" "$TARGET:$REMOTE_FILE" "$LOCAL_ROOT/"
rsync -avP -e "ssh -o ConnectTimeout=10" "$TARGET:$REMOTE_FILE.sha256" "$LOCAL_ROOT/"

LOCAL_BUNDLE="$LOCAL_ROOT/$(basename "$REMOTE_FILE")"
[ -s "$LOCAL_BUNDLE" ] || fail "가져온 번들이 비어 있음"
[ -s "$LOCAL_BUNDLE.sha256" ] || fail "체크섬 파일이 없음"

printf '[전송 완료] %s\n' "$LOCAL_BUNDLE"
if [ "$ANALYZE" = "true" ]; then
  python3 "$SCRIPT_DIR/analyze_mapping_run.py" \
    "$LOCAL_BUNDLE" --extract-root "$LOCAL_ROOT"
fi

printf 'LOCAL_BUNDLE=%s\n' "$LOCAL_BUNDLE"
