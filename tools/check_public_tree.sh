#!/usr/bin/env bash
# 공개 저장소에 Hiwonder system-image 전용 파일이 추적되지 않는지 검사한다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if ! git rev-parse --is-inside-work-tree > /dev/null 2>&1; then
  echo "[건너뜀] Git checkout이 아니므로 공개 tree 검사를 실행하지 않습니다."
  exit 0
fi

FORBIDDEN_PATTERN='(^|/)(proprietary|puppypi_control|robot_software_backup|hiwonder_software_backup|ActionGroups)(/|$)|(^|/)vendor/hiwonder(/|$)|(^|/)src/driver/puppy_control_cpp/private(/|$)|(^|/)(puppy_kinematics\.py|pwm_servo_control\.py|servo_controller\.py|action_group_control\.py)$|\.d6a$'

tracked="$(git ls-files | grep -E "$FORBIDDEN_PATTERN" || true)"
if [ -n "$tracked" ]; then
  echo "[오류] 공개 금지 파일이 Git에 추적되고 있습니다:"
  echo "$tracked"
  echo "원본과 로컬 C++ 엔진을 저장소 밖 또는 .gitignore 대상 경로로 옮기세요."
  exit 1
fi

echo "[통과] 공개 Git tree에 Hiwonder 로컬 전용 엔진/ActionGroups가 없습니다."
