#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"
REQUIREMENTS="${SCRIPT_DIR}/requirements.txt"
REQUIREMENTS_COPY="${VENV_DIR}/.puppypi_requirements.txt"

cd "${SCRIPT_DIR}"

if [[ -x "${VENV_DIR}/bin/python" ]] && \
   "${VENV_DIR}/bin/python" -c 'import tkinter' >/dev/null 2>&1; then
  NEED_VENV=0
else
  NEED_VENV=1
fi

if [[ "${NEED_VENV}" -eq 1 ]]; then
  PYTHON_BIN="${PUPPYPI_PYTHON:-}"
  if [[ -n "${PYTHON_BIN}" ]]; then
    if ! "${PYTHON_BIN}" -c 'import tkinter, venv' >/dev/null 2>&1; then
      echo "오류: PUPPYPI_PYTHON이 Tkinter 또는 venv를 제공하지 않습니다: ${PYTHON_BIN}" >&2
      exit 3
    fi
  else
    PYTHON_BIN=""
    for CANDIDATE in "$(command -v python3 2>/dev/null || true)" /usr/bin/python3; do
      if [[ -n "${CANDIDATE}" ]] && [[ -x "${CANDIDATE}" ]] && \
         "${CANDIDATE}" -c 'import tkinter, venv' >/dev/null 2>&1; then
        PYTHON_BIN="${CANDIDATE}"
        break
      fi
    done
  fi
  if [[ -z "${PYTHON_BIN}" ]]; then
    echo "오류: Tkinter와 venv를 모두 제공하는 Python 3를 찾지 못했습니다." >&2
    echo "Tk가 포함된 Python 3를 설치하거나 PUPPYPI_PYTHON=/경로/python3 을 지정하십시오." >&2
    exit 3
  fi
  echo "노트북 전용 Python 가상환경을 생성합니다: ${VENV_DIR}"
  "${PYTHON_BIN}" -m venv --clear "${VENV_DIR}"
fi

if [[ ! -f "${REQUIREMENTS_COPY}" ]] || ! cmp -s "${REQUIREMENTS}" "${REQUIREMENTS_COPY}"; then
  echo "노트북 Python 패키지를 설치합니다. 최초 실행에는 시간이 걸릴 수 있습니다."
  "${VENV_DIR}/bin/python" -m pip install --upgrade pip
  "${VENV_DIR}/bin/python" -m pip install -r "${REQUIREMENTS}"
  cp "${REQUIREMENTS}" "${REQUIREMENTS_COPY}"
fi

if ! "${VENV_DIR}/bin/python" -c 'import cv2, PIL, tkinter, yaml' >/dev/null 2>&1; then
  echo "오류: GUI 필수 모듈을 가져올 수 없습니다." >&2
  echo "Tk가 포함된 Python을 PUPPYPI_PYTHON으로 지정한 뒤 다시 실행하십시오." >&2
  exit 3
fi

exec "${VENV_DIR}/bin/python" laptop_client.py "$@"
