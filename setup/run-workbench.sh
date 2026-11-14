#!/usr/bin/env bash
# Run from the checkout, independently of the caller's working directory.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
PYTHON="${WORKBENCH_PYTHON:-/usr/bin/python3}"
if ! command -v -- "$PYTHON" >/dev/null 2>&1; then
  printf '找不到 Python：%s。请安装 Python 3，或设置 WORKBENCH_PYTHON 为解释器路径。\n' "$PYTHON" >&2
  exit 127
fi

# --check must remain usable without Qt or a graphical session.
CHECK_ONLY=false
for arg in "$@"; do
  case "$arg" in --check|--vm-check|--help|-h) CHECK_ONLY=true ;; esac
done
if [[ "$CHECK_ONLY" != true ]] && ! "$PYTHON" -c 'from PyQt5 import QtCore, QtWidgets' >/dev/null 2>&1; then
  printf '%s\n' \
    "工作台需要 PyQt5，当前解释器为：${PYTHON}" \
    'Ubuntu 系统 Python 可由管理员手动安装：sudo apt install python3-pyqt5' \
    '本脚本不会安装依赖。可先运行：bash setup/run-workbench.sh --check' >&2
  exit 1
fi

cd -- "$ROOT"
exec "$PYTHON" -m workbench "$@"
