#!/usr/bin/env bash
# Teacher-only provisioning inside a NEW Ubuntu 22.04 amd64 desktop VM.
# Default is a plan. --apply installs dependencies/builds; never starts ROS/robot.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT="$(cd -- "$HERE/.." && pwd -P)"
case "${1:---plan}" in
  --plan|--apply) action="${1:---plan}" ;;
  *) printf '用法：bash setup/prepare-student-vm.sh [--plan|--apply]\n' >&2; exit 2 ;;
esac
if [[ "$#" -gt 1 ]]; then
  printf '只允许一个参数；没有执行安装。\n' >&2
  exit 2
fi
if [[ "$action" == --plan ]]; then
  printf '教学虚拟机准备：现在支持从干净 Ubuntu 安装 ROS 和依赖。\n'
  # Keep the default plan usable when this compatibility wrapper is copied alone.
  if [[ -f "$HERE/install-student.sh" ]]; then
    exec bash "$HERE/install-student.sh" --plan
  fi
  printf '使用 setup/install-student.sh --install 执行联网安装。\n'
  exit 0
fi
exec bash "$HERE/install-student.sh" --install
