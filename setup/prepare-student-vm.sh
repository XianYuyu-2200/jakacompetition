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
cat <<'EOF'
教学虚拟机准备计划（不是 OVA 导出或虚拟机验收）：
前置：新建 Ubuntu 22.04 amd64 桌面虚拟机，安装 ROS 2 Humble 并配置其官方软件源。
1. 安装 ROS/Gazebo、Qt、二维码和图形诊断依赖（需要 sudo）。
2. 在本机源码路径编译；不复制开发者 build/install 目录。
3. 为当前学生账户安装桌面快捷方式。
4. 只读虚拟机预检，输出系统/OpenGL/版本报告；不启动仿真。
不连接真机，不删除用户配置，不配置 SSH/密码，不自动降低碰撞检查。
只有显式 --apply 才安装和编译。请在干净的教学虚拟机内使用。
EOF
[[ "$action" == --apply ]] || exit 0
if [[ "$EUID" -eq 0 ]]; then
  printf '请以教学普通用户运行；安装依赖时再由 sudo 提权。\n' >&2
  exit 2
fi
# This file is supplied by the guest operating system, not downloaded input.
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 22.04 || "$(uname -m)" != x86_64 ]]; then
  printf '拒绝安装：本交付候选只支持 Ubuntu 22.04 amd64。\n' >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf '请先按 ROS 官方说明安装 Humble；本脚本不会添加软件源或下载执行远程脚本。\n' >&2
  exit 2
fi
cd -- "$ROOT"
bash "$HERE/install-deps.sh"
ROS_DISTRO=humble bash "$HERE/build.sh"
bash "$HERE/install-workbench-desktop.sh"
printf '\n准备步骤完成；现在输出只读预检。预检通过也不等于仿真验收通过。\n'
bash "$HERE/run-workbench.sh" --vm-check
