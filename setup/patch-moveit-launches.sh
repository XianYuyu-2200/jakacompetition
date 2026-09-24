#!/usr/bin/env bash
# 关键步骤: MoveIt 自带的 moveit_configs_utils/launches.py 不认识
# use_rviz_sim / use_gazebo 这两个参数, 必须用 jaka_ros2/launches.py 覆盖它,
# 否则 demo.launch.py 的仿真模式不会生效(参数会被静默忽略, 仍然按真机逻辑启动)。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CUSTOM="${HERE}/../jaka_ros2/launches.py"

if [[ ! -f "${CUSTOM}" ]]; then
  echo "[ERROR] 找不到 ${CUSTOM}" >&2
  exit 1
fi

# ROS 的 setup.bash 会引用未定义变量, 必须临时关掉 nounset
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

PREFIX="$(ros2 pkg prefix moveit_configs_utils)"
TARGET="$(find "${PREFIX}" -path '*moveit_configs_utils/launches.py' -not -name '*.orig' | head -n1)"

if [[ -z "${TARGET}" ]]; then
  echo "[ERROR] 找不到 moveit_configs_utils/launches.py" >&2
  exit 1
fi

if [[ ! -f "${TARGET}.orig" ]]; then
  sudo cp "${TARGET}" "${TARGET}.orig"
  echo "[INFO] 已备份原文件 -> ${TARGET}.orig"
fi

sudo cp "${CUSTOM}" "${TARGET}"
# 清掉字节码缓存, 否则可能加载旧的 launches 模块
sudo rm -rf "${TARGET%/*}/__pycache__" 2>/dev/null || true

echo "[OK] 已替换 ${TARGET}"
echo "[NOTE] 以后 apt 升级 moveit 会覆盖该文件, 需要重新执行本脚本"
