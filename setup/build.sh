#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${HERE}/../jaka_ros2"

# ROS 的 setup.bash 会引用未定义变量, 必须临时关掉 nounset
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

colcon build --symlink-install

echo "[OK] 编译完成, 使用时先 source install/setup.bash"
