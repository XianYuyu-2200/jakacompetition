#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${HERE}/../jaka_ros2"

# ROS 的 setup.bash 会引用未定义变量, 必须临时关掉 nounset
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

case "${1:-}" in
  "") colcon build --symlink-install ;;
  --student)
    # Sequential packages and at most two compiler jobs keep VM memory bounded.
    # The workbench uses self-contained simulation launches, not a global MoveIt patch.
    MAKEFLAGS="-j2" CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install \
      --executor sequential --cmake-args -DBUILD_TESTING=OFF ;;
  *) echo "用法：bash setup/build.sh [--student]" >&2; exit 2 ;;
esac

echo "[OK] 编译完成, 使用时先 source install/setup.bash"
