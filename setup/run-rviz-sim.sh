#!/usr/bin/env bash
# 纯 RViz 仿真模式(不依赖真机, 不依赖 Gazebo, 使用 mock_components 假硬件)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${HERE}/../jaka_ros2/install/setup.bash"
set -u

exec ros2 launch jaka_minicobo_moveit_config demo.launch.py use_rviz_sim:=true
