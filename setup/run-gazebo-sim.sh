#!/usr/bin/env bash
# Gazebo (Ignition Fortress) 仿真模式: Gazebo 里跑物理, RViz 里 MoveIt 规划与执行
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${HERE}/../jaka_ros2/install/setup.bash"
set -u

exec ros2 launch jaka_minicobo_moveit_config demo_gazebo.launch.py
