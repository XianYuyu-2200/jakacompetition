#!/usr/bin/env bash
# Gazebo (Ignition Fortress) 仿真模式: Gazebo 里跑物理, RViz 里 MoveIt 规划与执行
#
# 注意: 这个脚本出来的 Gazebo 是**空世界**, 看不到赛场 ——
# 赛场几何只写在 MoveIt 规划场景里。想要 Gazebo 里有场地, 用:
#   ros2 launch jaka_competition_kit round.launch.py world:=gazebo
# 或者单独补一个: ros2 launch jaka_competition_kit gazebo_mirror.launch.py
# 说明见 docs/Gazebo仿真.md
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${HERE}/../jaka_ros2/install/setup.bash"
set -u

exec ros2 launch jaka_minicobo_moveit_config demo_gazebo.launch.py
