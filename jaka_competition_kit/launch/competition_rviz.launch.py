"""单独起一个能看到**赛场几何**的 RViz。

  ros2 launch jaka_competition_kit competition_rviz.launch.py

为什么要单独一个:`demo.launch.py` 用的是各型号自带的 `moveit.rviz`,
那份配置里 **没有** PlanningScene 显示, 所以工作台/工位板/料盒/工件
在 RViz 里根本看不见 —— 裁判看不到场上状态。

这个 launch 用工具包自带的 `config/competition.rviz`(基于同一份配置,
把相机拉近到台面), 可以离线跑, 也可以和 demo.launch.py 同时跑。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory("jaka_competition_kit"),
        "config", "competition.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("rviz_config", default_value=default_cfg),
        Node(
            package="rviz2",
            executable="rviz2",
            name="competition_rviz",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ])
