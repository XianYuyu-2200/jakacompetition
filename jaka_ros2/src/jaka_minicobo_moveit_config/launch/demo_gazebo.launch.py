"""Gazebo Sim launch for JAKA Mini Cobo.

The robot's gz_ros2_control plugin owns controller_manager inside Gazebo.  The
stock moveit_configs_utils helper also starts a second, external
ros2_control_node; that node cannot load the Gazebo-only hardware plugin and
crashes before the robot is spawned.  Keep the useful parts of that helper,
but spawn controllers after Gazebo has created the robot instead.
"""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_rsp_launch,
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description():
    package_path = Path(get_package_share_directory("jaka_minicobo_moveit_config"))
    kit_path = Path(get_package_share_directory("jaka_competition_kit"))

    use_gazebo = LaunchConfiguration("use_gazebo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    config = (
        MoveItConfigsBuilder(
            "jaka_minicobo", package_name="jaka_minicobo_moveit_config"
        )
        .robot_description(mappings={"use_gazebo": use_gazebo})
        .to_moveit_configs()
    )

    spawn_controllers = TimerAction(
        period=LaunchConfiguration("controller_spawn_delay"),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "jaka_minicobo_controller",
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("entity", default_value="jaka_minicobo"),
        DeclareLaunchArgument("use_gazebo", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("controller_spawn_delay", default_value="10.0"),
        DeclareLaunchArgument(
            "gui_config",
            default_value=str(kit_path / "config" / "gazebo_view_only.config"),
            description="Gazebo GUI 配置；工作台默认只显示 3D 视口",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
            condition=IfCondition(use_sim_time),
        ),
        generate_static_virtual_joint_tfs_launch(config),
        generate_rsp_launch(config),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ),
            launch_arguments={
                "gz_args": ["empty.sdf -r --gui-config ", LaunchConfiguration("gui_config")],
            }.items(),
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    name="spawn_robot",
                    output="screen",
                    arguments=[
                        "-topic",
                        "/robot_description",
                        "-z",
                        "0.06",
                    ],
                )
            ],
        ),
        generate_move_group_launch(config),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_path / "launch" / "moveit_rviz.launch.py")
            ),
            condition=IfCondition(use_rviz),
        ),
        # Do not start ros2_control_node here: gz_ros2_control creates the
        # controller manager when the robot is inserted into Gazebo.
        spawn_controllers,
    ])
