"""工作台的基础学习环境：单套假硬件 + 夹爪 + 赛场 RViz + 判分侧。

只准备环境，不自动出题、发令或执行抓取。不修改既有运动算法。
显式构建假硬件配置，不依赖开发机对 generate_demo_launch 的补丁。
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    kit = get_package_share_directory("jaka_competition_kit")
    return LaunchDescription([
        DeclareLaunchArgument("seed", default_value="246135"),
        DeclareLaunchArgument("output_dir", default_value="~/.local/share/jaka-workbench/results"),
        DeclareLaunchArgument("backend", default_value="gazebo",
                              description="gazebo=基础组 Gazebo；mock=轻量假硬件"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="可选启动 RViz；Gazebo 默认关闭以节省资源"),
        SetEnvironmentVariable("JAKA_GRIPPER", "1"),
        OpaqueFunction(function=_start_backend),
    ])


def _start_backend(context):
    backend = LaunchConfiguration("backend").perform(context).strip().lower()
    if backend == "gazebo":
        # Gazebo 主场景仍走已经存在的比赛 Gazebo 线：MoveIt 控制、判分事件、
        # 镜像赛场。参考程序不自动运行，由工作台按钮/学生程序控制。
        # The wrist RGB-D camera is part of the same Gazebo robot model. Start
        # its bridge here, alongside the competition stack, so the workbench
        # always receives camera frames whenever Gazebo is running.
        camera_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="workbench_camera_bridge",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "/gemini305/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/gemini305/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/gemini305/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            remappings=[
                ("/gemini305/image", "/gemini305/color/image_raw"),
                ("/gemini305/depth_image", "/gemini305/depth/image_raw"),
                ("/gemini305/camera_info", "/gemini305/color/camera_info"),
            ],
        )
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("jaka_competition_kit"), "launch", "round.launch.py"])),
            launch_arguments={
                "world": "gazebo", "track": "1", "seed": LaunchConfiguration("seed"),
                "team": "learning", "round": "practice", "run_reference": "false",
                "use_gripper": "true", "use_rviz": LaunchConfiguration("use_rviz"),
                "output_dir": LaunchConfiguration("output_dir"),
            }.items()), camera_bridge]
    if backend != "mock":
        raise RuntimeError(f"不支持的基础组 backend={backend!r}（可选 gazebo/mock）")

    cfg = (
        MoveItConfigsBuilder("jaka_minicobo", package_name="jaka_minicobo_moveit_config")
        .robot_description(mappings={
            "with_gripper": "true", "use_rviz_sim": "true", "use_gazebo": "false",
        })
        .to_moveit_configs()
    )
    kit = get_package_share_directory("jaka_competition_kit")
    return [
        generate_static_virtual_joint_tfs_launch(cfg),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[cfg.robot_description, {"use_sim_time": False}]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[cfg.to_dict(), {
                 "publish_robot_description": True, "publish_robot_description_semantic": True,
                 "publish_planning_scene": True, "publish_geometry_updates": True,
                 "publish_state_updates": True, "publish_transforms_updates": True,
                 "allow_trajectory_execution": True, "use_sim_time": False,
             }]),
        Node(package="controller_manager", executable="ros2_control_node", output="screen",
             parameters=[cfg.robot_description,
                         str(cfg.package_path / "config" / "ros2_controllers.yaml"),
                         {"use_sim_time": False}]),
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=["joint_state_broadcaster", "jaka_minicobo_controller",
                        "gripper_controller", "--controller-manager-timeout", "60"]),
        _rviz_node(cfg),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(kit, "launch", "judge.launch.py")),
            launch_arguments={"track": "1", "seed": LaunchConfiguration("seed"),
                              "team": "learning", "round": "practice",
                              "output_dir": LaunchConfiguration("output_dir")}.items()),
    ]


def _rviz_node(cfg):
    from launch.conditions import IfCondition
    return Node(package="rviz2", executable="rviz2", name="learning_rviz", output="screen",
                # 路径式启动可在重建 ament 安装树之前使用新增配置；安装后仍在相邻 config。
                arguments=["-d", str(Path(__file__).resolve().parent.parent / "config" / "workbench.rviz")],
                parameters=[cfg.to_dict(), {"use_sim_time": False}],
                condition=IfCondition(LaunchConfiguration("use_rviz")))
