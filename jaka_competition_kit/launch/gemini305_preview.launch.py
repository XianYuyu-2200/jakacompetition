"""腕部 Gemini 305 预览。

默认只起 Gazebo、机械臂和相机桥，不起 MoveIt，也不镜像整块赛场。
机械臂会被 gemini305_preview 节点送到识别位姿，彩色图和深度图写到
output_dir（默认仓库 design/gemini305_preview）。

  ros2 launch jaka_competition_kit gemini305_preview.launch.py

练习抓取（用训好的 YOLO，不是判分那条线）:

  export JAKA_GRIPPER=1
  ros2 launch jaka_competition_kit gemini305_preview.launch.py learn:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    kit = get_package_share_directory("jaka_competition_kit")
    description = get_package_share_directory("jaka_description")
    moveit = get_package_share_directory("jaka_minicobo_moveit_config")
    world = os.path.join(kit, "worlds", "gemini305_preview.sdf")
    xacro_file = os.path.join(moveit, "config", "jaka_minicobo.urdf.xacro")
    joints = os.path.join(kit, "config", "gemini305_preview_joints.yaml")
    repo = ""
    repo_out = os.path.join("/tmp", "gemini305_preview")
    # install 前缀在 jaka_ros2/install 下，源码在它的上一级仓库里。
    # 找到 design/腕部Gemini305.md 就写到旁边的 gemini305_preview。
    cursor = kit
    for _ in range(8):
        cursor = os.path.dirname(cursor)
        design = os.path.join(cursor, "design", "腕部Gemini305.md")
        if os.path.isfile(design):
            repo = cursor
            repo_out = os.path.join(cursor, "design", "gemini305_preview")
            break

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ", xacro_file,
                " use_gazebo:=true",
                " use_rviz_sim:=false",
                " with_gripper:=true",
                " initial_positions_file:=", joints,
            ]),
            value_type=str,
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"{world} -r"}.items(),
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )
    # 真正的 controller_manager 在 Gazebo 进程里的 ign_ros2_control 插件。
    # 再起一个 ros2_control_node 会因为 IgnitionSystem 加载失败而退出，那是预期噪音。
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-world", "gemini305_preview",
            "-name", "jaka_minicobo",
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", "0",
        ],
    )
    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "jaka_minicobo_controller",
            "gripper_controller",
            "--controller-manager-timeout", "60",
        ],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gemini305_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/gemini305/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/gemini305/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/gemini305/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/overview@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        remappings=[
            ("/gemini305/image", "/gemini305/color/image_raw"),
            ("/gemini305/depth_image", "/gemini305/depth/image_raw"),
            ("/gemini305/camera_info", "/gemini305/color/camera_info"),
            ("/overview", "/overview/image_raw"),
        ],
    )
    capture = Node(
        package="jaka_competition_kit",
        executable="gemini305_preview",
        name="gemini305_preview",
        output="screen",
        parameters=[{
            "output_dir": LaunchConfiguration("output_dir"),
            "joints_file": joints,
            "use_sim_time": False,
        }],
    )

    recognize = ExecuteProcess(
        cmd=["python3", "-m", "jaka_competition_kit.track1_recognize"],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("learn")),
    )

    def _learn_stack(context):
        """练习模式：只加 move_group，不再起一套假硬件控制器。"""
        flag = LaunchConfiguration("learn").perform(context).strip().lower()
        if flag not in ("1", "true", "yes", "on"):
            return []
        if not repo:
            raise RuntimeError("找不到仓库根目录，练习抓取脚本没法定位")
        os.environ["JAKA_GRIPPER"] = "1"
        from moveit_configs_utils import MoveItConfigsBuilder

        builder = MoveItConfigsBuilder(
            "jaka_minicobo", package_name="jaka_minicobo_moveit_config")
        builder.robot_description(mappings={
            "with_gripper": "true",
            "use_gazebo": "true",
            "use_rviz_sim": "false",
            "initial_positions_file": joints,
        })
        cfg = builder.to_moveit_configs()
        move_group = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                cfg.to_dict(),
                {
                    "publish_robot_description_semantic": True,
                    "allow_trajectory_execution": True,
                    "publish_planning_scene": True,
                    "publish_geometry_updates": True,
                    "publish_state_updates": True,
                    "publish_transforms_updates": True,
                    "monitor_dynamics": False,
                    "use_sim_time": True,
                },
            ],
        )
        # 控制器大约 12 秒起来。move_group 要等 /clock 和关节状态，不要跟 Gazebo 抢着起。
        delayed = [move_group]
        show_rviz = LaunchConfiguration("rviz").perform(context).strip().lower()
        if show_rviz in ("1", "true", "yes", "on"):
            rviz_config = os.path.join(moveit, "config", "moveit.rviz")
            delayed.append(Node(
                package="rviz2",
                executable="rviz2",
                name="learn_rviz",
                output="log",
                arguments=["-d", rviz_config],
                parameters=[cfg.to_dict(), {"use_sim_time": True}],
            ))
        grasp = os.path.join(repo, "perception", "track1_sim", "grasp.py")
        delayed.append(TimerAction(period=18.0, actions=[ExecuteProcess(
            cmd=["python3", "-u", grasp, "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
            additional_env={"JAKA_GRIPPER": "1", "PYTHONUNBUFFERED": "1"},
        )]))
        return [TimerAction(period=15.0, actions=delayed)]

    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value=repo_out),
        DeclareLaunchArgument(
            "learn", default_value="false",
            description="true：用训好的 YOLO 做识别抓取练习。不起轮廓识别，不起第二套控制器。"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="learn:=true 时要不要再开一个 MoveIt 的 RViz"),
        gazebo,
        rsp,
        bridge,
        TimerAction(period=6.0, actions=[spawn]),
        TimerAction(period=12.0, actions=[controllers_spawner]),
        TimerAction(period=14.0, actions=[capture]),
        TimerAction(period=16.0, actions=[recognize]),
        OpaqueFunction(function=_learn_stack),
    ])
