"""一键跑一轮完整比赛(仿真): MoveIt 假硬件 + 判分侧 + 参考实现。

  # 赛道一
  ros2 launch jaka_competition_kit round.launch.py track:=1 seed:=246135

  # 赛道二(自动换成 ref_track2)
  ros2 launch jaka_competition_kit round.launch.py track:=2 seed:=135246

**launch 不会自动发令** —— 计时必须掌握在裁判手里:

  ros2 service call /competition/generate std_srvs/srv/Trigger {}
  ros2 service call /competition/start    std_srvs/srv/Trigger {}

参数:
  world         rviz(默认, MoveIt 假硬件) / gazebo(真物理 + 赛场镜像到大屏)
  track         1(基础组) / 2(进阶组), 决定起 ref_track1 还是 ref_track2
  seed          题目种子, -1 表示随机
  team, round   写进成绩单的队伍名与轮次名
  run_reference 是否顺带起官方参考实现(默认 true; 队伍比赛时应设 false)
  vel_scale     参考实现的速度缩放
  gripper       mock(仿真) / jaka_io(真机)
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _sim_stack(context):
    """按 world 参数选仿真: rviz = MoveIt 假硬件; gazebo = 真物理 + 赛场镜像。

    两个 launch 声明的参数名不同(demo.launch.py 是 use_rviz_sim,
    demo_gazebo.launch.py 是 use_gazebo), 传错会直接报错, 所以分开构造。
    """
    world = LaunchConfiguration("world").perform(context)
    actions = []

    if world == "gazebo":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("jaka_minicobo_moveit_config"),
                "launch", "demo_gazebo.launch.py"])),
            launch_arguments={"use_gazebo": "true"}.items(),
        ))
        # 把场地(工作台/工位板/料盒/二维码/工件)镜像进 Gazebo,
        # 否则 Gazebo 画面只有机械臂悬在空地上。
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("jaka_competition_kit"),
                "launch", "gazebo_mirror.launch.py"])),
        ))
    else:
        # rviz / 空 / 拼错 -> 都按老的 RViz 假硬件线走, 不因为一个拼写错误就跑不起来
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("jaka_minicobo_moveit_config"),
                "launch", "demo.launch.py"])),
            launch_arguments={"use_rviz_sim": "true"}.items(),
        ))
    return actions


def generate_launch_description():
    track = LaunchConfiguration("track")

    sim = OpaqueFunction(function=_sim_stack)
    judge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("jaka_competition_kit"),
            "launch", "judge.launch.py"])),
        launch_arguments={
            "track": track,
            "seed": LaunchConfiguration("seed"),
            "team": LaunchConfiguration("team"),
            "round": LaunchConfiguration("round"),
        }.items(),
    )
    # ref_track1 / ref_track2 只差最后一位, 直接用 track 拼出可执行名
    reference = Node(
        package="jaka_competition_kit",
        executable=PythonExpression(["'ref_track' + '", track, "'"]),
        name="reference",
        output="screen",
        parameters=[{"vel_scale": LaunchConfiguration("vel_scale"),
                     "gripper": LaunchConfiguration("gripper")}],
        condition=IfCondition(LaunchConfiguration("run_reference")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="rviz",
                              description="rviz=假硬件 / gazebo=真物理+赛场镜像"),
        DeclareLaunchArgument("track", default_value="1"),
        DeclareLaunchArgument("seed", default_value="-1"),
        DeclareLaunchArgument("team", default_value="demo_team"),
        DeclareLaunchArgument("round", default_value="R1"),
        DeclareLaunchArgument("run_reference", default_value="true"),
        DeclareLaunchArgument("vel_scale", default_value="0.5"),
        DeclareLaunchArgument("gripper", default_value="mock"),
        sim, judge, reference,
    ])
