"""赛前场地自检: 对每个抓取点做一次规划(**不执行**), 确认场地合法。

  ros2 launch jaka_competition_kit selfcheck.launch.py track:=1 vel_scale:=0.3
  ros2 launch jaka_competition_kit selfcheck.launch.py track:=2 vel_scale:=0.3

它起的是官方参考实现并打开 `verify_only`: 所有抓取点只做 `/move_action`
规划、不执行, 所以**真机上也敢跑**。

两条赛道要分开跑 —— 赛道二的自检覆盖散放区, 赛道一只覆盖 6 个工位。
**全部 OK 才允许开赛**。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    track = LaunchConfiguration("track")
    return LaunchDescription([
        DeclareLaunchArgument("track", default_value="1",
                              description="1(工位阵) 或 2(散放区)"),
        DeclareLaunchArgument("vel_scale", default_value="0.3",
                              description="自检用的速度缩放, 真机建议 0.3"),
        Node(
            package="jaka_competition_kit",
            executable=PythonExpression(["'ref_track' + '", track, "'"]),
            name="competition_selfcheck",
            output="screen",
            parameters=[{"vel_scale": LaunchConfiguration("vel_scale"),
                         "verify_only": True}],
        ),
    ])
