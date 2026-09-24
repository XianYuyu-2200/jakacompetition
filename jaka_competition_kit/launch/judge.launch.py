"""只启动判分侧: 场景生成器 + 自动评分。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("track", default_value="1", description="赛道 1 或 2"),
        DeclareLaunchArgument("seed", default_value="-1", description="-1 表示随机"),
        DeclareLaunchArgument("team", default_value="team_unknown"),
        DeclareLaunchArgument("round", default_value=""),
        DeclareLaunchArgument("output_dir", default_value="~/.ros/jaka_competition"),

        Node(
            package="jaka_competition_kit",
            executable="scene_generator",
            name="competition_scene_generator",
            output="screen",
            parameters=[{
                "track": LaunchConfiguration("track"),
                "seed": LaunchConfiguration("seed"),
                "qr_output_dir": LaunchConfiguration("output_dir"),
            }],
        ),
        Node(
            package="jaka_competition_kit",
            executable="scorer",
            name="competition_scorer",
            output="screen",
            parameters=[{
                "team": LaunchConfiguration("team"),
                "round": LaunchConfiguration("round"),
                "output_dir": LaunchConfiguration("output_dir"),
            }],
        ),
    ])
