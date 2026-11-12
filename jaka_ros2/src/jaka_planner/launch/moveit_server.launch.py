import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        # Declare 'ip' and 'model' arguments
        DeclareLaunchArgument('ip', default_value='10.5.5.100', description='IP address'),
        DeclareLaunchArgument('model', default_value='default_model', description='Model name'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='Enable hand-drag teleop; flange Point toggles the gripper either way'),

        # Launch the 'moveit_server' node from the 'jaka_planner' package
        Node(
            package='jaka_planner',
            executable='moveit_server',  # the executable to run
            name='moveit_server',
            output='screen',
            parameters=[
                {'ip': LaunchConfiguration('ip')},  # Pass 'ip' parameter
                {'model': LaunchConfiguration('model')},  # Pass 'model' parameter
                {'teleop': ParameterValue(LaunchConfiguration('teleop'), value_type=bool)},
            ],
        ),
    ])
