"""把赛场镜像进 Gazebo —— 让 Gazebo 画面"有场地"。

  ros2 launch jaka_competition_kit gazebo_mirror.launch.py

参数名故意叫 ``gz_world`` 而不是 ``world``: round.launch.py 已经用 ``world``
表示"rviz 还是 gazebo", 而 IncludeLaunchDescription **会继承父级的
launch configuration** —— 同名的话 ``world:=gazebo`` 会把这里的 world
也变成 "gazebo", 服务名就成了 /world/gazebo/create, 一个模型都建不出来。

做两件事:
  1. 用 ros_gz_bridge 把 Gazebo 的 create / remove / set_pose 三个服务
     桥接成 ROS 服务(默认不桥接, 所以外面调不到);
  2. 起 gz_scene 节点, 订阅 /monitored_planning_scene, 把规划场景里的
     每一个碰撞体变成 Gazebo 模型。

⚠️ 这是**显示层**, 不参与判分; 判分只认 MoveIt 规划场景(RViz 那份)。
   默认 collision=false, Gazebo 里的场地几何只是视觉, 不参与物理,
   免得机械臂被静态几何顶住、轨迹跟踪变差。

工件放进料盒后, 判分侧会把它从规划场景里**移除**(`Scene.release_into_bin`:
已入库, 不再参与碰撞与规划)。这个 launch 默认把这些工件**冻结在最后位姿**
留在 Gazebo 里(`keep_removed:=true`), 否则料盒在画面里永远是空的 ——
"抓了 6 件"在 Gazebo 里看不出来。

配合 demo_gazebo.launch.py 使用(round.launch.py world:=gazebo 会自动带上)。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _svc(world, rest):
    """拼出 /world/<world>/<rest> 形式的桥接服务名。"""
    return PythonExpression(["'/world/' + '", world, "' + '/", rest, "'"])


def generate_launch_description():
    world = LaunchConfiguration("gz_world")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_scene_service_bridge",
        output="screen",
        arguments=[
            _svc(world, "create@ros_gz_interfaces/srv/SpawnEntity"),
            _svc(world, "remove@ros_gz_interfaces/srv/DeleteEntity"),
            _svc(world, "set_pose@ros_gz_interfaces/srv/SetEntityPose"),
        ],
    )
    mirror = Node(
        package="jaka_competition_kit",
        executable="gz_scene",
        name="competition_gazebo_mirror",
        output="screen",
        parameters=[{
            "world": world,
            "collision": LaunchConfiguration("collision"),
            "update_rate": LaunchConfiguration("update_rate"),
            "entity_prefix": LaunchConfiguration("entity_prefix"),
            "keep_removed": LaunchConfiguration("keep_removed"),
            "keep_removed_prefix": LaunchConfiguration("keep_removed_prefix"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gz_world", default_value="empty",
                              description="Gazebo world 名(默认 empty.sdf -> empty)"),
        DeclareLaunchArgument("collision", default_value="false",
                              description="true = 场地几何参与物理(机械臂可能被顶住)"),
        DeclareLaunchArgument("update_rate", default_value="10.0"),
        DeclareLaunchArgument("entity_prefix", default_value="",
                              description="给 Gazebo 实体名加统一前缀, 避免重名"),
        DeclareLaunchArgument(
            "keep_removed", default_value="true",
            description="工件入盒后会从规划场景里移除; true = 冻结在 Gazebo 画面里"
                        "(否则料盒看着永远是空的), false = 跟着删掉"),
        DeclareLaunchArgument("keep_removed_prefix", default_value="wp",
                              description="只对这几个前缀的实体做上面的冻结"),
        bridge, mirror,
    ])
