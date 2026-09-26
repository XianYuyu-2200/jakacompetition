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
  gripper       夹爪驱动: sim(仿真, 走 gripper_controller) / wheeltec(真机串口)
                / mock(不动夹爪) / jaka_io(老的真机 IO 夹爪)
  use_gripper   是否起夹爪控制器(默认跟着 JAKA_GRIPPER 环境变量走)

末端换 WHEELTEC 柔性夹爪(默认是规则书要求的裸法兰/小平行夹爪):

  JAKA_GRIPPER=1 ros2 launch jaka_competition_kit round.launch.py \
      world:=gazebo track:=1

这一个环境变量同时切三处: URDF 里的夹爪模型(with_gripper)、arena.yaml 的
抓取/转场/投放高度(tool.gripper)、夹爪控制器(use_gripper)。

JAKA 580mm 臂展 + 164.6mm 长的手指, 工位4/6(r=385mm)的**预抓取悬停点**
(208.6mm)和**原地竖直抬到转场高度**(232.6mm)都超出该半径的可达上限
195.9mm。参考实现不用改布局就能绕开: 悬停压到上限(保证 IK 有解 -> 不会随机
挑关节解翻腕), 转场走 L 形(先抬 -> 沿半径朝基座内收 -> 再抬, 见
`Executor.pick_and_place`)。逐点数字见 `ros2 run jaka_competition_kit arena_check`
和 docs/WHEELTEC柔性机械爪.md 第 4 节。

`ros2 launch` **跑完一轮不会自己退**。本 launch 的节点名是固定的, 上一套还活着
时再起一套, 第二套的 spawner 会连到第一套的 /controller_manager 上, 报
"Failed to configure controller" 然后 rviz2 / move_group 接连段错误 —— 看着
像命令写错了, 其实是两套在抢。所以启动前会先查一遍, 撞上就直接报错让你去跑
`bash setup/sim-clean.sh`(确实要并存就加 `allow_concurrent:=true`)。
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# 只有比赛仿真栈才有的标记 —— 别把 `ros2 run jaka_competition_kit arena_check`
# 这类一次性工具也算成"已经在跑"(它们的可执行文件名不一样)。
_GUARD_MARKERS = (
    "jaka_competition_kit/scorer",
    "jaka_competition_kit/scene_generator",
    "ign gazebo",
)


def _pids_with_marker(markers):
    """扫 /proc 找命令行里带任一 marker 的进程, 排除自己这条父进程链。"""
    chain, pid = set(), os.getpid()
    while pid > 1 and pid not in chain:
        chain.add(pid)
        try:
            with open(f"/proc/{pid}/status", encoding="utf-8") as fh:
                ppid = next((int(l.split()[1]) for l in fh
                             if l.startswith("PPid:")), 0)
        except OSError:
            break
        pid = ppid
    hits = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) in chain:
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as fh:
                cmd = fh.read().replace(b"\0", b" ").decode("utf-8", "replace")
        except OSError:
            continue
        if any(m in cmd for m in markers):
            hits.append((int(entry), cmd.strip()))
    return hits


def _guard_no_concurrent(context):
    """已经有仿真在跑就别再起一套, 直接报错说清怎么办。"""
    if LaunchConfiguration("allow_concurrent").perform(context).strip().lower() \
            in ("1", "true", "yes", "on"):
        return []
    hits = _pids_with_marker(_GUARD_MARKERS)
    if not hits:
        return []
    listed = "\n".join(f"    pid {p}: {c[:90]}" for p, c in hits[:5])
    more = f"\n    ... 还有 {len(hits) - 5} 个" if len(hits) > 5 else ""
    raise RuntimeError(
        "已经有一套比赛仿真在跑了, 不能再起第二套 —— 同名节点"
        "(/controller_manager, /robot_state_publisher, ign gazebo)会互相抢,\n"
        "新起的这套会在 'Failed to configure controller' 之后崩掉, "
        "报错看着却像命令写错了。\n"
        f"{listed}{more}\n"
        "  先停掉上一套:  bash setup/sim-clean.sh\n"
        "  确实要两套并存: 加 allow_concurrent:=true"
    )


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

    # 夹爪两片指由独立的 gripper_controller 驱动(位置控制)。
    # MoveIt 只执行机械臂的轨迹, 不会去启动这个控制器, 所以在这里 spawn。
    gripper_ctl = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager-timeout", "60"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gripper")),
    )

    return LaunchDescription([
        # 默认"装不装夹爪"跟着 JAKA_GRIPPER 走, 显式传 use_gripper:=... 可覆盖。
        # 用 PythonExpression 求出字面量 true/false —— IfCondition 只认
        # true/1/false/0, 直接写 $(optenv ...) 会报 invalid condition expression。
        DeclareLaunchArgument(
            "use_gripper",
            default_value=PythonExpression([
                "'true' if '", EnvironmentVariable("JAKA_GRIPPER", default_value="0"),
                "'.strip().lower() in ('1', 'true', 'yes', 'on') else 'false'"]),
            description="是否起夹爪控制器(末端装了 WHEELTEC 夹爪就跟着 JAKA_GRIPPER)"),
        DeclareLaunchArgument(
            "allow_concurrent", default_value="false",
            description="已经有仿真在跑时也允许再起一套(默认 false, 会直接报错)"),
        DeclareLaunchArgument("world", default_value="rviz",
                              description="rviz=假硬件 / gazebo=真物理+赛场镜像"),
        DeclareLaunchArgument("track", default_value="1"),
        DeclareLaunchArgument("seed", default_value="-1"),
        DeclareLaunchArgument("team", default_value="demo_team"),
        DeclareLaunchArgument("round", default_value="R1"),
        DeclareLaunchArgument("run_reference", default_value="true"),
        DeclareLaunchArgument("vel_scale", default_value="0.5"),
        DeclareLaunchArgument("gripper", default_value="sim"),
        # 冲突检查必须排在最前 —— launch 按顺序执行, 排在后面的话
        # Node/IncludeLaunchDescription 已经把进程起起来了。
        OpaqueFunction(function=_guard_no_concurrent),
        sim, gripper_ctl, judge, reference,
    ])
