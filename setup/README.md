# 环境搭建

按顺序执行即可。全部脚本默认针对 **Ubuntu 22.04 + ROS 2 Humble**。

```bash
bash setup/install-deps.sh          # 1. 装依赖 (需要 sudo)
bash setup/patch-moveit-launches.sh # 2. 打补丁 (关键, 见下)
bash setup/build.sh                 # 3. 编译
```

之后:

```bash
bash setup/run-rviz-sim.sh     # 纯 RViz 仿真(假硬件, 无需 Gazebo)
bash setup/run-gazebo-sim.sh   # Gazebo 物理仿真
```

想让 Gazebo 真的走独显(默认的远程桌面会话用不了独显, 见下面第 6 条):

```bash
bash setup/xorg-gpu.sh start :0
DISPLAY=:0 ros2 launch jaka_competition_kit round.launch.py world:=gazebo
bash setup/xorg-gpu.sh stop :0
```

`:0` 是 NoMachine 看不到的, 而 Gazebo 的 3D 区在 `:1001` 上不重绘。想在远程
桌面里看/操作 Gazebo:

```bash
bash setup/xorg-viewer.sh start    # VNC: 可拖拽操作, 只监听本机, 免 sudo
bash setup/xorg-viewer.sh fill     # Gazebo 起晚了就用这条把它铺满置顶
bash setup/xorg-viewer.sh view     # 查看窗口拖不动(卡在弹菜单状态)时重开它
bash setup/xorg-viewer.sh cam table  # 相机跑飞了, 把 Gazebo 机位对回赛场
bash setup/xorg-viewer.sh stop

# 或者只读转发(不装任何东西, 约 5 fps):
python3 setup/gazebo-viewer.py --from :0 --title Gazebo --crop 578x797+0+48
```

细节见 `docs/Gazebo仿真.md` 第 4.2.1 节。

## 为什么必须有第 2 步

仓库里的 `jaka_ros2/src/jaka_minicobo_moveit_config/launch/demo.launch.py`
只是调用了 MoveIt 的
`moveit_configs_utils.launches.generate_demo_launch()`。而 **系统自带的
`moveit_configs_utils/launches.py` 不认识 `use_rviz_sim` / `use_gazebo`**,
所以直接 `use_rviz_sim:=true` 只会被静默忽略,仿真模式根本不生效。

`jaka_ros2/launches.py` 是 JAKA 官方这份改过的版本(多了
`use_rviz_sim`、`use_gazebo`,以及 Gazebo 独立可视化/联动启动)。
必须把它覆盖到系统的 `moveit_configs_utils` 里。

脚本会先备份成 `launches.py.orig`。注意:**以后 apt 升级 moveit 会覆盖
这个文件, 需要重新执行第 2 步**。

## 依赖清单

除了常规 ROS 2 工具链,下面这些是关键:

- `ros-humble-moveit*` — 规划、可视化、setup assistant
- `ros-humble-moveit-configs-utils` — 被 demo.launch.py 调用的库
- `ros-humble-moveit-visual-tools` — `jaka_planner` 编译时必需
- `ros-humble-ros2-control` / `ros-humble-ros2-controllers` / `ros-humble-controller-manager`
- `ros-humble-joint-trajectory-controller`、`ros-humble-position-controllers`
- `ros-humble-xacro`、`ros-humble-joint-state-publisher[-gui]`
- `ros-humble-ros-gz`、`ros-humble-ign-ros2-control`、`ignition-fortress` — Gazebo 仿真

## 已知的坑

**1. Gazebo 模式会多起一个会崩溃的节点。**
`demo_gazebo.launch.py` 除了在 Gazebo 进程内加载 `gz_ros2_control` 插件,
还会另外起一个独立的 `ros2_control_node`。后者用的是
`<plugin>ign_ros2_control/IgnitionSystem</plugin>`,而这个插件只在 Gazebo
进程里存在,于是它会抛 `LibraryLoadException` 并打印一长串堆栈。
**这是无害的**,真正干活的 Gazebo 内插件工作正常。想去掉噪音,可在
`jaka_ros2/launches.py` 的 `generate_gazebo_launch()` 里删掉那个 node。

**2. `ign_ros2_control` 已改名 `gz_ros2_control`。**
Gazebo 日志里会有 deprecation 提示,不影响运行。若想干净,把
`jaka_minicobo_moveit_config/config/jaka_minicobo.ros2_control.xacro`
里的 `<plugin>ign_ros2_control/IgnitionSystem</plugin>` 换成
`gz_ros2_control/GazeboSimSystem`。

**3. `jaka_description` 的 rviz 配置原本是 ROS 1 类名。**
`jaka_<model>_urdf.rviz` 里写的是 `rviz/Grid`、`rviz/RobotModel` 这类
ROS 1 插件名,RViz2 会全部报
`Failed to load ... does not exist`,面板一片空白。
**本仓库里 6 个配置(minicobo + zu3/5/7/12/18)已经修好**:
面板改成 `rviz_common/*`,显示/工具/视图改成 `rviz_default_plugins/*`。

**4. `jaka_minicobo_rviz_control_launch.py` 还缺包。**
该 launch 依赖 `jaka_jog_panel`(另一个独立仓库,不在本项目里),
所以这条 launch 目前跑不起来。rviz 配置本身已修好。

**5. `warehouse_ros_mongo` 没有对应 apt 包。**
本次用 `ros-humble-moveit-ros-warehouse` 替代。只有 `db:=true` 的
warehouse 功能会受影响,仿真和规划不受影响。

**6. 默认的远程桌面会话里 OpenGL 只能走 Mesa 软渲染。**
这台机器有 RTX 5060 Ti, 但 `:1001` 不是 Xorg —— `nxnode.bin` 自己就是那个
X server, NVIDIA 的 GLX 客户端库驱动不了它。强行设
`__GLX_VENDOR_LIBRARY_NAME=nvidia` 会让 `glxinfo` 改口报 NVIDIA, 但
**所有 GL 窗口(含 Gazebo 的 3D 视口)全黑**。要用独显就
`bash setup/xorg-gpu.sh start`, 详见 `docs/Gazebo仿真.md` 第 4 节。
