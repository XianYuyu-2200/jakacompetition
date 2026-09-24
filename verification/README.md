# 仿真验证记录

本机环境:Ubuntu 22.04 + ROS 2 Humble + Gazebo Sim(Ignition Fortress)6.18。

## 一、URDF 运动学与 TF 交叉验证

```bash
source /opt/ros/humble/setup.bash
python3 verify_tf_vs_fk.py
```

脚本自己实现一遍前向运动学,再用 `robot_state_publisher` 发布 TF,两者对比。

实测结果(误差 0.000 mm):

| 位姿 | TF dummy_tcp (mm) | FK 计算 (mm) |
|---|---|---|
| 零位 | [0.00, -6.00, 766.80] | [0.00, -6.00, 766.80] |
| q2=+60°, q3=-30° | [-366.77, -6.00, 612.26] | [-366.77, -6.00, 612.26] |
| q2=-45°, q3=+80°, q5=45° | [-129.13, -6.00, 535.59] | [-129.13, -6.00, 535.59] |
| q1=30°, q4=90°, q6=120° | [-359.74, -96.39, 582.61] | [-359.74, -96.39, 582.61] |

## 二、MoveIt RViz 仿真(假硬件)

```bash
bash ../setup/run-rviz-sim.sh
# 另开一个终端:
source ../jaka_ros2/install/setup.bash
python3 moveit_plan_exec.py 0.4,0.9,-1.1,0,1.0,0.6
```

实测:`jaka_minicobo_controller` / `joint_state_broadcaster` 均 active;
MoveGroup 目标 `error_code=1`,29 个轨迹点,执行后 `/joint_states` =
`[0.4071, 0.8923, -1.0942, -0.0076, 1.0065, 0.5984]`。

- `screenshots/03-moveit-rviz-sim-startup.png` 启动后
- `screenshots/04-moveit-rviz-sim-after-execute.png` 规划执行后

## 三、Gazebo 仿真(真物理)

```bash
bash ../setup/run-gazebo-sim.sh
# 另开一个终端:
source ../jaka_ros2/install/setup.bash
python3 moveit_plan_exec.py 0.5,1.0,-1.2,0.2,0.9,0.7
```

实测:Gazebo `empty` world 里加载 `jaka_minicobo`,`gz_ros2_control` 激活控制器;
MoveGroup 目标 `error_code=1`,32 个轨迹点,Gazebo 侧 `/joint_states` 收敛到
`[0.499, 1.0048, -1.1976, 0.1969, 0.8989, 0.6975]`。

- `screenshots/05-gazebo-model-loaded.png` 执行前
- `screenshots/06-gazebo-after-trajectory.png` 执行后

## 四、其他两个脚本

`joint_pose_pub.py` — 手动摆姿态用,改 `/tmp/jaka_pose.json` 即可:

```bash
echo '{"q":[0,1.5708,0,0,0,0]}' > /tmp/jaka_pose.json
python3 joint_pose_pub.py
```

配合 `-d` 指定 rviz 配置,可以复现
`screenshots/01-rviz-urdf-zero-pose.png`(零位伸直)和
`screenshots/02-rviz-urdf-q2-90deg.png`(前折)。

`xwd2png.py` — 无 GUI 环境(如 Xvfb)下抓屏转 PNG:

```bash
xwd -root -display :0 -silent > /tmp/shot.xwd
python3 xwd2png.py /tmp/shot.xwd /tmp/shot.png
```

## 五、一个容易踩的坑

ROS 2 的 `robot_state_publisher` **只有在 `/joint_states` 的
`header.stamp` 有效时才发布动态 TF**(见其源码
`callbackJointState()` 里对 `state->header.stamp` 的比较)。

用 `ros2 topic pub` 手发关节角时时间戳是 0,于是 TF 一直不动、
RViz 里机器人不动 —— 这不是模型的问题。解决:发布时带上
`node.get_clock().now().to_msg()`,或者给 robot_state_publisher 加
`-p ignore_timestamp:=true`。本目录的脚本都已经带时间戳。
