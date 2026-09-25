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

### 三之二、赛场镜像进 Gazebo(新增)

上面那一节里 Gazebo 是**空世界** —— 只有地面、太阳和机械臂, 看不到赛场。
原因是工作台/工位板/料盒/工件只存在于 MoveIt 规划场景里, Gazebo 不认识。
`gz_scene` 节点(`launch/gazebo_mirror.launch.py`)把规划场景轮询出来、逐个
镜像成 Gazebo 模型。

```bash
ros2 launch jaka_competition_kit round.launch.py track:=1 seed:=246135 \
    world:=gazebo run_reference:=false
ros2 service call /competition/generate std_srvs/srv/Trigger "{}"
ros2 run jaka_competition_kit ref_track1 --ros-args -p vel_scale:=0.5
ros2 service call /competition/start std_srvs/srv/Trigger "{}"
```

实测:

| 项目 | 结果 |
|---|---|
| Gazebo 实体数 | 46(地面/太阳/机械臂 + 台面 + 6 工位板 + 料盒 5 件 + 二维码 + 6 工件) |
| 机械臂运动 | 真物理, `jaka_minicobo_controller` 每条轨迹 `Goal reached, success!` |
| 抓取判分 | **6/6** 全部成功 |
| 耗时 | **209.4 s** —— 对比假硬件线 102.7 s, 慢近一倍 |

- `screenshots/08-gazebo-arena-mirrored.png` 场地已镜像进 Gazebo
- `screenshots/09-gazebo-arena-running.png` 参考实现在 Gazebo 物理里执行中

> `round.launch.py` 默认会把参考实现一起起起来, 但它**只等题目 60 s**
> (`reference.py` 的 `wait_for_scene`)。Gazebo 那条线光起服务器 + 镜像就要
> 几十秒, 很容易还没来得及调 `/competition/generate` 它就自己退了, 日志里是
> `没有收到题目: 先调 /competition/generate`。要跑 Gazebo 线就加
> `run_reference:=false`, 等镜像完再手动 `ros2 run jaka_competition_kit ref_track1`。

⚠️ 三个坑记在这里:

1. 第一版截图里 **运行时 spawn 的几何有竖条纹瑕疵**(机械臂本体正常), 当时
   归因成"软渲染画质差"。**不是** —— 那是 `xwd2png.py` 自己的解析 bug: 它按
   `bytes_per_line` 而不是 `bits_per_pixel` 判断每像素几字节, 于是把 3 字节的
   数据当 4 字节读, 整张图横向错位。同一份 `.xwd` 用 ImageMagick 解出来是干净的,
   在软渲染和独显上都一样。解析器已修, `screenshots/08`、`09` 已重拍。
   详见 `docs/Gazebo仿真.md` 第 4.3 节。
2. Gazebo 线耗时 ≈ 假硬件线 × 2, **限时不能跨线复用**。
3. **Gazebo 线跑不满 6/6**, 而且不稳定。2026-09 复测: 同一 seed(246135)、
   同一命令下跑出 `3/6, 237.3s`, 失败集中在 `[4/6]` 之后, 报
   `FAIL(error_code=-4)`(MoveIt 的 `CONTROL_FAILED`, 不是规划失败),
   连带 `recover`/复位到 home 也一起失败。另外还有零星的
   `末端在 3.0s 内没有到位(容差 2mm)` 告警。假硬件线(RViz)则是稳定的
   6/6 —— 所以 Gazebo 这条线目前只适合"看画面 / 演示真物理", **不适合判分**,
   要判分还是走假硬件线。这个没查下去, 谁接谁往下挖。

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
