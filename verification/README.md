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

## 二点五、Gazebo 镜像的位姿形状回归测试

```bash
source /opt/ros/humble/setup.bash
source ../jaka_ros2/install/setup.bash
python3 check_gz_scene_pose.py
```

不启动 Gazebo, 只用假 TF 检查 `GazeboSceneMirror._to_world()` 的两条返回路径
(帧就是 `world` / 要过 TF) 返回同一种形状 `(x, y, z, (qx,qy,qz,qw))`, 并且
`_same_pose` 不会再抛 `TypeError`。

实测:`frame=world` 与 `frame=dummy_tcp` 都是 4 元组、平移合成正确、
`_compose` 保持 `(pos, quat)` 契约 -> 全部通过。

> 背景: 这两条路径形状不一致时, 工件一被 attach 就把镜像节点打死, 现象是
> "RViz 里工件抓进料盒了, Gazebo 里的工件一直躺在工位上"。
> 详见 `docs/Gazebo仿真.md` 第 2.4 节。

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

## 固定姿态、相机与四组顺序的隔离回放

在仓库根目录执行（会启动仿真并执行运动，只用于仿真；不要在此 ROS 域接入真机）：

```bash
/usr/bin/python3 verification/accept_workbench_fixed_poses.py \
  --poses jaka_competition_kit/config/track1_simulation_example.yaml \
  --output "$HOME/.local/share/jaka-workbench/acceptance/$(date +%Y%m%d-%H%M%S)" \
  --domain 94
```

要求输出目录尚不存在，ROS 域没有已有节点；现有仿真冲突仍由启动检查拒绝，不会清理用户进程。脚本自己加载 ROS 和本仓库安装环境，显式设置 `JAKA_GRIPPER=1`，同时隔离 `IGN_PARTITION`、`GZ_PARTITION`。请保留桌面图形会话。

默认 seed 为 `246135 1 2 3`（seed 不是二维码顺序），速度缩放 0.5。脚本等待机械臂、夹爪、控制器、MoveIt、裁判和相机都就绪；从二维码 PNG 解码，保持 MoveIt 碰撞检查并关闭笛卡尔回退。每轮要求程序 6/6、六次固定姿态执行、裁判收集顺序与生成顺序一致、相机在该轮有图像消息；失败后不自动重试。日志和 `report.json` 保存在输出目录，报告包括姿态与关键代码 SHA-256。

仅副本 `active-test-only.yaml` 在回放期间启用；退出时禁用并清理本脚本拥有的进程。原姿态和用户配置不改动。`remaining_owned_jobs` 应为空。直接强杀脚本或断电无法保证执行清理，恢复后应先检查隔离环境与副本状态。

**验收范围**：关节运动、二维码顺序、规划场景附着、裁判事件和相机数据联动。镜像工件没有完成物理摩擦夹持验证，程序分数不是实际比赛性能或真机许可。`verify_only` 只允许完整固定姿态的逐点规划，不支持的配置会拒绝，不能回退到实际运动。

可加 `--check-gripper-stall`：在完整回放之后，仅停用本脚本隔离仿真中的夹爪控制器，验证闭合请求在到位等待超时后返回失败，且执行器在附着/判分之前退出；随后恢复控制器。它不发送机械臂运动，不测试或认证实体急停。

## 全排列与失败中止的软件回归（不启动仿真）

```bash
source /opt/ros/humble/setup.bash
QT_QPA_PLATFORM=offscreen PYTHONPATH="$PWD:$PWD/jaka_competition_kit:$PYTHONPATH" \
  /usr/bin/python3 -m pytest -q workbench/test jaka_competition_kit/test
```

`test_track1_all_orders.py` 为全部 720 种排列各生成带前缀、裸数字两张二维码，实际解码后经过参考程序和固定姿态执行器，核对每个工位的基本姿态、路点、抓取与释放调用。场景 `qr_order` 故意放入错误内容，防止以真值代替解码。运动、TF 和夹爪使用测试替身；这些测试不证明 720 条 Gazebo 连续轨迹均可执行。

同一测试文件对 `432165` 顺序的全部 57 个动作逐一注入失败，对全部 36 个阶段末端反馈逐一注入 TF 异常，要求立即中止，不自动重试、不继续下一工位。另有空图、损坏图、多码歧义、非法载荷和判分事件异常测试。判分器拒绝缺失、长度错误、字符串、布尔值、NaN/Inf 坐标，不会将缺失坐标填为世界原点。

实际 Gazebo 回放仍须单独执行上节脚本；物理夹持与真机验收仍需独立证据。
