# jaka_competition_kit —— 比赛官方工具包

一句话:**把赛场摆出来、出题、计时、判分**四件事的官方实现。
参赛队和裁判都用这一个包,初赛(仿真)与决赛(真机)用**同一份配置**。

---
## 1. 三分钟跑一轮(仿真)

```bash
# 终端 1: MoveIt 假硬件 + RViz
source /opt/ros/humble/setup.bash
source ~/codex/codex-competition/jaka_ros2/install/setup.bash
ros2 launch jaka_minicobo_moveit_config demo.launch.py use_rviz_sim:=true

# 终端 2: 判分侧(场景生成器 + 自动评分)
ros2 launch jaka_competition_kit judge.launch.py track:=1 seed:=246135 team:=demo round:=R1

# 终端 3: 官方参考实现(同时也是难度标定基准)
ros2 run jaka_competition_kit ref_track1 --ros-args -p vel_scale:=1.0

# 终端 4: 裁判出题 + 发令
ros2 service call /competition/generate std_srvs/srv/Trigger "{}"
ros2 service call /competition/start    std_srvs/srv/Trigger "{}"
```

赛道二把 `track:=1` 换成 `track:=2`、`ref_track1` 换成 `ref_track2` 即可。

成绩自动写到 `~/.ros/jaka_competition/score_<时间戳>.json` / `.txt`。

> **想看到赛场长什么样**,再开一个终端:
> ```bash
> ros2 launch jaka_competition_kit competition_rviz.launch.py
> ```
> 各型号自带的 `moveit.rviz` **没有** PlanningScene 显示 —— 用它看不到
> 工作台/工位板/料盒/工件。这个 launch 用工具包自带的
> `config/competition.rviz`(相机已拉近到台面)。

### 1.1 想直接跑在 Gazebo 里?

```bash
# 一条命令: 真物理 + 赛场镜像 + 判分侧
ros2 launch jaka_competition_kit round.launch.py track:=1 seed:=246135 world:=gazebo
```

Gazebo 那条线默认是**空世界**, 赛场几何不会自己出现 —— 它在 MoveIt 规划
场景里, Gazebo 不认识。`world:=gazebo` 会顺带起 `gazebo_mirror.launch.py`,
把规划场景逐个镜像成 Gazebo 模型(工件夹起来时会跟着夹爪走)。

> ⚠️ `ros2 launch` 跑完一轮**不会自己退**。上一套还在跑时再起一套, 第二套会
> 卡在 `Failed to configure controller`, 接着 rviz2 / move_group 段错误, 很像
> "命令写错了"。重开一轮前先清场 `bash setup/sim-clean.sh`; launch 自带冲突
> 检查会直接拦住并提示, 确实要并存加 `allow_concurrent:=true`。

单独补一个镜像(比如你已经在跑 `demo_gazebo.launch.py`):

```bash
ros2 launch jaka_competition_kit gazebo_mirror.launch.py
```

> ⚠️ **Gazebo 线比 RViz 线慢近一倍**(209.4 s vs 102.7 s, 赛道一),
> 因为假硬件模式轨迹瞬间到位、Gazebo 要按真实速度走。
> **判分用哪条线, 限时就得按哪条线重新标定**, 不能混用。
>
> ⚠️ 默认的远程桌面会话里 OpenGL 只能走 Mesa 软渲染(独显在 `:1001` 上用不了,
> 因为那个 X server 是 NoMachine 自己实现的)。想真用上独显:
> `bash setup/xorg-gpu.sh start`, 见 `docs/Gazebo仿真.md` 第 4 节。
>
> 细节见 `docs/Gazebo仿真.md`。

---
## 2. 模块地图

| 模块 | 作用 |
|---|---|
| `config/arena.yaml` | **赛场几何的唯一真源**。改尺寸只改这里 |
| `config/rules.yaml` | 评分规则(分值、限时、判定容差) |
| `config/competition.rviz` | 能看到赛场几何的 RViz 配置 |
| `arena` | 读 `arena.yaml`;`validate()` 校核赛场是否合法 |
| `qr` | 二维码生成 / 解析(`JAKA1-######`) |
| `scene` | 规划场景操作(摆台面/料盒/工件, 附着/释放) |
| `backend` | MoveIt 动作客户端。仿真/真机同一接口 |
| `gripper` | 夹爪抽象:`SimGripper`(仿真, 发 `/gripper_controller/commands`) / `WheeltecGripper`(真机串口) / `MockGripper` / `JakaIOGripper` |
| `executor` | 抓取原语 + 事件上报(队伍直接用这个) |
| `gz_scene` | 把 MoveIt 规划场景镜像进 Gazebo(可选显示层, 不参与判分) |
| `scene_generator` | 出题节点 |
| `scorer` | 自动计时判分节点 |
| `ref_track1` / `ref_track2` | 官方参考实现 |

命令行工具:

```bash
ros2 run jaka_competition_kit arena_check   # 改完 arena.yaml 必须跑(含末端可达性逐点校核)
ros2 run jaka_competition_kit qr_tool       # 手动生成/解码二维码
```

### 2.1 末端执行器怎么切

默认是**裸法兰 / 行程几十毫米的小平行夹爪**(规则书写的就是它,
`arena.yaml` 的 `tool.gripper: none`)。要挂 WHEELTEC MS42DC 二指柔性爪:

```bash
JAKA_GRIPPER=1 ros2 launch jaka_competition_kit round.launch.py world:=gazebo track:=1
```

一个环境变量同时切 URDF 模型、抓取高度和夹爪控制器(三处必须一致,
拆开改就会出现"URDF 有夹爪、高度按法兰算"这种把手指戳进台面的组合)。
自备夹爪只要实现 `Gripper.open()/close(object_size)` 就能接入。

> ⚠ 柔性爪的指尖深度是 164.6mm(法兰面→刀尖),抓取高度会从 100mm 抬到 173.6mm,
> 预抓取悬停(再 +35mm)到 208.6mm,而 580mm 臂展在 r=385mm 处的工具朝下可达
> 上限只有 195.9mm —— 工位 4/6 的预抓取和"原地竖直抬到转场高度"(232.6mm)
> 都超了。`Executor` 用两条工业做法绕开: 悬停压到该半径上限(保证 IK 有解、
> 不翻腕), 转场走 **L 形**(先抬→沿半径内收→再抬, 见 `Arena.radius_for_tcp_z`)。
> 实测柔性爪跑完整轮赛道一 **6/6、顺序正确、69.6s、100.0 分**。
> 数字与可选方案见 `docs/WHEELTEC柔性机械爪.md` 第 4 节。

---
## 3. 接口约定

### 服务

| 服务 | 谁提供 | 作用 |
|---|---|---|
| `/competition/generate` | 场景生成器 | 出一轮新题并摆进场景 |
| `/competition/scene/reset` | 场景生成器 | 清空动态工件 |
| `/competition/start` | 判分器 | 裁判发令, 开始计时 |
| `/competition/abort` | 判分器 | 终止本轮 |
| `/competition/run/reset` | 判分器 | 清空判分状态 |

> 出题与判分是两个节点, 服务名各带前缀(`scene/`、`run/`)。
> **不要起同名服务**, 否则 `ros2 service call` 命中谁是随机的。

### 话题

| 话题 | 方向 | 内容 |
|---|---|---|
| `/competition/scene` | 出题 → 全体 | 本轮真值(工件位姿/尺寸/二维码), latched |
| `/competition/events` | 队伍 → 判分器 | 执行事件, JSON |
| `/competition/run_state` | 判分器 → 全体 | `idle`/`armed`/`running`/`finished` |
| `/competition/score` | 判分器 → 全体 | 成绩单 JSON |

### 事件协议

```json
{"event":"grasp",   "object_id":"wp_3", "pose":[x,y,z], "t": 12.34}
{"event":"release", "object_id":"wp_3", "pose":[x,y,z], "t": 45.67}
{"event":"abort",   "object_id":"wp_3", "pose":[x,y,z], "t": 30.12}
{"event":"collision",     "detail":"...", "t": 50.0}
{"event":"intervention",  "detail":"...", "t": 55.0}
```

**判分器不信任这些声明本身**:`grasp` 的 `pose` 必须落在该工件真值的
45mm 内,`release` 的 `pose` 必须落在料盒内 —— 否则事件被判无效。
`abort` 用于"这次尝试作废、工件已放回原位",不产生扣分。

---
## 4. 写自己的解法

最小骨架(仿真/真机通用):

```python
import rclpy
from jaka_competition_kit.arena import load_arena
from jaka_competition_kit.backend import MoveGroupBackend
from jaka_competition_kit.executor import Executor
from jaka_competition_kit.gripper import make_gripper
from jaka_competition_kit.scene import SceneClient

arena = load_arena()
backend = MoveGroupBackend(node, arena.group_name, arena.tcp_link,
                           arena.frame_id, vel_scale=0.5, acc_scale=0.5)
arm = Executor(node, arena, backend, SceneClient(), make_gripper("mock", node))

# obj 来自 /competition/scene 的真值(真实比赛要自己用视觉估)
arm.pick_and_place(obj["id"], obj["xy"][0], obj["xy"][1],
                   obj["grasp_tcp_z"], obj["z"], obj["size"],
                   arena.track1_bin, obj["release_tcp_z"],
                   approach_z=obj["approach_z"])
```

`Executor` 会自动上报 `grasp`/`release` 事件,计时判分随之生效。

调参考实现时可以打开逐动作计时,标定自己方案的瓶颈:

```bash
ros2 run jaka_competition_kit ref_track1 --ros-args -p debug_timing:=true
```

---
## 5. 真机

见 `../docs/真机部署.md`。要点:真机与仿真**只差"启动什么"**,
`-p gripper:=jaka_io` 切夹爪,其余代码不动。

---
## 6. 踩过的坑(改代码前先看)

按"会不会让整条流水线崩掉"排序。前 5 条是**致命**的,中了就基本跑不动。

### 6.1 规划场景(MoveIt)

1. **附着物体的 `REMOVE` 会被静默忽略** —— MoveIt 只处理
   `robot_state.is_diff == true` 的 diff。忘了置位, 服务照样返回
   `success=True`, 但幽灵工件一直挂在末端, 之后**所有规划都失败**。
   见 `scene.SceneClient.apply()`。
2. **必须在回调外自旋** —— 在服务回调里对自己 node 做
   `spin_until_future_complete` 会自我阻塞。所以 `SceneClient`
   自带 node + 后台执行器, 调用侧只用 `threading.Event`。
   (`SceneClient.close()` 负责停执行器、join 线程、销毁 node ——
   不调的话退出时会 `terminate called without an active exception`。)
3. **`attach` 必须给显式相对位姿** —— 只传 `id` 会让 MoveIt 把物体
   中心放到 TCP 原点, 直接和法兰重叠。用 `attach_at`。
4. **附着位姿要在末端"停稳"之后再算** —— 控制器还在路上就读 TF,
   算出来的相对位姿会带几毫米偏差。赛道二工件间隙只有 10mm,
   立刻和邻居报碰撞, 表现为**抓完抬不起来**(`lift` 返回 -2)。
   见 `Executor.wait_until_settled()`。
5. **场上只能有当前赛道的东西** —— 两条赛道的料盒中心几乎重合
   (赛道一 `(315,180)`、赛道二 `(310,200)`), 同时摆放时另一个料盒的
   侧墙会横在入盒路径上。赛道一的**工位板**和赛道二的**散放区**也大面积
   重叠, 同时摆放会让散放工件嵌进 3mm 高的工位板里。
   `scene_generator._build_static(track)` 只摆当前赛道。

### 6.2 几何与工具链

6. **台面高度是"上表面"** —— 按 `z_bottom` 摆会让台面整体抬高一个板厚,
   工件全埋进台面里。台面要按 `top_z - thickness/2` 居中。
7. **散放区要按"最近点"判可达性** —— 只查四角会漏掉近边中点。
   见 `arena.Box.nearest_point()`; 采样器同时查 `r_min` 和 `r_max`。
8. **释放高度要留够余量** —— MoveIt 的位置约束是个 5mm 的球,
   余量太小会报 `INVALID_MOTION_PLAN(-2)`。`release_margin` 现为 **12mm**。
9. **料盒不能做小** —— 150×100 的内腔(130×80)装不下 6 件 50mm 工件
   (10400mm² vs 15000mm²), 而且侧墙只剩 15mm 余量, 规划器末段一甩就
   报 `Computed path is not valid`。现为 **200×150**。
10. **换末端要连"抓取高度"一起换** —— 工件顶面离 TCP 多远, 由
   `Arena.grasp_offset(object_size)` 算: 裸法兰是 `flange_height + clearance`
   (46mm), 柔性爪是 `tip_depth + tip_clearance − 工件尺寸`(50mm 件 = 119.6mm)。
   两者差 **74mm**。只换 URDF 不改这里, 手指会直接扎进台面。
11. **末端越长, 够得到的半径越小** —— 工具轴朝下时 `J5` 恒在 TCP 正上方
   `wrist_len`(=URDF 里 J5→法兰的 159.3mm), 而 `J5` 被"大臂+小臂 420.5mm"
   的球锁住, 所以 `z_max(r) = 27.7 + √(420.5² − r²)` mm(实测完全吻合)。
   MoveIt 对"撞了"和"到不了"都只回 `error_code=99999`, 分不出来 ——
   用 `Arena.max_tcp_z()` / `arena_check` 直接算, 别靠试。

### 6.3 让它跑得快

12. **竖直进/出要走笛卡尔直线, 不要用 RRTConnect** —— RRTConnect 返回的是
    关节空间随机路径, 末端中途会甩。抓取/投放本来就是"竖直进、竖直出",
    见 `MoveGroupBackend.goto_xyz_straight()`(`/compute_cartesian_path`
    + `/execute_trajectory`), 分数 <0.95 时自动退回 RRTConnect。
    `pick_and_place` 的**下降/抬起/投放/退出**都走直线(下降和抬起是对称的,
    别只改一半); 关节空间下降在手腕奇异附近会划一道弧线, 工件容易蹭到邻居。
    探路/转场(approach、to_bin)走关节空间, 但目标必须是"最近 IK 解"(见下条),
    否则照样甩。
    **实测直线 0.4–0.7s; 关节空间转场 0.6–4.7s(看距离)。**
13. **位姿目标会让规划器"随机换关节解"** —— 只给末端位姿(5mm 球 + 姿态容差)时,
    同一个位姿对应多组关节解(肘上/肘下、手腕翻转, 以及 J1/J4/J6 绕 ±360° 的
    等效解), OMPL 采样到哪组**是随机的**。实测同一个"末端下移 10mm"的动作,
    规划出来的轨迹单关节行程高达 **538°**(J1 绕了整整一圈), 而正确答案只有 2.3°。
    见 `MoveGroupBackend.nearest_ik()`: 先解 IK, 按"单关节最大行程"挑最近的一组解,
    再把目标写成 `JointConstraint`; 解不出来或关节目标规划失败才退回位姿目标。

    a. **IK 求解器要用 LMA, 不能用 KDL** —— 见
       `jaka_minicobo_moveit_config/config/kinematics.yaml`。KDL 是牛顿迭代,
       本臂 J5 接近 ±90°(手腕奇异)时从给定种子经常不收敛, 插件随即改用
       **随机种子重启**: 实测同一个目标、同一个 seed 连续 10 次分别解出
       96°/154°/180° 三支, 还有 5 次直接"解不出"。这就是
       **"有几次抓取没走最优轨迹"的根因**。换成
       `lma_kinematics_plugin/LMAKinematicsPlugin`(阻尼最小二乘)后, 同目标
       同 seed 连续 10 次**完全一致**; `timeout` 也从 5ms 放宽到 50ms。
       **改完必须重启 `move_group`** —— 插件只在启动时加载。
    b. **种子要铺满各分支** —— 6R 臂同一个末端位姿一般有 8 组解(肩 ±180°、
       肘上/肘下、手腕翻转 = `J4+180° / J5 取反 / J6+180°`)。数值 IK 只收敛到
       种子附近那一支, 所以 `_ik_seeds()` 按这个结构生成 4~8 个种子各解一次,
       `ik_candidates()` 按行程从小到大**排序返回全部解**。实测 **182 组**
       (14 个真实可达位形 × 13 个抓取目标)全部取到全局最优(与 96 次随机种子的
       暴力枚举结果一致), 单次约 20ms。
       *注意*: 只枚举"当前角 ±360° 的等效表示 + 零位"是不够的 —— 当前角本身就是
       零位时这两者重合, 等于没枚举(实测 wp_5 会稳定地选到 180° 而不是 91°)。
    c. **避障解优先** —— `avoid_collisions=False` 才解出来的位形本身就在碰撞里,
       拿它当目标规划必然失败, 只能当兜底。
    d. **规划失败要换下一支解, 不要退回位姿目标** —— 最优的那一支解可能因为
       臂杆和场景碰撞而规划不出来(终点位姿本身没问题: 工件挂在末端, 换哪一支
       工件都在同一处, 但臂杆位置不同)。`goto()` 会按行程顺序最多试
       `max_ik_tries`(默认 3)支解, 只有全部失败才退回位姿目标。
       实测赛道二有两处 `to_bin` 因为少了这一步退化到位姿目标, 每处白跑
       10~12s, 而且关节解是随机的。备选解用 `quick` 预算(1 次尝试 / 1s),
       最坏耗时可控。
       **再加一层"放宽容差"**: 实测失败是**秒回 `-2`**(不是规划超时), 说明是
       目标位形被判非法、而不是规划不出来。这时把同一个关节目标的容差从
       1 mrad 放到 `joint_tol_loose`(0.02 rad ≈ 1.15°, 臂展 0.3m 上约 4~6mm,
       和位姿目标自带的 5mm 球同量级)**保留选定分支**再试一次。
       加了这一层之后赛道二的位姿兜底从每轮 1~2 次降到 **0 次**, 单轮
       159.9s → 128.0s。
    e. 把 IK 解绕回离当前角最近的那一圈(限位从 `/robot_description` 解析),
       否则 -180°/+180° 这类等价表示也会变成绕整圈。
    f. **转场先试笛卡尔直线(LIN), 再 PTP, 最后 OMPL** —— 竖直进/出、投放、
       退出用 `goto_xyz_straight()`; 工位 <-> 料盒的长转场也先试 LIN(见第 14 条:
       抬够高度后 LIN 全部走得通, 又直又短)。LIN 不行再交给
       `pilz_industrial_motion_planner` 的 `PTP`(各关节同步走关节空间直线 +
       梯形速度曲线, 工业现场搬运动作的标准做法, **确定性**), 规划只要 ~10ms;
       还不行才是 OMPL。
    **实测 Gazebo 线 173.2s → 52.1s, 6/6 与判分结果不变。**
14. **`joint_limits.yaml` 要打开加速度限位** —— 仓库默认
    `has_acceleration_limits: false` + 默认缩放 0.1。加速度限位缺失时
    `vel=1.0/acc=1.0` 反而比 `vel=1.0/acc=0.3` **更慢**(时间倒挂)。
    改成 `has_acceleration_limits: true / max_acceleration: 3.14`,
    默认缩放改 1.0。**改完必须重启 `demo.launch.py`** ——
    `move_group` 只在启动时读这个文件。
15. **失败了要把工件放回原位, 不能凭空扔掉** —— 判分器只认"掉到台面"才罚分。
    一次规划失败就把工件塞进料盒(或删掉), 会让判罚口径失真。
    见 `scene.put_back()` + `Executor.recover()`, 它会补发一个 `abort` 事件。
16. **抬着工件转场要"显式抬够", 否则反而抬得更高** —— 不看数据很难想到。
    工件挂在 TCP 下方 `grasp_offset + size` 处, 所以**转场高度**必须让工件的
    **底面**越过料盒侧墙和旁边工件的顶面:
    `tz = 障碍顶面 + 间隙 + grasp_offset + size`(见 `Arena.transfer_tcp_z()`)。
    - 原来 `lift`/`to_bin` 直接复用了"预抓取悬停高度"(抓取点 +35mm): 赛道一
      手里工件的底面只有 39mm, 而旁边工件顶面 52mm、料盒侧墙 39mm —— 必然撞。
      PTP 走不通, 退回 OMPL 就绕出一条**往外甩、往上拱**的大弧: 实测末端从
      135mm 拱到 **354mm**、半径甩到 **461mm**(超出 420mm 作业带)。看上去就是
      "机械臂抬得很高、不是最优轨迹"。
    - 显式抬到 159mm(赛道一) / 190~230mm(赛道二, 料盒侧墙 100mm)之后, 直线
      转场全部走通: **峰值高度 354mm → 160mm**, 赛道二 `to_bin` 合计
      **68.9s → 35.1s**, 单轮 128.0s → 122.4s。**"抬够" 与 "抬得高" 是反的:
      抬不够才真的抬得高。**
    - **`approach_lift`(预抓取悬停高度)和转场高度是两件事, 不要混用**: 前者
      贴着工件越好(现为 35mm), 后者按障碍物算。
    - 间隙可在 `arena.yaml` 的 `tool.transfer_margin` 里调(默认 10mm)。

### 6.4 Gazebo 镜像(gz_scene)

17. **不能用 `/monitored_planning_scene` 话题当数据源** —— 它是 VOLATILE 的,
    晚启动的节点收不到已有场景, 实测表现是"镜像一个模型都不生成"。
    改用 `/get_planning_scene` 服务轮询。
18. **Gazebo 的 create/remove/set_pose 默认不是 ROS 服务** ——
    要用 `ros_gz_bridge parameter_bridge` 显式桥接(launch 里已经做了)。
19. **launch 参数别叫 `world`** —— `IncludeLaunchDescription` 会继承父级的
    launch configuration, 而 `round.launch.py` 已经用 `world` 表示
    "rviz/gazebo"。同名的话服务名会变成 `/world/gazebo/create`, 建不出模型。
    所以镜像的参数叫 `gz_world`。
20. **地面要自动挪** —— 赛场 `z=0` 是基座安装面, 台面顶面在 `z=-1mm`,
    比 Gazebo 自带地面低, 不挪就只看得见地面。
21. **重起仿真是"按进程名杀干净 + 清 `/dev/shm`"两件事** ——
    只 `pkill -f round.launch.py` 会留下**孤儿的** `scorer` /
    `scene_generator` / `move_group` / `ign gazebo`。症状很好认但很迷惑:
    多发令一次、状态又自己跳回 `armed`(旧 scorer 在发状态)、
    `/competition/scene` 里少了字段(旧 generator 在发题)、
    或者 `approach` 跑了 100 多秒、`取不到 world -> dummy_tcp 的 TF`。
    杀进程时要用 `[x]xx` 这种写法避免 pkill 命中自己所在的命令行, 并且
    **不要在同一个 shell 里又写 `ros2 launch ...` 又 `pkill -f round.launch`**;
    杀完再 `find /dev/shm -maxdepth 1 \( -name 'fastrtps*' -o -name 'sem.fastrtps*' \) -delete`,
    否则新进程会 `Failed init_port ... open_and_lock_file failed`、TF 丢包。

细节见 `docs/Gazebo仿真.md`。

---
## 7. 标定限时(换硬件必做)

`rules.yaml` 里的限时**不是拍脑袋定的**, 是拿参考实现实跑取中位数再乘系数:

```bash
bash design/calibrate.sh 1 3      # 赛道一跑 3 轮
bash design/calibrate.sh 2 3      # 赛道二跑 3 轮
```

脚本会打印中位数、建议限时(中位数 × 1.4)与建议效率满分线(中位数 × 1.05)。
**每次换机械臂、换机器、换固件都要重跑**, 不要照抄本仓库的数字。
当前值见 `config/rules.yaml` 顶部的注释。
