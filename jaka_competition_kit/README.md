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
| `gripper` | 夹爪抽象(`MockGripper` / `JakaIOGripper`) |
| `executor` | 抓取原语 + 事件上报(队伍直接用这个) |
| `scene_generator` | 出题节点 |
| `scorer` | 自动计时判分节点 |
| `ref_track1` / `ref_track2` | 官方参考实现 |

命令行工具:

```bash
ros2 run jaka_competition_kit arena_check   # 改完 arena.yaml 必须跑
ros2 run jaka_competition_kit qr_tool       # 手动生成/解码二维码
```

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

### 6.3 让它跑得快

10. **竖直进/出要走笛卡尔直线, 不要用 RRTConnect** —— RRTConnect 返回的是
    关节空间随机路径, 末端中途会甩。抓取/投放本来就是"竖直进、竖直出",
    见 `MoveGroupBackend.goto_xyz_straight()`(`/compute_cartesian_path`
    + `/execute_trajectory`), 分数 <0.95 时自动退回 RRTConnect。
    探路/转场(approach、to_bin)仍走关节空间。
    **实测单次运动 0.3–0.6s, 而关节空间要 2.5–6s。**
11. **`joint_limits.yaml` 要打开加速度限位** —— 仓库默认
    `has_acceleration_limits: false` + 默认缩放 0.1。加速度限位缺失时
    `vel=1.0/acc=1.0` 反而比 `vel=1.0/acc=0.3` **更慢**(时间倒挂)。
    改成 `has_acceleration_limits: true / max_acceleration: 3.14`,
    默认缩放改 1.0。**改完必须重启 `demo.launch.py`** ——
    `move_group` 只在启动时读这个文件。
12. **失败了要把工件放回原位, 不能凭空扔掉** —— 判分器只认"掉到台面"才罚分。
    一次规划失败就把工件塞进料盒(或删掉), 会让判罚口径失真。
    见 `scene.put_back()` + `Executor.recover()`, 它会补发一个 `abort` 事件。

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
