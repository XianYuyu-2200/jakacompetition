# JAKA Mini 2 比赛资料包

围绕 **JAKA Mini 2 六轴协作机械臂**整理的开发/仿真/资料集合。
基础环境:Ubuntu 22.04 + ROS 2 Humble + Gazebo Sim (Ignition Fortress)。

> 命名提示:JAKA 官方 ROS 2 包里,mini 系列的型号目录名是 **`minicobo`**。
> MiniCobo 与 Mini 2 的运动学完全一致(同为 6 轴 / 580 mm 臂展 / 相同关节范围),
> 差别只在负载(1 kg vs 2 kg)、重量和供电,所以同一份 URDF 对两者都适用。
> 详见 `docs/JAKA-Mini2-参数速查.md`。

## 目录结构

```
codex-competition/
├── competition.md          赛题细则(定稿版)  <- 规则看这份
├── jaka_competition_kit/   比赛官方工具包(赛场定义 + 出题 + 计时判分 + 参考实现)
├── design/                 赛题设计工作区(规则校核 / 赛场尺寸实测 / 参考实现 / 配图)
├── jaka_ros2/              JAKA 官方 ROS 2 包(驱动 + MoveIt 配置 + 仿真), 源码
├── docs/                   官方 PDF 手册 + 参数速查 + 真机部署指南
├── setup/                  一键安装/打补丁/编译/启动脚本
├── verification/           仿真验证脚本 + 实测截图
├── dual_arm_demo/          双臂 operator/tracking 演示(基于本机械臂)
└── extras_linkerhand/      LinkerHand 灵巧手 ROS 2 SDK(末端执行器, 可选)
```

## 快速开始

```bash
bash setup/install-deps.sh           # 装依赖
bash setup/patch-moveit-launches.sh  # 必做! 否则仿真模式不生效
bash setup/build.sh                  # colcon build
bash setup/run-rviz-sim.sh           # 纯 RViz 仿真
bash setup/run-gazebo-sim.sh         # Gazebo 物理仿真
```

跑一轮完整的**比赛仿真**(出题 -> 计时 -> 判分 -> 成绩单):

```bash
source jaka_ros2/install/setup.bash
ros2 launch jaka_minicobo_moveit_config demo.launch.py use_rviz_sim:=true   # 终端 1
ros2 launch jaka_competition_kit judge.launch.py track:=1 seed:=246135      # 终端 2
ros2 run jaka_competition_kit ref_track1 --ros-args -p vel_scale:=1.0       # 终端 3
ros2 service call /competition/generate std_srvs/srv/Trigger "{}"           # 终端 4
ros2 service call /competition/start    std_srvs/srv/Trigger "{}"

# 想看到赛场(工作台/工位板/料盒/工件), 再开一个:
ros2 launch jaka_competition_kit competition_rviz.launch.py
```

> 自带的 `moveit.rviz` **没有** PlanningScene 显示, 用它看不到赛场几何 ——
> 所以工具包单独提供了一份 `competition.rviz`。

一键跑完整一轮(仿真, 会把仿真+判分+参考实现一起起起来):

```bash
ros2 launch jaka_competition_kit round.launch.py track:=1 seed:=246135 vel_scale:=1.0
```

详见 `jaka_competition_kit/README.md`(仿真)与 `docs/真机部署.md`(决赛真机)。

### 限时怎么来的

`rules.yaml` 的限时不是拍脑袋定的, 是拿参考实现实跑取中位数:

| 赛道 | 参考实现实测中位数 | 限时(×1.4) | 效率满分线(=中位数) |
|---|---|---|---|
| 一(6 件) | 102.7 s | **145 s** | **105 s** |
| 二(12 件) | 228.2 s | **320 s** | **230 s** |

换机器/换机械臂/换固件都要重跑 `bash design/calibrate.sh <赛道> 3`。

启动仿真后另开终端验证一次规划+执行:

```bash
source jaka_ros2/install/setup.bash
python3 verification/moveit_plan_exec.py 0.4,0.9,-1.1,0,1.0,0.6
```

## 已验证到什么程度

| 项目 | 结果 |
|---|---|
| URDF 运动学 vs TF | 4 个位姿误差 0.000 mm |
| MoveIt RViz 仿真 | 规划+执行成功, error_code=1, 29 个轨迹点 |
| Gazebo 物理仿真 | 规划+执行成功, error_code=1, 32 个轨迹点, Gazebo 关节收敛到目标 |

细节和截图见 `verification/README.md`。

## 每个目录里有什么

**jaka_competition_kit/** — 比赛本身的实现。`config/arena.yaml` 是**赛场几何的
唯一真源**,`config/rules.yaml` 是评分规则;两个 ROS 2 节点负责出题与自动判分;
`ref_track1` / `ref_track2` 是官方参考实现(也是限时标定基准)。
初赛(仿真)与决赛(真机)用**同一份配置、同一套判分**。

**design/** — 设计比赛本身要用的东西。`规则校核报告.md` 逐条列出
`competition.md` 里的错误与实测依据;`赛事设计指南.md` 是怎么把两个比赛落地的路线;
`arena_layout.png` / `reach_envelope.png` 可直接用作赛题细则配图;
`track1_pick_demo.py` 是赛道一的参考实现骨架(实测 6/6 成功)。
**改动规则或赛场尺寸前先看这里。**

**jaka_ros2/** — 主体。包含 `jaka_driver`(基于 `libjakaAPI.so` 的真机驱动)、
`jaka_planner`(MoveIt 桥接)、`jaka_msgs`、`jaka_description`(URDF + STL 网格),
以及各型号的 `jaka_<model>_moveit_config`。mini 系列对应
`jaka_minicobo_moveit_config`。另有官方中文/英文文档和改过的 `launches.py`。

> **网格已精简**:`jaka_description/meshes/` 下只保留了 mini 系列的
> `jaka_minicobo_meshes/`(2.7 MB),其余 23 个型号的 STL 已删除(省下约 178 MB)。
> 其他型号的 URDF / moveit_config / rviz 文件还在,但加载时会找不到网格。
> 如果确定整个比赛只做 mini 系列,可以把下面这些也删掉:
> `jaka_ros2/src/jaka_<其它型号>_moveit_config/`、
> `jaka_description/urdf/jaka_<其它型号>.urdf`、
> `jaka_description/config/jaka_<其它型号>_urdf.rviz`、
> `jaka_description/launch/jaka_<其它型号>_rviz_control_launch.py`。

注意:本目录**已经编译好**(`build/` + `install/`,用 `--symlink-install`),
所以 `source jaka_ros2/install/setup.bash` 就能直接跑。
改完 `src/` 里的 Python 代码**不用重新编译**(符号链接直通);
只有**新增文件**(比如新加一个 launch 或 config)才需要
`colcon build --symlink-install --packages-select jaka_competition_kit`。

> 交付成"纯源码包"(省约 148 MB)的话,删掉这三个目录即可,
> 用之前跑一次 `bash setup/build.sh`:
> ```bash
> find jaka_ros2/build jaka_ros2/install jaka_ros2/log -depth -delete
> ```

**docs/** — 从 JAKA 官网抓的官方 PDF(硬件用户手册 V03、产品选型手册、安全说明),
外加一份自己整理的 `JAKA-Mini2-参数速查.md`,里面有官方参数与 URDF 实测值的对照,
以及两者不一致的地方。

**setup/** — 环境搭建脚本,以及踩坑说明。**重点看 `setup/README.md`**,
里面记录了 5 个会让仿真直接跑不起来的问题。

**verification/** — 验证脚本(URDF/FK 交叉验证、MoveIt 规划执行、姿态发布、
抓屏转 PNG)和实测截图。

**dual_arm_demo/** — 原 `jaka_dual_readonly`,用两台 JAKA mini 做的
operator(192.168.0.101)/ tracking(192.168.0.102)只读跟随实验代码。
接真机时用,仿真里不需要。看 `ROLES.md`。

**extras_linkerhand/** — LinkerHand 灵巧手 ROS 2 SDK(原仓库的 `.git`
和 `build/install/log` 已剔除)。作为末端执行器备选,与机械臂本体无关,
不需要可以整个删掉。

## 比赛相关提醒

- **赛场尺寸有硬约束**:工具竖直向下时,竖直抓取可达带是**距基座 180–420mm**。
  改任何尺寸后必须跑 `ros2 run jaka_competition_kit arena_check`。
  详见 `design/规则校核报告.md`。

- URDF 里**没有负载参数**。抓取类任务需要自己在规划场景里加
  (`AttachedCollisionObject` 或 SRDF 里配置)。Mini 2 的实际负载上限是 2 kg。
- URDF 的 `joint_2` 行程是 ±119.7°,比官方手册的 ±125° 小 5°。
  动作设计如果贴近边界要留意。
- 真机开发用的是 `jaka_driver`;仿真用 MoveIt 的 mock 硬件或 Gazebo。
  两者**不能同时跑**(官方文档明确写了)。
- 赛事总则与赛题细则链接见 `web.md`。
- **限时不是硬编码的**:`config/rules.yaml` 一处生效, 改完记得同步
  `competition.md` 与 `design/规则校核报告.md`。
