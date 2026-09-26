# WHEELTEC MS42DC 二指柔性机械爪 —— 建模、仿真与真机控制

> 这份文档讲两件事:
> 1. `gripper.STEP` 怎么变成"机械臂上能用的东西"(仿真里能张合、能夹工件);
> 2. **这只夹爪装到 JAKA Mini 2 上之后,哪些点位够不到了** —— 第 4 节,
>    换末端之前一定要看,否则一轮比赛会在中途莫名其妙地反复"规划失败"。

相关目录:

| 位置 | 内容 |
|---|---|
| `jaka_ros2/src/wheeltec_gripper/` | 集成包:网格 + URDF 宏 + 实测几何参数 + 转换脚本 |
| `jaka_ros2/src/step_motor/` | 真机驱动(厂商 ROS 2 例程):串口 → 电机 |
| `jaka_ros2/src/serial_ros2/` | `step_motor` 依赖的串口库(厂商源码) |
| `WHEELTEC 柔性机械爪/` | 厂商原始资料(手册/固件/上位机/视频),索引见该目录的 `README.md` |
| `jaka_competition_kit/` | 比赛工具包:抓取高度、仿真夹爪驱动都按末端类型分了两套 |

---

## 1. STEP 能直接用吗?

**不能。** STEP 是 B-Rep 实体格式,MoveIt / Gazebo / RViz 只认三角网格
(STL / DAE / OBJ),而且 STEP 里没有"哪一块是底座、哪一块是手指"这种语义 ——
而夹爪的两片手指必须做成**两个能转的关节**,不拆开就没法张合。

所以流程是:STEP → 拆件 + 转 STL → 写 URDF 宏。转换脚本落在集成包里,
是可重跑的:

```bash
cd jaka_ros2/src/wheeltec_gripper
~/.venvs/stepmesh/bin/python tools/step_to_mesh.py     # 需要 trimesh + cascadio
```

脚本做四件事(全部有输出,改 STEP 后重跑即可):

| 产物 | 说明 |
|---|---|
| `meshes/gripper_base.stl` `gripper_jaw_l.stl` `gripper_jaw_r.stl` | 显示网格(1.5 万 / 4.9 千面) |
| `meshes/*_col.stl` | **碰撞包络**:底座 1 个包围盒(12 面),每片指 5 段深度切片(60 面) —— 手指是锥形,用包围盒会让它"凭空变粗"撞到工件 |
| `config/gripper.yaml` | 实测几何:指尖深度、转轴坐标、开口曲线、质量与惯量 |
| `urdf/wheeltec_gripper.urdf.xacro` | 装到 `dummy_tcp` 上的宏(两个转动关节) |

## 2. 实测几何(每次重跑脚本都会核对)

从 `step/gripper.STEP` 量出来、并且和实物照片对过的关键尺寸
(单位 mm,坐标系是 `dummy_tcp`,也就是机械臂法兰面,-z 是工具轴):

| 量 | 值 | 为什么重要 |
|---|---|---|
| 法兰底 | -40.0 | Link_6 网格伸出的高度,夹爪转接板贴在这里 |
| 掌底(电机/转接板最低点) | -123.5 | 工件顶面必须低于它 |
| 指根转轴 | (±35.9, 0, -125.9) | 两片指的转动中心,轴平行于 ±y |
| **指尖深度** | **-204.6** | **最要紧的一个数**:工件是夹在两片指之间的,抓取高度由它定 |
| 张开净距(在 -162.1 深处) | 75.96 | 最大能夹多宽 |
| 闭合速率 | 1.265 mm/° | 指间净距每闭合 1° 少这么多 |
| 关节行程 | 0 … 37.31° | 再合两片指就互相干涉 |
| 可夹工件宽度 | 29 … 76 mm | 由上面两个数推出来 |
| 质量 | 底座 300 g + 每片指 35 g | 惯量按凸包算,已写进 URDF |

## 3. 怎么用

### 3.1 仿真(Gazebo / RViz)

整只夹爪(URDF + 抓取高度 + 控制器)由**一个环境变量**切换:

```bash
JAKA_GRIPPER=1 ros2 launch jaka_competition_kit round.launch.py \
    world:=gazebo track:=1 seed:=246135
```

不设这个变量 = 规则书要求的那种"裸法兰 / 行程几十毫米的小平行夹爪"。

这个变量同时管三处,**不存在只改一半的组合**:

| 被它切换的东西 | 具体位置 |
|---|---|
| URDF 里挂不挂夹爪 | `jaka_minicobo.urdf.xacro` 的 `with_gripper`(读同一个变量) |
| 抓取/转场/投放高度 | `jaka_competition_kit/config/arena.yaml` 的 `tool.gripper` |
| `gripper_controller` 起不起 | `round.launch.py` 的 `use_gripper`(默认跟着变量走) |

仿真里两片指是 `ros2_control` 的 `JointGroupPositionController`(关节名
`gripper_jaw_l_joint` / `gripper_jaw_r_joint`,都约定"正角度 = 闭合"),
想手动试一下:

```bash
ros2 topic pub --once /gripper_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0.36, 0.36]}"   # 0.36 rad ≈ 20.6° ≈ 夹 50mm
ros2 topic pub --once /gripper_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"     # 张开
```

比赛流程里不用手发:`SimGripper` 会按工件实际宽度把角度算出来
(`arena.yaml` 的 `jaw_gap_open` / `jaw_gap_per_deg` / `jaw_limit_deg`)——
真机靠堵转自停,仿真没有这回事,不按尺寸收手指会直接穿过工件。

### 3.2 真机

```bash
# 1) 串口权限(插拔一次即可,规则写的是 /dev/motor_serial)
#    脚本里的 ATTRS{serial}=="56B8006534" 是厂商那根线的序列号:
#    换线要改成自己的(udevadm info -a -n /dev/ttyACM0 | grep serial),
#    或者干脆把那个条件删掉 —— 只按 USB VID/PID 认也行
sudo bash jaka_ros2/src/step_motor/ch9102_udev.sh

# 2) 起驱动
ros2 run step_motor motor_node            # 参数: usart_port_name=/dev/motor_serial
                                          #       serial_baud_rate=115200

# 3) 队伍代码里把夹爪切成真机驱动
ros2 launch jaka_competition_kit round.launch.py ... gripper:=wheeltec
# 或者参考实现单独跑: ros2 run jaka_competition_kit ref_track1 --ros-args -p gripper:=wheeltec
```

协议(厂商《驱控一体步进电机 ROS 控制使用手册》,实现见
`step_motor/src/motor_node.cpp` 与 `jaka_competition_kit/gripper.py` 的
`WheeltecGripper`):

| 项 | 值 |
|---|---|
| 话题 | `/motor_control`(`step_motor/msg/Motor`)→ 电机; `/motor_state` 回读 |
| 帧格式 | `7B` … `7D` + BCC(异或校验),帧长 11 字节 |
| `mode` | 2 = 相对角度模式(本项目只用这个) |
| `angle` | 相对转角,**放大 10 倍**发送(`0.1°` 为单位) |
| `dir` | 0 = 逆时针 = **张开**,1 = 顺时针 = **闭合** |
| `sub_divide` | 32(细分,越大越平滑) |
| `speed` | 放大 10 倍,单位 0.1 rad/s |
| 行程 | 全闭合 5.2 圈 ≈ 1872°;**顶到工件会堵转自停**,所以"发一个够大的角度"就是"夹紧" |
| 保护 | 堵转保护 + 输入反接保护(不支持热插拔) |

`dir` 的方向约定来自实物:闭合 = 顺时针。如果接线后方向反了,把
`WheeltecGripper` 的 `dir` 对调即可,不用改协议。

## 4. ⚠ 可达性:这只夹爪改变了"哪里够得到"

**这是本次集成最需要知道的结论。**

机械臂末端能不能到,取决于一个很简单的几何:工具轴竖直向下时,
`J5` 一定在 TCP 正上方 `wrist_len=159.3mm` 处(推导见
`Arena.max_tcp_z` 的注释,依据是 URDF 里 `joint_6` 的偏移方向与 Link_6 的
`rpy=-90°`),而 `J5` 必须落在以 `J2` 为心、`大臂+小臂=420.5mm` 为半径的球内。于是

```
r² + (z + 159.3 - 187.0)² ≤ 420.5²          (单位 mm)
=>  z_max(r) = 27.7 + sqrt(420.5² - r²)
```

用 40 个随机种子打 `/compute_ik` 实测(每次都是"全中 / 全不中"的硬边界,
和公式完全吻合):

| 半径 r | 公式上限 | 实测 |
|---|---|---|
| 355.0 mm | 253.1 mm | z=248.6 全中 |
| 362.8 mm(料盒中心) | 240.3 mm | z=240 全中 / z=245 全不中 |
| 385.4 mm(工位 4/6) | **195.9 mm** | z=195 全中 / z=200 全不中 |

而换上柔性夹爪之后,`grasp_tcp_z` 从 **100.0mm 抬到 213.6mm**
(工件顶面 54mm + 指尖深度 204.6mm + 刀尖余量 5mm − 工件高 50mm)。
逐点对一下:

| 目标 | r | 需要的 z | 该半径上限 | 结果 |
|---|---|---|---|---|
| 工位 1/2/3 抓取、预抓取 | 205–254 | 213.6 / 248.6 | 362.8 / 394.8 | ✔ |
| 工位 5 抓取、预抓取 | 355.0 | 213.6 / 248.6 | 253.1 | ✔(预抓取只剩 4.5mm) |
| **工位 4/6 抓取** | 385.4 | 213.6 | **195.9** | ✘ 差 17.7mm |
| **工位 4/6 预抓取** | 385.4 | 248.6 | **195.9** | ✘ |
| **料盒中心 转场** | 362.8 | 272.6 | **240.3** | ✘ 差 32.3mm |
| 料盒中心 投放 | 362.8 | 229.6 | 240.3 | ✔ |

随时可以重新打印这张表:

```bash
JAKA_GRIPPER=1 ros2 run jaka_competition_kit arena_check
```

> 2026-09 实测(`JAKA_GRIPPER=1` 跑一整轮赛道一): 工位 1/2/3/5 **都抓到了**
> 工件(夹爪张合、附着都正常), 但抬着工件去料盒(转场 272.6mm)全部规划失败;
> 工位 4/6 连预抓取(248.6mm)都到不了 —— 与上表逐点吻合。所以卡住的不是夹爪,
> 是"抬不够高"。

> 也就是说:用这只夹爪,赛道一的 6 个工位里有 2 个(4 和 6)抓不到,
> 工件也抬不到"越过其它工件"的转场高度。**这不是仿真参数没调好,
> 是 580mm 臂展 + 204.6mm 刀尖的几何结果** —— 真机同样够不到。

### 想让它跑通,可选的路

| 方案 | 说明 | 代价 |
|---|---|---|
| **换回短夹爪**(当前默认) | 规则书写的就是"电动平行夹爪,行程 0–30mm",和裸法兰差不多长 | 不用这只柔性爪 |
| **把工位阵内移** | `arena.yaml` 的 `array_center_r` 从 280 改到 ≈250:远端工位 r 从 385→355,`z_max` 195.9→253.1,抓取就够得着了 | 赛场布局变了,`competition.md` / `赛题细则` / 配图要同步改 |
| **料盒改用内侧投放点** | 判分只要求 TCP 落在料盒内缩 20mm 的范围内(x∈[235,395]、y∈[125,235]mm)。把投放点从中心 (315,180) 挪到 (245,180),r 从 363→304,`z_max` 240→318,转场高度就够得到 | 只解决"放不进料盒",工位 4/6 仍然抓不到 |
| **倾斜手腕下抓** | 绕工具轴先转 90°(让两片指沿切向张合),再向基座方向倾 15–20°:J5 沿刀尖方向内收 `159.3·sinθ`,r=385 处 `z_max` 能从 195.9 提到 240 以上 | 抓取姿态、附着位姿、预抓取点都要重算;两片指不再水平,夹锥形/薄件时接触会变 |

当前仓库采取的是第一条(默认不装柔性爪),柔性爪的模型、驱动、参数都在,
随时可以用 `JAKA_GRIPPER=1` 挂上去做演示或二次开发。

## 5. 常见问题

**Q: `ros2 launch ... round.launch.py gripper:=wheeltec` 之后夹爪不动?**
`motor_node` 起了吗(`ros2 run step_motor motor_node`)?串口开了吗
(`/dev/motor_serial`)?另外 `ref_track1` 的 `gripper` 参数与 `use_gripper`
互相独立:仿真用 `sim`,真机用 `wheeltec`。

**Q: 仿真里手指穿过工件?**
说明 `SimGripper` 不知道工件尺寸(传了 `None`),它会把手指合到底。
检查是不是绕过了 `Executor.grasp()` 直接调了 `gripper.close()`。

**Q: Gazebo 里夹爪是隐形的(但 RViz 里看得见)?**
新加的网格包必须在 `package.xml` 里导出 `<gazebo_ros gazebo_model_path="${prefix}/.."/>`
(`wheeltec_gripper` 与 `jaka_description` 都导出了)。Gazebo 不认 `package://`,
`ros_gz_sim` 把它转成 `model://<包名>/...` 之后只在 `GZ_SIM_RESOURCE_PATH` 里找,
而这条路径**只收声明了该导出的包**。漏掉时的现象是 Gazebo 启动日志里刷
`Unable to find file with URI [model://wheeltec_gripper/meshes/...]`,
**显示网格和碰撞体一起加载失败** —— 夹爪看不见, 而且不参与物理接触
(手指会直接穿过工件)。改了 `package.xml` 要重新 `colcon build` 再起仿真。

**Q: 机械臂带着夹爪"莫名其妙规划失败"?**
先看是哪一类:`JAKA_GRIPPER=1 ros2 run jaka_competition_kit arena_check`
会打印每个目标的可达上限。MoveIt 对"撞了"和"根本到不了"都只回一个
`error_code=99999`,光看返回值分不出来,所以工具包把这条几何算成了公式。

**Q: 夹爪开口是 76mm,能夹 76mm 的工件吗?**
不能夹满。29–76mm 是"指间净距"的范围,工件要留一点缝(仿真里是
`jaw_gap_extra`,默认 4mm),而且指间净距随深度变化(上宽下窄),
梯形/锥形工件会卡在某个深度上 —— 这正是"柔性"爪的自适应夹持方式。

**Q: 厂商资料里那两个夹爪尺寸(98×113×153mm / 72×64×100mm)和这里对不上?**
那是《柔性机械爪开发手册》(舵机版)里另外两个型号的标称外形。
本仓库用的是 `gripper.STEP` 实测值:法兰以下总高 164.6mm、张到最大时
最宽 117.8mm,与随资料附带的 `2.三维模型/gripper.stp` 是同一只夹爪的
不同版本导出。
