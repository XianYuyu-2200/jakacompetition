# JAKA Mini 2 参数速查与 URDF 对照

## 1. 官方参数(Mini 2 vs MiniCobo)

来源:`JAKA Mini Series Hardware User Manual` V03(2025-08-15, 见同目录 PDF)
Table 5-1 Robot technical specifications。

| 项目 | JAKA MiniCobo | JAKA Mini 2 |
|---|---|---|
| 负载(Payload) | 1 kg (2.2 lb) | 2 kg (4.4 lb) |
| 重量(含线缆) | 9.4 kg | 9.9 kg |
| 臂展(Reach) | 580 mm (22.8 in) | 580 mm |
| 重复定位精度 | ±0.1 mm | ±0.1 mm |
| 自由度 | 6 | 6 |
| 工具典型速度 | 1 m/s | 1 m/s |
| 平均功耗 | 150 W | 180 W |
| 防护等级 | IP40 | IP40 |
| 基座直径 | 124 mm | 124 mm |
| 关节 1 | ±360° | ±360° |
| 关节 2 | ±125° | ±125° |
| 关节 3 | ±130° | ±130° |
| 关节 4 | ±360° | ±360° |
| 关节 5 | ±120° | ±120° |
| 关节 6 | ±360° | ±360° |
| 控制柜 | MiniCab(24 V DC) | MiniCab(48 V DC) |
| 电源适配器 | GST280A24-C6P(24 V) | HRP-300N3-48(48 V) |

要点:**MiniCobo 与 Mini 2 的运动学一致**(同为 6 轴、580 mm 臂展、相同关节范围),
差别只在负载、重量、功耗和供电。所以 ROS 包里的同一份 URDF 对两者都适用。

## 2. ROS 包内 URDF 实测值

文件:`jaka_ros2/src/jaka_description/urdf/jaka_minicobo.urdf`
关节顺序:`world -> Link_0 -> joint_1..joint_6 -> Link_6 -> dummy_tcp`

| 量 | URDF 值 | 说明 |
|---|---|---|
| 肩高(joint_1 原点 z) | 187.0 mm | 基座安装面到 J2 轴线 |
| 大臂(joint_3 偏移) | 210.0 mm | |
| 小臂(joint_4 偏移) | 210.5 mm | |
| 腕部(joint_6 偏移) | 159.3 mm | |
| 三连杆和 | 579.8 mm | ≈ 官方 580 mm 臂展 ✔ |
| 零位 TCP | [0, -6, 766.8] mm | 手臂完全伸直的姿态 |
| 总质量(各连杆和) | 9.355 kg | 接近 MiniCobo 的 9.4 kg |

关节限位对照:

| 关节 | URDF (rad) | URDF (°) | 官方 | 结论 |
|---|---|---|---|---|
| joint_1 | ±6.28 | ±359.8 | ±360 | ✔ |
| joint_2 | ±2.09 | ±119.7 | ±125 | ✘ 比官方小约 5° |
| joint_3 | ±2.27 | ±130.1 | ±130 | ✔ |
| joint_4 | ±6.28 | ±359.8 | ±360 | ✔ |
| joint_5 | ±2.09 | ±119.7 | ±120 | ✔ |
| joint_6 | ±6.28 | ±359.8 | ±360 | ✔ |

其他:各关节最大速度 1.57 rad/s(90°/s);关节 effort 限位统一写 100(占位值, 无实际意义)。

## 3. 需要注意的地方

- URDF 内 **没有负载(payload)字段**。做抓取类任务时, 负载要自己在
  MoveIt 的 SRDF/规划场景里加, 或者用 `AttachedCollisionObject`。
- `joint_2` 行程比官方手册小 5°, 如果比赛动作需要贴 `±125°` 边界,
  要么改 URDF 的 `limit`, 要么在赛前留出余量。改 URDF 后记得
  `colcon build --symlink-install --packages-select jaka_description`。
- URDF 总质量 9.355 kg 更接近 MiniCobo(9.4 kg)而不是 Mini 2(9.9 kg)。
  纯运动学仿真无影响; 若要做力矩/动力学比对, 需要用实测惯量替换。
- 真机负载上限以**控制器里配置的型号**为准(Mini 2 = 2 kg)。

## 4. 资料来源

- `docs/JAKA-Mini系列-硬件用户手册-V03-EN.pdf` — 官方硬件手册(110 页),
  含技术参数、SDK 说明、安全限值、电源规格。抓取自 jaka.com 官方下载页。
- `docs/JAKA-产品选型手册-2026.pdf` — 全系选型手册(图片版 PDF)。
- `docs/JAKA-Mini系列-安全说明-多语言.pdf` — 安全警示(安装/断电开关高度等)。
- `jaka_ros2/jaka_ros2_documentation-中文版.md` — ROS 2 包官方中文文档。
