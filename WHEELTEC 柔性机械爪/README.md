# WHEELTEC 柔性机械爪 —— 厂商原始资料索引

这是随夹爪一起拿到的厂商资料, 原样放在这里(约 547 MB, 大头是视频和 Windows
上位机)。**真正被项目用到的部分已经抽成 ROS 2 包放进 `jaka_ros2/src/`**,
这份目录只是"来源和依据"。

## 三个型号,别搞混

| 子目录 | 是什么 | 本项目 |
|---|---|---|
| `MS42DC步进电机版柔性机械爪用户资料_V3.0.-20251031/` | **当前用的这只**:驱控一体步进电机,USB 串口/CAN/TTL | ✅ 驱动在 `jaka_ros2/src/step_motor/` |
| `WHEELTEC_睿尔曼柔性机械爪资料资料_2025.04.15/` | 适配睿尔曼(RM)机械臂的版本,含同一只爪的 `2.三维模型/gripper.stp` | 只作建模参照 |
| `舵机版柔性机械爪用户资料/` | 老款舵机版(点位/按键控制),不是本项目这只 | 不用 |

## 有用的文件在哪

| 文件 | 用途 |
|---|---|
| `MS42DC.../1.用户教程/MS42DC步进电机机械爪用户手册-20260324.pdf` | 电控接口、性能参数、堵转保护、烧录 |
| `MS42DC.../5.ROS例程与教程/驱控一体步进电机ROS控制使用手册.pdf` | `/motor_control` 话题与帧格式(本项目驱动照它实现) |
| `MS42DC.../5.ROS例程与教程/源码/ROS2.zip` | 厂商 ROS 2 例程源码(`step_motor` + `serial` + 键盘控制) |
| `MS42DC.../3.程序固件/` | 电机固件源码(闭环圈数、电流等参数) |
| `MS42DC.../2.软件与工具/WHEELTEC串口调试助手.exe` | Windows 上位机,标定/试爪用 |
| `WHEELTEC_睿尔曼.../2.三维模型/gripper.stp` | 同一只爪的另一版 CAD 导出(与 `gripper.STEP` 零件一致) |
| `舵机版柔性机械爪用户资料/柔性机械爪开发手册.pdf` | 夹爪原理与"柔性"夹持方式的说明 |

## 被抽进项目的东西

```
jaka_ros2/src/wheeltec_gripper/step/gripper.STEP   ← 用户提供的三维图(1.1 MB)
jaka_ros2/src/wheeltec_gripper/meshes/             ← 由它转出的 STL(见 tools/step_to_mesh.py)
jaka_ros2/src/wheeltec_gripper/config/gripper.yaml ← 实测几何(指尖深度/转轴/开口/惯量)
jaka_ros2/src/wheeltec_gripper/urdf/               ← 装到机械臂末端的 URDF 宏
jaka_ros2/src/step_motor/                          ← 厂商 ROS 2 驱动(由上面 ROS2.zip 抽出)
jaka_ros2/src/serial_ros2/                         ← 驱动依赖的串口库(包名是 serial)
```

**整体怎么用、可达性有什么坑,看 `docs/WHEELTEC柔性机械爪.md`。**

## 体积

`*.mp4`、`*.exe`、`*.7z`、`*.rar` 等大二进制由本目录的 `.gitignore` 排除,
不进版本库(原理图、手册、源码 zip 这些小文件照旧提交)。
