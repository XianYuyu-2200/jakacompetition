# 腕部 Gemini 305 仿真设计

记于 2026-10-19。相机已经装进 URDF，并用 Gazebo 预览看过一帧。现行赛场数字以 `jaka_competition_kit/config/arena.yaml` 为准。

厂家页：<https://www.orbbec.com/gemini-305/>。下面尺寸和视场角从该页规格表摘出，用的是 **Gemini 305**（USB），不是 Gemini 305g。

## 结论

在 `dummy_tcp` 上、法兰的 **−Y 侧**固定一台 Gemini 305。工具朝下时，这一侧是世界 +Y，离开基座、朝散放区远端。光轴与工具轴平行，朝工具 +Z（抓住时竖直向下）。相机不是 TCP，不改 `wrist_len`，不改抓取高度。

仿真里用 Gazebo Fortress（本机 `libignition-gazebo6` 6.18.0）的一只 `rgbd_camera` 近似对齐后的深度和彩色。不模拟 18 mm 基线的双目视差，也不接 Orbbec SDK。

## 用哪一款

| | Gemini 305（采用） | Gemini 305g（不采用） |
|---|---|---|
| 用途 | 腕部，USB Type-C | 腕部，另加 GMSL2 |
| 外形 | 42 × 42 × 23 mm | 42 × 42 × 40 mm |
| 重量 | 68 g ± 10 g | 116 g ± 10 g |
| 安装孔 | 底面 1×1/4-20 UNC + 2×M3；背面 2×M3 | 底面孔更多 |

深度是双目，基线 18 mm。工作距离 4 cm 到 100 cm 以上，理想距离 7–50 cm。对齐后的深度/彩色视场是 **H 88° × V 65° ± 3°**（1280×800，在 20 cm 处标的深度视场）。彩色全画幅更宽（H 94° × V 68°），对齐输出不要用这个宽视场。

## 装在哪

坐标都在 `dummy_tcp` 里。这个坐标系原点在法兰贴合面，**+Z 朝外**（工具朝下夹时指向台面），Link_6 的材料在 −Z 一侧。

网格实测：

- Link_6 包络大约 x ∈ [−39, 38] mm，y ∈ [−37, 37] mm，z ∈ [−39, 0] mm。法兰半径约 38 mm。
- 柔性爪底座（装上时）x ∈ [−40, 40] mm，y ∈ [−31.5, 31.5] mm，z ∈ [0, 83.5] mm。两指绕 ±Y 在 ±X 侧开合，指厚只有约 ±11 mm（沿 Y）。

所以相机不能放在 ±X：那是手指扫掠的方向。±Y 的间隙一样。2026-10-19 从靠基座的 +Y 改到 −Y，对着 Gazebo 里手腕的另一侧。

相机外壳中心相对 `dummy_tcp`：

```text
x = 0
y = -0.0615 m
z = -0.0095 m
rpy = 0 0 π
```

这样 42×42×23 mm 的盒子是：

- 沿 Y：−82.5–−40.5 mm。离法兰（−37.5 mm）留 3 mm，离爪底座（−31.5 mm）留 9 mm。
- 沿 Z：−21–+2 mm。镜头所在的前面比法兰面探出 2 mm，大部分壳体缩在法兰侧面，不伸进手指那一截。
- 沿 X：±21 mm，落在法兰宽度里面，但不和法兰重叠。

```text
                 工具 +Z（朝台面）
                      |
        指 (−X)       |        指 (+X)
                      |
                      |   法兰
                      |
  相机中心在 −Y = −61.5 mm ----+
```

不装夹爪时位置不变，父连杆仍然是 `dummy_tcp`。质量按 68 g。碰撞体用这个盒子，不要用相机网格去和手指做精细碰撞。

MoveIt 里豁免 `gemini305_link` 与 `Link_6`（相邻）。与 `gripper_base` 留了 9 mm，先不豁免；规划垫片如果误报再加。不要放进 `touch_links`：工件不该碰到相机。

## 坐标系

`gemini305_link` 的轴直接用 ROS 光学坐标，不再套 RealSense 那种 `rpy="-π/2 0 -π/2"`：

- +Z 光轴，与 `dummy_tcp` +Z 同向
- +X 图像向右，与 `dummy_tcp` −X 同向（绕工具 Z 转了 180°）
- +Y 图像向下，与 `dummy_tcp` −Y 同向

`gemini305_link` 就是 `gemini305_color_optical_frame`。深度和彩色对齐，共用这一帧。真机以后用 Orbbec SDK 时也发到这个帧名，感知节点不区分仿真和真机。

现有抓取姿态 `tool_down_quat: [1, 0, 0, 0]`（xyzw，绕 X 转 180°）把工具 +Z 转到世界 −Z，工具 +Y 转到世界 −Y。相机在工具 −Y，所以外壳落到世界 +Y。识别位姿必须沿用这个姿态，**不要再转 joint 6**。转 90° 相机会甩到手指那一侧，既挡画面，也和上面的间隙假设相反。

## 仿真传感器

传感器写在 link 的 `<gazebo>` 里，只有 Gazebo 会用。RViz 假硬件没有图像。

Ignition Fortress 的传感器写在该 link 的 `<gazebo>` 里，类型 `rgbd_camera`。水平视场取 88° = 1.53589 rad，分辨率默认 **640×400 @ 15 Hz**（1280×800 的一半，宽高比不变）。Gazebo 用宽高比推出的垂直视场约 62.2°，落在标称 65°±3° 里面，不再改宽高比去凑 65°。

近裁剪 0.04 m（最近工作距离），远裁剪 1.20 m（盖住 100 cm 的工作距离并留一点余量）。

默认不用 1280×800 @ 30 Hz。这场仿真还要跑 MoveIt，全分辨率的 RGBD 太重。要和真机分辨率对齐时再开，不作为默认。

话题（`ros_gz_bridge`，Gazebo → ROS，只在 `world:=gazebo` 时起）：

| Gazebo | ROS | 类型 |
|---|---|---|
| `/gemini305/image` | `/gemini305/color/image_raw` | `sensor_msgs/Image` |
| `/gemini305/depth_image` | `/gemini305/depth/image_raw` | `sensor_msgs/Image` |
| `/gemini305/points` | `/gemini305/points` | `sensor_msgs/PointCloud2` |
| `/gemini305/camera_info` | `/gemini305/color/camera_info` | `sensor_msgs/CameraInfo` |

深度是光轴上的理想射线深度，没有双目空洞、没有 18 mm 基线、没有厂家噪声。手指会进画面，但在上沿附近，不盖住中心：爪底座在光学 −Y（画面上方）约 30 mm，伸到 z≈80 mm 时离光轴约 19°，垂直半视场约 31°，所以指尖在画面上侧，散放区仍在中下部。检测框照旧不是抓取点。

## 识别时看不看得全散放区

散放区 x ∈ [−190, 190] mm，y ∈ [190, 365] mm，中心 (0, 277.5) mm。

法兰放在散放区中心、工具朝下时，相机光轴打在台面上的点是 (0, 339) mm（沿世界 +Y 偏 61.5 mm）。散放区离这个点最远的边是 y=190 mm，距离 149 mm。

640×400、水平 88° 时，垂直半幅约 `h × 0.604`。要包住 149 mm，相机离台面至少 **247 mm**。水平方向更宽，不是限制项。

识别高度的几何目标仍是相机离台面 **320 mm**（法兰几乎同高，只差相机中心那 9.5 mm）。竖直朝下、法兰在散放区中心时，0.32 m 会顶到 `joint_5` 限位（URDF ±2.09 rad）。预览因此用法兰高度 **0.28 m**，关节角在 `config/gemini305_preview_joints.yaml`。这一帧里五件工件都在画面里，手指在画面上侧。0.32 m 那条仍然是几何目标，不是已经解出来的关节角。

按 0.32 m 算时：

- 理想量程 7–50 cm 之内。
- 视场在台面上大约 ±309 mm（X）× 从光轴算 ±193 mm（Y），散放区整块在里面。最紧的是靠基座那条边（世界 −Y），余量约 44 mm；沿 X 余量约 119 mm。
- 40 mm 高的工件顶面离相机约 280 mm，远于 40 mm 近裁剪。
- 指尖在法兰前方 165 mm，法兰在 320 mm 时指尖离台约 155 mm，不会戳到工件。这只说明这个高度几何上可行，不是已经标定的关节角。

`home` 的 z=220 mm 不够看全，不要拿 home 当识别位姿。

## 已经装进模型

1. `jaka_description/urdf/gemini305.urdf.xacro` 固定挂在 `dummy_tcp` 上，不随夹爪开关。数字只读 `jaka_competition_kit/config/gemini305.yaml`。
2. SRDF 豁免了 `gemini305_link` 与 `Link_6`、`dummy_tcp`。
3. 预览世界 `jaka_competition_kit/worlds/gemini305_preview.sdf` 带传感器插件，场上是基础组 6 个几何体。`empty.sdf` 没有传感器插件，正式比赛那场 Gazebo 还不会出图。
4. 2026-10-19 改到 −Y 后又跑了一帧。法兰在 (0, 0.2775, 0.28)，工具朝下。相机在世界 (0, 0.339, 0.289)，也就是离开基座的那一侧。腕部彩色图里五件都在，两指仍在画面上侧，画面下沿是散放区远端外面的空地。图在 `design/gemini305_preview/`。`mount.png` 还是改位之前的近景，不要拿它当现在的安装。
5. 预览世界里原来的五块色块换成了基础组 6 个几何体（正方体、长方体、圆柱、六角柱、三角柱、圆台），摆在腕部画面中部两排。工位近排 y=205 mm 会顶到手指，所以这里不放在工位坐标上。识别节点 `track1_recognize` 只看轮廓和深度，不看颜色。2026-10-19 这场预览里 6 个都认对，位置也对上（6/6）。结果图在 `design/track1_recognize/result.png`，JSON 在同目录 `detections.json`，话题是 `/track1/detections`。六角柱、三角柱、圆台不用 OBJ：Fortress 会吃掉网格自带材质，SDF 颜色上不去，画面是白的。改成同外形的 STL，颜色跟方块、圆柱一样由 SDF 的 diffuse 决定。分类规则没改。

再看这一帧：

```bash
source /opt/ros/humble/setup.bash
source jaka_ros2/install/setup.bash
ros2 launch jaka_competition_kit gemini305_preview.launch.py
```

Gazebo 窗口留着。彩色图、深度图写到 `design/gemini305_preview/`。停掉用 `bash setup/sim-clean.sh`。

还没做的是把这场桥接接进 `round.launch.py world:=gazebo`。那条线现在仍打开 `empty.sdf`，而且真正被调用的是 `/opt/ros` 里的 `moveit_configs_utils/launches.py`，要靠 `setup/patch-moveit-launches.sh` 才会换成仓库里的 `jaka_ros2/launches.py`。

MuJoCo 的 `mujoco/jaka_mini2` 不在这次范围内。

## 明确不做

- 不把相机当 TCP，不改可达带 180–420 mm，不改柔性爪的 `tip_depth`。
- 不用 Gemini 305g 的 40 mm 厚度和 116 g。若实物换成 305g，只加长壳体沿 −Z 的尺寸，Y 向安装位置不用动。
- 不在仿真里复现双目匹配。赛道二那 20 种日用品的 YOLO 仍要在真机、这只相机、这个识别位姿下采集，仿真里的盒子不能代替那批数据。基础组 6 个几何体可以在这场预览里练习采集和训练，见 `perception/track1_sim/README.md`。那是学校没有设备时的练习，不是决赛权重。
