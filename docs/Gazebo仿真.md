# Gazebo 仿真线(可选, 用来出转播画面)

初赛判分以 **RViz / MoveIt 规划场景** 为准; Gazebo 是**可选的显示层**。
本文说清楚它现在能做什么、为什么以前看着"空"、以及用它判分必须先做什么。

## 1. 为什么以前的 Gazebo 只有一台悬空的机械臂

`demo_gazebo.launch.py` 的世界是 `empty.sdf` —— 里面只有
`ground_plane` / `sun` / `jaka_minicobo` 三样东西。

赛场(工作台 / 工位板 / 料盒 / 二维码卡 / 工件)只存在于 **MoveIt 的规划场景**
里(`/apply_planning_scene` 下发的 `CollisionObject`)。Gazebo 不认识规划场景,
所以它**不会**渲染这些几何。旧截图 `05-gazebo-model-loaded.png` 里那个
"机械臂悬在一片空地上"的画面就是这么来的 —— 不是 Gazebo 不好看, 是场地
从来没进过 Gazebo。

## 2. 现在: 把规划场景镜像进 Gazebo

```
gz_scene(节点)  --轮询-->  /get_planning_scene
                --差分-->  /world/<world>/{create,set_pose,remove}
```

- **代码**: `jaka_competition_kit/jaka_competition_kit/gz_scene.py`
- **launch**: `jaka_competition_kit/launch/gazebo_mirror.launch.py`
- **一键**: `ros2 launch jaka_competition_kit round.launch.py world:=gazebo`

镜像的行为:

| 规划场景变化 | Gazebo 动作 |
|---|---|
| 多了个碰撞体 | `create`(带位姿) |
| 位姿变了 | `set_pose` |
| 没了 | `remove` |
| 几何形状变了 | 删掉重建( Gazebo 改不了几何) |

工件被 `attach` 到末端时, 它在规划场景里的位姿是**相对末端连杆**的,
镜像节点用 TF 换算到 `world` 再下发 —— 所以 Gazebo 里能看到工件跟着
夹爪走。

### 2.1 三个必须知道的实现细节

1. **数据来源是 `/get_planning_scene` 服务轮询, 不是
   `/monitored_planning_scene` 话题。** 后者是 VOLATILE 的, 晚启动的节点
   收不到已有场景, 实测表现是"镜像一个模型都不生成"。轮询还天然避开了
   "话题发的是全量还是增量 diff"的歧义。
2. **Gazebo 的 create/remove/set_pose 默认不是 ROS 服务**, 需要
   `ros_gz_bridge parameter_bridge` 显式桥接。`gazebo_mirror.launch.py`
   已经把这三个桥好了。
3. **launch 参数叫 `gz_world`, 不能叫 `world`。**
   `IncludeLaunchDescription` 会**继承父级的 launch configuration**, 而
   `round.launch.py` 已经用 `world` 表示"rviz 还是 gazebo"。同名的话
   `world:=gazebo` 会被继承进来, 服务名变成 `/world/gazebo/create`
   (不存在), 一个模型都建不出来 —— 这个坑实测踩过。

### 2.2 地面会被自动挪到场地下面

赛场坐标里 `z=0` 是**基座安装面**, 台面顶面在 `z=-1mm`, 也就是整个台面
在 Gazebo 自带地面**以下**。不处理的话 Gazebo 里只看得见地面, 看不见台面。

镜像节点会按当前场景里最低的那个几何(`pose.z - 半高`)把 `ground_plane`
挪下去(`align_ground`, 默认开)。

### 2.3 默认不参与物理

Gazebo 里的这些几何默认 `collision=false`, 只是视觉。原因是它们是
"跟着规划场景走的静态模型", 如果参与物理, 机械臂会被它们顶住、轨迹跟踪
变差(实测会报 `末端在 3.0s 内没有到位`)。

要物理接触就 `collision:=true`, 但请自己确认轨迹还跟得上。

## 3. 用 Gazebo 判分之前必须重新标定限时

**这是最容易出事的一点。**

同一套参考实现, 两条线的耗时差**接近一倍**:

| 线 | 控制器 | 赛道一 6 件耗时 |
|---|---|---|
| RViz(假硬件) | mock 硬件, 瞬时执行 | **102.7 s**(中位数) |
| Gazebo(真物理) | `gz_ros2_control` + 真物理 | **209.4 s**(单次实测, 6/6) |

原因: 假硬件模式下轨迹是"瞬间到位", Gazebo 里关节要按真实速度走完,
还要等物理收敛。

按赛题现在的限时(赛道一 145 s), 用 Gazebo 跑参考实现只会拿到
"超时, 计 4/6"。所以:

- **判分用哪条线, 限时就按哪条线标定**;
- 两条线不能混用同一个限时, 否则对选手不公平;
- 换线之后重跑 `bash design/calibrate.sh <赛道> 3`。

## 4. 这台机器上的渲染质量(重要)

本机 **没有 GPU**, Gazebo 走的是 Mesa **llvmpipe 软件渲染**
(`~/.ignition/rendering/ogre2.log`: `GL_RENDERER = llvmpipe`)。在这个环境下:

- 机械臂本体(URDF 网格)和地面渲染正常;
- 运行时 **spawn 进来的几何(台面/工位板/料盒/工件)会出现竖条纹瑕疵**
  —— 已排除是抓图工具的问题(条纹周期约 9px, 是渲染出来的),
  也排除了阴影(`cast_shadows=false`、关掉太阳投影都没用)。

有独显的机器上不会这样。**如果比赛要投大屏, 请用带 GPU 的机器跑 Gazebo。**
截图见 `verification/screenshots/08/09`。

## 5. 常用命令

```bash
# 一键: 真物理 + 赛场镜像 + 判分侧
ros2 launch jaka_competition_kit round.launch.py track:=1 seed:=246135 world:=gazebo

# 或者: 正在跑 demo_gazebo / round.launch.py 时, 单独补一个镜像
ros2 launch jaka_competition_kit gazebo_mirror.launch.py

# 参数
#   gz_world     Gazebo world 名, 默认 empty(对应 empty.sdf)
#   collision    场地几何是否参与物理, 默认 false
#   update_rate  轮询频率, 默认 10 Hz
#   entity_prefix 给 Gazebo 实体名加前缀, 默认空
```

调整 Gazebo 视角(转播机位):

```bash
ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera \
  --reptype ignition.msgs.Boolean --timeout 3000 \
  --req 'pose: {position: {x: 0.82, y: -0.70, z: 0.52},
                orientation: {x: -0.170161, y: 0.086650, z: 0.874718, w: 0.445428}}'
```

## 6. 建议

- **判分 / 成绩**: 用 RViz(MoveIt 规划场景是唯一真源, 限时也是按它标的)。
- **转播 / 观众看**: 用 Gazebo, 画面有真实感、有关节运动、有影子。
- 两者可以同时开: `round.launch.py world:=gazebo` 本身就会把两个都起起来。
