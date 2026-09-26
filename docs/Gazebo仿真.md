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

## 4. 画面: 独显是有的, 但远程桌面里用不上(重要)

先把结论摆清楚, 因为这一段踩过坑、也写错过:

| 问题 | 答案 |
|---|---|
| 这台机器有没有独显 | **有**。NVIDIA RTX 5060 Ti, 驱动 580.178.04, 16311 MiB |
| 默认的远程桌面会话(`:1001`)能不能用它渲染 | **不能**, 只能走 Mesa llvmpipe 软渲染 |
| 另起一个无头 Xorg 能不能用 | **能**, `bash setup/xorg-gpu.sh start` 一条命令 |

### 4.1 为什么默认会话用不上独显

`:1001` 不是 Xorg。`lsof /tmp/.X11-unix/X1001` 会告诉你, 那个 socket 属于
**`nxnode.bin`** —— NoMachine 自己实现的虚拟 X server。NVIDIA 的 GLX
客户端库没法往这种 X server 上呈现画面。拿 `glxgears` 做对照, 同一台机器:

```bash
DISPLAY=:1001 glxgears                                   # -> 正常画出齿轮
DISPLAY=:1001 __GLX_VENDOR_LIBRARY_NAME=nvidia glxgears  # -> 窗口全黑
```

加了这个变量之后 `glxinfo` 确实会改口报 `NVIDIA GeForce RTX 5060 Ti`,
`~/.ignition/rendering/ogre2.log` 里也会写 `GL_VENDOR = NVIDIA Corporation`,
**但 Gazebo 的 3D 视口是全黑的** —— 渲染结果根本没送到屏幕上。

所以: **不要在远程桌面会话里设 `__GLX_VENDOR_LIBRARY_NAME=nvidia`**。
这个坑一度被当成"修复"写进 `README` 和 launch 文件里, 2026-09 已全部撤掉。

### 4.2 想真的用上独显

GPU 上没接显示器(`nvidia-smi -q` 显示 `Display Attached: No`)也能起 Xorg,
靠 `AllowEmptyInitialConfiguration` + `UseDisplayDevice "none"` 走 NoScanout:

```bash
bash setup/xorg-gpu.sh start :0
DISPLAY=:0 glxinfo -B | grep -E "renderer|direct"
# OpenGL renderer string: NVIDIA GeForce RTX 5060 Ti/PCIe/SSE2
# direct rendering: Yes

DISPLAY=:0 ros2 launch jaka_competition_kit round.launch.py world:=gazebo

bash setup/xorg-gpu.sh stop :0
```

两个注意点:

- `:0` 上**没有窗口管理器**, 新窗口不会自动置顶。X 不保存被遮挡的像素,
  `xwd` / `import -window` 抓到的是"屏幕上那块区域", 所以抓图前要先
  `xdotool windowraise <wid>` 把它抬上来, 否则抓到的是盖在它上面的 RViz;
- 反过来, `:1001` 上 Gazebo 的 Ogre 视口在这台机器上会**只画一帧就不重绘**
  (Qt 面板照常刷新, 3D 区冻住)。要拍 Gazebo 的画面, 用 `:0`。

第二条 2026-09 用屏幕级抓帧量化确认过, 不是抓图工具的假象: 机械臂在动、
`/joint_states` 在变的情况下, 直接抓 `:1001` 上 Gazebo 窗口的区域, 8 秒里
只差 ~300 像素(同期 RViz 是 ~15000 像素, 转发窗口是 ~6200 像素)。换 Ogre1
引擎(`--render-engine-gui ogre`)更糟 —— 在 `:1001` 上连窗口都开不出来。

### 4.2.1 在远程桌面里看 `:0` 上的 Gazebo(两种办法)

`:1001` 上 3D 区不重绘、`:0` 又不在 NoMachine 里 —— 先把 Gazebo 线起在 `:0`
(GUI 和 server 都在 `:0`, 独显负责渲染):

```bash
bash setup/xorg-gpu.sh start :0
DISPLAY=:0 ros2 launch jaka_competition_kit round.launch.py world:=gazebo \
    track:=1 seed:=246135 vel_scale:=0.5 run_reference:=false

# 出题 + 发令(DISPLAY 无所谓)
ros2 service call /competition/generate std_srvs/srv/Trigger "{}"
ros2 service call /competition/start    std_srvs/srv/Trigger "{}"
```

然后选一种把画面搬到你面前:

**(A) VNC —— 能拖拽、能操作, 推荐**

```bash
bash setup/xorg-viewer.sh start      # 起 VNC 服务 + 打开查看窗口
bash setup/xorg-viewer.sh stop       # 用完关掉
```

- 服务端只听 `127.0.0.1`(`-localhost`), 不对外开端口; 本地回环连接不需要密码;
- 窗口里就是 `:0` 的桌面, 左键拖=转视角, 中键拖=平移, 滚轮=缩放;
- **拖不动 / 转不了视角时先看这条**(2026-09 实测过一次): 先分清是 Gazebo 没反应,
  还是**事件根本没传到 `:0`**。判据: 指针在查看窗口里移来移去, 而
  `DISPLAY=:0 xdotool getmouselocation` 读到的 `:0` 指针**纹丝不动** —— 那就是查看
  窗口没转发输入, 跟 Gazebo 无关(当时同时验证: 直接在 `:0` 上合成一次拖拽, 视角是
  转得动的)。处理顺序: 先在查看窗口内部点一下让它拿到焦点, 再拖; 还不行就重开一个
  查看窗口 —— `bash setup/xorg-viewer.sh view`(实测重开后立刻恢复转发)。
  顺带: 滚轮缩放是**围着鼠标光标**缩的, 光标不在赛场上时连缩几下镜头就飘进地面里,
  画面只剩一片灰, 看起来更像"卡死"; 用 `bash setup/xorg-viewer.sh cam table` 复位机位;
- **`start` 会顺手把 Gazebo 铺满 `:0` 并置顶**(见下), 没赶上就补一条
  `bash setup/xorg-viewer.sh fill`;
- 实测(x11vnc 0.9.16 + gvncviewer 1.3.0, 1920x1080): 两端各占 5~6% CPU,
  画面流畅可交互; 拖拽转视角后 `:0` 侧的 Gazebo 画面确实跟着变;
- 本机没装这两个包时脚本会自己 `apt-get download` 解到 `~/.local/opt/vnc` ——
  **不需要 sudo**。想装进系统就 `sudo apt install x11vnc tigervnc-viewer`;
- **卡不卡取决于最后一跳, 不取决于独显**(2026-09 实测): Gazebo 侧确实在独显上
  跑(`nvidia-smi` 里 `ign gazebo gui` 是 GPU 进程、304MiB 显存, ogre2.log 是
  NVIDIA, 仿真实时率 100%), 但 `:1001` 本身是 NoMachine 的虚拟 X server, 独显
  接不上 —— VNC 解码、重绘、gnome-shell 合成、NoMachine 再编码给你本地客户端,
  这几步全是 CPU。拖动实测: x11vnc 4%、gvncviewer 2%、gnome-shell 36%、
  转发帧率 ≥19fps, 本机这头没有瓶颈, 所以卡在 NoMachine 那条链路上。
  想更顺就少传像素: `ZOOM=50 bash setup/xorg-viewer.sh start`(窗口边长减半,
  要传的像素少到 1/4), 或者别把 Gazebo 铺满(1280x720 就够看);
- 另外一个"白烧"的地方: GUI 静止时也吃 ~191% CPU —— 无头 Xorg 是
  `UseDisplayDevice none`, 没有垂直同步, 帧率无上限(窗口 1280x720 和
  1920x1080 实测都是 191%)。用 `__GL_SYNC_TO_VBLANK=1` 限帧会掉到 2%,
  但 GUI 就不再出画面了(显存 304MiB→2MiB、渲染日志停更), 所以不能用;
- 踩过的坑: 本机环境里 `WAYLAND_DISPLAY=no`, x11vnc 会把它当成 Wayland 会话
  直接 `Wayland display server detected. Exiting.` —— 脚本里用 `env -u` 去掉了。

> **"Gazebo 黑屏"是怎么来的(2026-09 复现确认)**: `:0` 上没有窗口管理器, 于是
> 1) 后启动的 RViz 和 Gazebo 都落在 `(0,0)`, 没人管叠放顺序, 1200x975 的 RViz
> 把 1000x845 的 Gazebo **整个盖住**; 2) 根窗口是黑的, 窗口又只占屏幕一角, 剩下
> 一大片就是纯黑。所以在 VNC 窗口里看到的是"一个 RViz + 一片黑", 很容易当成
> Gazebo 挂了 —— 其实 Gazebo 在底下好好跑着。
> 解法就是把 Gazebo 拉到 `(0,0)` 铺满并置顶(脚本已自动做):
>
> ```bash
> bash setup/xorg-viewer.sh fill
> export DISPLAY=:0 && pkill -f moveit.rviz   # 可选: :0 上的 RViz 看不见, 还抢位置
> ```

**(B) 只读转发 —— 不装任何东西**

```bash
python3 setup/gazebo-viewer.py --from :0 --title Gazebo --crop 578x797+0+48
```

- 每帧 `xwd` 抓一次贴到窗口里, 只读, 不动仿真、不进判分链路; 1000x845 的
  Gazebo 窗口约 5 fps, 占用 10% CPU 左右;
- 帧率 `--fps`, 大小 `--scale`, `--crop WxH+X+Y` 用来裁掉右侧属性面板;
- 只依赖 `xwd`、`xdotool`、PIL、tkinter(这台机器都有, 不用装 ffmpeg);
- 抓图前会把目标窗口置顶(`:0` 没有窗口管理器, 不置顶抓到的是盖在它上面的
  RViz), 不想动叠放顺序加 `--no-raise`。

两种办法都只是"看", 机位都还是用第 5 节那条 `ign service /gui/move_to/pose`
调(`:0` 上的 GUI 一样吃)。

### 4.3 曾经被当成"软渲染瑕疵"的那层竖条纹

`verification/screenshots/08/09` 的早期版本里, 运行时 spawn 的几何
(台面 / 工位板 / 料盒 / 工件) 上罩着一层很明显的竖条纹, 当时归因成
"没有独显 / llvmpipe 画质差"。

**不是渲染的问题, 是抓图工具的 bug。** `verification/xwd2png.py` 原先按
`bytes_per_line` 去猜"每像素几字节", 而 xwd 写出来的这两个字段可以不一致 ——
本机抓 1000 像素宽的窗口时是 `bits_per_pixel=24` 但 `bytes_per_line=4000`,
于是它按 4 字节去读一份 3 字节的数据, 整张图被横向错位, 就长出了条纹。
同一份 `.xwd` 用 ImageMagick 解出来是**干净的**, 在软渲染和独显上都是。

现在 `xwd2png.py` 改成按 `bits_per_pixel` 取宽度、并从文件尾倒推像素起点
(跳过 xwd 写在像素前的调色板段)。不放心的话绕开它直接用 ImageMagick:

```bash
import -window <wid> shot.png
```

顺带纠正另一个结论: **软渲染下 Gazebo 的画面本身是正常的**, 代价只是 CPU
(`ign gazebo gui` 会吃掉好几个核)。以前说"投大屏要换带独显的机器"是错的,
在这台机器上要的是按 4.2 起一个 Xorg。

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

**相机跑飞了(画面只剩一片灰)最省事的救法** —— 不用算四元数, 直接把镜头对到
某个模型上:

```bash
ign service -s /gui/move_to --reqtype ignition.msgs.StringMsg \
  --reptype ignition.msgs.Boolean --timeout 3000 --req 'data: "table"'
```

实测踩过的坑: Gazebo 的滚轮缩放是**围着鼠标光标**缩的。光标没落在赛场上时,
连缩几下镜头就一路飘到地面里去了, 看起来像"画面卡住不动"(其实是全屏都是地面)。
要么先用中键把赛场拖到视口中央再缩, 要么直接跑上面那条 `move_to` 复位。

## 6. 建议

- **判分 / 成绩**: 用 RViz(MoveIt 规划场景是唯一真源, 限时也是按它标的)。
- **转播 / 观众看**: 用 Gazebo, 画面有真实感、有关节运动、有影子。
- 两者可以同时开: `round.launch.py world:=gazebo` 本身就会把两个都起起来。
