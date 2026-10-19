# 没有设备时：在仿真里练采集、训练、抓取

给学校里没有机械臂、没有相机、也没有工件的参赛队。这是练习，不是决赛提交的模型。

规则仍以 `competition.md` 为准。赛道一正式比赛**不要求认类别**，按二维码顺序抓固定工位。这个练习把两件事放在一起：腕部相机读二维码决定抓取顺序，YOLO 加顶面深度算出每件的实际位置，再放进料盒。用的是基础组 6 个几何体：正方体、长方体、圆柱体、六角柱、三角柱、圆台。

两场仿真不要一起开。开新的一场之前先 `bash setup/sim-clean.sh`。

## 1. 采集

腕部 Gemini 305 的预览世界里已经有这 6 个几何体。先开预览：

```bash
source /opt/ros/humble/setup.bash
source jaka_ros2/install/setup.bash
ros2 launch jaka_competition_kit gemini305_preview.launch.py
```

另开一个终端，等腕部画面出来再采：

```bash
source /opt/ros/humble/setup.bash
source jaka_ros2/install/setup.bash
python3 perception/track1_sim/capture_sim.py --samples 24
```

脚本会把 6 件在画面里挪几个位置、转一个角度，存下彩色图和 YOLO 框，然后放回原来的演示位置。框不是手画的：仿真知道位姿，把几何体八个角投到图像上。学生要打开 `preview/` 看框有没有包住物体，包不住就不要拿去训练。

图片在 `dataset/incoming/<批次>/`，不进 git。

这批图只认仿真里的这 6 个几何体。赛道二的 20 种日用品在仿真里只是盒子和圆柱，外观和实物对不上，**不能**用这里的方法代替真机采集。那 20 类仍走 `perception/track2_yolo/`，等真机上的腕部相机。

## 2. 训练

用赛道二那个已经装好 YOLO11 的虚拟环境。口头说的 YOLOv11，文件名是 `yolo11n.pt`。

```bash
perception/track2_yolo/.venv/bin/python perception/track1_sim/train.py
```

默认 30 个 epoch。仿真里每一张都是独立摆放，所以按图片划分 train / val 是可以的；真机连拍不能这么划。权重在 `runs/track1/weights/best.pt`，不进 git。没有 GPU 时把 `--batch 8` 降到 4，会慢很多，但流程一样。

看一帧：

```bash
perception/track2_yolo/.venv/bin/python - << 'PY'
from ultralytics import YOLO
model = YOLO("perception/track1_sim/runs/track1/weights/best.pt")
model.predict(source="perception/track1_sim/dataset/images/val", save=True, project="perception/track1_sim/runs", name="pred")
PY
```

## 3. 用训练结果按二维码顺序抓取

这一步用的就是上面训出来的 `runs/track1/weights/best.pt`。

练习按基础组来。6 个几何体随机换到 6 个固定工位，腕部相机在识别位姿读画面下沿的二维码，内容是 `JAKA1-######`。6 位数字是工位号 1–6 的一个排列：近排从 −x 到 +x 是 1、2、3，远排是 4、5、6。程序只按**画面里解出来的顺序**抓。解不出来就停，不会偷偷改用生成时的那串数字。

抓取点不是工位的名义坐标。YOLO 只给出类别和框，框里面工件顶面的深度才反投影成世界坐标，再放进画面外的练习料盒。不要去夹检测框的中心。

工位是这套腕部预览世界里够得到的位置（近排 y=0.255，远排 y=0.338，x=−0.120 / 0 / 0.120），不是比赛场地上那套 150 mm 网格。正式比赛的二维码在 `(0, 110)`，由俯拍相机读。这里没有俯拍相机，卡片放在远排垫板外侧，让腕部相机看得全。编号和码的格式跟 `competition.md` 0.4 节一样。

判分用的 `round.launch.py` 不走这条链路，`ref_track1` 也不读这个权重。正式比赛赛道一仍然按二维码抓固定工位，不要求认类别。

两场仿真不要一起开。先清掉上一场：

```bash
bash setup/sim-clean.sh
```

再开练习这场。`JAKA_GRIPPER=1` 必须有，预览世界里的机械臂装的是 WHEELTEC 柔性爪，抓取高度按指尖算：

```bash
source /opt/ros/humble/setup.bash
source jaka_ros2/install/setup.bash
export JAKA_GRIPPER=1
ros2 launch jaka_competition_kit gemini305_preview.launch.py learn:=true
```

这场会自己做完这些事：

1. 打开带腕部 Gemini 305 和 6 个几何体的 Gazebo，把机械臂送到识别位姿。
2. 6 个几何体随机换到 6 个工位，并摆上这一轮的 `JAKA1-######`。
3. 只额外起一个 `move_group`，接到 Gazebo 里已经有的 `jaka_minicobo_controller`。不再起一套假硬件控制器。
4. 腕部画面解出抓取顺序。解不出来就报错停下。
5. 用 YOLO 那个 Python 3.12 虚拟环境跑 `best.pt`（ROS 节点本身是 3.10，不能直接 import，二维码解码也走系统的 Python 3.10）。按二维码顺序，把每个框里的顶面深度换成世界坐标，交给 `Executor.pick_and_place`。
6. Gazebo 里的网格不会跟着规划场景走，抓起来之后脚本用 `set_pose` 让它跟着 TCP，放进画面外的练习料盒后停住。

看结果：

- Gazebo 窗口里，工件按二维码顺序离开垫板、被夹到右前方的蓝色料盒。
- 运行时会弹出「基础组识别」窗口，是腕部相机的实时画面。底栏先显示正在读码，解出来之后显示顺序。检测框和置信度跟着画面更新。抓取用的仍是识别位姿上的那一帧。关掉窗口或按 q 结束。定格图在 `perception/track1_sim/runs/grasp/detect.jpg`，读码用的原图在同目录的 `frame.jpg`。
- 话题 `/track1/yolo` 是这一帧的 JSON，里面有 `order` 和每件的 `station`。

想换自己训的权重，仿真开着的时候另开一个终端再跑一次（手臂会先回到识别位姿）：

```bash
source /opt/ros/humble/setup.bash
source jaka_ros2/install/setup.bash
export JAKA_GRIPPER=1
python3 perception/track1_sim/grasp.py --ros-args -p use_sim_time:=true \
    -p weights:=perception/track1_sim/runs/track1/weights/best.pt
```

这 6 类只存在于仿真。赛道二的 20 种日用品仍要在真机上采集，不能把这个 `best.pt` 当成赛道二的模型。
