# 赛道二 · YOLO11 二十类检测

口头说的 YOLOv11，在 Ultralytics 里叫 **YOLO11**。权重文件是 `yolo11n.pt`，没有 `yolov11n.pt`。

当前进度见 [`进度记录.md`](进度记录.md)。用法以本文为准。

这场比赛的 20 种工件是固定的，模型要把这 20 个 id 都认出来。每一轮从桌上抽走 4 种（每类抽走 1 种），**抽走的是桌上的东西，不是从模型里删类**。认出来以后映射到四只料盒：`life` 生活、`tool` 工具、`toy` 玩具、`fruit` 水果。放错盒子不计分。

检测框不是抓取点。框只回答「是哪一种、在画面哪里」。抓取点以后取框内深度最高那一片的中位数，不要夹框中心。

类别编号以 `jaka_competition_kit/config/arena.yaml` 为准，顺序就是 YOLO 的 class id。`classes.py` 会核对这份顺序，对不上就退出，不要改编号去迁就旧权重。

| id | sku | 中文 | 料盒 |
|---:|---|---|---|
| 0 | bottle | 方形旅行分装瓶 | life 生活 |
| 1 | tissue | 迷你硬质抽纸盒 | life 生活 |
| 2 | soap | 椭圆香皂 | life 生活 |
| 3 | sponge | 洗碗海绵块 | life 生活 |
| 4 | mouse | 迷你无线鼠标 | life 生活 |
| 5 | driver | 短柄绝缘螺丝刀 | tool 工具 |
| 6 | pliers_nose | 迷你尖嘴钳 | tool 工具 |
| 7 | pliers_cut | 迷你斜口钳 | tool 工具 |
| 8 | tape | 闭合钢卷尺 | tool 工具 |
| 9 | brush | 短柄油漆刷 | tool 工具 |
| 10 | brick | 大颗粒积木块 | toy 玩具 |
| 11 | duck | 小黄鸭 | toy 玩具 |
| 12 | car | 玩具小车 | toy 玩具 |
| 13 | plush | 短毛毛绒方块 | toy 玩具 |
| 14 | dino | 塑料小恐龙 | toy 玩具 |
| 15 | apple | 苹果 | fruit 水果 |
| 16 | orange | 橙子 | fruit 水果 |
| 17 | lemon | 柠檬 | fruit 水果 |
| 18 | pear | 梨 | fruit 水果 |
| 19 | kiwi | 猕猴桃 | fruit 水果 |

易混的是 `pliers_nose` / `pliers_cut`，以及苹果 / 橙子。这两对要单独加拍，不要只靠总类数。

## 环境

在本目录建虚拟环境。RTX 5060 Ti（sm_120）配驱动 580 时，用 CUDA 13 的 PyTorch，也就是 `requirements.txt` 里锁的 `torch==2.14.1`。

```bash
cd perception/track2_yolo
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r requirements.txt
.venv/bin/python check_runtime.py
.venv/bin/python classes.py
```

没有 uv 时：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -U pip
.venv/bin/python -m pip install -r requirements.txt
```

`check_runtime.py` 只确认 GPU 能把 YOLO11n 的网络跑起来，不代表已经会认这 20 类。
预训练文件是 `weights/yolo11n.pt`（COCO 权重，不是这批工件）。`train.py` 会自己去下；GitHub 超时时用浏览器下到这个路径，文件名不要改。

## 采集

相机在腕部。每次抓取前臂都回到**同一个关节角**再认。采集必须在这个位姿做，用比赛时的那只相机、同一张桌子、同一路灯光。不要手持，也不要换焦距。

输入法切到英文。

```bash
.venv/bin/python capture_label.py --list-cameras
.venv/bin/python capture_label.py --camera 0
# 单类连拍时锁死 id，例如只拍尖嘴钳:
.venv/bin/python capture_label.py --camera 0 --id pliers_nose
```

按键：`]` / `[` 或 Tab 换类，Space 冻结，拖拽画框，Backspace 撤销，Enter 保存，Esc 丢掉这一帧，`q` 退出。混合场景在同一冻结帧里换类再画下一个框。松手时选中的类会写进这个框。

建议数量（都在识别位姿下）：

- 每种单放约 60 张。在散放区内换位置，平面内大约 ±20°。长边大致沿画面水平，和比赛摆放一致。
- `pliers_nose` 和 `pliers_cut` 并排再拍约 30 张，苹果和橙子同样。
- 混合场景约 40 张：8 到 16 件，间隙至少 10mm，不叠放。每一件都要框到，漏框等于教模型把工件当背景。
- 换一批摆放或一批光照就重新开一个 session。验证集要来自**另一批**，不要从同一串连拍里抽。

图片在 `dataset/incoming/<批次>/`，旁边是同名的 YOLO txt。这些图不进 git。

## 划分和训练

```bash
.venv/bin/python split_dataset.py
.venv/bin/python train.py
```

`split_dataset.py` 按批次整批划进 train 或 val，默认最新的约 20% 批次做验证。只有一批时会拒绝；`--allow-frame-split` 仅供调试，那个 val 分数不能当真。

`train.py` 默认 `yolo11n`、640、最多 100 个 epoch、平面旋转 ±10°、不做上下翻转。nano 不够准时先补易混类的图，再考虑 `--model yolo11s.pt`。显存不够就把 `--batch 16` 降到 8。

权重在 `runs/track2*/weights/best.pt`，不进 git。

## 查看结果

```bash
.venv/bin/python predict.py --source 0
.venv/bin/python predict.py --source /path/to/frame.jpg
```

终端会打印置信度、sku、中文名和料盒 id。程序里用：

```python
from predict import detect_bgr, load_detector

model = load_detector("runs/track2/weights/best.pt")
dets = detect_bgr(model, bgr, conf=0.25, device=0)
# det.id, det.bin_id, det.conf, det.xyxy
```

没有训练出 `best.pt` 之前，不要把 `yolo11n.pt` 拿去认这 20 类。
