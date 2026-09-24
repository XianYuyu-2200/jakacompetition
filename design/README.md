# 赛题设计工作区

这里放的是**设计这两个比赛本身**要用的东西:可行性实测、规则校核、赛场尺寸、
参考实现骨架和配图。

## 一句话结论

`competition.md` 已按实测修正**定稿**。初稿有 **1 个致命几何错误**(工位阵放得太远)、
**1 个参数写错**(重复定位精度)、**2 个规则自相矛盾**、**1 个时间预算不成立**;
改为定稿版的过程中把整条流水线实跑跑通,又发现 **8 个静态算不出来的问题**
(见 `规则校核报告.md` 附二)并重新标定了限时(见附三)。

下面所有数字都是在本机用 `jaka_minicobo` URDF + MoveIt 真跑出来的,不是估算。

## 文件清单

| 文件 | 内容 |
|---|---|
| `规则校核报告.md` | 逐条列出 `competition.md` 的问题 + 实测依据 + 修改建议 |
| `赛事设计指南.md` | 怎么从零把这两个比赛办起来(赛场/软件/裁判/赛程) |
| `arena_layout.png` | 赛场布局图(原稿 vs 建议),可直接放进赛题细则 |
| `reach_envelope.png` | 竖直抓取可达域曲线 |
| `workspace_check.py` | 工作空间剖面 + FK 自检 |
| `table_clearance.py` | 桌面干涉校核(用真实 STL 网格) |
| `arena_report.py` | 布局方案打分汇总 |
| `make_layout_figure.py` | 生成上面两张图 |
| `track1_pick_demo.py` | 赛道一参考实现骨架(早期版本,现用 `jaka_competition_kit` 里的 `ref_track1`) |
| `scatter_probe.py` | 用 `/compute_ik` 扫散放区,确认整块可达(赛道二尺寸的依据) |
| `calibrate.sh` | **限时标定工具**:反复跑参考实现,输出中位数与建议限时 |
| `arena_check.py` | 赛场校核入口(与 `ros2 run jaka_competition_kit arena_check` 等价) |
| `competition_scene_track1.png` | RViz 里的赛道一实景(工作台/工位板/料盒/二维码) |

## 实测数据(全部可复现)

测试条件:基座安装面 = 台面 = `z=0`,工具坐标系 `dummy_tcp`,工具轴**严格竖直向下**,
场景里有 900×900mm 工作台,通过 MoveIt `/compute_ik` + `/check_state_validity`。

| 抓取高度 z | 竖直下抓可达半径 r | 带宽 |
|---:|---:|---:|
| 30 mm | 180 – 420 mm | 240 mm |
| 50 mm | 180 – 420 mm | 240 mm |
| 80 mm | 170 – 415 mm | 245 mm |
| 120 mm | 155 – 406 mm | 251 mm |
| 160 mm | 120 – 397 mm | 277 mm |
| 200 mm | 85 – 380 mm | 295 mm |

放宽到**允许倾角 ≤15°**(z=50mm):130 – 459 mm。但靠近边界时解的数量会急剧变少
(r=451mm 处 380 万次采样里只有 37 个可行构型),工程上不可靠。

### 结论:有效作业带 = 距基座 180–420mm

这一条应该写进赛题细则,两条赛道共用。

> 早期版本按"留安全余量"保守写成 200–390mm,后来用 `/compute_ik`
> 逐点实测,确认工具竖直向下时 **180–420mm** 整段可达(z=166mm 处整块可达),
> 赛题细则与 `arena.yaml` 最终采用 180–420mm。

## 复现方法

```bash
# 1) 编译(只需一次)
bash ../setup/build.sh

# 2) 纯离线的几何校核,不需要 ROS 跑起来
python3 workspace_check.py      # FK 自检 + 工作空间剖面
python3 table_clearance.py      # 桌面干涉
python3 arena_report.py         # 布局方案打分
python3 make_layout_figure.py   # 出图

# 3) 需要 MoveIt 的校核: 先起仿真, 再另开终端
bash ../setup/run-rviz-sim.sh
# --- 另开一个终端 ---
source ../jaka_ros2/install/setup.bash
python3 ../verification/check_arena_layout.py   # 每个工位能不能求得无碰撞 IK
python3 ../verification/reach_scan.py           # 竖直抓取的可达半径上限
python3 ../verification/near_scan.py            # 竖直抓取的最近可达半径
python3 -u track1_pick_demo.py 246135           # 赛道一完整抓取循环
```

## 参考实现的实测表现

`track1_pick_demo.py` 在建议布局(工位阵中心 280mm)上 **6/6 全部成功**:

| 配置 | 每件耗时 | 6 件总耗时 |
|---|---:|---:|
| 速度缩放 0.3, 规划尝试 5 次 | ~51 s | 306 s |
| 速度缩放 1.0, 规划尝试 1 次 | ~23 s | ~140 s(推算) |

→ 原稿"限时 90 秒 / 60 秒内完成得效率满分"对 6 个工件**不成立**,
详见 `规则校核报告.md` 第 7 条。
