"""赛场几何定义与合法性校核。

`config/arena.yaml` 是唯一真源;本模块把它读成对象,并提供:
  - 各元素的精确坐标与尺寸
  - 合法性校核(所有元素必须落在有效作业带内)
  - 生成 MoveIt 规划场景所需的几何体
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import yaml

def _repo_config(name: str) -> str:
    """源码目录下的 config 路径(脱离 ROS 安装时使用)。"""
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "config", name)


def default_config_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("jaka_competition_kit"),
                            "config", "arena.yaml")
    except Exception:                                 # 允许脱离 ROS 运行
        return _repo_config("arena.yaml")


@dataclass
class Box:
    """一个矩形区域(俯视), 单位 m。"""
    center: Tuple[float, float]
    size: Tuple[float, float]

    @property
    def x_range(self) -> Tuple[float, float]:
        return (self.center[0] - self.size[0] / 2.0,
                self.center[0] + self.size[0] / 2.0)

    @property
    def y_range(self) -> Tuple[float, float]:
        return (self.center[1] - self.size[1] / 2.0,
                self.center[1] + self.size[1] / 2.0)

    def contains(self, x: float, y: float, shrink: float = 0.0) -> bool:
        x0, x1 = self.x_range
        y0, y1 = self.y_range
        return (x0 + shrink) <= x <= (x1 - shrink) and (y0 + shrink) <= y <= (y1 - shrink)

    def corners(self) -> List[Tuple[float, float]]:
        x0, x1 = self.x_range
        y0, y1 = self.y_range
        return [(x0, y0), (x0, y1), (x1, y0), (x1, y1)]

    def nearest_point(self) -> Tuple[float, float]:
        """矩形内离原点最近的点(x 与 y 各自向 0 收拢)。

        区域是否可达必须按**最近点**判 —— 只看近角会漏掉一条更靠近基座的边。
        矩形上离原点最远的点一定是四个角之一(凸函数在矩形上的最大值在角点),
        最近点则可能落在边中点。
        """
        x0, x1 = self.x_range
        y0, y1 = self.y_range
        cx = 0.0 if x0 <= 0.0 <= x1 else min(abs(x0), abs(x1))
        cy = 0.0 if y0 <= 0.0 <= y1 else min(abs(y0), abs(y1))
        return (cx, cy)

    def farthest_point(self) -> Tuple[float, float]:
        return max(self.corners(), key=lambda p: radius(*p))


def radius(x: float, y: float) -> float:
    return math.hypot(x, y)


@dataclass
class Arena:
    raw: Dict
    config_path: str = ""

    # ---- 基本参数 ----
    @property
    def frame_id(self) -> str:
        return self.raw.get("frame_id", "world")

    @property
    def group_name(self) -> str:
        return self.raw.get("group_name", "jaka_minicobo")

    @property
    def tcp_link(self) -> str:
        return self.raw.get("tcp_link", "dummy_tcp")

    @property
    def tool_down_quat(self) -> Tuple[float, float, float, float]:
        return tuple(self.raw.get("tool_down_quat", [1.0, 0.0, 0.0, 0.0]))

    @property
    def work_band(self) -> Dict:
        return self.raw["work_band"]

    @property
    def r_min(self) -> float:
        return float(self.work_band["r_min"])

    @property
    def r_max(self) -> float:
        return float(self.work_band["r_max"])

    @property
    def table(self) -> Dict:
        return self.raw["table"]

    @property
    def home(self) -> Tuple[float, float, float]:
        h = self.raw.get("home", {"xy": [0.0, 0.28], "z": 0.22})
        return (float(h["xy"][0]), float(h["xy"][1]), float(h["z"]))

    # ---- 赛道一 ----
    def station_xy(self, idx: int) -> Tuple[float, float]:
        """工位编号 1..6 -> (x, y)。近排 1-3, 远排 4-6, 从 -x 到 +x。"""
        if not 1 <= idx <= 6:
            raise ValueError(f"工位编号必须是 1..6, 收到 {idx}")
        t1 = self.raw["track1"]
        pitch = float(t1["station_pitch"])
        r0 = float(t1["array_center_r"])
        row, col = divmod(idx - 1, 3)
        return ((col - 1) * pitch, r0 + (row - 0.5) * pitch)

    def stations(self) -> List[Tuple[int, float, float]]:
        return [(i, *self.station_xy(i)) for i in range(1, 7)]

    @property
    def track1_bin(self) -> Box:
        b = self.raw["track1"]["bin"]
        return Box(tuple(b["center"]), tuple(b["size"]))

    @property
    def track1_bin_height(self) -> float:
        return float(self.raw["track1"]["bin"]["height"])

    @property
    def qr(self) -> Box:
        q = self.raw["track1"]["qr"]
        return Box(tuple(q["center"]), (q["size"], q["size"]))

    @property
    def t1_grasp_z(self) -> float:
        return float(self.raw["track1"]["grasp_z"])

    @property
    def t1_release_z(self) -> float:
        return float(self.raw["track1"]["release_z"])

    # ---- 末端几何 ----
    @property
    def flange_height(self) -> float:
        return float(self.raw["tool"]["flange_height"])

    @property
    def clearance(self) -> float:
        return float(self.raw["tool"]["clearance"])

    @property
    def approach_lift(self) -> float:
        return float(self.raw["tool"]["approach_lift"])

    @property
    def touch_links(self) -> List[str]:
        return list(self.raw["tool"].get("touch_links", ["dummy_tcp", "Link_6"]))

    def grasp_tcp_z(self, object_top_z: float) -> float:
        """工件顶面高度 -> 抓取时 TCP 应有的高度。

        URDF 里 Link_6 在 TCP 下方还延伸 ``flange_height``,
        所以 TCP 必须抬到工件顶面之上, 否则法兰会插进工件、规划直接失败。
        """
        return object_top_z + self.flange_height + self.clearance

    def approach_z(self, grasp_z: float) -> float:
        return grasp_z + self.approach_lift

    @property
    def bin_floor(self) -> float:
        return float(self.raw["tool"].get("bin_floor", 0.008))

    @property
    def release_margin(self) -> float:
        return float(self.raw["tool"].get("release_margin", 0.003))

    @property
    def grasp_offset(self) -> float:
        """抓取时工件中心到 TCP 的距离(工件挂在 TCP 下方这么远)。"""
        return self.flange_height + self.clearance

    def release_tcp_z(self, object_size: float) -> float:
        """把工件放进料盒时 TCP 应有的高度。

        工件抓起来后一直挂在 TCP 下方 ``grasp_offset + size/2`` 处,
        所以 TCP 不能直接下到料盒里 —— 要按工件实际占位反推。
        """
        return (self.bin_floor + self.release_margin + self.grasp_offset
                + object_size)

    # ---- 赛道二 ----
    @property
    def scatter(self) -> Box:
        s = self.raw["track2"]["scatter"]
        return Box(tuple(s["center"]), tuple(s["size"]))

    @property
    def track2_bin(self) -> Box:
        b = self.raw["track2"]["bin"]
        return Box(tuple(b["center"]), tuple(b["size"]))

    @property
    def track2_bin_height(self) -> float:
        return float(self.raw["track2"]["bin"]["height"])

    @property
    def t2_objects_per_round(self) -> int:
        return int(self.raw["track2"]["objects_per_round"])

    @property
    def t2_min_gap(self) -> float:
        return float(self.raw["track2"]["min_gap"])

    @property
    def t2_grasp_z(self) -> float:
        return float(self.raw["track2"]["grasp_z"])

    @property
    def t2_release_z(self) -> float:
        return float(self.raw["track2"]["release_z"])

    # ---- 校核 ----
    def validate(self) -> List[str]:
        """返回问题列表;空列表表示赛场合法。

        判据:
          - **抓取点**(工位中心、料盒中心)必须落在有效作业带内 —— 机械臂只需要够到中心;
          - **散放区最近点与最远角**都必须落在作业带内 —— 区内任意位置都可能放工件;
          - 料盒与工位/散放区不得相交;
          - 所有元素必须落在台面之内。
        """
        problems: List[str] = []
        lo, hi = self.r_min, self.r_max

        def check_point(name: str, x: float, y: float):
            r = radius(x, y)
            if r < lo:
                problems.append(f"{name}: r={r*1000:.0f}mm < 下界 {lo*1000:.0f}mm")
            if r > hi:
                problems.append(f"{name}: r={r*1000:.0f}mm > 上界 {hi*1000:.0f}mm")

        # 抓取点
        for i, x, y in self.stations():
            check_point(f"工位{i} 中心", x, y)
        check_point("赛道一料盒 中心", *self.track1_bin.center)
        check_point("赛道二料盒 中心", *self.track2_bin.center)

        # 散放区:工件可能落在区内任意位置, 所以"最近点"和"最远角"都要在带内。
        # 只查四角是不够的 —— 近边中点往往比近角离基座更近。
        nx, ny = self.scatter.nearest_point()
        check_point(f"散放区最近点({nx*1000:.0f},{ny*1000:.0f})", nx, ny)
        for cx, cy in self.scatter.corners():
            check_point(f"散放区角({cx*1000:.0f},{cy*1000:.0f})", cx, cy)

        # 二维码只需相机可见, 不要求机械臂可达
        qr_r = radius(*self.qr.center)
        if qr_r > hi:
            problems.append(f"二维码卡 r={qr_r*1000:.0f}mm > 上界 {hi*1000:.0f}mm "
                            f"(虽不需机械臂够到, 但应靠近工作区中心)")

        # 相交检查。
        # 注意: 两个赛道的料盒**故意重叠**(赛道一 (315,180) / 赛道二 (310,200)),
        # 因为两条赛道不会同时开赛、场上只摆一个。所以这里不检查 bin_t1 vs bin_t2,
        # 但场景生成器必须保证同时只放一个 —— 见 scene_generator._build_static()。
        regions = ([f"工位{i}" for i, _, _ in self.stations()]
                   + ["散放区"])
        boxes = ([Box((x, y), (100e-3, 100e-3)) for _, x, y in self.stations()]
                 + [self.scatter])
        for nm, box in (("赛道一料盒", self.track1_bin),
                        ("赛道二料盒", self.track2_bin)):
            for rn, ob in zip(regions, boxes):
                gx = min(box.x_range[1], ob.x_range[1]) - max(box.x_range[0], ob.x_range[0])
                gy = min(box.y_range[1], ob.y_range[1]) - max(box.y_range[0], ob.y_range[0])
                if gx > 0 and gy > 0:
                    problems.append(f"{nm} 与 {rn} 相交 "
                                    f"({gx*1000:.0f}×{gy*1000:.0f}mm), 需留出间隙")

        # 台面包含检查
        t = self.table
        tb = Box(tuple(t["center"]), tuple(t["size"]))
        items = [(f"工位{i}", Box((x, y), (100e-3, 100e-3)))
                 for i, x, y in self.stations()]
        items.append(("赛道一料盒", self.track1_bin))
        items.append(("赛道二料盒", self.track2_bin))
        items.append(("散放区", self.scatter))
        for nm, ob in items:
            if not (tb.x_range[0] <= ob.x_range[0] and ob.x_range[1] <= tb.x_range[1]
                    and tb.y_range[0] <= ob.y_range[0] and ob.y_range[1] <= tb.y_range[1]):
                problems.append(f"{nm} 超出台面范围 {tb.size[0]*1000:.0f}×{tb.size[1]*1000:.0f}mm")
        return problems


def load_arena(path: str | None = None) -> Arena:
    path = path or default_config_path()
    with open(path, "r", encoding="utf-8") as f:
        return Arena(raw=yaml.safe_load(f), config_path=path)


def load_rules(path: str | None = None) -> Dict:
    if path is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory("jaka_competition_kit"),
                                "config", "rules.yaml")
        except Exception:
            path = _repo_config("rules.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)
