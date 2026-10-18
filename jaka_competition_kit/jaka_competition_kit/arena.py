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

# 工件底面离支撑面(工位标记板 / 台面)抬起的高度: 1mm, 避免与支撑面贴死
# 导致接触误判。场景生成器按它摆放工件(见 scene_generator), 抓取高度校核
# 必须用同一个值 —— 差 1mm 就会让 arena_check 的"末端几何"和"末端可达性"
# 打印出两套互相矛盾的数字。
OBJECT_LIFT = 0.001


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

    # ---- 末端几何 ----
    @property
    def gripper(self) -> str:
        """末端类型: ``wheeltec``(WHEELTEC 二指柔性爪) / ``none``(实心法兰)。"""
        return str(self.raw["tool"].get("gripper", "none")).lower()

    @property
    def has_gripper(self) -> bool:
        return self.gripper not in ("none", "false", "")

    @property
    def flange_height(self) -> float:
        """Link_6 网格伸出 dummy_tcp 下方的长度(= 机械臂法兰安装面)。"""
        return float(self.raw["tool"]["flange_height"])

    @property
    def tip_depth(self) -> float:
        """TCP 到夹爪最低点(刀尖)的距离。实测自 3D 模型, 见 wheeltec_gripper。"""
        return float(self.raw["tool"].get("tip_depth", self.flange_height))

    @property
    def tip_clearance(self) -> float:
        """刀尖离"被抓工件底面所在的平面"要留的余量。

        它同时是抓取时刀尖离台面/工位板的余量(工件就坐在那上面)。
        """
        return float(self.raw["tool"].get("tip_clearance",
                                          self.raw["tool"]["clearance"]))

    @property
    def clearance(self) -> float:
        return float(self.raw["tool"]["clearance"])

    @property
    def approach_lift(self) -> float:
        return float(self.raw["tool"]["approach_lift"])

    @property
    def reach_pull_margin(self) -> float:
        """L 形转场"内收"时给可达上限留的余量(见 ``Executor.pick_and_place``)。"""
        return float(self.raw["tool"].get("reach_pull_margin", 0.010))

    # ---- 机械臂可达性 ----
    # 目标位姿"规划不出来"时, 先分清是**撞了**还是**根本到不了**: MoveIt 两种
    # 情况都只回一个 error_code=99999(FAILURE), 光看返回值分不出来。
    # 下面这几行用臂长直接算可达上限, 不用跑 IK —— 见 max_tcp_z。
    @property
    def _arm(self) -> Dict:
        return self.raw.get("arm", {})

    @property
    def shoulder_z(self) -> float:
        """J2 轴线高度 = 可达球的球心高度。"""
        return float(self._arm.get("shoulder_z", 0.187))

    @property
    def reach_radius(self) -> float:
        """大臂 + 小臂 = J2 到 J5 的最大距离。"""
        return float(self._arm.get("upper_plus_forearm", 0.4205))

    @property
    def wrist_len(self) -> float:
        """J5 到 dummy_tcp 的距离(沿工具轴)。"""
        return float(self._arm.get("wrist_len", 0.1593))

    def max_tcp_z(self, r: float) -> float:
        """工具轴竖直向下时, 水平半径 ``r`` 处 TCP 能到的**最高**高度。

        推导(全部来自 jaka_minicobo.urdf, 不依赖 IK 求解器):

          * ``joint_6`` 的原点写在 J5 的 ``+y`` 上, 而 Link_6 的 ``+z``(工具轴)
            按 ``rpy=-90°`` 展开后**也**是 J5 的 ``+y`` —— 也就是说 J5->TCP
            这一段永远躺在工具轴上。工具朝下 => J5 恒在 TCP 正上方 ``wrist_len``。
          * J5 必须落在以 J2 为心、``大臂+小臂`` 为半径的球内。

        于是 ``r² + (z + wrist_len - shoulder_z)² ≤ (大臂+小臂)²``。

        实测吻合(40 个随机种子打 ``/compute_ik``, 每次都是"全中/全不中"的硬边界):

          ==========  ============  ============
          r (mm)      公式上限(mm)  实测
          ==========  ============  ============
          385.4        195.9        195 全中 / 200 全不中
          362.8        240.3        240 全中 / 245 全不中
          355.0        253.1        248.6 全中
          ==========  ============  ============
        """
        if r >= self.reach_radius:
            return float("nan")
        return (self.shoulder_z - self.wrist_len
                + math.sqrt(self.reach_radius ** 2 - r * r))

    def radius_for_tcp_z(self, z: float, margin: float = 0.0) -> float:
        """``max_tcp_z`` 的反函数: 想站到高度 ``z``, 水平半径最大能放到多少。

        ``z_max(r) = 27.7 + sqrt(R² - r²)``  =>  ``r_max(z) = sqrt(R² - (z + w - h)²)``
        (R = 大臂+小臂, w = wrist_len, h = shoulder_z)。够不到返回 nan。
        用途: 长末端竖直抬不到转场高度时, 算"往基座收多少才抬得起来"(见 Executor)。
        """
        dy = z + self.wrist_len - self.shoulder_z + margin
        R = self.reach_radius
        if abs(dy) > R:
            return float("nan")
        return math.sqrt(R * R - dy * dy)

    def reach_ok(self, r: float, z: float, margin: float = 0.0) -> bool:
        top = self.max_tcp_z(r)
        return (not math.isnan(top)) and z <= top - margin

    def reach_top_z(self, x: float, y: float, margin: float = 0.0) -> float:
        """``(x, y)`` 处 TCP 能到的最高高度(可再留 ``margin``)。够不到返回 nan。"""
        top = self.max_tcp_z(math.hypot(x, y))
        return top if math.isnan(top) else top - margin

    def clamp_tcp_z(self, x: float, y: float, z: float,
                    margin: float = 0.005) -> Tuple[float, bool]:
        """把 TCP 目标高度压进可达范围内 -> ``(z, 是否被压过)``。

        为什么要压而不是直接放过: ``max_tcp_z`` 只是**必要条件**, 真到不了的时候
        ``/compute_ik`` 直接返回无解, 而参考实现此时会退化成"只给位姿目标"的兜底
        模式 —— 规划器于是**随机挑**一组关节解(实测挑到了 J4/J6 翻 180° 的腕部
        另一支), 之后整轮都在翻腕, 看起来就是"解算很奇怪"。

        压到"该半径够得到的最高点"之后 IK 一定有解, "最近关节角分支"这条工业做法
        才真正生效 —— 动作确定、不翻腕。预抓取悬停本来就是个余量, 压掉几毫米
        不影响抓取; 但**转场高度不能只压不报**, 压了就意味着手里的工件会撞到东西,
        调用方要拿 ``clamped`` 去告警(见 ``Executor.pick_and_place``)。
        """
        top = self.reach_top_z(x, y, margin)
        if math.isnan(top):
            return z, False
        return (z, False) if z <= top else (top, True)

    def reach_rows(self) -> List[Tuple[str, float, float, float, bool]]:
        """末端必须到的高度逐点校核 -> (名称, r, z, 该半径上限, 是否可达)。

        含赛道一 6 个工位的抓取/预抓取, 以及料盒中心与料盒内缘的转场/投放高度。
        ``arena_check`` 直接打印这张表。
        """
        rows: List[Tuple[str, float, float, float, bool]] = []
        sz = self.track1_max_workpiece_size
        top = self.station_top + OBJECT_LIFT + sz
        for i, x, y in self.stations():
            z = self.grasp_tcp_z(top, sz)
            rows.append((f"工位{i} 抓取", math.hypot(x, y), z,
                         self.max_tcp_z(math.hypot(x, y)), self.reach_ok(math.hypot(x, y), z)))
            za = self.approach_z(z)
            rows.append((f"工位{i} 预抓取", math.hypot(x, y), za,
                         self.max_tcp_z(math.hypot(x, y)), self.reach_ok(math.hypot(x, y), za)))
        binr = math.hypot(*self.track1_bin.center)
        gz0 = self.grasp_tcp_z(top, sz)
        for name, r, z in (("料盒中心 转场", binr,
                            self.transfer_tcp_z(gz0, sz, self.track1_bin)),
                           ("料盒中心 投放", binr, self.release_tcp_z(sz))):
            rows.append((name, r, z, self.max_tcp_z(r), self.reach_ok(r, z)))
        sz2 = self.track2_max_object_size
        top2 = OBJECT_LIFT + sz2
        gz2 = self.grasp_tcp_z(top2, sz2)
        for b in self.track2_bins():
            r = math.hypot(*b["box"].center)
            for name, z in ((f"{b['name']}盒 转场", self.transfer_tcp_z(gz2, sz2, b["box"])),
                            (f"{b['name']}盒 投放", self.release_tcp_z(sz2))):
                rows.append((name, r, z, self.max_tcp_z(r), self.reach_ok(r, z)))
        return rows

    @property
    def touch_links(self) -> List[str]:
        return list(self.raw["tool"].get("touch_links", ["dummy_tcp", "Link_6"]))

    def grasp_offset(self, object_size: float) -> float:
        """TCP 到**工件顶面**的距离 —— 也就是工件"挂"在 TCP 下方多深。

        两种末端的区别就在这一个数上:

        - 没有夹爪 (``gripper: none``): TCP 下方是一根实心法兰, 工件只能整个
          吊在它下面, 所以工件顶面必须低于工具底面 ``clearance``:
          ``flange_height + clearance``。
        - WHEELTEC 柔性夹爪: 工件是**夹在两片指之间**的。两片指有 ``tip_depth``
          那么长, 指间净距又随深度变大(往下越张越开), 所以工件可以坐得很深 ——
          最深的合法位置是"刀尖刚好到工件底面"那一档:
          ``tip_depth + tip_clearance - object_size``。

          这一条决定了竞赛里的作业高度: 赛道一 50mm 工件 -> 顶面在 TCP 下方
          119.6mm(不带夹爪时是 46mm)。抬得高不高、能不能夹到台面上的小工件,
          全看这个数对不对 —— 它必须**正好**等于 URDF 里刀尖到 TCP 的距离。
        """
        if not self.has_gripper:
            return self.flange_height + self.clearance
        return self.tip_depth + self.tip_clearance - object_size

    def grasp_tcp_z(self, object_top_z: float, object_size: float) -> float:
        """工件顶面高度 -> 抓取时 TCP 应有的高度(工件落在指间最深处)。"""
        return object_top_z + self.grasp_offset(object_size)

    def approach_z(self, grasp_z: float) -> float:
        return grasp_z + self.approach_lift

    @property
    def bin_floor(self) -> float:
        return float(self.raw["tool"].get("bin_floor", 0.008))

    @property
    def release_margin(self) -> float:
        return float(self.raw["tool"].get("release_margin", 0.003))

    def release_tcp_z(self, object_size: float) -> float:
        """把工件放进料盒时 TCP 应有的高度。

        工件底面离 TCP ``grasp_offset(size) + size`` ——
        用夹爪时它等于 ``tip_depth + tip_clearance``(刀尖位置), 与工件尺寸无关;
        用实心法兰时是 ``flange_height + clearance + size``。
        """
        return (self.bin_floor + self.release_margin
                + self.grasp_offset(object_size) + object_size)

    # ---- 转场高度 ----
    def _is_track1_bin(self, bin_box: Box) -> bool:
        return all(abs(a - b) < 1e-9
                   for a, b in zip(self.track1_bin.center, bin_box.center))

    def _bin_height(self, bin_box: Box) -> float:
        """这只料盒的侧墙高度。赛道二四只盒子高度可以各自不同。"""
        if self._is_track1_bin(bin_box):
            return self.track1_bin_height
        for b in self.track2_bins():
            if all(abs(a - c) < 1e-9 for a, c in zip(b["box"].center, bin_box.center)):
                return b["height"]
        return self.track2_bin_height

    def bin_top_z(self, bin_box: Box) -> float:
        """料盒**侧墙顶面**高度。"""
        return float(self.table["top_z"]) + self._bin_height(bin_box)

    @property
    def station_top(self) -> float:
        """工位标记板顶面高度 —— 工件坐在它上面(与 scene_generator 一致)。"""
        return float(self.raw["track1"]["station_thickness"])

    @property
    def track1_max_workpiece_size(self) -> float:
        return max(float(w["size"]) for w in self.raw["track1"]["workpieces"])

    @property
    def track2_max_object_size(self) -> float:
        """赛道二工件里最高的一件(竖直尺寸)。转场高度要越过它。"""
        return max(float(o["dims"][2]) for o in self.track2_objects())

    def source_area_top_z(self, bin_box: Box) -> float:
        """工件区最高的东西有多高(工位板 + 最高的一件工件, 或桌面 + 最高工件)。"""
        if self._is_track1_bin(bin_box):
            return (float(self.table["top_z"])
                    + float(self.raw["track1"]["station_thickness"])
                    + OBJECT_LIFT
                    + self.track1_max_workpiece_size)
        return float(self.table["top_z"]) + self.track2_max_object_size

    def transfer_tcp_z(self, grasp_z: float, object_size: float,
                       bin_box: Box) -> float:
        """**抬着工件转场**时 TCP 该到的高度。

        工件抓起来后挂在 TCP 下方 ``grasp_offset(size) + size`` 处(那是它的
        **底面**, 见 grasp_offset)。要让工件越过一个高 ``obs`` 的障碍, TCP 至少
        得抬到 ``obs + 间隙 + grasp_offset(size) + size``。障碍取料盒侧墙与工件区
        最高件中较高者。

        用 WHEELTEC 夹爪时 ``grasp_offset + size`` 恒等于刀尖深度, 所以它同时也
        保证了刀尖不撞障碍。

        为什么必须显式抬够 —— 这一条不看数据很难想到: 抬不够时关节空间的直线
        会撞, 规划器(退回 OMPL)就绕出一条**往外甩、往上拱**的大弧。实测赛道一
        抬 135mm 时末端拱到 **354mm**、半径甩到 460mm(超出 420mm 作业带);
        显式抬到 159mm(= 本函数在 gripper: none 下的结果)之后总高度反而
        降到 160mm 上下, 又直又短。
        "抬够" 和 "抬得高" 是反直觉的: **抬不够才真的抬得高**。

        间隙可在 ``arena.yaml`` 的 ``tool.transfer_margin`` 里调(默认 10mm)。

        注意本函数只给出"该到多高", 并**不假设一步竖直抬得上去** —— 长末端在
        远端工位原地抬不到这个高度时, ``Executor.pick_and_place`` 会改走
        L 形(先抬到该半径上限 -> 沿半径朝基座内收 -> 再抬到本高度, 见
        ``radius_for_tcp_z``)。
        """
        margin = float(self.raw["tool"].get("transfer_margin", 0.010))
        obs = max(self.bin_top_z(bin_box), self.source_area_top_z(bin_box))
        return max(self.approach_z(grasp_z),
                   obs + margin + self.grasp_offset(object_size)
                   + object_size)

    # ---- 赛道二 ----
    @property
    def scatter(self) -> Box:
        s = self.raw["track2"]["scatter"]
        return Box(tuple(s["center"]), tuple(s["size"]))

    def track2_bins(self) -> List[Dict]:
        """四只分类料盒。``box`` 是俯视矩形, ``id`` 与工件类别一致。"""
        bins = []
        for b in self.raw["track2"]["bins"]:
            bins.append({
                "id": b["id"],
                "name": b["name"],
                "box": Box(tuple(b["center"]), tuple(b["size"])),
                "height": float(b["height"]),
            })
        return bins

    def track2_bin_by_id(self, bin_id: str) -> Dict:
        for b in self.track2_bins():
            if b["id"] == bin_id:
                return b
        raise KeyError(f"没有类别料盒 {bin_id!r}")

    @property
    def track2_bin(self) -> Box:
        """兼容旧调用: 返回第一只赛道二料盒。新代码请用 ``track2_bin_by_id``。"""
        return self.track2_bins()[0]["box"]

    @property
    def track2_bin_height(self) -> float:
        return max(b["height"] for b in self.track2_bins())

    @property
    def t2_objects_per_round(self) -> int:
        return int(self.raw["track2"]["objects_per_round"])

    @property
    def t2_min_gap(self) -> float:
        return float(self.raw["track2"]["min_gap"])

    def track2_categories(self) -> List[Dict]:
        return list(self.raw["track2"]["categories"])

    def track2_objects(self) -> List[Dict]:
        """20 件标准工件, 每件带上类别。"""
        out = []
        for cat in self.track2_categories():
            for obj in cat["objects"]:
                item = dict(obj)
                item["category"] = cat["id"]
                item["category_name"] = cat["name"]
                item["bin_id"] = cat["id"]
                out.append(item)
        return out

    def sample_track2(self, rng) -> Tuple[List[Dict], List[Dict]]:
        """每类 5 选 4。返回 (上场的 16 件, 每类被抽走的那 1 件)。"""
        k = int(self.raw["track2"]["pick_per_category"])
        picked: List[Dict] = []
        withheld: List[Dict] = []
        for cat in self.track2_categories():
            choices = []
            for obj in cat["objects"]:
                item = dict(obj)
                item["category"] = cat["id"]
                item["category_name"] = cat["name"]
                item["bin_id"] = cat["id"]
                choices.append(item)
            chosen = rng.sample(choices, k)
            chosen_ids = {c["id"] for c in chosen}
            picked.extend(chosen)
            withheld.extend(c for c in choices if c["id"] not in chosen_ids)
        rng.shuffle(picked)
        return picked, withheld

    def pack_track2(self, objects: List[Dict], rng) -> List[Dict] | None:
        """把工件摆进散放区。长边沿 +x, 货架装箱, 行内和行间至少留 ``min_gap``。

        成功返回带 ``xy`` / ``world_dims`` / ``yaw_deg`` 的列表; 摆不下返回 None。
        竖直尺寸不参与占地。圆柱不旋转。
        """
        area = self.scatter
        gap = self.t2_min_gap
        x0, x1 = area.x_range
        y0, y1 = area.y_range
        width, height = x1 - x0, y1 - y0
        items = []
        for obj in objects:
            sx, sy, sz = (float(v) for v in obj["dims"])
            yaw = 0
            if obj.get("shape") != "cylinder" and sy > sx:
                sx, sy = sy, sx
                yaw = 90
            items.append({**obj, "_sx": sx, "_sy": sy, "_sz": sz, "_yaw": yaw})
        # 不要先洗牌再只按短边排序: 相同短边的先后会改变行的填充,
        # 有些抽法会多出一行, 175mm 的散放区放不下。按短边、长边、id
        # 排成全序后, 货架装箱对同一组工件是确定的, 5000 个种子都放得下。
        items.sort(key=lambda t: (-t["_sy"], -t["_sx"], str(t.get("id", ""))))
        rows: List[Dict] = []
        for it in items:
            best = None
            best_h = 1e9
            for row in rows:
                extra = it["_sx"] if row["used"] == 0.0 else it["_sx"] + gap
                if row["used"] + extra <= width + 1e-9 and it["_sy"] <= row["h"] + 1e-9 and row["h"] < best_h:
                    best, best_h = row, row["h"]
            if best is None:
                rows.append({"h": it["_sy"], "used": it["_sx"], "items": [it]})
            else:
                extra = it["_sx"] if best["used"] == 0.0 else it["_sx"] + gap
                best["used"] += extra
                best["items"].append(it)
        total_h = sum(r["h"] for r in rows) + gap * max(0, len(rows) - 1)
        if total_h > height + 1e-6:
            return None
        slack_y = height - total_h
        y = y0 + (rng.uniform(0.0, slack_y) if slack_y > 1e-6 else 0.0)
        placed: List[Dict] = []
        for row in rows:
            slack_x = width - row["used"]
            x = x0 + (rng.uniform(0.0, slack_x) if slack_x > 1e-6 else 0.0)
            for it in row["items"]:
                cx = x + it["_sx"] / 2.0
                cy = y + (row["h"] - it["_sy"]) / 2.0 + it["_sy"] / 2.0
                kept = {k: v for k, v in it.items() if not k.startswith("_")}
                kept["xy"] = [cx, cy]
                kept["yaw_deg"] = it["_yaw"]
                kept["world_dims"] = [it["_sx"], it["_sy"], it["_sz"]]
                placed.append(kept)
                x += it["_sx"] + gap
            y += row["h"] + gap
        return placed

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
        for b in self.track2_bins():
            check_point(f"赛道二{b['name']}盒 中心", *b["box"].center)

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
        t2_bins = self.track2_bins()
        for nm, box in [("赛道一料盒", self.track1_bin)] + [
                (f"赛道二{b['name']}盒", b["box"]) for b in t2_bins]:
            for rn, ob in zip(regions, boxes):
                gx = min(box.x_range[1], ob.x_range[1]) - max(box.x_range[0], ob.x_range[0])
                gy = min(box.y_range[1], ob.y_range[1]) - max(box.y_range[0], ob.y_range[0])
                if gx > 0 and gy > 0:
                    problems.append(f"{nm} 与 {rn} 相交 "
                                    f"({gx*1000:.0f}×{gy*1000:.0f}mm), 需留出间隙")
        for i, a in enumerate(t2_bins):
            for b in t2_bins[i + 1:]:
                ba, bb = a["box"], b["box"]
                gx = min(ba.x_range[1], bb.x_range[1]) - max(ba.x_range[0], bb.x_range[0])
                gy = min(ba.y_range[1], bb.y_range[1]) - max(ba.y_range[0], bb.y_range[0])
                if gx > 0 and gy > 0:
                    problems.append(f"赛道二{a['name']}盒 与 {b['name']}盒 相交")

        # 台面包含检查
        t = self.table
        tb = Box(tuple(t["center"]), tuple(t["size"]))
        items = [(f"工位{i}", Box((x, y), (100e-3, 100e-3)))
                 for i, x, y in self.stations()]
        items.append(("赛道一料盒", self.track1_bin))
        items.extend((f"赛道二{b['name']}盒", b["box"]) for b in self.track2_bins())
        items.append(("散放区", self.scatter))
        for nm, ob in items:
            if not (tb.x_range[0] <= ob.x_range[0] and ob.x_range[1] <= tb.x_range[1]
                    and tb.y_range[0] <= ob.y_range[0] and ob.y_range[1] <= tb.y_range[1]):
                problems.append(f"{nm} 超出台面范围 {tb.size[0]*1000:.0f}×{tb.size[1]*1000:.0f}mm")

        problems.extend(self._audit_track2_library())
        return problems

    def _audit_track2_library(self) -> List[str]:
        """工件库必须是 4 类 × 5 件, 并且随机抽 16 件后摆得进散放区。"""
        import random
        problems: List[str] = []
        cats = self.track2_categories()
        per = int(self.raw["track2"]["per_category"])
        pick = int(self.raw["track2"]["pick_per_category"])
        if len(cats) != 4:
            problems.append(f"赛道二应有 4 个类别, 实际 {len(cats)}")
        ids = []
        bin_ids = {b["id"] for b in self.track2_bins()}
        for cat in cats:
            objs = cat.get("objects", [])
            if len(objs) != per:
                problems.append(f"类别 {cat.get('name')} 应有 {per} 件, 实际 {len(objs)}")
            if cat.get("id") not in bin_ids:
                problems.append(f"类别 {cat.get('id')} 没有同名料盒")
            ids.extend(f"{cat['id']}:{o['id']}" for o in objs)
        if len(ids) != len(set(ids)):
            problems.append("赛道二工件 id 在类别内重复")
        if pick * len(cats) != self.t2_objects_per_round:
            problems.append("objects_per_round 不等于 类别数 × 每类抽取数")
        fails = []
        for seed in range(40):
            rng = random.Random(seed)
            picked, withheld = self.sample_track2(rng)
            if len(picked) != self.t2_objects_per_round or len(withheld) != len(cats):
                fails.append(seed)
                continue
            placed = self.pack_track2(picked, rng)
            if not placed or len(placed) != len(picked):
                fails.append(seed)
                continue
            if not self._packed_ok(placed):
                fails.append(seed)
        if fails:
            problems.append(f"赛道二装箱失败的种子: {fails[:8]}"
                            + (" ..." if len(fails) > 8 else ""))
        return problems

    def _packed_ok(self, placed: List[Dict]) -> bool:
        gap = self.t2_min_gap - 1e-6
        area = self.scatter
        boxes = []
        for it in placed:
            x, y = it["xy"]
            sx, sy, _sz = it["world_dims"]
            box = Box((x, y), (sx, sy))
            if not area.contains(x, y):
                return False
            if (box.x_range[0] < area.x_range[0] - 1e-6 or box.x_range[1] > area.x_range[1] + 1e-6
                    or box.y_range[0] < area.y_range[0] - 1e-6 or box.y_range[1] > area.y_range[1] + 1e-6):
                return False
            r = radius(x, y)
            if r < self.r_min or r > self.r_max:
                return False
            boxes.append(box)
        for i, a in enumerate(boxes):
            for b in boxes[i + 1:]:
                gx = min(a.x_range[1], b.x_range[1]) - max(a.x_range[0], b.x_range[0])
                gy = min(a.y_range[1], b.y_range[1]) - max(a.y_range[0], b.y_range[0])
                # 分离轴: 至少在一个方向上留出 gap
                sep_x = -gx
                sep_y = -gy
                if sep_x < gap and sep_y < gap:
                    return False
        return True


GRIPPER_ENV = "JAKA_GRIPPER"


def _env_tool(raw: Dict) -> Dict:
    """``JAKA_GRIPPER`` 环境变量覆盖 ``tool.gripper``。

    为什么要一个环境变量: 换末端要同时改三处(arena.yaml 的抓取高度、URDF 的
    ``with_gripper``、launch 的 ``use_gripper``), 三处漏一处就会撞工件或让
    MoveIt 觉得末端比实际短。让 xacro 和本模块读**同一个**变量, 换末端就只剩
    一个开关:

        JAKA_GRIPPER=1 ros2 launch jaka_competition_kit round.launch.py ...

    取值: 1/true/yes/on/wheeltec -> wheeltec; 0/false/no/off/none -> none;
    没设(或空) -> 用 arena.yaml 里写的值。
    """
    v = os.environ.get(GRIPPER_ENV, "").strip().lower()
    if v in ("", "default"):
        return raw
    tool = raw.setdefault("tool", {})
    tool["gripper"] = "wheeltec" if v in ("1", "true", "yes", "on", "wheeltec") else "none"
    return raw


def load_arena(path: str | None = None) -> Arena:
    path = path or default_config_path()
    with open(path, "r", encoding="utf-8") as f:
        return Arena(raw=_env_tool(yaml.safe_load(f)), config_path=path)


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
