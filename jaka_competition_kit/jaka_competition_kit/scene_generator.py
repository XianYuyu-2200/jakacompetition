"""场景生成器 —— 官方出题程序。

职责:
  1. 把工作台 / 工位标记 / 料盒 / 二维码卡摆进 MoveIt 规划场景;
  2. 为赛道一生成一轮二维码序令, 并写出 PNG(裁判打印用);
  3. 为赛道二在散放区内随机撒 12 件工件(互不重叠);
  4. 把本轮**真值**发布到 ``/competition/scene``(transient_local, 供判分使用)。

服务:
  ``/competition/generate``     (std_srvs/Trigger) 出一轮新题并摆进场景
  ``/competition/scene/reset``  (std_srvs/Trigger) 清空动态工件, 只留静态设施

注意: 复位服务名下必须带 ``scene/``。判分器(scorer)也有一个
``/competition/reset``, 两者同名会互相抢服务, 调用时命中谁完全随机。

用法:
  ros2 run jaka_competition_kit scene_generator
  ros2 service call /competition/generate std_srvs/srv/Trigger "{}"
"""
from __future__ import annotations

import json
import os
import random
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
import rclpy.node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .arena import OBJECT_LIFT, Arena, Box, load_arena, radius
from .qr import make_order, parse, payload, render_png
from .scene import SceneClient, bin_container, box_object, cylinder_object

OBJECT_LIBRARY = [
    "纸杯", "半瓶矿泉水", "钢卷尺", "钥匙串", "橡皮擦",
    "有线鼠标", "订书机", "眼镜盒", "便签本", "马克笔",
    "胶带座", "塑料螺丝刀", "梳子", "门禁卡套", "橡皮圈",
    "小号扳手", "茶杯垫", "塑料夹子", "名片盒", "小号毛绒玩具",
]


class SceneGenerator(rclpy.node.Node):
    def __init__(self):
        super().__init__("competition_scene_generator")
        self.declare_parameter("arena_config", "")
        self.declare_parameter("qr_output_dir", os.path.expanduser("~/.ros/jaka_competition"))
        self.declare_parameter("track", 1)
        self.declare_parameter("seed", -1)

        cfg = self.get_parameter("arena_config").value
        self.arena: Arena = load_arena(cfg or None)
        self.qr_dir = os.path.expanduser(str(self.get_parameter("qr_output_dir").value))
        os.makedirs(self.qr_dir, exist_ok=True)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._scene_pub = self.create_publisher(String, "/competition/scene", latched)
        self.current: Dict = {}

        # SceneClient 自带 node + 后台执行器, 在服务回调里阻塞调用也不会自我阻塞。
        self._srv_cb_group = ReentrantCallbackGroup()
        self.scene = SceneClient()
        self.create_service(Trigger, "/competition/generate", self._on_generate,
                            callback_group=self._srv_cb_group)
        self.create_service(Trigger, "/competition/scene/reset", self._on_reset,
                            callback_group=self._srv_cb_group)

        # 启动时先清一次: 上一次进程可能是被 kill 的, 末端上会挂着幽灵工件,
        # 不摘掉的话后面每次规划都会失败。
        self._clear_dynamic()
        self._build_static()
        self.get_logger().info(
            f"场景生成器就绪。有效作业带 "
            f"{self.arena.r_min*1000:.0f}-{self.arena.r_max*1000:.0f}mm, "
            f"赛道一工位阵中心 {self.arena.raw['track1']['array_center_r']*1000:.0f}mm")

    # ---------- 静态设施 ----------
    def _build_static(self, track: Optional[int] = None):
        """摆台面 / 工位 / **本轮那个**料盒。

        ⚠️ 两个赛道的料盒位置是重叠的(赛道一 (315,180)、赛道二 (310,200)),
        因为两条赛道不会同时开赛。**绝对不能两个料盒一起放进规划场景** ——
        另一个料盒的侧墙会横在工件入盒的路径上, 实测表现为
        "Computed path is not valid"(工件与 bin_t2_wall_y0 接触),
        而且是时好时坏。只放当前赛道的那个。
        """
        if track is None:
            track = int(self.get_parameter("track").value)
        a = self.arena
        objs = []
        t = a.table
        # 注意: 台面高度说的是"上表面", z_bottom 是"下表面", 别搞混。
        # 按 z_bottom 放会让台面整体抬高一个板厚, 工件就全埋进台面里了。
        objs.append(box_object(
            "table", a.frame_id,
            (t["center"][0], t["center"][1],
             float(t["top_z"]) - float(t["thickness"]) / 2.0),
            (t["size"][0], t["size"][1], t["thickness"])))
        th = float(a.raw["track1"]["station_thickness"])
        self.station_top = th
        ss = float(a.raw["track1"]["station_size"])
        station_ids = [f"station_{i}" for i in range(1, 7)]
        if track == 1:
            for i, x, y in a.stations():
                objs.append(box_object(f"station_{i}", a.frame_id, (x, y, 0.0),
                                       (ss, ss, th), z_bottom=0.0))
        else:
            # 赛道二的散放区 y[190,350] 与赛道一的工位阵 y[155,405] **大面积重叠**,
            # 把工位板留在场上, 散放的工件就会嵌进 3mm 高的板子里
            # (实测报 station_5 与附着工件碰撞)。只摆当前赛道需要的东西。
            self.scene.remove_existing(station_ids)
        active = "bin_t1" if track == 1 else "bin_t2"
        stale = "bin_t2" if track == 1 else "bin_t1"
        if track == 1:
            objs += bin_container("bin_t1", a.frame_id, a.track1_bin,
                                  a.track1_bin_height)
        else:
            objs += bin_container("bin_t2", a.frame_id, a.track2_bin,
                                  a.track2_bin_height)
        # 把另一条赛道的料盒彻底删掉(换赛道重出题时用得上)
        self.scene.remove_existing([f"{stale}_{part}" for part in
                                    ("floor", "wall_x0", "wall_x1",
                                     "wall_y0", "wall_y1")])
        if not self.scene.apply(objs):
            self.get_logger().error("静态场景下发失败")
        else:
            parts = ["工作台"]
            if track == 1:
                parts.append(f"工位板(高 {th*1000:.0f}mm)")
            parts += [active, "二维码卡"]
            self.get_logger().info(" / ".join(parts) + " 已摆放")

    # ---------- 出题 ----------
    def _on_generate(self, request, response):
        track = int(self.get_parameter("track").value)
        seed = int(self.get_parameter("seed").value)
        if seed < 0:
            seed = random.randrange(1 << 30)
        rng = random.Random(seed)

        # 每次出题都重下一遍静态场景: 幂等(同一批 id 重新 ADD), 但能扛住
        # "move_group 中途重启 -> 规划场景被清空" 这种情况。
        self._build_static(track)
        self._clear_dynamic()
        if track == 1:
            q = self.arena.qr
            self.scene.apply([box_object("qr_card", self.arena.frame_id,
                                         (q.center[0], q.center[1], 0.0),
                                         (q.size[0], q.size[1], 0.002), z_bottom=0.0)])
        if track == 1:
            info = self._generate_track1(rng, seed)
        elif track == 2:
            info = self._generate_track2(rng, seed)
        else:
            response.success = False
            response.message = f"赛道只能是 1 或 2, 收到 {track}"
            return response

        self.current = info
        msg = String()
        msg.data = json.dumps(info, ensure_ascii=False)
        self._scene_pub.publish(msg)
        response.success = True
        response.message = json.dumps(
            {k: v for k, v in info.items() if k != "objects"}, ensure_ascii=False)
        self.get_logger().info(f"已生成赛道{track} 题目 seed={seed}")
        return response

    def _on_reset(self, request, response):
        self._clear_dynamic()
        self._build_static()
        response.success = True
        response.message = "场景已复位"
        return response

    def _all_workpiece_ids(self) -> list:
        return [f"wp_{i}" for i in range(1, 21)]

    def _clear_dynamic(self):
        ids = self._all_workpiece_ids()
        # 先摘掉可能还挂在末端上的(attached 工件在世界坐标里是"不存在"的,
        # 对世界下发 REMOVE 完全无效), 再清世界里的。
        # 用 *_existing 版本: 只动场景里真的有的东西, 否则 MoveIt 会为每个
        # 不存在的 id 刷一行 ERROR("Attached body 'wp_7' not found"), 一次出题
        # 几十行噪声, 现场看不清真正的问题。
        self.scene.detach_existing(ids)
        self.scene.remove_existing(ids)
        self.current = {}

    def _generate_track1(self, rng: random.Random, seed: int) -> Dict:
        t1 = self.arena.raw["track1"]
        lib = t1["workpieces"]
        order = make_order(seed, require_derangement=True)
        text = payload(order)
        png = os.path.join(self.qr_dir, f"qr_track1_{seed}.png")
        render_png(text, png)

        objs, manifest = [], []
        for i, (x, y) in enumerate(
                [self.arena.station_xy(k) for k in range(1, 7)], start=1):
            lib_item = lib[i - 1]
            size = float(lib_item["size"])
            obj_id = f"wp_{i}"
            # 工件坐在工位标记板上, 底面 z = 板厚 + OBJECT_LIFT(避免贴面误判)
            z_bottom = self.station_top + OBJECT_LIFT
            objs.append(box_object(obj_id, self.arena.frame_id, (x, y, 0.0),
                                   (size, size, size), z_bottom=z_bottom))
            top = z_bottom + size
            manifest.append({
                "id": obj_id, "station": i, "label": lib_item["name"],
                "xy": [x, y], "size": size, "z": z_bottom + size / 2.0,
                "top_z": top, "shape": "box",
                "grasp_tcp_z": self.arena.grasp_tcp_z(top, size),
                "approach_z": self.arena.approach_z(self.arena.grasp_tcp_z(top, size)),
                "release_tcp_z": self.arena.release_tcp_z(size),
            })
        self.scene.apply(objs)
        return {"track": 1, "seed": seed, "qr_payload": text, "qr_order": order,
                "qr_png": png, "objects": manifest,
                "bin": {"center": list(self.arena.track1_bin.center),
                        "size": list(self.arena.track1_bin.size),
                        "height": self.arena.track1_bin_height}}

    def _generate_track2(self, rng: random.Random, seed: int) -> Dict:
        t2 = self.arena.raw["track2"]
        n = self.arena.t2_objects_per_round
        area: Box = self.arena.scatter
        lo, hi = t2["object_size_range"]
        gap = self.arena.t2_min_gap

        def attempt() -> List[Tuple[float, float, float]]:
            """一次随机序列投放(随机顺序 + 拒绝采样)。"""
            placed: List[Tuple[float, float, float]] = []
            for _ in range(4000):
                if len(placed) >= n:
                    break
                size = rng.uniform(lo, hi)
                half = size / 2.0
                x = rng.uniform(area.x_range[0] + half, area.x_range[1] - half)
                y = rng.uniform(area.y_range[0] + half, area.y_range[1] - half)
                r = radius(x, y)
                # 内外边界都要判: 太靠近基座会撞本体, 太远则 IK 无解。
                if r < self.arena.r_min or r > self.arena.r_max:
                    continue
                if all(abs(x - px) >= half + ph + gap
                       or abs(y - py) >= half + ph + gap for px, py, ph in placed):
                    placed.append((x, y, half))
            return placed

        # 单次贪心投放经常"先摆的大件把后面挤死"。整轮重来几十次,
        # 取最好的一次 —— 实测 300 个种子 100% 能放下 12 件。
        placed = attempt()
        best = list(placed)
        for _ in range(120):
            if len(best) >= n:
                break
            cur = attempt()
            if len(cur) > len(best):
                best = cur
        placed = best
        if len(placed) < n:
            self.get_logger().warn(
                f"只放下了 {len(placed)}/{n} 件, 请放宽散放区或减小工件尺寸上限")
        else:
            self.get_logger().info(
                f"散放区投放 {len(placed)}/{n} 件, "
                f"r = {min(radius(x, y) for x, y, _ in placed)*1000:.0f} .. "
                f"{max(radius(x, y) for x, y, _ in placed)*1000:.0f} mm")

        objs, manifest = [], []
        labels = rng.sample(OBJECT_LIBRARY, k=min(len(placed), len(OBJECT_LIBRARY)))
        for i, (x, y, half) in enumerate(placed, start=1):
            obj_id = f"wp_{i}"
            size = half * 2
            shape = "cylinder" if i % 4 == 0 else "box"
            if shape == "cylinder":
                objs.append(cylinder_object(obj_id, self.arena.frame_id,
                                            (x, y, half + OBJECT_LIFT), half, size))
            else:
                objs.append(box_object(obj_id, self.arena.frame_id, (x, y, 0.0),
                                       (size, size, size), z_bottom=OBJECT_LIFT))
            top = 2 * half + OBJECT_LIFT
            manifest.append({
                "id": obj_id, "label": labels[i - 1], "xy": [x, y],
                "size": size, "z": half + OBJECT_LIFT, "top_z": top, "shape": shape,
                "grasp_tcp_z": self.arena.grasp_tcp_z(top, size),
                "approach_z": self.arena.approach_z(self.arena.grasp_tcp_z(top, size)),
                "release_tcp_z": self.arena.release_tcp_z(size),
            })
        self.scene.apply(objs)
        return {"track": 2, "seed": seed, "objects": manifest,
                "bin": {"center": list(self.arena.track2_bin.center),
                        "size": list(self.arena.track2_bin.size),
                        "height": self.arena.track2_bin_height}}


def main(args=None):
    rclpy.init(args=args)
    node = SceneGenerator()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
