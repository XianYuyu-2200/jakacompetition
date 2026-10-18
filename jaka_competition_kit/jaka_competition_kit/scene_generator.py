"""场景生成器 —— 官方出题程序。

职责:
  1. 把工作台 / 工位标记 / 料盒 / 二维码卡摆进 MoveIt 规划场景;
  2. 为赛道一生成一轮二维码序令, 并写出 PNG(裁判打印用);
  3. 为赛道二每类抽 4 件(共 16 件)摆进散放区, 并摆好四只分类料盒;
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
from typing import Dict, List, Optional

import rclpy
import rclpy.node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .arena import OBJECT_LIFT, Arena, load_arena, radius
from .qr import make_order, parse, payload, render_png
from .scene import SceneClient, bin_container, box_object, cylinder_object

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
            # 赛道二的散放区 y[190,365] 与赛道一的工位阵 y[155,405] **大面积重叠**,
            # 把工位板留在场上, 散放的工件就会嵌进 3mm 高的板子里
            # (实测报 station_5 与附着工件碰撞)。只摆当前赛道需要的东西。
            self.scene.remove_existing(station_ids)
        parts = ("floor", "wall_x0", "wall_x1", "wall_y0", "wall_y1")
        t2_prefixes = [f"bin_{b['id']}" for b in a.track2_bins()] + ["bin_t2"]
        if track == 1:
            objs += bin_container("bin_t1", a.frame_id, a.track1_bin,
                                  a.track1_bin_height)
            stale = [f"{pre}_{part}" for pre in t2_prefixes for part in parts]
            active = "bin_t1"
        else:
            for b in a.track2_bins():
                objs += bin_container(f"bin_{b['id']}", a.frame_id, b["box"], b["height"])
            stale = [f"bin_t1_{part}" for part in parts] + [f"bin_t2_{part}" for part in parts]
            active = "四只分类料盒"
            # 二维码只属于赛道一, 换到赛道二时拿掉
            stale.append("qr_card")
        self.scene.remove_existing(stale)
        if not self.scene.apply(objs):
            self.get_logger().error("静态场景下发失败")
        else:
            parts = ["工作台"]
            if track == 1:
                parts.append(f"工位板(高 {th*1000:.0f}mm)")
                parts.append("二维码卡")
            parts.append(active)
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
        catalog = [f"wp_{o['category']}_{o['id']}" for o in self.arena.track2_objects()]
        # 旧版赛道二用过 wp_1..wp_20, 一起清掉, 避免换题后幽灵工件还挂着
        legacy = [f"wp_{i}" for i in range(1, 21)]
        return catalog + legacy

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
        """每类 5 选 4, 共 16 件, 按类别放进对应料盒。"""
        picked, withheld = self.arena.sample_track2(rng)
        placed = self.arena.pack_track2(picked, rng)
        if not placed or len(placed) != len(picked):
            self.get_logger().error("散放区摆不下 16 件, 请检查 arena.yaml")
            placed = placed or []
        else:
            rs = [radius(it["xy"][0], it["xy"][1]) for it in placed]
            counts = {}
            for it in placed:
                counts[it["category_name"]] = counts.get(it["category_name"], 0) + 1
            self.get_logger().info(
                f"散放区投放 {len(placed)}/{self.arena.t2_objects_per_round} 件 "
                + " ".join(f"{k}{v}" for k, v in counts.items())
                + f", r = {min(rs)*1000:.0f} .. {max(rs)*1000:.0f} mm")

        objs, manifest = [], []
        for it in placed:
            x, y = it["xy"]
            sx, sy, sz = (float(v) for v in it["world_dims"])
            shape = it.get("shape", "box")
            obj_id = f"wp_{it['category']}_{it['id']}"
            if shape == "cylinder":
                objs.append(cylinder_object(
                    obj_id, self.arena.frame_id,
                    (x, y, OBJECT_LIFT + sz / 2.0), sx / 2.0, sz))
            else:
                objs.append(box_object(
                    obj_id, self.arena.frame_id, (x, y, 0.0),
                    (sx, sy, sz), z_bottom=OBJECT_LIFT))
            top = OBJECT_LIFT + sz
            grasp_z = self.arena.grasp_tcp_z(top, sz)
            manifest.append({
                "id": obj_id,
                "label": it["name"],
                "category": it["category"],
                "category_name": it["category_name"],
                "bin_id": it["bin_id"],
                "xy": [x, y],
                "yaw_deg": it.get("yaw_deg", 0),
                "dims": [sx, sy, sz],
                "size": sz,
                "grasp_width": min(sx, sy),
                "z": OBJECT_LIFT + sz / 2.0,
                "top_z": top,
                "shape": shape,
                "grasp_tcp_z": grasp_z,
                "approach_z": self.arena.approach_z(grasp_z),
                "release_tcp_z": self.arena.release_tcp_z(sz),
            })
        if objs:
            self.scene.apply(objs)
        bins = [{
            "id": b["id"],
            "name": b["name"],
            "center": list(b["box"].center),
            "size": list(b["box"].size),
            "height": b["height"],
        } for b in self.arena.track2_bins()]
        return {"track": 2, "seed": seed, "objects": manifest, "bins": bins,
                "withheld": [{"id": w["id"], "name": w["name"],
                              "category": w["category"],
                              "category_name": w["category_name"]} for w in withheld]}


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
