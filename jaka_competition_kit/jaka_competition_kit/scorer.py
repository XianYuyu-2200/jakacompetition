"""自动计时与判分。

设计原则:**不信任队伍上报的事件本身**。
  - 抓取事件必须发生在该工件真值位置的 ``grasp_xy_m`` 容差内;
  - 释放事件必须落在料盒内缩 ``bin_xy_m`` 的区域里;
  - 顺序、掉件、多抓全部由事件序列 + 真值几何推断。

主题 / 服务:
  ``/competition/scene``   (in,  latched) 本轮真值
  ``/competition/events``  (in)          队伍执行事件(grasp/release/abort/collision)
  ``/competition/start``      (srv Trigger) 裁判发令, 开始计时
  ``/competition/abort``      (srv Trigger) 终止本轮
  ``/competition/run/reset``  (srv Trigger) 清空判分状态(须与出题前一起调用)
  ``/competition/score``   (out, latched) 成绩单 JSON

成绩单同时写到 ``~/.ros/jaka_competition/score_<时间>.json`` / ``.txt``。
"""
from __future__ import annotations

import json
import math
import os
import time
from typing import Dict, List, Optional

import rclpy
import rclpy.node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .arena import load_arena, load_rules, radius


class Scorer(rclpy.node.Node):
    def __init__(self):
        super().__init__("competition_scorer")
        self.declare_parameter("arena_config", "")
        self.declare_parameter("rules_config", "")
        self.declare_parameter("output_dir", os.path.expanduser("~/.ros/jaka_competition"))
        self.declare_parameter("team", "team_unknown")
        self.declare_parameter("round", "")

        self.arena = load_arena(self.get_parameter("arena_config").value or None)
        self.rules = load_rules(self.get_parameter("rules_config").value or None)
        self.out_dir = os.path.expanduser(str(self.get_parameter("output_dir").value))
        os.makedirs(self.out_dir, exist_ok=True)

        self.track: Optional[int] = None
        self.scene: Dict = {}
        self.state = "idle"                 # idle | armed | running | finished
        self.t_start: Optional[float] = None
        self.t_end: Optional[float] = None

        self.correct_order: List[int] = []
        self.order_ptr = 0
        self.collected: List[Dict] = []
        self.out_of_order: List[Dict] = []
        self.dropped: List[Dict] = []
        self.collisions = 0
        self.interventions = 0
        self.holding: Optional[str] = None
        self.log: List[str] = []

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/competition/scene", self._on_scene, latched)
        self.create_subscription(String, "/competition/events", self._on_event, 50)
        self._score_pub = self.create_publisher(String, "/competition/score", latched)
        self._state_pub = self.create_publisher(String, "/competition/run_state", latched)
        self.create_service(Trigger, "/competition/start", self._on_start)
        self.create_service(Trigger, "/competition/abort", self._on_abort)
        self.create_service(Trigger, "/competition/run/reset", self._on_reset)
        self.create_timer(0.2, self._tick)
        self._publish_state()
        self.get_logger().info("判分器就绪,等待 /competition/generate 与 /competition/start")

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self._state_pub.publish(msg)

    # ---------- 输入 ----------
    def _on_scene(self, msg: String):
        self.scene = json.loads(msg.data)
        self.track = self.scene.get("track")
        self.correct_order = list(self.scene.get("qr_order", []))
        self._reset_round()
        self.state = "armed"
        self._publish_state()
        n = len(self.scene.get("objects", []))
        self.get_logger().info(f"收到赛道{self.track} 真值: {n} 件工件")

    def _on_event(self, msg: String):
        try:
            ev = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"事件不是合法 JSON: {msg.data!r}")
            return
        kind = ev.get("event", "")
        if kind == "grasp":
            self._handle_grasp(ev)
        elif kind == "release":
            self._handle_release(ev)
        elif kind == "abort":
            # 队伍主动声明"这次尝试作废, 工件已放回原位" —— 不计掉落、不计扣分。
            # 判分器仍然要它发生在持有期间, 否则就是伪造事件。
            oid = ev.get("object_id", "")
            if self.holding == oid:
                self.holding = None
                self._log(f"↺ {oid} 本次尝试作废(工件放回原位, 不计掉落)")
            else:
                self._log(f"作废事件引用了未持有的 {oid!r}, 忽略")
        elif kind == "collision":
            self.collisions += 1
            self._log(f"碰撞 #{self.collisions}: {ev.get('detail','')}")
            if self.collisions >= self.rules[f"track{self.track or 1}"]["abort_after_collisions"]:
                self._finish("累计碰撞达到上限")
        elif kind == "intervention":
            self.interventions += 1
            self._log(f"人工干预 #{self.interventions}: {ev.get('detail','')}")
            if self.interventions >= 2:
                self._finish("累计干预 2 次,成绩无效")
        else:
            self._log(f"未知事件: {kind}")

    # ---------- 判分核心 ----------
    def _object_truth(self, obj_id: str) -> Optional[Dict]:
        for o in self.scene.get("objects", []):
            if o["id"] == obj_id:
                return o
        return None

    def _handle_grasp(self, ev: Dict):
        if self.state != "running":
            self._log(f"非比赛时段忽略抓取: {ev.get('object_id')}")
            return
        obj_id = ev.get("object_id", "")
        truth = self._object_truth(obj_id)
        pose = ev.get("pose") or [0, 0, 0]
        if truth is None:
            self._log(f"抓取事件引用了不存在的工件 {obj_id!r}, 不计分")
            return
        tol = self.rules["tolerance"]["grasp_xy_m"]
        d = math.hypot(pose[0] - truth["xy"][0], pose[1] - truth["xy"][1])
        if d > tol:
            self._log(f"抓取点偏离 {obj_id} 真值 {d*1000:.0f}mm > {tol*1000:.0f}mm, 判无效")
            return
        if self.holding is not None:
            self.dropped.append({"id": self.holding, "t": self.elapsed()})
            self._log(f"{self.holding} 未投放即被替换, 判掉落")
        self.holding = obj_id
        self._log(f"抓取 {obj_id} (距真值 {d*1000:.0f}mm)")

    def _handle_release(self, ev: Dict):
        if self.state != "running":
            return
        obj_id = ev.get("object_id", "")
        pose = ev.get("pose") or [0, 0, 0]
        if self.holding != obj_id:
            self._log(f"释放 {obj_id} 但当前并未持有它, 忽略")
            return
        self.holding = None
        truth = self._object_truth(obj_id)
        if truth is None:
            return

        bin_cfg = self.scene.get("bin", {})
        center = bin_cfg.get("center", [0, 0])
        size = bin_cfg.get("size", [0, 0])
        tol = self.rules["tolerance"]["bin_xy_m"]
        inside = (abs(pose[0] - center[0]) <= size[0] / 2 - tol
                  and abs(pose[1] - center[1]) <= size[1] / 2 - tol
                  and pose[2] >= 0.0)
        if not inside:
            self.dropped.append({"id": obj_id, "t": self.elapsed(),
                                 "pose": list(pose)})
            self._log(f"{obj_id} 释放点不在料盒内, 判掉落")
            return

        entry = {"id": obj_id, "t": self.elapsed(), "label": truth.get("label", "")}
        if self.track == 1:
            station = truth.get("station")
            expected = (self.correct_order[self.order_ptr]
                        if self.order_ptr < len(self.correct_order) else None)
            entry["station"] = station
            if station == expected:
                self.order_ptr += 1
                entry["order_ok"] = True
                self.collected.append(entry)
                self._log(f"✔ 工位{station} 入盒 (顺序正确)")
            else:
                entry["order_ok"] = False
                self.out_of_order.append(entry)
                self._log(f"✘ 工位{station} 入盒但顺序错误(期望 {expected})")
        else:
            self.collected.append(entry)
            self._log(f"✔ {truth.get('label','')} 入盒 ({len(self.collected)} 件)")

        if self._all_collected():
            self._finish("全部工件入盒")

    def _all_collected(self) -> bool:
        target = 6 if self.track == 1 else len(self.scene.get("objects", []))
        return len(self.collected) >= target and target > 0

    # ---------- 计时 ----------
    def elapsed(self) -> float:
        if self.t_start is None:
            return 0.0
        return (self.t_end if self.t_end is not None else time.time()) - self.t_start

    def _tick(self):
        if self.state != "running":
            return
        limit = self.rules[f"track{self.track}"]["time_limit_s"]
        if self.elapsed() >= limit:
            self._finish("超时")

    def _on_start(self, request, response):
        self._reset_round()
        self.t_start = time.time()
        self.t_end = None
        self.state = "running"
        self._publish_state()
        response.success = True
        response.message = "计时开始"
        self.get_logger().info("=== 计时开始 ===")
        return response

    def _on_abort(self, request, response):
        self._finish("裁判终止")
        response.success = True
        response.message = "本轮已终止"
        return response

    def _on_reset(self, request, response):
        self._reset_round()
        self.state = "idle"
        self._publish_state()
        response.success = True
        response.message = "判分器已复位"
        return response

    def _reset_round(self):
        self.t_start = self.t_end = None
        self.order_ptr = 0
        self.collected, self.out_of_order, self.dropped = [], [], []
        self.collisions = self.interventions = 0
        self.holding = None
        self.log = []

    # ---------- 结算 ----------
    def _finish(self, reason: str):
        if self.state == "finished":
            return
        self.t_end = time.time()
        self.state = "finished"
        self._publish_state()
        sheet = self._compute(reason)
        msg = String()
        msg.data = json.dumps(sheet, ensure_ascii=False)
        self._score_pub.publish(msg)
        self._write_files(sheet)
        self.get_logger().info(
            f"=== 结束({reason}) 用时 {sheet['elapsed_s']:.1f}s 总分 {sheet['total']:.1f} ===")

    def _compute(self, reason: str) -> Dict:
        track = self.track or 1
        cfg = self.rules[f"track{track}"]
        elapsed = self.elapsed()
        base_cfg = cfg["base"]
        eff_cfg = cfg["efficiency"]
        pen_cfg = cfg["penalties"]

        n_ok = len(self.collected)
        base = min(n_ok, base_cfg["max_objects"]) * base_cfg["per_object"]

        if elapsed <= eff_cfg["full_marks_within_s"]:
            efficiency = float(eff_cfg["max"])
        elif track == 1:
            over = elapsed - eff_cfg["full_marks_within_s"]
            efficiency = max(0.0, eff_cfg["max"] - math.ceil(over / 5.0) * eff_cfg["penalty_per_5s"])
        else:
            span = max(1e-6, cfg["time_limit_s"] - eff_cfg["full_marks_within_s"])
            frac = max(0.0, (cfg["time_limit_s"] - elapsed) / span)
            efficiency = eff_cfg["max"] * frac

        p_collision = self.collisions * pen_cfg["collision"]
        p_drop = len(self.dropped) * pen_cfg.get("drop", 0.0)
        p_interv = self.interventions * pen_cfg["intervention"]
        penalties = p_collision + p_drop + p_interv

        invalid = self.interventions >= cfg["invalidate_after_interventions"]
        total = 0.0 if invalid else max(0.0, base + efficiency - penalties)

        return {
            "track": track,
            "track_name": cfg["name"],
            "team": self.get_parameter("team").value,
            "round": self.get_parameter("round").value,
            "seed": self.scene.get("seed"),
            "finish_reason": reason,
            "started": self.t_start is not None,
            "elapsed_s": round(elapsed, 2),
            "time_limit_s": cfg["time_limit_s"],
            "generated_order": self.correct_order if track == 1 else None,
            "collected": self.collected,
            "collected_count": n_ok,
            "out_of_order": self.out_of_order,
            "dropped": self.dropped,
            "collisions": self.collisions,
            "interventions": self.interventions,
            "scores": {
                "base": round(base, 1),
                "efficiency": round(efficiency, 1),
                "collision_penalty": round(p_collision, 1),
                "drop_penalty": round(p_drop, 1),
                "intervention_penalty": round(p_interv, 1),
                "innovation_bonus": 0.0,
            },
            "invalid": invalid,
            "total": round(total, 1),
            "log": self.log,
        }

    def _write_files(self, sheet: Dict):
        stamp = time.strftime("%Y%m%d-%H%M%S")
        json_path = os.path.join(self.out_dir, f"score_{stamp}.json")
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(sheet, f, ensure_ascii=False, indent=2)
        with open(os.path.join(self.out_dir, f"score_{stamp}.txt"), "w", encoding="utf-8") as f:
            f.write(self.format_sheet(sheet))
        self.get_logger().info(f"成绩单已写入 {json_path}")

    @staticmethod
    def format_sheet(s: Dict) -> str:
        sc = s["scores"]
        lines = [
            "=" * 52,
            f"  {s['track_name']}  成绩单",
            "=" * 52,
            f"  队伍: {s['team']}    轮次: {s['round']}    题目种子: {s['seed']}",
            f"  结束原因: {s['finish_reason']}",
            f"  用时: {s['elapsed_s']:.1f} s / 限时 {s['time_limit_s']:.0f} s",
            "-" * 52,
        ]
        if s["generated_order"]:
            lines.append(f"  二维码顺序: {''.join(map(str, s['generated_order']))}")
            got = [str(e.get("station")) for e in s["collected"]]
            lines.append(f"  正确入盒顺序: {''.join(got) if got else '(无)'}")
        lines += [
            f"  成功入盒: {s['collected_count']} 件",
            f"  错序: {len(s['out_of_order'])} 件   掉落: {len(s['dropped'])} 件",
            f"  碰撞: {s['collisions']} 次   人工干预: {s['interventions']} 次",
            "-" * 52,
            f"  基础分   {sc['base']:>7.1f}",
            f"  效率分   {sc['efficiency']:>7.1f}",
            f"  碰撞扣分 {-sc['collision_penalty']:>7.1f}",
            f"  掉落扣分 {-sc['drop_penalty']:>7.1f}",
            f"  干预扣分 {-sc['intervention_penalty']:>7.1f}",
            "-" * 52,
            f"  总  分   {s['total']:>7.1f}" + ("   [成绩无效]" if s["invalid"] else ""),
            "=" * 52,
        ]
        return "\n".join(lines) + "\n"

    def _log(self, text: str):
        self.log.append(f"[{self.elapsed():6.1f}s] {text}")
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = Scorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
