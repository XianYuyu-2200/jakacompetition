"""参考实现的公共骨架。

参考实现有两个用途:
  1. 给参赛队一个能跑通的最小示例(仿真/真机同一份代码);
  2. 给主办方做**难度标定** —— 拿它在目标硬件上跑 10 次取中位数,
     乘以 1.8 就是合理的单轮限时。

赛道一/赛道二只在"抓取顺序怎么定"上不同,其余(等题、等发令、计时、
失败复位)完全一致,所以公共部分放在这里。
"""
from __future__ import annotations

import json
import time
from typing import Dict, List, Optional

import rclpy
import rclpy.node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from .arena import load_arena
from .backend import MoveGroupBackend
from .executor import Executor
from .gripper import make_gripper
from .scene import SceneClient


class ReferenceBase(rclpy.node.Node):
    """等题 -> 等发令 -> 按顺序抓取投放 -> 汇总。"""

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.declare_parameter("arena_config", "")
        self.declare_parameter("vel_scale", 0.5)
        self.declare_parameter("verify_only", False)
        # 默认 sim: 仿真的 fingers 会张开/闭合(走 /gripper_controller/commands);
        # 真机用 wheeltec(串口) / jaka_io(工具端 IO) / mock(不驱动)
        self.declare_parameter("gripper", "sim")
        self.declare_parameter("debug_timing", False)

        self.arena = load_arena(str(self.get_parameter("arena_config").value) or None)
        self.scene: Dict = {}
        self.run_state = "idle"

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/competition/scene", self._on_scene, latched)
        self.create_subscription(String, "/competition/run_state", self._on_state, latched)

        vel = float(self.get_parameter("vel_scale").value)
        # 加速度缩放跟着速度缩放走 —— 固定 0.3 会让"全速"名不副实(实测慢一倍)。
        self.backend = MoveGroupBackend(
            self, self.arena.group_name, self.arena.tcp_link, self.arena.frame_id,
            vel_scale=vel, acc_scale=vel,
            planning_attempts=2, allowed_planning_time=2.0)
        self.scene_cli = SceneClient()
        self.gripper = make_gripper(self.get_parameter("gripper").value, self,
                                    arena=self.arena)
        self.arm = Executor(self, self.arena, self.backend, self.scene_cli, self.gripper,
                            timing_log=bool(self.get_parameter("debug_timing").value))
        self.done = False

    # ---------- 回调 ----------
    def _on_scene(self, msg: String):
        self.scene = json.loads(msg.data)
        self.get_logger().info(
            f"收到赛道{self.scene.get('track')} 题目, seed={self.scene.get('seed')}, "
            f"{len(self.scene.get('objects', []))} 件工件")

    def _on_state(self, msg: String):
        self.run_state = msg.data
        self.get_logger().info(f"比赛状态 -> {self.run_state}")

    # ---------- 等待 ----------
    def wait_for_state(self, target: str, timeout: float = 180.0) -> bool:
        """等裁判发令(state 变 running)后才动作, 保证计时公平。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.run_state == target:
                return True
        return False

    def wait_for_scene(self, timeout: float = 60.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scene:
                return True
        return False

    # ---------- 子类实现 ----------
    def plan_order(self) -> List[Dict]:
        """返回本轮要按顺序抓取的工件真值列表。"""
        raise NotImplementedError

    # ---------- 主循环 ----------
    def run(self):
        if not self.wait_for_scene():
            self.get_logger().error("没有收到题目: 先调 /competition/generate")
            return
        verify_only = bool(self.get_parameter("verify_only").value)
        if verify_only:
            # 自检只做规划、不执行, 也没必要等裁判发令 —— 否则"赛前场地自检"
            # 会一直卡在等 /competition/start 上。真机自检尤其需要它马上出结果。
            self.get_logger().info("自检模式: 不等发令, 直接逐点规划(不执行)")
        else:
            self.get_logger().info("题目已就绪, 等待裁判发令 (/competition/start) ...")
            if not self.wait_for_state("running"):
                self.get_logger().error("等待发令超时, 退出")
                return
            self.get_logger().info("发令! 开始执行")

        objects = self.plan_order()
        if not objects:
            self.get_logger().error("没有可抓的工件, 退出")
            return
        bin_box = (self.arena.track1_bin if self.scene.get("track") == 1
                   else self.arena.track2_bin)
        t0 = time.time()
        ok = 0
        total = len(objects)
        for k, obj in enumerate(objects, 1):
            x, y = obj["xy"]
            r = (x ** 2 + y ** 2) ** 0.5
            tag = f"工位{obj['station']} " if obj.get("station") else ""
            self.get_logger().info(
                f"[{k}/{total}] {tag}{obj.get('label','')} "
                f"x={x*1000:.0f} y={y*1000:.0f} r={r*1000:.0f}mm")
            if verify_only:
                res = self.backend.validate_pose((x, y, obj["grasp_tcp_z"]),
                                                 self.arena.tool_down_quat)
                self.get_logger().info(f"    规划自检: {res}")
                ok += int(res.ok)
                continue

            # 赛道二允许单件重试 2 次(见 competition.md), 赛道一同样受益。
            good = False
            for attempt in range(3):
                good = self.arm.pick_and_place(
                    obj["id"], x, y, obj["grasp_tcp_z"], obj["z"], obj["size"],
                    bin_box, obj["release_tcp_z"], approach_z=obj["approach_z"])
                if good:
                    break
                if self.arm.last_failure == "unreachable":
                    # 几何够不到(见 docs/WHEELTEC柔性机械爪.md 第 4 节): 重试 3 次
                    # 只是把同一段几何再算 3 遍。确定性失败就不要假装能重试成功。
                    self.get_logger().warn(
                        "    几何不可达, 不重试: 换短末端或按文档第 4 节的方案调布局")
                    break
                self.get_logger().warn(f"    第 {attempt + 1} 次失败, 复位后重试")
            self.get_logger().info(
                f"    {'成功' if good else '失败'} (累计 {time.time() - t0:.1f}s)")
            ok += int(good)

        self.get_logger().info(
            f"参考实现结束: {ok}/{total}, 总耗时 {time.time() - t0:.1f}s")
        self.done = True


def spin(node: rclpy.node.Node) -> None:
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.scene_cli.close()
        except Exception:                                # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()
