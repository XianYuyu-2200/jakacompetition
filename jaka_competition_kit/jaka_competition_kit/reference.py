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
from .fixed_poses import default_fixed_pose_path, is_complete, load_fixed_poses
from .gripper import make_gripper
from .scene import SceneClient


def object_height(obj: Dict) -> float:
    """Return the workpiece height, accepting both current and legacy manifests.

    ``size`` is the historical height field. Newer scene descriptions also carry
    ``dims``; use its Z dimension when consuming a manifest that omitted ``size``.
    """
    if "size" in obj:
        return float(obj["size"])
    dims = obj.get("dims")
    if isinstance(dims, (list, tuple)) and len(dims) == 3:
        return float(dims[2])
    raise ValueError(
        f"工件 {obj.get('id', '<unknown>')} 缺少 size，且 dims 不是三维尺寸")


class ReferenceBase(rclpy.node.Node):
    """等题 -> 等发令 -> 按顺序抓取投放 -> 汇总。"""

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.declare_parameter("arena_config", "")
        self.declare_parameter("vel_scale", 0.5)
        self.declare_parameter("verify_only", False)
        # 默认 sim: 仿真的 fingers 会张开/闭合(走 /gripper_controller/commands);
        # 真机用 wheeltec(串口) / jaka_io(工具端 NPN: DO_1 张, DO_2 合) / mock
        self.declare_parameter("gripper", "sim")
        self.declare_parameter("io_open_index", 0)    # 工具端 DO_1
        self.declare_parameter("io_close_index", 1)   # 工具端 DO_2
        self.declare_parameter("io_settle", 0.5)
        self.declare_parameter("debug_timing", False)
        self.declare_parameter("fixed_poses_config", default_fixed_pose_path())
        self.declare_parameter("fixed_poses_backend", "simulation")
        self.declare_parameter("fixed_poses_device_profile", "jaka_mini_2_gemini305_gripper_sim")
        self.declare_parameter("allow_cartesian_fallback", True)

        self.arena = load_arena(str(self.get_parameter("arena_config").value) or None)
        pose_path = str(self.get_parameter("fixed_poses_config").value) or default_fixed_pose_path()
        try:
            self.fixed_poses = load_fixed_poses(pose_path)
        except (OSError, ValueError) as exc:
            self.get_logger().warning(f"固定姿态配置不可用，将使用基础笛卡尔流程：{exc}")
            self.fixed_poses = {}
        self.fixed_poses_backend = str(self.get_parameter("fixed_poses_backend").value)
        self.fixed_poses_device_profile = str(
            self.get_parameter("fixed_poses_device_profile").value)
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
        gripper_mode = str(self.get_parameter("gripper").value)
        gripper_kwargs = {}
        if gripper_mode.lower() in ("jaka_io",):
            gripper_kwargs = {
                "open_index": int(self.get_parameter("io_open_index").value),
                "close_index": int(self.get_parameter("io_close_index").value),
                "settle": float(self.get_parameter("io_settle").value),
            }
        self.gripper = make_gripper(gripper_mode, self, arena=self.arena,
                                    **gripper_kwargs)
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
    def wait_for_state(self, target: str, timeout: Optional[float] = None) -> bool:
        """等裁判发令; 默认不限时, 可指定 timeout 用于自检。"""
        t0 = time.monotonic()
        while rclpy.ok() and (timeout is None or time.monotonic() - t0 < timeout):
            if self.run_state == target:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.run_state == target

    def wait_for_scene(self, timeout: Optional[float] = None) -> bool:
        """等裁判出题; 默认不限时, 避免场景尚未准备好时参考程序退出。"""
        t0 = time.monotonic()
        while rclpy.ok() and (timeout is None or time.monotonic() - t0 < timeout):
            if self.scene:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return bool(self.scene)

    # ---------- 子类实现 ----------
    def plan_order(self) -> List[Dict]:
        """返回本轮要按顺序抓取的工件真值列表。"""
        raise NotImplementedError

    # ---------- 主循环 ----------
    def run(self):
        self.get_logger().info("等待裁判出题 (/competition/generate) ...")
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
                self.get_logger().error("等待发令中断, 退出")
                return
            self.get_logger().info("发令! 开始执行")

        objects = self.plan_order()
        if not objects:
            self.get_logger().error("没有可抓的工件, 退出")
            return
        t0 = time.time()
        ok = 0
        total = len(objects)
        fixed_ready = is_complete(
            self.fixed_poses, backend=self.fixed_poses_backend,
            device_profile=self.fixed_poses_device_profile)
        if self.scene.get("track") == 1 and not fixed_ready:
            allow_fallback = bool(self.get_parameter("allow_cartesian_fallback").value)
            if not allow_fallback:
                self.get_logger().error(
                    "固定姿态配置未完成或 profile 不匹配，且 allow_cartesian_fallback=false；"
                    "为避免意外执行其它策略，本轮停止。")
                return
            self.get_logger().warning(
                "固定姿态配置未完成或 profile 不匹配；按显式参数 "
                "allow_cartesian_fallback=true 回退到笛卡尔基础流程。")
        for k, obj in enumerate(objects, 1):
            if self.scene.get("track") == 1:
                bin_box = self.arena.track1_bin
            else:
                bin_box = self.arena.track2_bin_by_id(obj["bin_id"])["box"]
            x, y = obj["xy"]
            r = (x ** 2 + y ** 2) ** 0.5
            tag = f"工位{obj['station']} " if obj.get("station") else ""
            self.get_logger().info(
                f"[{k}/{total}] {tag}{obj.get('label','')} "
                f"x={x*1000:.0f} y={y*1000:.0f} r={r*1000:.0f}mm")
            use_fixed = (self.scene.get("track") == 1 and fixed_ready
                         and bool(obj.get("station")))
            if use_fixed and verify_only:
                phases = [self.fixed_poses["objects"][str(obj["station"])][p]
                          for p in ("approach_joints", "grasp_joints", "lift_joints")]
                phases += [self.fixed_poses["place"][p] for p in
                           ("approach_joints", "release_joints", "retreat_joints")]
                results = [self.backend.goto_joints(j, plan_only=True) for j in phases]
                ok += int(all(r.ok for r in results))
                self.get_logger().info(f"    固定姿态规划自检: {sum(r.ok for r in results)}/{len(results)}")
                continue

            # 赛道二允许单件重试 2 次(见 competition.md), 赛道一同样受益。
            good = False
            for attempt in range(3):
                if use_fixed:
                    good = self.arm.fixed_pick_and_place(obj, bin_box, self.fixed_poses)
                    if good:
                        self.get_logger().info("    使用已记录固定关节姿态")
                else:
                    good = self.arm.pick_and_place(
                        obj["id"], x, y, obj["grasp_tcp_z"], obj["z"], object_height(obj),
                        bin_box, obj["release_tcp_z"], approach_z=obj["approach_z"],
                        dims=obj.get("dims"), shape=obj.get("shape", "box"),
                        grasp_width=obj.get("grasp_width"))
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
