"""运动后端。

**仿真与真机使用完全相同的接口**:两者都是向 MoveGroup 的 ``/move_action``
发目标。区别只在启动了什么:

  仿真  → ``ros2 launch jaka_minicobo_moveit_config demo.launch.py use_rviz_sim:=true``
  真机  → ``ros2 launch jaka_planner moveit_server.launch.py ip:=<ip> model:=minicobo``
          ``ros2 launch jaka_minicobo_moveit_config demo.launch.py``  (不带 use_rviz_sim)

所以本模块不需要知道自己在仿真还是真机。
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, MotionPlanRequest, OrientationConstraint,
                             PlanningOptions, PositionConstraint)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


@dataclass
class MoveResult:
    ok: bool
    error_code: int
    elapsed: float
    planning_time: float = 0.0

    def __bool__(self) -> bool:
        return self.ok

    def __str__(self) -> str:
        return (f"{'OK' if self.ok else 'FAIL'}(error_code={self.error_code}, "
                f"{self.elapsed:.2f}s)")


class MoveGroupBackend:
    def __init__(self, node, group_name: str = "jaka_minicobo",
                 tcp_link: str = "dummy_tcp", frame_id: str = "world",
                 vel_scale: float = 0.3, acc_scale: float = 0.3,
                 planning_attempts: int = 3, allowed_planning_time: float = 3.0,
                 orientation_tol: float = 0.05):
        self.node = node
        self.group = group_name
        self.tcp_link = tcp_link
        self.frame_id = frame_id
        self.vel_scale = vel_scale
        self.acc_scale = acc_scale
        self.planning_attempts = planning_attempts
        self.allowed_planning_time = allowed_planning_time
        self.orientation_tol = orientation_tol

        from rclpy.action import ActionClient
        self._ac = ActionClient(node, MoveGroup, "/move_action")
        if not self._ac.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("/move_action 不可用 —— move_group 起了吗?")

        self._joints: List[str] = []
        self._positions: List[float] = []
        node.create_subscription(JointState, "/joint_states", self._on_joint, 10)

    # ---------- 状态 ----------
    def _on_joint(self, msg: JointState):
        self._joints = list(msg.name)
        self._positions = list(msg.position)

    def joint_state(self, timeout: float = 5.0) -> Optional[List[float]]:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if len(self._positions) >= 6 and any(abs(v) > 1e-9 for v in self._positions):
                return self._positions[:6]
        return self._positions[:6] if len(self._positions) >= 6 else None

    # ---------- 运动 ----------
    def _goal(self, position: Sequence[float], quat: Sequence[float],
              plan_only: bool) -> MoveGroup.Goal:
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in position)
        ps.pose.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]),
                                         z=float(quat[2]), w=float(quat[3]))

        pc = PositionConstraint()
        pc.header = ps.header
        pc.link_name = self.tcp_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(ps.pose)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = ps.header
        oc.link_name = self.tcp_link
        oc.orientation = ps.pose.orientation
        oc.absolute_x_axis_tolerance = self.orientation_tol
        oc.absolute_y_axis_tolerance = self.orientation_tol
        oc.absolute_z_axis_tolerance = self.orientation_tol
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)

        req = MotionPlanRequest()
        req.group_name = self.group
        req.num_planning_attempts = self.planning_attempts
        req.allowed_planning_time = self.allowed_planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        req.goal_constraints.append(c)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = not plan_only
        return goal

    def goto(self, position: Sequence[float], quat: Sequence[float],
             plan_only: bool = False) -> MoveResult:
        goal = self._goal(position, quat, plan_only)
        t0 = time.time()
        fut = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return MoveResult(False, -1, time.time() - t0)
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf)
        res = rf.result().result
        return MoveResult(res.error_code.val == 1, res.error_code.val, time.time() - t0)

    def goto_xyz(self, x: float, y: float, z: float,
                 quat: Sequence[float], plan_only: bool = False) -> MoveResult:
        return self.goto((x, y, z), quat, plan_only)

    # ---------- 直线运动 ----------
    def goto_xyz_straight(self, x: float, y: float, z: float,
                          quat: Sequence[float], max_step: float = 0.005,
                          min_fraction: float = 0.95) -> MoveResult:
        """让末端沿**空间直线**走到目标点(笛卡尔路径), 失败返回 ok=False。

        为什么需要它: OMPL(RRTConnect)返回的是关节空间里的随机路径,
        末端在中间过程会甩来甩去。抓取/投放本来就是"竖直进、竖直出",
        这一甩会让末端夹着的工件蹭到旁边的工件或料盒侧墙,规划直接报
        `Computed path is not valid`(-2)。

        典型场景: 赛道二工件之间只有 10mm 间隙, 关节空间路径十有八九会蹭到。
        """
        from moveit_msgs.srv import GetCartesianPath
        from moveit_msgs.action import ExecuteTrajectory
        from rclpy.action import ActionClient

        if not hasattr(self, "_cart"):
            self._cart = self.node.create_client(GetCartesianPath,
                                                 "/compute_cartesian_path")
            if not self._cart.wait_for_service(timeout_sec=15.0):
                return MoveResult(False, -6, 0.0)
            self._exec_ac = ActionClient(self.node, ExecuteTrajectory,
                                         "/execute_trajectory")
            self._exec_ac.wait_for_server(timeout_sec=15.0)

        ps = Pose()
        ps.position.x, ps.position.y, ps.position.z = float(x), float(y), float(z)
        ps.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]),
                                    z=float(quat[2]), w=float(quat[3]))
        req = GetCartesianPath.Request()
        req.header.frame_id = self.frame_id
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.group_name = self.group
        req.link_name = self.tcp_link
        req.waypoints = [ps]
        req.max_step = max_step
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale

        t0 = time.time()
        fut = self._cart.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
        res = fut.result()
        dt = time.time() - t0
        if res is None or res.fraction < min_fraction:
            frac = None if res is None else res.fraction
            self.node.get_logger().debug(
                f"直线路径不可用(fraction={frac}), 回退到关节空间规划")
            return MoveResult(False, -2, dt)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        gh = self._exec_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, gh, timeout_sec=30.0)
        handle = gh.result()
        if handle is None or not handle.accepted:
            return MoveResult(False, -1, time.time() - t0)
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120.0)
        res2 = rf.result()
        code = res2.result.error_code.val if res2 is not None else -1
        return MoveResult(code == 1, code, time.time() - t0)

    def validate_pose(self, position: Sequence[float],
                      quat: Sequence[float]) -> MoveResult:
        """只规划不执行 —— 用于赛前自检。"""
        return self.goto(position, quat, plan_only=True)


def quat_down() -> Tuple[float, float, float, float]:
    """工具轴竖直向下(绕 X 转 180°)。"""
    return (1.0, 0.0, 0.0, 0.0)


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)
