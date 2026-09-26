"""运动后端。

**仿真与真机使用完全相同的接口**:两者都是向 MoveGroup 的 ``/move_action``
发目标。区别只在启动了什么:

  仿真  → ``ros2 launch jaka_minicobo_moveit_config demo.launch.py use_rviz_sim:=true``
  真机  → ``ros2 launch jaka_planner moveit_server.launch.py ip:=<ip> model:=minicobo``
          ``ros2 launch jaka_minicobo_moveit_config demo.launch.py``  (不带 use_rviz_sim)

所以本模块不需要知道自己在仿真还是真机。

**位姿目标为什么不够**: 只约束末端位姿时, 同一个位姿对应多组关节解
(肘上/肘下, 手腕翻转, 以及 J1/J4/J6 绕 ±360° 的等效解)。OMPL 采样到哪组
是随机的, 于是"J1 转几度就够"的动作经常变成整条手臂绕大圈。
本模块因此改成**工业做法**: 先解 IK, 挑一组"离当前关节角最近"的解,
再把目标写成 ``JointConstraint``(关节空间目标) —— 规划器不再有分支可选。

**IK 求解器**: 用 ``lma_kinematics_plugin/LMAKinematicsPlugin``
(见 ``jaka_minicobo_moveit_config/config/kinematics.yaml``)。默认的 KDL
插件是牛顿迭代, 本臂 J5 接近 ±90°(手腕奇异)时从给定种子经常不收敛,
插件会改用随机种子重启 —— 同一个目标反复求解会分别落到不同的关节分支上,
表现为"抓取动作有时走最优轨迹、有时绕大圈"。LMA 是阻尼最小二乘, 从种子
出发几乎总能收敛, 结果可复现(实测同目标同种子连续 10 次完全一致)。
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             OrientationConstraint, PlanningOptions,
                             PositionConstraint, RobotState)
from moveit_msgs.srv import GetPositionIK
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String


@dataclass
class MoveResult:
    ok: bool
    error_code: int
    elapsed: float
    planning_time: float = 0.0
    joint_travel: float = 0.0      # 规划出来的轨迹里, 单关节的最大行程(rad)
    via: str = "pose"              # pose=位姿目标 / joint=最近 IK 关节目标

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
                 orientation_tol: float = 0.05,
                 joint_names: Optional[Sequence[str]] = None,
                 use_nearest_ik: bool = True,
                 use_ptp: bool = True,
                 ik_timeout: float = 0.05,
                 ik_budget: float = 2.0,
                 max_ik_tries: int = 3,
                 joint_tol: float = 0.001,
                 joint_tol_loose: float = 0.02,
                 detour_warn_deg: float = 40.0):
        self.node = node
        self.group = group_name
        self.tcp_link = tcp_link
        self.frame_id = frame_id
        self.vel_scale = vel_scale
        self.acc_scale = acc_scale
        self.planning_attempts = planning_attempts
        self.allowed_planning_time = allowed_planning_time
        self.orientation_tol = orientation_tol
        self.joint_names: List[str] = list(joint_names) if joint_names else [
            f"joint_{i}" for i in range(1, 7)]
        # 关掉就退回老的"纯位姿目标"行为(排障/对比用)
        self.use_nearest_ik = use_nearest_ik
        # 关掉就不试 Pilz PTP, 所有关节目标都交给 OMPL(排障/对比用)
        self.use_ptp = use_ptp
        self.ik_timeout = ik_timeout
        self.ik_budget = ik_budget
        # 关节目标规划失败时, 最多再试几个"次优解"(见 goto)
        self.max_ik_tries = max_ik_tries
        self.joint_tol = joint_tol
        # 兜底用: 关节目标被判定非法时, 放宽到这点容差再试一次(见 goto)。
        # 0.02 rad ≈ 1.15°, 在 0.3m 臂展上约 4~6mm —— 和位姿目标自带的
        # 5mm 球同量级, 但**保留了选定的那一支 IK 解**, 不会随机换支。
        self.joint_tol_loose = joint_tol_loose
        # 规划轨迹比"关节目标本身需要的行程"多出这么多度就告警(见 _log_move)
        self.detour_warn_deg = detour_warn_deg

        from rclpy.action import ActionClient
        self._ac = ActionClient(node, MoveGroup, "/move_action")
        if not self._ac.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("/move_action 不可用 —— move_group 起了吗?")

        self._pos_by_name: dict = {}
        node.create_subscription(JointState, "/joint_states", self._on_joint, 10)

        # 关节限位: 用来把 IK 解"绕回离当前角最近的那一圈"。
        # 拿不到就按 ±2π 兜底(本机器人 J1/J4/J6 就是 ±6.28)。
        self._limits: dict = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, "/robot_description",
                                 self._on_robot_description, latched)

        self._ik_client = None
        self._last_ik_log = -1.0

    # ---------- 状态 ----------
    def _on_joint(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._pos_by_name[name] = float(pos)

    def _on_robot_description(self, msg: String):
        import re
        for m in re.finditer(r'<joint\s+name="([^"]+)"[^>]*>(.*?)</joint>',
                             msg.data, re.S):
            name, body = m.group(1), m.group(2)
            lim = re.search(r'<limit[^>]*lower="([-0-9.eE]+)"[^>]*'
                            r'upper="([-0-9.eE]+)"', body)
            if lim:
                self._limits[name] = (float(lim.group(1)), float(lim.group(2)))

    def joint_state(self, timeout: float = 5.0) -> Optional[List[float]]:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if all(n in self._pos_by_name for n in self.joint_names) and \
                    any(abs(self._pos_by_name[n]) > 1e-9 for n in self.joint_names):
                return [self._pos_by_name[n] for n in self.joint_names]
        if all(n in self._pos_by_name for n in self.joint_names):
            return [self._pos_by_name[n] for n in self.joint_names]
        return None

    # ---------- IK: 挑"离当前关节角最近"的那组解 ----------
    def _wrap_toward(self, name: str, value: float, ref: float) -> float:
        """把角度绕到离 ref 最近的那一圈(限位内)。

        J1/J4/J6 是 ±360°, 同一个物理姿态可以写成 -180° 或 +180°。
        IK 返回哪个表示是任意的, 不归一化就会出现"绕整圈"。
        """
        lo, hi = self._limits.get(name, (-2 * math.pi, 2 * math.pi))
        best, best_d = value, abs(value - ref)
        for k in (-2, -1, 1, 2):
            cand = value + 2 * math.pi * k
            if lo - 1e-9 <= cand <= hi + 1e-9 and abs(cand - ref) < best_d - 1e-9:
                best, best_d = cand, abs(cand - ref)
        return best

    def _ik_once(self, position, quat, seed, avoid_collisions: bool):
        """一次 IK 求解; 解不出来返回 None。seed 是 6 个关节角(rad)。"""
        if self._ik_client is None:
            self._ik_client = self.node.create_client(GetPositionIK, "/compute_ik")
            if not self._ik_client.wait_for_service(timeout_sec=15.0):
                self.node.get_logger().warn("/compute_ik 不可用, 退回位姿目标")
                self.use_nearest_ik = False
                return None
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group
        req.ik_request.ik_link_name = self.tcp_link
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.timeout = Duration(sec=int(self.ik_timeout),
                                          nanosec=int((self.ik_timeout % 1) * 1e9))
        rs = RobotState()
        rs.joint_state.name = list(self.joint_names)
        rs.joint_state.position = [float(v) for v in seed]
        req.ik_request.robot_state = rs

        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = \
            (float(v) for v in position)
        ps.pose.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]),
                                         z=float(quat[2]), w=float(quat[3]))
        req.ik_request.pose_stamped = ps

        fut = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        if not all(n in sol for n in self.joint_names):
            return None
        return [sol[n] for n in self.joint_names]

    def nearest_ik(self, position, quat) -> Optional[List[float]]:
        """解 IK 并返回"最省事"的一组关节角。

        打分用的是工业里最直白的口径: **单关节最大行程**(≈ 最短到位时间,
        因为各关节 max_velocity 相同), 关节总行程做次要判据。

        为什么要枚举而不是"种子=当前角解一次就完事": 6R 臂同一个末端位姿一般
        有 8 组解(肩 ±180° / 肘上肘下 / 手腕翻转), 数值 IK 只收敛到种子附近的
        那一支。种子给当前角时, 结果经常不是行程最短的那一支 —— 参赛队看到的
        就是"有几次抓取没走最优轨迹"。

        实现见 ``ik_candidates``; 这里只是取其中最省事的那一支。
        """
        cands = self.ik_candidates(position, quat)
        return cands[0] if cands else None

    def ik_candidates(self, position, quat) -> List[List[float]]:
        """全部候选 IK 解, 按"单关节最大行程"从小到大排序(可能为空)。

        做法: 用 ``_ik_seeds`` 把各分支的种子铺开各解一次。避障解优先 ——
        ``avoid_collisions=False`` 才解出来的位形本身就在碰撞里, 拿它当目标
        规划必然失败, 只作兜底(避障解非空时直接丢掉这些)。

        **为什么要排序返回全部而不是只给最优解**: 最优的那一支可能因为臂杆
        和场景碰撞而规划不出来(终点位姿没问题 —— 工件挂在末端, 换哪一支工件
        都在同一处, 但臂杆位置不同)。这时应该退而求其次换一支解, 而不是直接
        退回"位姿目标"让规划器随机挑解。实测赛道二有两处 `to_bin` 正是如此,
        退化后每处白跑 10~12s。

        性能: 结构种子只有 4~8 个、一次 IK ≈ 3ms, 整个函数 20~40ms;
        """
        cur = self.joint_state(timeout=1.0)
        if cur is None:
            return []
        deadline = time.time() + self.ik_budget

        def score(sol):
            return (max(abs(a - b) for a, b in zip(sol, cur)),
                    sum(abs(a - b) for a, b in zip(sol, cur)))

        clean: List[List[float]] = []      # 避障解出来的
        risky: List[List[float]] = []      # 放开避障才解出来的(兜底)

        def add(sol, is_clean: bool):
            sol = [self._wrap_toward(n, v, c)
                   for n, v, c in zip(self.joint_names, sol, cur)]
            if any(max(abs(a - b) for a, b in zip(sol, t)) < 1e-6
                   for t in clean + risky):
                return
            (clean if is_clean else risky).append(sol)

        for seed in self._ik_seeds(cur):
            if time.time() > deadline:
                self.node.get_logger().debug("IK 候选枚举超时, 用已有结果")
                break
            sol = self._ik_once(position, quat, seed, True)
            if sol is not None:
                add(sol, True)
                continue
            sol = self._ik_once(position, quat, seed, False)
            if sol is not None:
                add(sol, False)

        pool = clean or risky
        return sorted(pool, key=score)

    def _ik_seeds(self, cur: Sequence[float]) -> List[List[float]]:
        """覆盖 6R 臂各离散 IK 分支的种子, 按"离当前角近"优先排序。

        6R 臂同一个末端位姿一般有 8 组解: 肩部转 ±180°、肘上/肘下、手腕翻转
        (``J4+180°, J5 取反, J6+180°``)。数值 IK 只收敛到种子附近的那一支,
        所以要找行程最小的解, 就得把种子铺到各支上。

        实测这组种子在赛道一/二的 13 个抓取目标上都能取到全局最优分支
        (结果与 96 个随机种子的暴力枚举一致), 耗时却只有它的 1/10。
        """
        pi = math.pi
        out: List[List[float]] = []

        def add(seed):
            s = [self._wrap_toward(n, v, c)
                 for n, v, c in zip(self.joint_names, seed, cur)]
            if not all(self._limits.get(n, (-2 * pi, 2 * pi))[0] - 1e-9 <= v
                       <= self._limits.get(n, (-2 * pi, 2 * pi))[1] + 1e-9
                       for n, v in zip(self.joint_names, s)):
                return
            if any(max(abs(a - b) for a, b in zip(s, t)) < 1e-6 for t in out):
                return
            out.append(s)

        def wrist_flip(s):
            s = list(s)
            s[3] += pi
            s[4] = -s[4]
            s[5] += pi
            return s

        def elbow_flip(s):
            s = list(s)
            s[1] = -s[1]
            s[2] = -s[2]
            return s

        add(cur)                                    # 同侧解, 最常用
        for k in (-1, 1):                           # 同分支的 ±360° 等效表示
            add([c + 2 * pi * k for c in cur])
        add([0.0] * len(cur))                       # 零位: 关节角表示差很远的一支
        for dj1 in (0.0, pi, -pi):                  # 肩部反向
            base = list(cur)
            base[0] += dj1
            add(base)
            add(wrist_flip(base))
            add(wrist_flip(elbow_flip(base)))
        return out

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

    def _goal_joint(self, joints: Sequence[float], plan_only: bool,
                    planner: Optional[str] = None,
                    quick: bool = False,
                    tol: Optional[float] = None) -> MoveGroup.Goal:
        """关节空间目标。

        tolerance 给一个很小的值(默认 1 mrad): 0 会让采样规划器很难"正好"
        采到目标点, 太小反而规划失败; 1 mrad ≈ 0.06°, 远小于本赛题的精度需求。

        ``planner`` 给定时走 **Pilz 工业运动规划器**(目前只支持 ``"PTP"``):
        point-to-point, 各关节同步走一条关节空间直线 + 梯形速度曲线 —— 这是
        工业现场"从 A 点搬到 B 点"的标准做法, 结果**确定性**, 不会像 OMPL
        那样偶尔甩一条绕路的轨迹出来。带工件走直线会撞时 PTP 会规划失败,
        由调用方退回 OMPL。

        ``quick`` 给备选解用: 规划预算砍到 1 次尝试 / 1s, 免得"连试 3 支解"
        的最坏耗时比直接退回位姿目标还长。

        ``tol`` 覆盖容差(默认 ``joint_tol``)。用 ``joint_tol_loose`` 再试一次
        可以救回"目标位形被判定非法、但附近有合法位形"的情况 —— 位姿目标之所以
        常常还能过, 就是因为它自带 5mm 位置容差。
        """
        c = Constraints()
        for name, val in zip(self.joint_names, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = self.joint_tol if tol is None else tol
            jc.tolerance_below = self.joint_tol if tol is None else tol
            jc.weight = 1.0
            c.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = self.group
        req.num_planning_attempts = 1 if quick else self.planning_attempts
        req.allowed_planning_time = 1.0 if quick else self.allowed_planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        req.goal_constraints.append(c)
        if planner is not None:
            req.pipeline_id = "pilz_industrial_motion_planner"
            req.planner_id = planner

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = not plan_only
        return goal

    @staticmethod
    def _travel_of(result) -> float:
        """规划结果里单关节的最大行程(rad) —— 用来看这次动作"绕没绕圈"。"""
        jt = result.planned_trajectory.joint_trajectory
        if not jt.points:
            return 0.0
        cols = list(zip(*[list(p.positions) for p in jt.points]))
        return max((max(seq) - min(seq)) for seq in cols) if cols else 0.0

    def _run(self, goal: MoveGroup.Goal, t0: float, via: str) -> MoveResult:
        fut = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return MoveResult(False, -1, time.time() - t0, via=via)
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf)
        res = rf.result().result
        return MoveResult(res.error_code.val == 1, res.error_code.val,
                          time.time() - t0, joint_travel=self._travel_of(res),
                          via=via)

    def goto(self, position: Sequence[float], quat: Sequence[float],
             plan_only: bool = False, use_ik: Optional[bool] = None) -> MoveResult:
        """走到指定位姿。

        三级降级, 越靠前越"工业":

          1. ``PTP`` —— 最近 IK 关节目标 + Pilz 工业规划器(PTP)。各关节同步走
             关节空间直线, 确定性、不绕路。转场(approach/to_bin)的首选。
          2. ``joint`` —— 同一个关节目标交给 OMPL。PTP 失败(带工件直着走会撞)
             时用, 代价是路径可能绕。
          3. ``pose`` —— 只给末端位姿。会让规划器随机换关节解, 只在 IK 解不出来
             或前两级都规划失败时兜底 —— 宁可动作难看, 也不能过不去。

        前两级都失败时, 先换 IK 候选解(最多 ``max_ik_tries`` 支)再来一轮,
        实在不行才到第 3 级 —— 因为退回位姿目标 = 放任规划器随机换关节解,
        正是这个模块要消灭的行为。
        """
        use_ik = self.use_nearest_ik if use_ik is None else use_ik

        if use_ik:
            t0 = time.time()
            cands = self.ik_candidates(position, quat)
            for i, joints in enumerate(cands[:max(1, self.max_ik_tries)]):
                cur = self.joint_state(timeout=0.0) or joints
                move = max(abs(a - b) for a, b in zip(joints, cur))
                quick = i > 0                      # 备选解用短预算, 见 _goal_joint
                plans = [("joint", self._goal_joint(joints, plan_only, quick=quick))]
                if self.use_ptp:
                    plans[0:0] = [("ptp", self._goal_joint(
                        joints, plan_only, "PTP", quick=quick))]
                # 精确关节目标被判非法时(实测 -2 秒回, 不是规划超时),
                # 放宽到 0.02 rad 再来一次: 仍然锁在同一支解上, 但不至于
                # 因为几毫米的碰撞判定丢掉整支解、退化到"位姿目标随机挑解"。
                plans.append(("joint_loose", self._goal_joint(
                    joints, plan_only, quick=quick, tol=self.joint_tol_loose)))
                last = None
                for via, goal in plans:
                    res = self._run(goal, t0, via)
                    if res.ok:
                        self._log_move(move, res)
                        return res
                    last = res
                self.node.get_logger().info(
                    f"[ik] 第 {i + 1} 支候选解规划失败(error_code={last.error_code}), "
                    f"换下一支(共 {len(cands)} 支)")
            if cands:
                self.node.get_logger().warn(
                    f"[ik] {len(cands)} 支候选解都规划失败 —— 退回位姿目标"
                    f"(规划器会随机挑解, 这一步可能绕路)")
            else:
                # 一个候选解都没有 = /compute_ik 对所有种子都无解, 基本只有一个
                # 原因: 目标点在这个半径上太高(见 Arena.max_tcp_z)。必须说清楚,
                # 因为**退化成位姿目标之后规划器会随机挑一组关节解** —— 实测挑到
                # J4/J6 翻 180° 的腕部另一支, 之后整轮都在翻腕, 看起来就是
                # "抓取放置的解算很奇怪"。正确做法是调用方先把目标压进可达范围
                # (Executor 用 Arena.clamp_tcp_z 做这件事)。
                r = math.hypot(float(position[0]), float(position[1]))
                self.node.get_logger().warn(
                    f"[ik] 目标 ({position[0]:.3f},{position[1]:.3f},{position[2]:.3f}) "
                    f"r={r * 1000:.0f}mm 对全部种子都无逆解 —— 目标多半超出了该半径的"
                    f"可达上限(用 Arena.max_tcp_z(r) / arena_check 核对); "
                    f"现退回位姿目标, 规划器会**随机挑**关节解(实测会翻腕)")

        t0 = time.time()
        return self._run(self._goal(position, quat, plan_only), t0, "pose")

    def _log_move(self, move: float, res: MoveResult) -> None:
        """记录这次关节目标走得值不值。

        ``规划轨迹`` 明显大于 ``最大关节行程`` 说明规划器绕路了(带工件绕障是
        正当的, 否则就是白跑) —— 排查"这次动作怎么这么慢"看这一行。
        """
        detour = math.degrees(res.joint_travel) - math.degrees(move)
        if abs(move - self._last_ik_log) <= 0.05 and detour <= self.detour_warn_deg:
            return
        self._last_ik_log = move
        msg = (f"[ik] {res.via}: 最大关节行程 {math.degrees(move):.1f}° "
               f"(规划轨迹 {math.degrees(res.joint_travel):.1f}°)")
        if detour > self.detour_warn_deg:
            self.node.get_logger().warn(f"{msg} —— 规划器多绕了 {detour:.0f}°")
        else:
            self.node.get_logger().info(msg)

    def goto_xyz(self, x: float, y: float, z: float,
                 quat: Sequence[float], plan_only: bool = False,
                 use_ik: Optional[bool] = None) -> MoveResult:
        return self.goto((x, y, z), quat, plan_only, use_ik=use_ik)

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
                return MoveResult(False, -6, 0.0, via="cartesian")
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
            return MoveResult(False, -2, dt, via="cartesian")

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        gh = self._exec_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, gh, timeout_sec=30.0)
        handle = gh.result()
        if handle is None or not handle.accepted:
            return MoveResult(False, -1, time.time() - t0, via="cartesian")
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120.0)
        res2 = rf.result()
        code = res2.result.error_code.val if res2 is not None else -1
        travel = 0.0
        if res.solution.joint_trajectory.points:
            cols = list(zip(*[list(p.positions)
                              for p in res.solution.joint_trajectory.points]))
            travel = max((max(seq) - min(seq)) for seq in cols) if cols else 0.0
        return MoveResult(code == 1, code, time.time() - t0,
                          joint_travel=travel, via="cartesian")

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
