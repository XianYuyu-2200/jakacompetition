"""抓取原语与事件上报 —— 本工具包的核心。

队伍只需要调用这里的原语, 计时与判分就自动生效(Executor 会向
``/competition/events`` 发布事件, scorer 独立校验)。

事件协议(JSON 文本, 主题 ``/competition/events``):
    {"event": "grasp",   "object_id": "station_3", "pose": [x,y,z], "t": 12.34}
    {"event": "release", "object_id": "station_3", "pose": [x,y,z], "t": 45.67}
    {"event": "abort",   "object_id": "station_3", "pose": [x,y,z], "t": 30.12}
    {"event": "collision", "detail": "...", "t": ...}
    {"event": "intervention", "detail": "...", "t": ...}

scorer 不信任这些声明本身:抓取点必须落在该工件真值附近,释放点必须落在料盒内。
"""
from __future__ import annotations

import json
import math
import time
from typing import Dict, Optional, Sequence, Tuple

import rclpy
from std_msgs.msg import String

import numpy as np
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener

from .arena import Arena, Box
from .backend import MoveGroupBackend
from .gripper import Gripper
from .backend import MoveResult
from .scene import SceneClient, box_object


class Executor:
    def __init__(self, node, arena: Arena, backend: MoveGroupBackend,
                 scene: SceneClient, gripper: Gripper,
                 pre_grasp_z: float = 0.150, vel_scale: Optional[float] = None,
                 timing_log: bool = False):
        self.node = node
        self.arena = arena
        self.backend = backend
        self.scene = scene
        self.gripper = gripper
        self.pre_grasp_z = pre_grasp_z
        # 打开后每次运动都打一条耗时日志(标定限时、找时间都花在哪了很有用)
        self.timing_log = timing_log
        if vel_scale is not None:
            self.backend.vel_scale = vel_scale
        self._pub = node.create_publisher(String, "/competition/events", 50)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self.tcp_xyz: Tuple[float, float, float] = (0.0, 0.0, self.pre_grasp_z)
        self.quat = arena.tool_down_quat
        # 上一次 pick_and_place 为什么失败: "" = 成功 / "unreachable"(几何够不到,
        # 重试没有意义) / "motion"(规划或执行失败, 值得重试)
        self.last_failure = ""

    # ---------- 事件 ----------
    def emit(self, event: str, object_id: str = "", detail: str = "",
             pose: Optional[Sequence[float]] = None) -> None:
        msg = String()
        msg.data = json.dumps({
            "event": event,
            "object_id": object_id,
            "detail": detail,
            "pose": [float(v) for v in (pose if pose is not None else self.tcp_xyz)],
            "t": time.time(),
        }, ensure_ascii=False)
        self._pub.publish(msg)

    def report_collision(self, detail: str = "") -> None:
        self.emit("collision", detail=detail)

    def report_intervention(self, detail: str = "") -> None:
        self.emit("intervention", detail=detail)

    # ---------- 原语 ----------
    def _move(self, x: float, y: float, z: float, label: str = "",
              straight: bool = False):
        """straight=True 时优先走笛卡尔直线(LIN), 不行再退回关节空间规划。

        **凡是"末端该走空间直线"的动作都要开**: 竖直的下降/抬起/投放/退出,
        以及工位与料盒之间的转场(approach / to_bin)。

        为什么转场也必须走 LIN: 关节空间的直线 **不是** 空间里的直线。跨基座
        转场(比如工位4 `(-150,355)` -> 料盒 `(315,180)`)时 PTP 的关节直线会
        和场景碰撞, 退回 OMPL 后它会绕出一条**往外甩、往上拱**的路径 ——
        实测末端从指令高度 135mm 拱到 **354mm**、半径甩到 460mm(超出 420mm
        作业带)。LIN 走的是半径 0.28~0.39m 的那条空间直线, 又直又短。
        """
        t0 = time.time()
        res = MoveResult(False, -1, 0.0)
        if straight:
            res = self.backend.goto_xyz_straight(x, y, z, self.quat)
            if res.ok:
                self.tcp_xyz = (x, y, z)
                if self.timing_log:
                    self.node.get_logger().info(
                        f"    [timing] {label:9s} ok(直线) {time.time() - t0:5.2f}s")
                return res
        res = self.backend.goto_xyz(x, y, z, self.quat)
        if self.timing_log:
            self.node.get_logger().info(
                f"    [timing] {label:9s} {'ok ' if res.ok else 'FAIL'} "
                f"{time.time() - t0:5.2f}s ({res.via})")
        if res.ok:
            self.tcp_xyz = (x, y, z)
        else:
            self.node.get_logger().warn(f"运动失败 {label} ({x:.3f},{y:.3f},{z:.3f}) -> {res}")
        return res

    def move_to(self, x: float, y: float, z: float, label: str = "move"):
        return self._move(x, y, z, label)

    def approach(self, x: float, y: float, z: Optional[float] = None):
        # 这里**不**走 LIN: 一局开始时机械臂处在零位(工具朝天, TCP 在 z≈0.77),
        # 和目标(工具朝下)差了 180° 姿态, 笛卡尔直线要边平移边翻转, 实测
        # 第一段 approach 从 3.3s 涨到 30.6s。转场交给关节规划(PTP)更快。
        return self._move(x, y, z if z is not None else self.pre_grasp_z, "approach")

    def descend(self, z: float):
        x, y, _ = self.tcp_xyz
        return self._move(x, y, z, "descend", straight=True)

    def lift(self, z: Optional[float] = None):
        x, y, _ = self.tcp_xyz
        return self._move(x, y, z if z is not None else self.pre_grasp_z,
                          "lift", straight=True)

    # ---------- 位姿 ----------
    def tcp_pose(self, timeout: float = 3.0):
        """从 TF 读取末端在赛场坐标系下的位姿 (xyz, quat)。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            try:
                tr = self._tf_buffer.lookup_transform(
                    self.arena.frame_id, self.arena.tcp_link, rclpy.time.Time())
                t, q = tr.transform.translation, tr.transform.rotation
                return np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w])
            except Exception:                            # noqa: BLE001
                continue
        raise RuntimeError(
            f"取不到 {self.arena.frame_id} -> {self.arena.tcp_link} 的 TF")

    def wait_until_settled(self, tol: float = 0.002, timeout: float = 3.0) -> bool:
        """等末端**真的走到**了指令位置(而不是控制器还在路上)。

        为什么必须等: 附着位姿是拿当前 TF 和工件真值算出来的。
        如果控制器还没到位就读 TF, 算出来的相对位姿会带上这几毫米的偏差,
        工件被"贴"在偏离中心的位置 —— 赛道二工件之间只有 10mm 间隙,
        立刻就和邻居 / 工位板 / 台面报碰撞, 表现为抓完抬不起来(-2)。
        """
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                xyz, _ = self.tcp_pose(timeout=0.5)
            except RuntimeError:
                return False
            if max(abs(a - b) for a, b in zip(xyz, self.tcp_xyz)) <= tol:
                return True
        self.node.get_logger().warn(
            f"末端在 {timeout:.1f}s 内没有到位(容差 {tol*1000:.0f}mm), 附着位姿可能有偏差")
        return False

    def grasp(self, object_id: str, size: float, obj_world_xyz,
              obj_quat=(0.0, 0.0, 0.0, 1.0)) -> bool:
        """闭合夹爪, 并把工件按**真实世界位姿**附着到末端。

        关键:必须显式算出工件相对 TCP 的位姿再 attach。
        只给 id 的话 MoveIt 会认为物体位姿是相对末端的, 把物体中心放到 TCP
        原点上, 立刻和法兰重叠, 之后所有规划都返回 INVALID_MOTION_PLAN。
        """
        # 夹爪按工件实际宽度收 —— 仿真里没有堵转保护, 不告诉它尺寸手指
        # 会直接穿过工件(真机靠堵转自停, 传不传都一样)
        self.gripper.close(object_size=size)
        self.wait_until_settled()
        tcp_xyz, tcp_quat = self.tcp_pose()
        rel = _relative_pose(tcp_xyz, tcp_quat,
                             np.asarray(obj_world_xyz, float),
                             np.asarray(obj_quat, float))
        prim = box_object(object_id, self.arena.tcp_link, (0, 0, 0),
                          (size, size, size))
        ok = self.scene.attach_at(object_id, self.arena.tcp_link, prim, rel,
                                  touch_links=self.arena.touch_links)
        if not ok:
            self.node.get_logger().error(f"附着 {object_id} 失败")
        self.emit("grasp", object_id=object_id)
        return ok

    def release(self, object_id: str, bin_box: Box) -> bool:
        """张开夹爪 + 把工件登记为"已入盒"(从场景中移除)。"""
        self.gripper.open()
        ok = self.scene.release_into_bin(object_id, self.arena.tcp_link)
        self.emit("release", object_id=object_id)
        return bool(ok)

    def go_to_bin(self, bin_box: Box, z: Optional[float] = None):
        return self._move(bin_box.center[0], bin_box.center[1],
                          z if z is not None else self.pre_grasp_z,
                          "to_bin", straight=True)

    def recover(self, drop_object: Optional[str] = None,
                restore: Optional[Tuple[float, float, float, float]] = None) -> None:
        """一次尝试失败后的复位。

        ``drop_object`` 给定时表示"工件还在夹爪上、这次尝试要作废":
        把工件**放回原位**再摘掉, 并向判分器上报 ``abort`` 事件。

        为什么不直接把它扔掉: 判分器只认"工件掉到台面"才罚分。
        一次规划失败把工件凭空塞进料盒(或凭空删掉), 会让判分口径失真 ——
        重试成功也照样被记一次掉落。所以这里如实还原, 由 ``abort`` 事件
        告诉判分器"这次不算", 不产生任何扣分。
        """
        if drop_object:
            try:
                self.gripper.open()
            except Exception:                            # noqa: BLE001
                pass
            if restore is not None:
                x, y, z_bottom, size = restore
                self.scene.put_back(drop_object, self.arena.frame_id,
                                    (x, y, z_bottom), size)
            else:
                self.scene.detach_any([drop_object])
            self.emit("abort", object_id=drop_object)
        hx, hy, hz = self.arena.home
        res = self._move(hx, hy, hz, "recover")
        if not res.ok:
            self.node.get_logger().warn(f"复位到 home 也失败了: {res}")

    # ---------- 组合动作 ----------
    def pick_and_place(self, object_id: str, x: float, y: float,
                       grasp_tcp_z: float, object_center_z: float, size: float,
                       bin_box: Box, release_z: float,
                       approach_z: Optional[float] = None) -> bool:
        """一次完整的抓取投放: 预抓取 -> 下降 -> 闭合 -> 抬起 -> 到料盒 -> 松开。

        任何一步失败都会调用 recover() 摘掉工件并退回安全高度;
        否则附着在末端的工件会让后续所有规划都失败。
        """
        az = approach_z if approach_z is not None else grasp_tcp_z + self.arena.approach_lift
        # **转场平面**和"预抓取高度"是两件事, 不能混用:
        #   az —— 悬停在抓取点上方多少(贴近工件, 越短越好)
        #   tz —— 抬着工件在场地里走时 TCP 该到的高度。必须让手里的工件
        #         越过料盒侧墙和旁边工件的顶面, 否则规划器会绕出一条大弧
        #         (实测末端从 135mm 拱到 354mm)。见 Arena.transfer_tcp_z()。
        tz = self.arena.transfer_tcp_z(grasp_tcp_z, size, bin_box)

        # ---- 可达性预检: 够不到的位姿**不发指令** ----
        # 悬停高度只是个余量, 压到该半径够得到的最高点即可(IK 一定有解,
        # "最近关节角分支"才会生效; 直接下发够不到的位姿会让规划器随机挑解,
        # 实测挑到 J4/J6 翻 180° 的腕部另一支, 之后整轮都在翻腕)。
        self.last_failure = ""
        az_want = az
        az, az_clamped = self.arena.clamp_tcp_z(x, y, az)
        if az_clamped:
            self.node.get_logger().warn(
                f"工位({x:.3f},{y:.3f}) 预抓取悬停 {az_want * 1000:.1f}mm 超出该半径上限"
                f" {self.arena.reach_top_z(x, y) * 1000:.1f}mm, 压到 {az * 1000:.1f}mm"
                f"(够不到的位姿 IK 无解 -> 规划器随机挑关节解 -> 翻腕)")
        # ---- L 形转场: 竖直抬不到转场高度时, 先"抬到该半径上限 -> 内收 -> 再抬" ----
        # 长末端(指尖比臂展还占地方)在远端工位会遇到"原地竖直抬不到 tz"。这不是
        # 死路: ``z_max`` 只跟**半径**有关, 往基座收 20mm 就能多抬 40mm。关键是
        # 这一段水平移动发生在**还没抬够高度**的时候, 只能朝基座方向走 —— 邻件都在
        # 同一圈上, 沿半径内收不会经过它们; 一旦抬到 tz, 工件底面已高过全场最高的
        # 工件, 再横穿赛场就安全了。
        tz_top = self.arena.reach_top_z(x, y)
        pull = None                       # (x, y, z): 内收路点
        if not math.isnan(tz_top) and tz > tz_top:
            r = math.hypot(x, y)
            r_need = self.arena.radius_for_tcp_z(tz + self.arena.reach_pull_margin)
            if math.isnan(r_need) or r_need >= r:
                self.node.get_logger().error(
                    f"工位({x:.3f},{y:.3f}) r={r * 1000:.0f}mm 够不到转场高度 "
                    f"{tz * 1000:.1f}mm(该半径上限 {tz_top * 1000:.1f}mm): 末端竖直向下时 "
                    f"工件挂深 {self.arena.grasp_offset(size) * 1000 + size * 1000:.1f}mm"
                    f"(指尖深 {self.arena.tip_depth * 1000:.1f}mm), 收到基座附近也抬不起来 "
                    f"—— 跳过本件(见 docs/WHEELTEC柔性机械爪.md 第 4 节)")
                self.node.get_logger().error(
                    f"运动失败 lift ({x:.3f},{y:.3f},{tz:.3f}) -> 几何不可达")
                self.last_failure = "unreachable"
                return False
            f = r_need / r
            pull = (x * f, y * f, tz_top)
            self.node.get_logger().warn(
                f"工位({x:.3f},{y:.3f}) r={r * 1000:.0f}mm 原地抬不到转场高度 "
                f"{tz * 1000:.1f}mm(上限 {tz_top * 1000:.1f}mm), 改走 L 形转场: "
                f"抬到 {tz_top * 1000:.1f}mm -> 沿半径内收到 r={r_need * 1000:.0f}mm -> "
                f"再抬到 {tz * 1000:.1f}mm")
        if not self.arena.reach_ok(math.hypot(x, y), grasp_tcp_z):
            self.node.get_logger().error(
                f"工位({x:.3f},{y:.3f}) 抓取高度 {grasp_tcp_z * 1000:.1f}mm 超出该半径上限 "
                f"{self.arena.max_tcp_z(math.hypot(x, y)) * 1000:.1f}mm —— 跳过本件")
            self.last_failure = "unreachable"
            return False

        # 工件原始底面高度: 复位时按它放回原位
        z_bottom = object_center_z - size / 2.0
        back = (x, y, z_bottom, size)
        if not self.approach(x, y, az).ok:
            self.last_failure = "motion"
            self.recover()
            return False
        # 下降和抬起对称, 也走笛卡尔直线: 关节空间规划在手腕奇异附近可能
        # 换 IK 分支, 末端划一道弧线出来(工件蹭到邻居 / 工位板)。
        if not self._move(x, y, grasp_tcp_z, "descend", straight=True).ok:
            self.last_failure = "motion"
            self.recover()
            return False
        if not self.grasp(object_id, size, (x, y, object_center_z)):
            self.last_failure = "motion"
            self.recover()
            return False
        # 抬升: 够得到就一步竖直抬到转场高度; 长末端够不到就走 L 形三段。
        if pull is None:
            if not self._move(x, y, tz, "lift", straight=True).ok:
                self.last_failure = "motion"
                self.recover(object_id, back)
                return False
        else:
            if not self._move(x, y, pull[2], "lift", straight=True).ok:
                self.last_failure = "motion"
                self.recover(object_id, back)
                return False
            if not self._move(pull[0], pull[1], pull[2], "pull", straight=True).ok:
                self.last_failure = "motion"
                self.recover(object_id, back)
                return False
            if not self._move(pull[0], pull[1], tz, "lift2", straight=True).ok:
                self.last_failure = "motion"
                self.recover(object_id, back)
                return False
        if not self.go_to_bin(bin_box, tz).ok:
            self.last_failure = "motion"
            self.recover(object_id, back)
            return False
        if not self._move(bin_box.center[0], bin_box.center[1], release_z,
                          "place", straight=True).ok:
            self.last_failure = "motion"
            self.recover(object_id, back)
            return False
        self.release(object_id, bin_box)
        self._move(bin_box.center[0], bin_box.center[1], tz, "retreat", straight=True)
        return True


def _quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _matrix_to_quat(m):
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return np.array([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                         (m[1, 0] - m[0, 1]) / s, 0.25 * s])
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    if i == 0:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return np.array([0.25 * s, (m[0, 1] + m[1, 0]) / s,
                         (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return np.array([(m[0, 1] + m[1, 0]) / s, 0.25 * s,
                         (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s])
    s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return np.array([(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s,
                     0.25 * s, (m[1, 0] - m[0, 1]) / s])


def _relative_pose(tcp_xyz, tcp_quat, obj_xyz, obj_quat):
    """求物体在 TCP 坐标系下的位姿。"""
    Rt = _quat_to_matrix(np.asarray(tcp_quat, float))
    Ro = _quat_to_matrix(np.asarray(obj_quat, float))
    R_rel = Rt.T @ Ro
    p_rel = Rt.T @ (np.asarray(obj_xyz, float) - np.asarray(tcp_xyz, float))
    q_rel = _matrix_to_quat(R_rel)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in p_rel)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = \
        (float(v) for v in q_rel)
    return pose


def _pose(x: float, y: float, z: float):
    from geometry_msgs.msg import Pose
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.w = 1.0
    return p
