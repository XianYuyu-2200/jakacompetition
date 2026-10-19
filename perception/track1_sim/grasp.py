#!/usr/bin/env python3
"""用训好的 YOLO 权重，在腕部相机仿真里按基础组二维码顺序抓取。

这是给没有真机的参赛队看的练习，不是决赛判分。
判分仍走 round.launch.py：参考实现读二维码后去固定工位，不读这个权重。

这一场是基础组的练法。6 个几何体随机放在 6 个固定工位上，腕部相机先读
二维码（JAKA1-######），再按工位号顺序抓。检测框只回答「是哪一类、大概在
画面哪一块」。抓取点用框内工件顶面的深度，反投影到世界坐标，不用工位的
名义坐标。不要夹检测框的中心。

  export JAKA_GRIPPER=1
  python3 perception/track1_sim/grasp.py --ros-args -p use_sim_time:=true

预览世界和 move_group 要先开着。一条命令见 perception/track1_sim/README.md。
"""
from __future__ import annotations

import json
import math
import os
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parent))
from shapes import SHAPES  # noqa: E402
from round_qr import deal, order_picks, read_order, show_qr  # noqa: E402

from jaka_competition_kit.arena import Box, load_arena  # noqa: E402
from jaka_competition_kit.backend import MoveGroupBackend  # noqa: E402
from jaka_competition_kit.executor import Executor  # noqa: E402
from jaka_competition_kit.gripper import make_gripper  # noqa: E402
from jaka_competition_kit.scene import SceneClient, bin_container, box_object  # noqa: E402



WINDOW = "基础组识别"

_VIEWER = r"""
import sys
import cv2

path = sys.argv[1]
window = "基础组识别"
cv2.namedWindow(window, cv2.WINDOW_NORMAL)
cv2.moveWindow(window, 40, 40)
seen = 0.0
shown = False
while True:
    try:
        stamp = __import__("os").path.getmtime(path)
    except OSError:
        stamp = 0.0
    if stamp != seen:
        image = cv2.imread(path)
        if image is not None:
            cv2.resizeWindow(window, int(image.shape[1]), int(image.shape[0]))
            cv2.imshow(window, image)
        seen = stamp
    key = cv2.waitKey(30) & 0xFF
    visible = cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE)
    if visible >= 1:
        shown = True
    if key in (ord("q"), 27):
        break
    if shown and visible < 1:
        break
"""


_FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"



def _focus(image: np.ndarray, picks: list[dict], margin: int = 36) -> np.ndarray:
    """裁掉大片空桌面，识别窗口里主要留下工件。"""
    if not picks:
        return image
    height, width = image.shape[:2]
    boxes = [item.get("tight", item["xyxy"]) for item in picks]
    x1 = max(0, int(min(box[0] for box in boxes)) - margin)
    y1 = max(0, int(min(box[1] for box in boxes)) - margin - 18)
    x2 = min(width, int(max(box[2] for box in boxes)) + margin)
    y2 = min(height, int(max(box[3] for box in boxes)) + margin)
    if x2 - x1 < 80 or y2 - y1 < 80:
        return image
    return image[y1:y2, x1:x2]


def _recognition_board(image: np.ndarray, picks: list[dict], status: str) -> np.ndarray:
    """图像下面加一栏：类别和这一帧的置信度。OpenCV 画不了中文，这里用字体画。"""
    from PIL import Image, ImageDraw, ImageFont

    rows = max(len(picks), 1)
    bar_h = 28 + 26 * rows + 8
    canvas = np.full((image.shape[0] + bar_h, image.shape[1], 3), 248, np.uint8)
    canvas[:image.shape[0]] = image
    pil = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil)
    font = ImageFont.truetype(_FONT_PATH, 18)
    small = ImageFont.truetype(_FONT_PATH, 16)
    draw.text((8, image.shape[0] + 4), status, font=font, fill=(20, 20, 20))
    if not picks:
        draw.text((8, image.shape[0] + 30), "还没有检测结果", font=small, fill=(80, 80, 80))
    for index, item in enumerate(picks):
        y = image.shape[0] + 30 + 26 * index
        percent = item["conf"] * 100.0
        color = (20, 120, 40) if percent >= 70 else (160, 90, 10)
        station = item.get("station")
        title = f"{index + 1}. 工位{station} {item['name']}" if station else f"{index + 1}. {item['name']}"
        draw.text((8, y), title, font=small, fill=(20, 20, 20))
        draw.text((210, y), f"{percent:.0f}%", font=small, fill=color)
    out = cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)
    return out


WORLD = "gemini305_preview"
# 和 capture_sim.py 里标框用的是同一只相机。只在识别位姿下成立。
CAM_XYZ = (0.0, 0.339, 0.289)
JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
# 练习料盒放在画面外、够得到、又不压到两排工件的位置。不是赛场那只料盒。
BIN_CENTER = (0.28, 0.10)
BIN_SIZE = (0.16, 0.12)
BIN_HEIGHT = 0.040
# 工件中心比台面垫板再抬一点，让网格落在料盒底板上面。
BIN_Z_LIFT = 0.007
SLOTS = (
    (-0.040, -0.025), (0.000, -0.025), (0.040, -0.025),
    (-0.040, 0.025), (0.000, 0.025), (0.040, 0.025),
)
SHAPE_BY_ID = {s.id: s for s in SHAPES}
KNOWN = set(SHAPE_BY_ID)


def _package_dir() -> Path:
    return Path(__file__).resolve().parent


def _joints_file() -> Path:
    here = _package_dir()
    for cursor in [here, *here.parents]:
        path = cursor / "jaka_competition_kit" / "config" / "gemini305_preview_joints.yaml"
        if path.is_file():
            return path
    raise FileNotFoundError("找不到 gemini305_preview_joints.yaml")


def load_joint_targets() -> dict[str, float]:
    with open(_joints_file(), encoding="utf-8") as fh:
        loaded = yaml.safe_load(fh)["initial_positions"]
    return {name: float(loaded[name]) for name in JOINTS}


def tool_quat(yaw: float) -> tuple[float, float, float, float]:
    """工具朝下，再绕世界 Z 转 yaw。yaw=0 时就是 arena 的 tool_down_quat。

    手指轴在夹爪坐标系的 Y。朝下且 yaw=0 时，手指沿世界 Y 开合。
    """
    half = yaw / 2.0
    # Rz(yaw) * Rx(pi)，Rx(pi) 的四元数是 (1, 0, 0, 0)。
    return (math.cos(half), math.sin(half), 0.0, 0.0)


def _quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _mul_quat(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _relative(tcp_p, tcp_q, obj_p, obj_q):
    rot = _quat_to_R(tcp_q)
    pos = rot.T @ (np.asarray(obj_p, float) - np.asarray(tcp_p, float))
    inv = (-tcp_q[0], -tcp_q[1], -tcp_q[2], tcp_q[3])
    quat = _mul_quat(inv, obj_q)
    return pos, quat


def _compose(parent_p, parent_q, child_p, child_q):
    rot = _quat_to_R(parent_q)
    pos = np.asarray(parent_p, float) + rot @ np.asarray(child_p, float)
    return pos, _mul_quat(parent_q, child_q)


def yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def pixels_to_world(u, v, z_opt, info) -> tuple[float, float, float] | None:
    """光学系：+X 朝世界 −X，+Y 朝世界 +Y，光轴朝下。"""
    if info is None or z_opt <= 0.05:
        return None
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    if fx <= 1 or fy <= 1:
        return None
    x_opt = (u - cx) / fx * z_opt
    y_opt = (v - cy) / fy * z_opt
    return (-x_opt + CAM_XYZ[0], CAM_XYZ[1] + y_opt, CAM_XYZ[2] - z_opt)


def depth_to_meters(raw) -> np.ndarray:
    depth = np.asarray(raw)
    if depth.dtype == np.uint16:
        return depth.astype(np.float32) / 1000.0
    depth = depth.astype(np.float32)
    finite = depth[np.isfinite(depth)]
    if finite.size and float(np.max(finite)) > 50.0:
        depth = depth / 1000.0
    return depth


def top_of_box(depth: np.ndarray, xyxy, info):
    """框里高出桌面 1–7 cm 的那一层才是工件顶面。

    近排的框会挨到手指。手指离相机更近，不能拿「最近的像素」当顶面。
    """
    height, width = depth.shape[:2]
    whole = np.isfinite(depth) & (depth > 0.05) & (depth < 1.0)
    if int(np.count_nonzero(whole)) < 100:
        return None
    table = float(np.percentile(depth[whole], 90))
    x1 = max(0, int(math.floor(xyxy[0])))
    y1 = max(0, int(math.floor(xyxy[1])))
    x2 = min(width, int(math.ceil(xyxy[2])))
    y2 = min(height, int(math.ceil(xyxy[3])))
    if x2 - x1 < 8 or y2 - y1 < 8:
        return None
    # 缩一圈，躲开框边上的桌面。
    mx = max(1, int(0.08 * (x2 - x1)))
    my = max(1, int(0.08 * (y2 - y1)))
    x1, y1, x2, y2 = x1 + mx, y1 + my, x2 - mx, y2 - my
    if x2 - x1 < 6 or y2 - y1 < 6:
        return None
    roi = depth[y1:y2, x1:x2]
    valid = np.isfinite(roi) & (roi > 0.05) & (roi < 0.8)
    above = table - roi
    mask = valid & (above >= 0.012) & (above <= 0.075)
    if int(np.count_nonzero(mask)) < 20:
        return None
    ys, xs = np.nonzero(mask)
    u = float(x1 + np.median(xs))
    v = float(y1 + np.median(ys))
    z = float(np.median(roi[mask]))
    fx = float(info.k[0]) if info is not None else 0.0
    fy = float(info.k[4]) if info is not None else 0.0
    if fx <= 1 or fy <= 1:
        return None
    extent_x = (float(xs.max() - xs.min()) + 1.0) * z / fx
    extent_y = (float(ys.max() - ys.min()) + 1.0) * z / fy
    cov_x = xs.astype(np.float64) - xs.mean()
    cov_y = ys.astype(np.float64) - ys.mean()
    ang = 0.5 * math.atan2(float(np.mean(2 * cov_x * cov_y)),
                           float(np.mean(cov_x * cov_x - cov_y * cov_y)))
    # 图像 +u 是世界 −X，图像 +v 是世界 +Y。
    yaw = math.atan2(math.sin(ang), -math.cos(ang))
    pad = 3
    return {
        "u": u,
        "v": v,
        "z": z,
        "extent_x": extent_x,
        "extent_y": extent_y,
        "yaw": yaw,
        "tight": (
            float(max(0, x1 + int(xs.min()) - pad)),
            float(max(0, y1 + int(ys.min()) - pad)),
            float(min(width - 1, x1 + int(xs.max()) + pad)),
            float(min(height - 1, y1 + int(ys.max()) + pad)),
        ),
    }


def choose_pinch(extent_x: float, extent_y: float) -> tuple[float, float]:
    """挑手指张得开的那个方向。yaw=0 沿世界 Y 夹，yaw=90° 沿世界 X 夹。"""
    extent_x = min(max(extent_x, 0.02), 0.09)
    extent_y = min(max(extent_y, 0.02), 0.09)
    if extent_y <= extent_x or extent_y <= 0.064:
        width = min(extent_y, 0.066)
        return 0.0, max(0.024, width)
    return math.pi / 2.0, max(0.024, min(extent_x, 0.066))


def run_yolo(image: Path, weights: Path, conf: float) -> list[dict]:
    venv = _package_dir().parent / "track2_yolo" / ".venv" / "bin" / "python"
    script = _package_dir() / "infer.py"
    if not venv.is_file():
        raise RuntimeError(f"没有 YOLO 虚拟环境 {venv}")
    proc = subprocess.run(
        [str(venv), str(script), str(image), "--weights", str(weights),
         "--conf", f"{conf:.2f}"],
        check=False, capture_output=True, text=True, timeout=180,
    )
    line = ""
    for item in proc.stdout.splitlines():
        if item.startswith("YOLO_JSON="):
            line = item[len("YOLO_JSON="):]
    if proc.returncode != 0 or not line:
        detail = (proc.stderr or proc.stdout or "").strip()
        raise RuntimeError(f"推理失败（退出 {proc.returncode}）: {detail[-800:]}")
    return list(json.loads(line).get("detections", []))


class _PoseLink:
    """一个常驻进程改位姿。每帧新开 ign service 来不及，网格会停在桌上。"""

    def __init__(self):
        binary = _package_dir() / "pose_daemon"
        self.proc = None
        if binary.is_file():
            self.proc = subprocess.Popen(
                [str(binary), WORLD],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                text=True, bufsize=1,
            )
        self.lock = threading.Lock()

    def set_pose(self, name, xyz, quat) -> bool:
        if self.proc is None or self.proc.poll() is not None or self.proc.stdin is None:
            return False
        x, y, z = (float(v) for v in xyz)
        qx, qy, qz, qw = (float(v) for v in quat)
        line = f"{name} {x:.5f} {y:.5f} {z:.5f} {qx:.5f} {qy:.5f} {qz:.5f} {qw:.5f}\n"
        with self.lock:
            try:
                self.proc.stdin.write(line)
                self.proc.stdin.flush()
                reply = self.proc.stdout.readline() if self.proc.stdout else ""
            except (BrokenPipeError, OSError):
                return False
        return reply.strip() == "ok"


_POSE_LINK: _PoseLink | None = None


def set_model_pose(name: str, xyz, quat, timeout_ms: int = 800) -> bool:
    global _POSE_LINK
    if _POSE_LINK is None:
        _POSE_LINK = _PoseLink()
    if _POSE_LINK.set_pose(name, xyz, quat):
        return True
    x, y, z = (float(v) for v in xyz)
    qx, qy, qz, qw = (float(v) for v in quat)
    req = (
        f'name: "{name}", position: {{x: {x:.5f}, y: {y:.5f}, z: {z:.5f}}}, '
        f"orientation: {{x: {qx:.5f}, y: {qy:.5f}, z: {qz:.5f}, w: {qw:.5f}}}"
    )
    result = subprocess.run(
        ["ign", "service", "-s", f"/world/{WORLD}/set_pose",
         "--reqtype", "ignition.msgs.Pose",
         "--reptype", "ignition.msgs.Boolean",
         "--timeout", str(timeout_ms), "--req", req],
        check=False, capture_output=True, text=True,
    )
    return result.returncode == 0 and "true" in (result.stdout or "").lower()



LIVE_FRAME = Path("/tmp/track1_live.bin")
LIVE_STATUS = Path("/tmp/track1_live_status.txt")


class FramePump(Node):
    """抓取时主线程会卡住，相机画面改由这个节点持续写出去。"""

    def __init__(self):
        super().__init__(
            "track1_live_pump",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.bridge = CvBridge()
        self.seq = 0
        self.create_subscription(
            Image, "/gemini305/color/image_raw", self._on_color, qos_profile_sensor_data)

    def _on_color(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if image is None or image.ndim != 3:
            return
        height, width = image.shape[:2]
        self.seq = (self.seq + 1) & 0x7FFFFFFF
        blob = struct.pack("<QII", self.seq, width, height) + image.tobytes()
        tmp = LIVE_FRAME.with_suffix(".tmp")
        tmp.write_bytes(blob)
        tmp.replace(LIVE_FRAME)


class GazeboFollow(Node):
    """规划场景里的附着不会带动 Gazebo 网格。抓起来之后用 set_pose 跟着 TCP。"""

    def __init__(self):
        super().__init__(
            "track1_gz_follow",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.lock = threading.Lock()
        self.current = None
        self.job = None
        self.slot = 0
        self._warned = False
        self.create_subscription(String, "/competition/events", self._on_event, 50)
        self.create_timer(0.1, self._tick)

    def arm(self, spec: dict) -> None:
        with self.lock:
            self.current = spec

    def _on_event(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            spec = self.current
        if not spec or data.get("object_id") != spec["id"]:
            return
        event = data.get("event")
        if event == "grasp":
            self._begin(spec)
        elif event == "release":
            self._drop(spec)
        elif event == "abort":
            self._restore(spec)

    def _tcp(self):
        try:
            tf = self.buffer.lookup_transform("world", "dummy_tcp", Time())
        except Exception:
            return None
        tr = tf.transform.translation
        q = tf.transform.rotation
        return (
            np.array([tr.x, tr.y, tr.z]),
            (q.x, q.y, q.z, q.w),
        )

    def _begin(self, spec):
        tcp = self._tcp()
        if tcp is None:
            self.get_logger().warn(f"还没有 TCP，{spec['model']} 先留在原地")
            return
        rel_p, rel_q = _relative(tcp[0], tcp[1], spec["center"], spec["quat"])
        with self.lock:
            self.job = {"model": spec["model"], "p": rel_p, "q": rel_q}
        self._warned = False
        self.get_logger().info(f"{spec['model']} 跟着夹爪")

    def _drop(self, spec):
        with self.lock:
            self.job = None
            index = self.slot
            self.slot += 1
        dx, dy = SLOTS[index % len(SLOTS)]
        xyz = (BIN_CENTER[0] + dx, BIN_CENTER[1] + dy, spec["rest_z"] + BIN_Z_LIFT)
        set_model_pose(spec["model"], xyz, spec["quat"], timeout_ms=1500)
        self.get_logger().info(
            f"{spec['model']} 放进练习料盒 ({xyz[0]:.3f}, {xyz[1]:.3f})")

    def _restore(self, spec):
        with self.lock:
            self.job = None
        set_model_pose(spec["model"], spec["home"], spec["quat"], timeout_ms=1500)
        self.get_logger().info(f"{spec['model']} 放回原位")

    def _tick(self):
        with self.lock:
            job = None if self.job is None else dict(self.job)
        if job is None:
            return
        tcp = self._tcp()
        if tcp is None:
            return
        pos, quat = _compose(tcp[0], tcp[1], job["p"], job["q"])
        if not set_model_pose(job["model"], pos, quat, timeout_ms=250) and not self._warned:
            self._warned = True
            self.get_logger().warn(f"移动 {job['model']} 失败，画面里的网格可能还停在原地")


class LearnGrasp(Node):
    def __init__(self, follower: GazeboFollow):
        super().__init__("track1_learn_grasp")
        self.declare_parameter("weights", "")
        self.declare_parameter("conf", 0.40)
        self.declare_parameter("vel_scale", 0.4)
        self.follower = follower
        self.targets = load_joint_targets()
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.info = None
        self.color_stamp = -1.0
        self.depth_stamp = -1.0
        self.joints = None
        self.create_subscription(
            Image, "/gemini305/color/image_raw", self._on_color, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/gemini305/depth/image_raw", self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, "/gemini305/color/camera_info", self._on_info, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.pub = self.create_publisher(String, "/track1/yolo", 10)
        self.image_pub = self.create_publisher(Image, "/track1/yolo_image", 10)
        self._shown = None
        self._viewer = None
        self.traj = ActionClient(
            self, FollowJointTrajectory,
            "/jaka_minicobo_controller/follow_joint_trajectory")

    def _stamp(self, msg) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _on_color(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.color_stamp = self._stamp(msg)

    def _on_depth(self, msg):
        self.depth = depth_to_meters(self.bridge.imgmsg_to_cv2(msg))
        self.depth_stamp = self._stamp(msg)

    def _on_info(self, msg):
        self.info = msg

    def _on_joints(self, msg):
        self.joints = msg

    def _spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._shown is not None and os.environ.get("DISPLAY"):
                cv2.waitKey(1)

    def _at_pose(self) -> bool:
        if self.joints is None:
            return False
        current = dict(zip(self.joints.name, self.joints.position))
        return all(name in current and abs(current[name] - value) < 0.08
                   for name, value in self.targets.items())

    def _send_pose(self) -> None:
        if not self.traj.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("关节控制器没起来")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [self.targets[name] for name in JOINTS]
        point.time_from_start = Duration(sec=4, nanosec=0)
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = Duration(sec=3, nanosec=0)
        future = self.traj.send_goal_async(goal)
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        accepted = future.result()
        if accepted is None or not accepted.accepted:
            raise RuntimeError("识别位姿被控制器拒绝")
        self.get_logger().info("手臂不在识别位姿，已重新发关节轨迹")

    def _wait_pose(self, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._at_pose():
                return True
        return False

    def _wait_frame(self, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.color is None or self.depth is None or self.info is None:
                continue
            if self.color.shape[:2] != self.depth.shape[:2]:
                continue
            if abs(self.color_stamp - self.depth_stamp) > 0.3:
                continue
            return True
        return False

    def _weights(self) -> Path:
        given = str(self.get_parameter("weights").value)
        path = Path(given) if given else _package_dir() / "runs" / "track1" / "weights" / "best.pt"
        if not path.is_file():
            raise RuntimeError(
                f"没有训练结果 {path}。先跑 perception/track1_sim/train.py。"
                "这个练习只用仿真里这 6 个几何体的权重。")
        return path

    def _draw(self, picks: list[dict], status: str) -> np.ndarray:
        """绿框是检测框，红叉是顶面抓取点。底下列出这一帧的置信度。"""
        canvas = self.frame.copy()
        for item in picks:
            x1, y1, x2, y2 = (int(v) for v in item.get("tight", item["xyxy"]))
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (40, 170, 60), 2)
            u, v = int(item["u"]), int(item["v"])
            cv2.drawMarker(canvas, (u, v), (20, 20, 220), cv2.MARKER_CROSS, 16, 2)
            label = f"{item['conf'] * 100:.0f}%"
            cv2.rectangle(canvas, (x1, max(0, y1 - 18)), (x1 + 52, y1), (40, 170, 60), -1)
            cv2.putText(canvas, label, (x1 + 4, max(14, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        canvas = _focus(canvas, picks)
        return _recognition_board(canvas, picks, status)

    def _live_path(self) -> Path:
        path = _package_dir() / "runs" / "grasp" / "live.jpg"
        path.parent.mkdir(parents=True, exist_ok=True)
        return path

    def _note(self, text: str) -> None:
        LIVE_STATUS.write_text(text + "\n", encoding="utf-8")
        self._ensure_live()

    def _ensure_live(self) -> None:
        """窗口单独用 YOLO 环境跑，相机每帧都刷新，检测框跟着更新。"""
        if not os.environ.get("DISPLAY"):
            return
        if self._viewer is not None and self._viewer.poll() is None:
            return
        venv = _package_dir().parent / "track2_yolo" / ".venv" / "bin" / "python"
        script = _package_dir() / "live_view.py"
        if not venv.is_file() or not script.is_file():
            self.get_logger().warn(f"实时窗口开不了，缺少 {venv}")
            return
        env = os.environ.copy()
        env.setdefault("QT_QPA_PLATFORM", "xcb")
        env["QT_LOGGING_RULES"] = "qt.qpa.fonts=false"
        self._viewer = subprocess.Popen(
            [str(venv), str(script), "--weights", str(self._weights())],
            start_new_session=True,
            env=env,
        )
        self._shown = True
        self.get_logger().info("识别窗口是实时画面，检测框跟着相机更新")

    def _hold_window(self) -> None:
        """抓完也不关掉。关掉窗口或在窗口里按 q 才结束。"""
        if self._viewer is None:
            return
        self.get_logger().info("识别窗口保持打开。关掉窗口或按 q 结束。")
        while rclpy.ok() and self._viewer.poll() is None:
            rclpy.spin_once(self, timeout_sec=0.2)

    def _publish(self, picks: list[dict], canvas: np.ndarray) -> None:
        payload = {
            "count": len(picks),
            "weights": "track1_sim/best.pt",
            "order": list(getattr(self, "qr_order", [])),
            "objects": [{
                "id": item["id"],
                "name": item["name"],
                "station": item.get("station"),
                "conf": item["conf"],
                "bbox": [round(v, 1) for v in item["xyxy"]],
                "grasp_px": [round(item["u"], 1), round(item["v"], 1)],
                "world_xyz": [round(v, 4) for v in item["world"]],
            } for item in picks],
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8"))
        out = _package_dir() / "runs" / "grasp"
        out.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out / "detect.jpg"), canvas)

    def _locate_all(self, detections: list[dict]) -> list[dict]:
        best: dict[str, dict] = {}
        for det in detections:
            sid = str(det.get("id", ""))
            if sid not in KNOWN:
                continue
            if sid in best and float(det["conf"]) <= best[sid]["conf"]:
                continue
            best[sid] = det
        picks = []
        for sid, det in best.items():
            top = top_of_box(self.depth, det["xyxy"], self.info)
            if top is None:
                self.get_logger().warn(f"{sid} 的框里没有可用的顶面深度，跳过")
                continue
            world = pixels_to_world(top["u"], top["v"], top["z"], self.info)
            if world is None:
                continue
            x, y, z = world
            if not (-0.22 <= x <= 0.22 and 0.20 <= y <= 0.43 and 0.02 <= z <= 0.09):
                self.get_logger().warn(
                    f"{sid} 反投影到 ({x:.3f},{y:.3f},{z:.3f})，不在散放区，当误检丢掉")
                continue
            shape = SHAPE_BY_ID[sid]
            height = float(shape.bounds[5] - shape.bounds[4])
            bottom = max(z - height, 0.004)
            size = z - bottom
            yaw, width = choose_pinch(top["extent_x"], top["extent_y"])
            ex = min(max(top["extent_x"], 0.025), 0.090)
            ey = min(max(top["extent_y"], 0.025), 0.090)
            picks.append({
                "id": sid,
                "name": shape.name,
                "conf": float(det["conf"]),
                "xyxy": det["xyxy"],
                "tight": top["tight"],
                "u": top["u"],
                "v": top["v"],
                "world": (x, y, z),
                "center": (x, y, bottom + size / 2.0),
                "size": size,
                "dims": (ex, ey, size),
                "pinch_yaw": yaw,
                "grasp_width": width,
                "visual_quat": yaw_quat(top["yaw"]),
                "rest_z": float(shape.center_z),
                "model": f"shape_{sid}",
                "object_id": f"wp_{sid}",
            })
        picks.sort(key=lambda item: (item["world"][1], item["world"][0]))
        return picks

    def _build_scene(self, picks: list[dict], arena) -> None:
        frame = arena.frame_id
        table = arena.raw["table"]
        cx, cy = table["center"]
        sx, sy = table["size"]
        thick = float(table["thickness"])
        top = float(table["top_z"])
        objects = [box_object(
            "table", frame, (cx, cy, top - thick / 2.0), (sx, sy, thick))]
        bin_box = Box(BIN_CENTER, BIN_SIZE)
        objects.extend(bin_container("bin_learn", frame, bin_box, BIN_HEIGHT))
        for item in picks:
            objects.append(box_object(
                item["object_id"], frame, item["center"], item["dims"]))
        if not self.scene.apply(objects):
            raise RuntimeError("规划场景写不进去")
        self.get_logger().info(
            f"规划场景：台面、练习料盒、{len(picks)} 个工件")

    def run(self) -> None:
        if not bool(self.get_parameter("use_sim_time").value):
            raise RuntimeError("必须带 use_sim_time:=true，不然 TF 和 Gazebo 时钟对不上")
        weights = self._weights()
        self.get_logger().info(f"练习抓取使用训练结果 {weights}")
        arena = load_arena()
        if not arena.has_gripper:
            raise RuntimeError("预览机械臂装的是 WHEELTEC 夹爪，但 arena 仍按裸法兰算高度。"
                               "请用 JAKA_GRIPPER=1 启动。")
        links = arena.raw["tool"].setdefault("touch_links", [])
        if "gemini305_link" not in links:
            links.append("gemini305_link")
        vel = float(self.get_parameter("vel_scale").value)
        backend = MoveGroupBackend(
            self, arena.group_name, arena.tcp_link, arena.frame_id,
            vel_scale=vel, acc_scale=vel,
            planning_attempts=3, allowed_planning_time=3.0)
        self.scene = SceneClient()
        gripper = make_gripper("sim", self, arena=arena)
        arm = Executor(self, arena, backend, self.scene, gripper, timing_log=True)
        gripper.open()

        shot = _package_dir() / "runs" / "grasp"
        shot.mkdir(parents=True, exist_ok=True)
        assignment, generated, qr_text = deal(set_model_pose)
        show_qr(qr_text, shot)
        placed = " ".join(f"{sid}:{name}" for name, sid in assignment)
        self.get_logger().info(f"工位随机摆放 {placed}")
        self.get_logger().info(f"二维码写入 {qr_text}，等腕部画面解码")
        self._qr_text = qr_text
        self._qr_generated = generated
        # 这一句会打开实时窗口。先别改成别的状态，等码解出来再换。
        self._note("正在读取二维码")
        self.get_logger().info("等腕部相机和识别位姿")
        if not self._wait_pose(25.0):
            self._send_pose()
            if not self._wait_pose(20.0):
                raise RuntimeError("手臂没有到识别位姿，外参不能用")
        self._spin(0.8)
        if not self._wait_frame(20.0):
            raise RuntimeError("没有同时拿到彩色图和深度图")

        image = shot / "frame.jpg"
        order, seen = [], ""
        # 卡片刚塞进 Gazebo，头一两帧可能还是白底。多看几帧，仍只认画面。
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not order:
            self._spin(0.25)
            if self.color is None or self.info is None:
                continue
            self.frame = self.color.copy()
            order, seen = read_order(self.frame, self.info, CAM_XYZ)
        if self.color is not None:
            self.frame = self.color.copy()
            cv2.imwrite(str(image), self.frame)
        if not order:
            raise RuntimeError(
                f"腕部画面没有解出二维码。这一轮写的是 {self._qr_text}。"
                f"看 {image}")
        if order != self._qr_generated:
            self.get_logger().warn(
                f"解出 {seen}，和写入的 {self._qr_text} 不一致，按画面里的顺序抓")
        else:
            self.get_logger().info(f"二维码 {seen} -> 顺序 {order}")
        self.qr_order = order
        order_text = "→".join(str(n) for n in order)
        self._note(f"顺序 {order_text}，正在识别")
        conf = float(self.get_parameter("conf").value)
        self.get_logger().info("用 best.pt 看这一帧")
        detections = run_yolo(image, weights, conf)
        picks = self._locate_all(detections)
        picks, missing = order_picks(picks, self.qr_order)
        for station in missing:
            self.get_logger().warn(f"工位 {station} 没有对上检测，这一件跳过")
        canvas = self._draw(picks, "识别完成，开始抓取")
        self._publish(picks, canvas)
        self._note("识别完成，开始抓取")
        if not picks:
            raise RuntimeError("这一帧没有可用的检测。看 perception/track1_sim/runs/grasp/detect.jpg")
        names = " ".join(f"{item['name']}({item['conf']:.2f})" for item in picks)
        self.get_logger().info(f"准备抓 {len(picks)} 个：{names}")
        self._build_scene(picks, arena)

        bin_box = Box(BIN_CENTER, BIN_SIZE)
        ok = 0
        for index, item in enumerate(picks, 1):
            x, y, top_z = item["world"]
            grasp_z = arena.grasp_tcp_z(top_z, item["size"])
            approach = arena.approach_z(grasp_z)
            release = arena.release_tcp_z(item["size"])
            arm.quat = tool_quat(item["pinch_yaw"])
            self.get_logger().info(
                f"[{index}/{len(picks)}] 工位{item['station']} {item['name']} "
                f"顶面 ({x:.3f},{y:.3f},{top_z:.3f}) "
                f"TCP {grasp_z:.3f} 夹持宽 {item['grasp_width']*1000:.0f}mm")
            self._note(f"正在抓取 {index}/{len(picks)}  工位{item['station']} {item['name']}")
            spec = {
                "id": item["object_id"],
                "model": item["model"],
                "center": item["center"],
                "home": item["center"],
                "rest_z": item["rest_z"],
                "quat": item["visual_quat"],
            }
            self.follower.arm(spec)
            good = False
            for attempt in range(2):
                good = arm.pick_and_place(
                    item["object_id"], x, y, grasp_z, item["center"][2], item["size"],
                    bin_box, release, approach_z=approach, dims=item["dims"],
                    shape="box", grasp_width=item["grasp_width"])
                if good or arm.last_failure == "unreachable":
                    break
                self.get_logger().warn(f"    第 {attempt + 1} 次没抓起来，再试一次")
            ok += int(good)
            self.get_logger().info(f"    {'成功' if good else '失败'}")
        self.get_logger().info(f"学习抓取结束: {ok}/{len(picks)}")
        self._note(f"抓取结束 {ok}/{len(picks)}")
        self._hold_window()


def main() -> None:
    os.environ.setdefault("JAKA_GRIPPER", "1")
    rclpy.init()
    follower = GazeboFollow()
    pump = FramePump()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(follower)
    executor.add_node(pump)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    node = LearnGrasp(follower)
    try:
        node.run()
    finally:
        try:
            node.scene.close()
        except Exception:
            pass
        node.destroy_node()
        executor.shutdown()
        follower.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
