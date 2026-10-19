"""把机械臂送到识别位姿，并把腕部 Gemini 305 的彩色/深度图存下来。

只给 gemini305_preview.launch.py 用。关节角读
config/gemini305_preview_joints.yaml，不在这里再写一遍。
"""
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = (
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
)


class Preview(Node):
    def __init__(self):
        super().__init__("gemini305_preview")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("joints_file", "")
        self.declare_parameter("move_sec", 4.0)
        self.declare_parameter("timeout_sec", 70.0)
        self.output_dir = self.get_parameter("output_dir").value
        joints_file = self.get_parameter("joints_file").value
        with open(joints_file, encoding="utf-8") as fh:
            loaded = yaml.safe_load(fh)["initial_positions"]
        self.target = {name: float(loaded[name]) for name in JOINTS}
        self.move_sec = float(self.get_parameter("move_sec").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.overview = None
        self.joints = None
        self.sent = False
        self.goal_finished = False
        self.pose_ready_at = None
        self.color_at = 0.0
        self.depth_at = 0.0
        self.overview_at = 0.0
        self.t0 = time.monotonic()
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/jaka_minicobo_controller/follow_joint_trajectory",
        )
        self.create_subscription(
            Image, "/gemini305/color/image_raw", self._on_color, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/gemini305/depth/image_raw", self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/overview/image_raw", self._on_overview, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            "目标关节 " + ", ".join(f"{n}={self.target[n]:.4f}" for n in JOINTS))

    def _on_color(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.color_at = time.monotonic()

    def _on_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg)
        self.depth_at = time.monotonic()

    def _on_overview(self, msg):
        self.overview = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.overview_at = time.monotonic()

    def _on_joints(self, msg):
        self.joints = msg

    def _pose_ok(self):
        if self.joints is None:
            return False
        current = dict(zip(self.joints.name, self.joints.position))
        for name, value in self.target.items():
            if name not in current or abs(current[name] - value) > 0.08:
                return False
        return True

    def _send(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [self.target[name] for name in JOINTS]
        point.time_from_start = Duration(sec=int(self.move_sec), nanosec=0)
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = Duration(sec=3, nanosec=0)
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info("已发送识别位姿轨迹")

    def _on_goal_response(self, future):
        goal = future.result()
        if goal is None or not goal.accepted:
            self.get_logger().error("轨迹被拒绝，下一拍再发")
            self.sent = False
            return
        result = goal.get_result_async()
        result.add_done_callback(self._on_result)

    def _on_result(self, future):
        self.goal_finished = True
        status = future.result().status
        self.get_logger().info(f"轨迹结束，状态码 {status}")

    def _tick(self):
        elapsed = time.monotonic() - self.t0
        if not self.sent:
            if self.client.server_is_ready():
                self.sent = True
                self._send()
            elif elapsed > 45.0:
                self._finish("控制器没起来")
            return
        if self._pose_ok():
            if self.pose_ready_at is None:
                self.pose_ready_at = time.monotonic()
                self.get_logger().info("关节已到识别位姿")
        else:
            self.pose_ready_at = None
        fresh = (
            self.pose_ready_at is not None
            and self.color is not None
            and self.color_at >= self.pose_ready_at
            and time.monotonic() - self.pose_ready_at > 0.8
        )
        if fresh and (self.depth is not None or time.monotonic() - self.pose_ready_at > 3.0):
            self._finish("ok")
            return
        if elapsed > self.timeout_sec:
            self._finish("超时，按手头的画面保存")

    def _colorize_depth(self, depth):
        values = np.asarray(depth)
        if values.dtype == np.uint16:
            meters = values.astype(np.float32) / 1000.0
        else:
            meters = values.astype(np.float32)
        valid = np.isfinite(meters) & (meters > 0.04) & (meters < 1.2)
        gray = np.zeros(meters.shape[:2], np.uint8)
        if np.any(valid):
            samples = meters[valid]
            low, high = np.percentile(samples, (2, 98))
            if high <= low:
                high = low + 1e-3
            scaled = np.clip((meters - low) / (high - low), 0.0, 1.0)
            gray = (scaled * 255.0).astype(np.uint8)
            gray[~valid] = 0
        return cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)

    def _finish(self, reason):
        os.makedirs(self.output_dir, exist_ok=True)
        written = []
        panels = []
        if self.color is not None:
            path = os.path.join(self.output_dir, "color.png")
            cv2.imwrite(path, self.color)
            written.append(path)
            panels.append(self._labeled(self.color, "wrist color"))
        if self.depth is not None:
            depth_color = self._colorize_depth(self.depth)
            path = os.path.join(self.output_dir, "depth.png")
            cv2.imwrite(path, depth_color)
            raw = os.path.join(self.output_dir, "depth_raw.npy")
            np.save(raw, np.asarray(self.depth))
            written.append(path)
            panels.append(self._labeled(depth_color, "wrist depth"))
        if self.overview is not None:
            path = os.path.join(self.output_dir, "overview.png")
            cv2.imwrite(path, self.overview)
            written.append(path)
            panels.append(self._labeled(self.overview, "overview"))
        if panels:
            sheet = self._sheet(panels)
            path = os.path.join(self.output_dir, "preview.png")
            cv2.imwrite(path, sheet)
            written.append(path)
        if self.joints is not None:
            current = dict(zip(self.joints.name, self.joints.position))
            pose = os.path.join(self.output_dir, "joints.txt")
            with open(pose, "w", encoding="utf-8") as fh:
                for name in JOINTS:
                    got = current.get(name, float("nan"))
                    fh.write(f"{name} target={self.target[name]:.4f} actual={got:.4f}\n")
            written.append(pose)
        self.get_logger().info(reason)
        for path in written:
            self.get_logger().info("SAVED " + path)
        if not written:
            self.get_logger().error("没有画面可存")
        raise SystemExit(0 if self.color is not None else 2)

    @staticmethod
    def _labeled(image, text):
        canvas = image.copy()
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 28), (0, 0, 0), -1)
        cv2.putText(canvas, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        return canvas

    @staticmethod
    def _sheet(panels):
        height = 400
        resized = []
        for panel in panels:
            scale = height / panel.shape[0]
            width = max(1, int(panel.shape[1] * scale))
            resized.append(cv2.resize(panel, (width, height), interpolation=cv2.INTER_AREA))
        return np.hstack(resized)


def main():
    rclpy.init()
    node = Preview()
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = int(exc.code) if isinstance(exc.code, int) else 0
    else:
        code = 0
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
