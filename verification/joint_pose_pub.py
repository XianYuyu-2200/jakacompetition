#!/usr/bin/env python3
"""按 /tmp/jaka_pose.json 里的关节角持续发布 /joint_states。

改姿态只需覆盖 json 文件, 例如:
  echo '{"q":[0,1.5708,0,0,0,0]}' > /tmp/jaka_pose.json

注意: 必须写 header.stamp, ROS 2 的 robot_state_publisher 只在时间戳
推进时才发布动态 TF。
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

POSE_FILE = "/tmp/jaka_pose.json"
JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def main():
    rclpy.init()
    node = Node("joint_pose_pub")
    pub = node.create_publisher(
        JointState, "/joint_states",
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    while rclpy.ok():
        try:
            with open(POSE_FILE, encoding="utf-8") as fh:
                q = json.load(fh)["q"]
        except Exception:  # noqa: BLE001
            q = [0.0] * 6
        msg = JointState()
        msg.name = JOINTS
        msg.position = [float(v) for v in q]
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(0.03)


if __name__ == "__main__":
    main()
