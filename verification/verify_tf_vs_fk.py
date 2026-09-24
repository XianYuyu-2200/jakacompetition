#!/usr/bin/env python3
"""用 robot_state_publisher 的 TF 与独立实现的前向运动学(FK)交叉验证 URDF。

做法:
  1) 起一个 robot_state_publisher 加载 URDF;
  2) 按给定关节角发布 /joint_states(必须带 header.stamp, 否则 ROS 2 的
     robot_state_publisher 不会发布动态 TF!);
  3) 读取 Link_0 -> dummy_tcp 的 TF, 与本脚本自己算的 FK 比较。

用法:
  python3 verify_tf_vs_fk.py [urdf 路径]
"""
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_URDF = os.path.join(
    HERE, "..", "jaka_ros2", "src", "jaka_description", "urdf", "jaka_minicobo.urdf")
URDF = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.abspath(DEFAULT_URDF)

SEQ = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def load_urdf(path):
    root = ET.parse(path).getroot()
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        joints[j.get("name")] = dict(
            type=j.get("type"),
            xyz=np.array([float(v) for v in (o.get("xyz") if o is not None else "0 0 0").split()]),
            rpy=np.array([float(v) for v in (o.get("rpy") if o is not None else "0 0 0").split()]),
        )
    return joints


def rpy_to_matrix(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def forward_kinematics(joints, q):
    T = np.eye(4)
    for i, name in enumerate(SEQ):
        j = joints[name]
        M = np.eye(4)
        M[:3, :3] = rpy_to_matrix(j["rpy"])
        M[:3, 3] = j["xyz"]
        c, s = np.cos(q[i]), np.sin(q[i])
        Rz = np.eye(4)
        Rz[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        T = T @ M @ Rz
    return T[:3, 3] * 1000.0  # m -> mm


def main():
    joints = load_urdf(URDF)

    rsp = subprocess.Popen(
        ["/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher", URDF],
        env=dict(os.environ),
        stdout=open("/tmp/rsp_verify.log", "w"),
        stderr=subprocess.STDOUT,
    )
    time.sleep(3)

    rclpy.init()
    node = Node("verify_tf_vs_fk")
    pub = node.create_publisher(
        JointState, "/joint_states",
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    buf = Buffer()
    TransformListener(buf, node)

    poses = [
        ("zero", [0, 0, 0, 0, 0, 0]),
        ("q2=+60,q3=-30", [0, math.radians(60), math.radians(-30), 0, 0, 0]),
        ("q2=-45,q3=+80,q5=45", [0, math.radians(-45), math.radians(80), 0, math.radians(45), 0]),
        ("q1=30,q4=90,q6=120",
         [math.radians(30), math.radians(30), math.radians(20),
          math.radians(90), math.radians(-40), math.radians(120)]),
    ]

    print(f"URDF: {URDF}")
    print(f"{'pose':22s} {'TF dummy_tcp (mm)':34s} {'FK computed (mm)':34s} {'err(mm)':>8s}")
    ok = True
    for name, q in poses:
        t0 = time.time()
        while time.time() - t0 < 2.0:
            msg = JointState()
            msg.name = SEQ
            msg.position = [float(v) for v in q]
            msg.header.stamp = node.get_clock().now().to_msg()  # 必须有时间戳
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.3)
        try:
            tr = buf.lookup_transform("Link_0", "dummy_tcp", rclpy.time.Time())
            t = tr.transform.translation
            tf_mm = np.array([t.x, t.y, t.z]) * 1000.0
        except Exception as exc:  # noqa: BLE001
            print(f"{name:22s} TF ERROR {exc}")
            ok = False
            continue
        fk_mm = forward_kinematics(joints, q)
        err = np.linalg.norm(tf_mm - fk_mm)
        print(f"{name:22s} [{tf_mm[0]:8.2f},{tf_mm[1]:8.2f},{tf_mm[2]:8.2f}]     "
              f"[{fk_mm[0]:8.2f},{fk_mm[1]:8.2f},{fk_mm[2]:8.2f}]     {err:8.3f}")
        if err > 0.05:
            ok = False

    print()
    print("RESULT:", "MATCH (TF == FK, <0.05mm)" if ok else "MISMATCH")
    rclpy.shutdown()
    rsp.terminate()


if __name__ == "__main__":
    main()
