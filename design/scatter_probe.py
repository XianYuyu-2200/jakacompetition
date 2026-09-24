#!/usr/bin/env python3
"""用 MoveIt /compute_ik 实测"工具竖直向下"时给定高度的可达区域。

用于给赛道二散放区定尺寸: 散放区里任意一点都必须可达,
且预抓取点(更高)也要可达 —— 后者才是真正的瓶颈。
"""
import sys, math, rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped, Quaternion

Q_DOWN = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


class Probe(Node):
    def __init__(self):
        super().__init__("scatter_probe")
        self.cli = self.create_client(GetPositionIK, "/compute_ik")
        self.cli.wait_for_service(timeout_sec=20.0)

    def ik(self, x, y, z, timeout=2.0):
        req = GetPositionIK.Request()
        r = PositionIKRequest()
        r.group_name = "jaka_minicobo"
        r.ik_link_name = "dummy_tcp"
        r.avoid_collisions = True
        r.timeout.sec = 0
        r.timeout.nanosec = 300_000_000
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation = Q_DOWN
        r.pose_stamped = ps
        req.ik_request = r
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None:
            return None
        return fut.result().error_code.val == 1


def main():
    rclpy.init()
    p = Probe()
    xs = [i / 100.0 for i in range(-20, 21, 1)]     # -0.20 .. 0.20
    ys = [i / 100.0 for i in range(5, 46, 1)]       #  0.05 .. 0.45
    for z in (0.10, 0.166):
        rows = []
        for y in ys:
            line = ""
            for x in xs:
                ok = p.ik(x, y, z, timeout=1.5)
                line += "?" if ok is None else ("#" if ok else ".")
            rows.append((y, line))
        print(f"\n=== z = {z*1000:.0f} mm  (# = IK 成功, . = 失败) ===")
        print("      x: " + "".join(f"{int(round(x*100))%10}" for x in xs))
        for y, line in rows:
            print(f"y={y*1000:4.0f}  {line}")
        # 每行的可达 x 范围
        print("  行内可达 x 范围(mm):")
        for y, line in rows:
            idx = [i for i, c in enumerate(line) if c == "#"]
            if idx:
                print(f"    y={y*1000:3.0f} : {xs[idx[0]]*1000:6.0f} .. {xs[idx[-1]]*1000:6.0f}  (n={len(idx)})")
            else:
                print(f"    y={y*1000:3.0f} : 无")
    p.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
