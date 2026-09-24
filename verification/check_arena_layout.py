#!/usr/bin/env python3
"""用真实 MoveIt 栈(FCL + URDF 碰撞网格)校核赛场工位布局。

做三件事:
  1. 往规划场景里放一块工作台(box), 台面 z=-1mm;
  2. 对每个候选工位求垂直下抓的 IK(/compute_ik);
  3. 检查该 IK 解是否与工作台/自身碰撞(/check_state_validity)。

用法(先启动 demo.launch.py use_rviz_sim:=true):
  source jaka_ros2/install/setup.bash
  python3 verification/check_arena_layout.py
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject,
                             PlanningScene, PositionIKRequest, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetPositionIK,
                             GetStateValidity)
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

GROUP = "jaka_minicobo"
IK_LINK = "dummy_tcp"
FRAME = "world"
TABLE_TOP = -0.001          # 台面 z(m), 略低于基座安装面以免贴面误判


def topdown_quat():
    """工具 z 轴指向 -Z(竖直向下): 绕 X 转 180°。"""
    return Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


class ArenaCheck(Node):
    def __init__(self):
        super().__init__("arena_check")
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.valid_cli = self.create_client(GetStateValidity, "/check_state_validity")
        for c, n in ((self.scene_cli, "apply_planning_scene"),
                     (self.ik_cli, "compute_ik"),
                     (self.valid_cli, "check_state_validity")):
            if not c.wait_for_service(timeout_sec=20.0):
                raise RuntimeError(f"服务不可用: {n}")

    def add_table(self, size=(0.9, 0.9), thickness=0.02, center_xy=(0.0, 0.30)):
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [size[0], size[1], thickness]
        obj = CollisionObject()
        obj.header.frame_id = FRAME
        obj.id = "workbench"
        obj.primitives.append(prim)
        pose = PoseStamped()
        pose.header.frame_id = FRAME
        pose.pose.position.x = center_xy[0]
        pose.pose.position.y = center_xy[1]
        pose.pose.position.z = TABLE_TOP - thickness / 2.0
        pose.pose.orientation.w = 1.0
        obj.primitive_poses.append(pose.pose)
        obj.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        fut = self.scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        return fut.result() and fut.result().success

    ORDER = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    def ik(self, x, y, z, avoid_collisions=True):
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation = topdown_quat()
        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = GROUP
        req.ik_request.ik_link_name = IK_LINK
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout.sec = 1
        req.ik_request.avoid_collisions = avoid_collisions
        rs = RobotState()
        rs.joint_state.name = self.ORDER
        rs.joint_state.position = [0.0, 0.6, -0.6, 0.0, 0.6, 0.0]
        req.ik_request.robot_state = rs
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        js = res.solution.joint_state
        return [dict(zip(js.name, js.position))[j] for j in self.ORDER]

    def valid(self, q, table=True):
        rs = RobotState()
        rs.joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        rs.joint_state.position = [float(v) for v in q]
        req = GetStateValidity.Request()
        req.robot_state = rs
        req.group_name = GROUP
        fut = self.valid_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None:
            return None, ["service failed"]
        contacts = [f"{c.contact_body_1}<->{c.contact_body_2}" for c in res.contacts]
        return res.valid, contacts


def main():
    rclpy.init()
    node = ArenaCheck()
    print("加工作台到规划场景 ...", node.add_table())
    time.sleep(1.0)

    pitch = 0.150
    print("说明: 'IK+避障' = 在含工作台的场景里直接求得无碰撞 IK 解(理想结果)")
    print(f"\n{'方案':>16s} {'工位':>6s} {'x':>7s} {'y':>7s} {'IK':>5s} {'避障IK':>10s} {'备注':>10s}")
    for R in (0.300, 0.350, 0.400):
        for row in (0, 1):
            for col in (0, 1, 2):
                x = (col - 1) * pitch
                y = R + (row - 0.5) * pitch
                z = 0.050   # 抓取高度 50mm
                nm = ('近' if row == 0 else '远') + str(col + 1)
                z = 0.050
                q = node.ik(x, y, z, avoid_collisions=True)
                if q is not None:
                    print(f"{R*1000:13.0f}mm {nm:>6s} {x*1000:7.0f} {y*1000:7.0f} "
                          f"{'有':>5s} {'无':>10s} {'无':>10s}  <-- 直接用")
                    continue
                q2 = node.ik(x, y, z, avoid_collisions=False)
                if q2 is None:
                    print(f"{R*1000:13.0f}mm {nm:>6s} {x*1000:7.0f} {y*1000:7.0f} "
                          f"{'无':>5s} {'-':>10s} {'-':>10s}  <-- 不可达")
                    continue
                v1, _ = node.valid(q2)
                print(f"{R*1000:13.0f}mm {nm:>6s} {x*1000:7.0f} {y*1000:7.0f} "
                      f"{'有':>5s} {'-':>10s} {'-':>10s}  <-- 可达但所有 IK 解都碰撞")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
