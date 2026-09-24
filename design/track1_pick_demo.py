#!/usr/bin/env python3
"""赛道一参考实现骨架: 二维码序令定点抓取。

演示整条链路(不需要真机, mock 硬件即可):
  1. 往规划场景放工作台 + 收纳盒;
  2. 读一个 6 位抓取顺序(模拟二维码内容);
  3. 按顺序对每个工位做 预抓取 -> 下降 -> 闭合 -> 抬起 -> 移到料盒 -> 松开;
  4. 每一步都通过 MoveGroup action 规划 + 执行, 记录耗时。

用法(先启动 demo.launch.py use_rviz_sim:=true):
  python3 design/track1_pick_demo.py 246135
"""
import os, sys, time
import rclpy
import rclpy.action
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, MotionPlanRequest, OrientationConstraint,
                             PlanningOptions, PositionConstraint)
from shape_msgs.msg import SolidPrimitive

VEL = float(os.environ.get("JAKA_VEL", "0.3"))
ATTEMPTS = int(os.environ.get("JAKA_ATTEMPTS", "5"))
ALLOWED = float(os.environ.get("JAKA_ALLOWED", "5.0"))
GROUP = "jaka_minicobo"
IK_LINK = "dummy_tcp"
FRAME = "world"
STATION_PITCH = 0.150
ARRAY_CENTER_R = 0.280          # 推荐值(原稿 0.350 会让最外工位超界)
GRASP_Z = 0.050
PREGRASP_Z = 0.150
BIN_XY = (-0.330, 0.230)        # 料盒位置
STATION_Q = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)   # 工具竖直向下


def station_xy(idx):
    """idx 1..6 -> (x, y)。近排 1-3, 远排 4-6, 从左到右。"""
    row, col = divmod(idx - 1, 3)
    x = (col - 1) * STATION_PITCH
    y = ARRAY_CENTER_R + (row - 0.5) * STATION_PITCH
    return x, y


class PickDemo(Node):
    def __init__(self):
        super().__init__("track1_pick_demo")
        self._ac = rclpy.action.ActionClient(self, MoveGroup, "move_action")
        self._ac.wait_for_server()
        self.get_logger().info("MoveGroup action 已连接")

    def goto(self, x, y, z, label=""):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.num_planning_attempts = ATTEMPTS
        req.allowed_planning_time = ALLOWED
        req.max_velocity_scaling_factor = VEL
        req.max_acceleration_scaling_factor = VEL

        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation = STATION_Q

        pc = PositionConstraint()
        pc.header = ps.header
        pc.link_name = IK_LINK
        s = SolidPrimitive(); s.type = SolidPrimitive.SPHERE; s.dimensions = [0.005]
        pc.constraint_region.primitives.append(s)
        pc.constraint_region.primitive_poses.append(ps.pose)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = ps.header
        oc.link_name = IK_LINK
        oc.orientation = STATION_Q
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)

        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False       # 规划并执行
        goal.planning_options.replan = True

        t0 = time.time()
        fut = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, time.time() - t0, "目标被拒绝"
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        res = rf.result().result
        dt = time.time() - t0
        code = res.error_code.val
        return code == 1, dt, f"error_code={code}"

    def run(self, order):
        print(f"\n二维码解析出的抓取顺序: {order}")
        total = 0.0
        ok_cnt = 0
        for i, idx in enumerate(order, 1):
            sx, sy = station_xy(idx)
            steps = [
                ("预抓取", sx, sy, PREGRASP_Z),
                ("下降",   sx, sy, GRASP_Z),
                ("抬起",   sx, sy, PREGRASP_Z),
                ("到料盒", BIN_XY[0], BIN_XY[1], PREGRASP_Z),
            ]
            line = f"  [{i}/6] 工位{idx} (x={sx*1000:.0f}, y={sy*1000:.0f}) "
            oks = []
            t_st = time.time()
            for nm, px, py, pz in steps:
                ok, dt, info = self.goto(px, py, pz, nm)
                oks.append(ok)
                if not ok:
                    line += f"{nm}失败({info}) "
                    break
            dt_all = time.time() - t_st
            total += dt_all
            if all(oks):
                ok_cnt += 1
                line += f"✔ 用时 {dt_all:.2f}s"
            print(line)
        print(f"\n配置: 速度缩放={VEL}, 规划尝试={ATTEMPTS}, 规划时限={ALLOWED}s")
        print(f"成功抓取 {ok_cnt}/6, 总耗时 {total:.2f}s (限时 90s)")
        return ok_cnt


def main():
    rclpy.init()
    node = PickDemo()
    # 放工作台
    from check_arena_layout import ArenaCheck
    helper = ArenaCheck()
    print("工作台:", helper.add_table())
    r = rclpy.executors.SingleThreadedExecutor()
    r.add_node(node); r.add_node(helper)

    order = sys.argv[1] if len(sys.argv) > 1 else "246135"
    order = [int(c) for c in order]
    node.run(order)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
