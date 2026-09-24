#!/usr/bin/env python3
"""向 MoveIt 的 /move_action 发一个关节空间目标, 完成一次规划+执行。

前置: 已经起了 demo.launch.py (use_rviz_sim:=true 或 demo_gazebo.launch.py)。

用法:
  python3 moveit_plan_exec.py                 # 默认目标
  python3 moveit_plan_exec.py 0.4,0.9,-1.1,0,1.0,0.6
"""
import sys

import rclpy
from geometry_msgs.msg import Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    WorkspaceParameters,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Header

GROUP = "jaka_minicobo"
JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def main():
    target = ([float(x) for x in sys.argv[1].split(",")]
              if len(sys.argv) > 1 else [0.4, 0.9, -1.1, 0.0, 1.0, 0.6])

    rclpy.init()
    node = Node("moveit_plan_exec")
    client = ActionClient(node, MoveGroup, "/move_action")
    if not client.wait_for_server(timeout_sec=15):
        print("NO ACTION SERVER")
        sys.exit(1)
    print("action server available")

    req = MotionPlanRequest()
    req.group_name = GROUP
    req.num_planning_attempts = 10
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.3
    req.max_acceleration_scaling_factor = 0.3
    req.workspace_parameters = WorkspaceParameters(
        header=Header(stamp=node.get_clock().now().to_msg(), frame_id="Link_0"),
        min_corner=Vector3(x=-1.0, y=-1.0, z=-0.5),
        max_corner=Vector3(x=1.0, y=1.0, z=1.5),
    )

    goal_constraints = Constraints()
    for name, value in zip(JOINTS, target):
        goal_constraints.joint_constraints.append(JointConstraint(
            joint_name=name, position=value,
            tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
    req.goal_constraints.append(goal_constraints)

    goal = MoveGroup.Goal()
    goal.request = req
    options = PlanningOptions()
    options.plan_only = False            # False = 规划并执行
    options.look_around = False
    options.replan = False
    options.planning_scene_diff.is_diff = True
    options.planning_scene_diff.robot_state.is_diff = True
    goal.planning_options = options

    print("sending goal ->", target)
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=20)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        print("GOAL REJECTED")
        sys.exit(1)

    print("goal accepted, planning + executing ...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=120)
    result = result_future.result()
    if result is None:
        print("NO RESULT (timeout)")
        sys.exit(1)

    res = result.result
    traj = res.planned_trajectory.joint_trajectory
    print("error_code:", res.error_code.val)
    print("planning_time: %.3f s" % res.planning_time)
    print("trajectory points:", len(traj.points))
    if traj.points:
        print("final positions:", [round(v, 4) for v in traj.points[-1].positions])
    print("RESULT:", "SUCCESS(1)" if res.error_code.val == 1 else f"ERROR({res.error_code.val})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
