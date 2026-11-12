"""命令行试一下工具端 NPN 夹爪。

示教器上点 DO_1 张开、点 DO_2 闭合。这里做同一件事, 两路互斥:

  ros2 run jaka_competition_kit gripper_io open
  ros2 run jaka_competition_kit gripper_io close

需要 jaka_planner 已经登录机械臂(服务 /jaka_planner/set_io)。
没装新的入口脚本时也可以:

  python3 -m jaka_competition_kit.gripper_io open
"""
from __future__ import annotations

import sys

import rclpy

from .gripper import JakaIOGripper


def main(argv=None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    cmd = args[0] if args else ""
    if cmd not in ("open", "close"):
        print("用法: gripper_io open|close", file=sys.stderr)
        print("  open  = 工具端 NPN DO_1 置 1, DO_2 置 0", file=sys.stderr)
        print("  close = 工具端 NPN DO_2 置 1, DO_1 置 0", file=sys.stderr)
        return 2
    rclpy.init()
    node = rclpy.create_node("gripper_io")
    try:
        gripper = JakaIOGripper(node)
        ok = gripper.open() if cmd == "open" else gripper.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
