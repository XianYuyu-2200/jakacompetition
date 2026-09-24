#!/usr/bin/env python3
"""扫描: 在"工具严格竖直向下"约束下, 不同台面高度/半径的 IK 可达边界。

这是比赛规则里"工作区间"数字的实测依据。
用法(先启动 demo.launch.py use_rviz_sim:=true):
  python3 verification/reach_scan.py
"""
import math, sys
import rclpy
from check_arena_layout import ArenaCheck

def main():
    rclpy.init()
    node = ArenaCheck()
    print("加工作台 ...", node.add_table())
    print("\n== 工具严格竖直向下时的可达半径上限 ==")
    print(f"{'抓取高度z(mm)':>13s} {'最大可达半径 r(mm)':>20s}")
    for zmm in (30, 50, 80, 120, 160, 200):
        z = zmm / 1000.0
        lo, hi = 50, 620
        # 二分找边界(允许 5mm 误差)
        while hi - lo > 5:
            mid = (lo + hi) / 2
            if node.ik(mid/1000.0, 0.0, z) is not None:
                lo = mid
            else:
                hi = mid
        print(f"{zmm:13d} {lo:20.0f}")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
