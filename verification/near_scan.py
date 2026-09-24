#!/usr/bin/env python3
"""扫描严格竖直抓取时的"最近可达半径"。"""
import rclpy
from check_arena_layout import ArenaCheck

def main():
    rclpy.init(); node = ArenaCheck()
    node.add_table()
    print(f"{'抓取高度z(mm)':>13s} {'最近可达半径 r(mm)':>20s}")
    for zmm in (30, 50, 80, 120, 160, 200):
        z = zmm/1000.0
        # 从 5mm 向外找第一个能求得 IK 的半径(步长 5mm)
        found = None
        for r in range(5, 260, 5):
            if node.ik(r/1000.0, 0.0, z) is not None:
                found = r; break
        print(f"{zmm:13d} {str(found) if found else '无解':>20s}")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
