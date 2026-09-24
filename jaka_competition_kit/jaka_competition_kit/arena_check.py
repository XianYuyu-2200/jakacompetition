"""赛场定义自检 —— 改完 arena.yaml 必须跑一遍。

用法: ros2 run jaka_competition_kit arena_check
"""
from __future__ import annotations

import math
import sys

from .arena import load_arena, load_rules


def main(args=None):
    a = load_arena()
    print(f"赛场定义: {a.config_path}")
    print(f"有效作业带: r = {a.r_min*1000:.0f} .. {a.r_max*1000:.0f} mm\n")
    print("赛道一布局:")
    for i, x, y in a.stations():
        print(f"  工位{i}: ({x*1000:7.1f}, {y*1000:6.1f}) mm   r = "
              f"{math.hypot(x, y)*1000:6.1f} mm")
    b = a.track1_bin
    print(f"  料盒  : ({b.center[0]*1000:7.1f}, {b.center[1]*1000:6.1f}) mm   r = "
          f"{math.hypot(*b.center)*1000:6.1f} mm")
    q = a.qr
    print(f"  二维码: ({q.center[0]*1000:7.1f}, {q.center[1]*1000:6.1f}) mm   r = "
          f"{math.hypot(*q.center)*1000:6.1f} mm  (仅需相机可见)")
    print("\n赛道二布局:")
    s = a.scatter
    print(f"  散放区: {s.size[0]*1000:.0f} x {s.size[1]*1000:.0f} mm, "
          f"中心 ({s.center[0]*1000:.0f}, {s.center[1]*1000:.0f})")
    nx, ny = s.nearest_point()
    print(f"          最近点 ({nx*1000:.0f}, {ny*1000:.0f}) r = "
          f"{math.hypot(nx, ny)*1000:.0f} mm  <- 判可达性的关键点")
    print("          四角 r = " +
          ", ".join(f"{math.hypot(*c)*1000:.0f}" for c in s.corners()) + " mm")
    b2 = a.track2_bin
    print(f"  料盒  : ({b2.center[0]*1000:7.1f}, {b2.center[1]*1000:6.1f}) mm   r = "
          f"{math.hypot(*b2.center)*1000:6.1f} mm")

    problems = a.validate()
    print()
    if problems:
        print("校核失败:")
        for p in problems:
            print("  ✘", p)
        sys.exit(1)
    print("校核通过 ✔  所有抓取点与散放区均在有效作业带内")

    r = load_rules()
    print(f"\n评分规则: 赛道一限时 {r['track1']['time_limit_s']:.0f}s "
          f"(效率满分 {r['track1']['efficiency']['full_marks_within_s']:.0f}s), "
          f"赛道二限时 {r['track2']['time_limit_s']:.0f}s "
          f"(效率满分 {r['track2']['efficiency']['full_marks_within_s']:.0f}s)")


if __name__ == "__main__":
    main()
