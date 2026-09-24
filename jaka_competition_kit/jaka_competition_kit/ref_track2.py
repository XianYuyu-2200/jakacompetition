"""赛道二参考实现 —— 无序抓取的难度标定基准。

赛道二**不给顺序**, 由程序自己决定先抓哪一件。这里用最简单的一条基线策略:
**按到基座的水平距离从小到大抓**(近的先抓)。

这么做的好处是每条路径都最短、最容易规划成功; 局限是它完全没看工件长什么样,
也不挑抓取点。真正的比赛方案应该用视觉 + 抓取点生成, 这条基线只用来:

  - 验证赛场几何与工具链没问题;
  - 给"180 秒够不够"提供下界估计。

用法:
  ros2 run jaka_competition_kit ref_track2 --ros-args -p vel_scale:=1.0
  ros2 run jaka_competition_kit ref_track2 --ros-args -p verify_only:=true

参数同 ref_track1。
"""
from __future__ import annotations

from typing import Dict, List

import rclpy

from .reference import ReferenceBase, spin


class Track2Reference(ReferenceBase):
    def __init__(self):
        super().__init__("ref_track2")

    def plan_order(self) -> List[Dict]:
        objs = list(self.scene.get("objects", []))
        objs.sort(key=lambda o: (o["xy"][0] ** 2 + o["xy"][1] ** 2) ** 0.5)
        self.get_logger().info(
            "抓取顺序(由近及远): " + " -> ".join(
                f"{o.get('label','')}({((o['xy'][0]**2+o['xy'][1]**2)**0.5)*1000:.0f}mm)"
                for o in objs))
        return objs


def main(args=None):
    rclpy.init(args=args)
    spin(Track2Reference())


if __name__ == "__main__":
    main()
