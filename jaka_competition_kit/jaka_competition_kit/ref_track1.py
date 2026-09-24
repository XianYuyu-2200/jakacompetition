"""赛道一参考实现 —— 也是出题方的难度标定基准。

流程: 等题目 -> **真实解码二维码**(不读真值) -> 按顺序抓取投放。
拿它跑 10 次取中位数, 就能标定单轮限时与效率分门槛。

用法:
  ros2 run jaka_competition_kit ref_track1 --ros-args -p vel_scale:=1.0
  ros2 run jaka_competition_kit ref_track1 --ros-args -p verify_only:=true

参数:
  vel_scale   运动速度缩放(0.3 保守 / 1.0 全速)
  verify_only 只规划不执行(赛前场地自检)
  gripper     mock(仿真) / jaka_io(真机)
"""
from __future__ import annotations

from typing import Dict, List

import rclpy

from .qr import decode_image, parse
from .reference import ReferenceBase, spin


class Track1Reference(ReferenceBase):
    def __init__(self):
        super().__init__("ref_track1")

    def plan_order(self) -> List[Dict]:
        png = self.scene.get("qr_png", "")
        text = decode_image(png) if png else ""
        if not text:
            self.get_logger().warn(f"二维码解码失败({png})")
            return []
        order = parse(text)
        self.get_logger().info(f"二维码内容 {text!r} -> 抓取顺序 {order}")
        by_station = {o["station"]: o for o in self.scene["objects"]}
        return [by_station[s] for s in order if s in by_station]


def main(args=None):
    rclpy.init(args=args)
    spin(Track1Reference())


if __name__ == "__main__":
    main()
