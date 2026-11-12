#!/usr/bin/env python3
"""基础组学生模板：读取二维码 PNG，使用固定工位训练坐标执行。

不是实时相机视觉，也不是物理摩擦夹持验证。先在工作台启动基础环境、
生成题目，选择本文件运行，并在工作台发令。ReferenceBase 负责等待场景
和发令；学生只需修改 plan_order，不改 Executor、碰撞规则或 IK。

须在已配置的 ROS 2 环境中运行；建议由工作台启动并传入 ROS 参数：
  python3 student_track1.py --ros-args -p gripper:=sim -p vel_scale:=0.5

本地程序受信任且无沙箱。不要选择不可信的第三方 Python 文件。
"""
from __future__ import annotations

from typing import Dict, List

import rclpy

from jaka_competition_kit.qr import decode_image, parse
from jaka_competition_kit.ref_track1 import Track1Reference
from jaka_competition_kit.reference import spin


class StudentTrack1(Track1Reference):
    def plan_order(self) -> List[Dict]:
        """练习入口：PNG 解码 -> 工位顺序 -> 固定工位对象列表。

        禁止用 scene['qr_order'] / scene['qr_payload'] 代替真实解码。
        这里仍使用场景中的固定训练坐标，不进行目标检测或深度定位。
        默认实现保持二维码要求的顺序；随意改序会影响判分。
        """
        png = self.scene.get("qr_png", "")
        if not png:
            self.get_logger().error("场景没有二维码 PNG，请重新生成基础组题目")
            return []
        try:
            text = decode_image(png)
            if not text:
                self.get_logger().error(f"未能从图片识读二维码：{png}")
                return []
            station_order = parse(text)
        except (RuntimeError, ValueError, OSError) as exc:
            self.get_logger().error(f"二维码识读失败：{exc}")
            return []

        # TODO（教学）：在这里加入自己的格式校验或顺序显示，保持正确工位顺序。
        by_station = {obj["station"]: obj for obj in self.scene.get("objects", [])}
        missing = [station for station in station_order if station not in by_station]
        if missing:
            self.get_logger().error(f"训练场景缺少工位 {missing}，停止执行")
            return []
        self.get_logger().info(f"学生程序识读 {text!r}，顺序：{station_order}")
        return [by_station[station] for station in station_order]


def main(args=None):
    rclpy.init(args=args)
    spin(StudentTrack1())


if __name__ == "__main__":
    main()
