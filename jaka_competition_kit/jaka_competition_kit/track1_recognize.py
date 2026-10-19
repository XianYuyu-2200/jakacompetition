#!/usr/bin/env python3
"""用腕部 Gemini 305 认基础组的 6 个几何体。

只订彩色图、深度图和相机内参。外形规则在 track1_shapes.py，不看颜色。
认出来的结果画在图上，并发到 /track1/detections（JSON 文本）。

  python3 -m jaka_competition_kit.track1_recognize
"""
from __future__ import annotations

import json
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from jaka_competition_kit.track1_shapes import (
    PLACEMENTS,
    SHAPE_BY_ID,
    SHAPES,
    classify_contour,
    height_mask,
    self_check,
)


def _font(size: int):
    try:
        from PIL import ImageFont
    except ImportError:
        return None
    candidates = (
        ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 2),
        ("/usr/share/fonts/truetype/arphic/uming.ttc", 0),
    )
    for path, index in candidates:
        if os.path.isfile(path):
            try:
                return ImageFont.truetype(path, size, index=index)
            except TypeError:
                return ImageFont.truetype(path, size)
            except OSError:
                continue
    return None


def _label_y(box_y, img_h, text_h=26):
    """近排工件贴着画面上沿，标签画在框上方会被切掉，改画到框内顶边。"""
    above = box_y - text_h
    if above >= 2:
        return above
    return min(box_y + 4, max(2, img_h - text_h - 2))


def _draw_labels(image_bgr, items):
    """items: (x, y, w, h, text, ok). 中文走 PIL，没有字体就退回英文。"""
    font = _font(22)
    img_h = image_bgr.shape[0]
    if font is None:
        canvas = image_bgr.copy()
        for x, y, w, h, text, ok in items:
            color = (40, 170, 60) if ok else (40, 40, 210)
            cv2.rectangle(canvas, (x, y), (x + w, y + h), color, 2)
            cv2.putText(canvas, text, (x, _label_y(y, img_h, 18) + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        return canvas
    from PIL import Image, ImageDraw
    rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    pil = Image.fromarray(rgb)
    draw = ImageDraw.Draw(pil)
    for x, y, w, h, text, ok in items:
        color = (40, 170, 60) if ok else (210, 40, 40)
        draw.rectangle((x, y, x + w, y + h), outline=color, width=2)
        draw.text((x, _label_y(y, img_h)), text, font=font, fill=color)
    return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)


class Track1Recognizer(Node):
    def __init__(self):
        super().__init__("track1_recognize")
        self_check()
        self.declare_parameter("output_dir", "")
        self.declare_parameter("min_area", 600)
        self.declare_parameter("max_area", 20000)
        self.output_dir = self.get_parameter("output_dir").value or self._default_dir()
        self.min_area = int(self.get_parameter("min_area").value)
        self.max_area = int(self.get_parameter("max_area").value)
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.info = None
        self.color_at = 0.0
        self.depth_at = 0.0
        self.last_save = 0.0
        self.best = None
        self.pub = self.create_publisher(String, "/track1/detections", 10)
        self.create_subscription(
            Image, "/gemini305/color/image_raw", self._on_color, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/gemini305/depth/image_raw", self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, "/gemini305/color/camera_info", self._on_info, qos_profile_sensor_data)
        self.create_timer(0.4, self._tick)
        names = "、".join(s.name for s in SHAPES)
        self.get_logger().info(f"基础组识别已启动，等腕部相机。目标：{names}")

    def _default_dir(self) -> str:
        here = os.path.abspath(os.path.dirname(__file__))
        for _ in range(6):
            here = os.path.dirname(here)
            design = os.path.join(here, "design", "腕部Gemini305.md")
            if os.path.isfile(design):
                return os.path.join(here, "design", "track1_recognize")
        return "/tmp/track1_recognize"

    def _on_color(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.color_at = time.monotonic()

    def _on_depth(self, msg):
        raw = np.asarray(self.bridge.imgmsg_to_cv2(msg))
        if raw.dtype == np.uint16:
            depth = raw.astype(np.float32) / 1000.0
        else:
            depth = raw.astype(np.float32)
            finite = depth[np.isfinite(depth)]
            if finite.size and float(np.max(finite)) > 50.0:
                depth = depth / 1000.0
        self.depth = depth
        self.depth_at = time.monotonic()

    def _on_info(self, msg):
        self.info = msg

    def _pixels_to_world(self, u: float, v: float, z_opt: float):
        """光轴深度是光学系的 z。没有内参就只回像素。"""
        if self.info is None or z_opt <= 0:
            return None
        fx = self.info.k[0]
        fy = self.info.k[4]
        cx = self.info.k[2]
        cy = self.info.k[5]
        if fx <= 1 or fy <= 1:
            return None
        # 这里只给出相对画面中心的台面偏移，真正的世界坐标要再乘相机姿态。
        # 节点不依赖 TF：用已知的识别位姿把光学系对齐到世界。
        # 工具朝下且相机在世界 +Y 时，光学 +X 是世界 -X，光学 +Y 是世界 +Y。
        x_opt = (u - cx) / fx * z_opt
        y_opt = (v - cy) / fy * z_opt
        # 相机光轴落点。法兰在 (0, 0.2775, 0.28)，外壳在世界 +Y 61.5 mm、上移 9.5 mm。
        cam_y = 0.2775 + 0.0615
        cam_z = 0.28 + 0.0095
        world_x = -x_opt
        world_y = cam_y + y_opt
        world_z = cam_z - z_opt
        return (world_x, world_y, world_z)

    def _detect(self, depth: np.ndarray):
        mask = height_mask(depth)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        height, width = mask.shape
        found = []
        for contour in contours:
            area = abs(cv2.contourArea(contour))
            if area < self.min_area or area > self.max_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if x <= 1 or y <= 1 or x + w >= width - 1 or y + h >= height - 1:
                continue
            kind, features = classify_contour(contour, depth)
            m = cv2.moments(contour)
            if m["m00"] == 0:
                continue
            u = m["m10"] / m["m00"]
            v = m["m01"] / m["m00"]
            core = depth[max(0, int(v) - 2):int(v) + 3, max(0, int(u) - 2):int(u) + 3]
            z = float(np.median(core)) if core.size else 0.0
            world = self._pixels_to_world(u, v, z)
            found.append({
                "id": kind,
                "name": SHAPE_BY_ID[kind].name,
                "bbox": [int(x), int(y), int(w), int(h)],
                "center_px": [round(u, 1), round(v, 1)],
                "world_xyz": None if world is None else [round(n, 4) for n in world],
                "area": int(area),
                **features,
            })
        return found, mask

    def _score(self, found):
        """用摆放坐标核对，不参与分类。对上的记 ok。"""
        truth = {sid: (x, y) for sid, x, y, _yaw in PLACEMENTS}
        used = set()
        for item in found:
            item["ok"] = False
            world = item["world_xyz"]
            if world is None or item["id"] not in truth or item["id"] in used:
                continue
            tx, ty = truth[item["id"]]
            if abs(world[0] - tx) < 0.04 and abs(world[1] - ty) < 0.04:
                item["ok"] = True
                used.add(item["id"])
        ids = [item["id"] for item in found]
        complete = len(found) == 6 and len(set(ids)) == 6 and set(ids) == set(truth)
        located = sum(1 for item in found if item["ok"])
        return complete, located

    def _tick(self):
        if self.color is None or self.depth is None:
            return
        if abs(self.color_at - self.depth_at) > 0.5:
            return
        if self.color.shape[:2] != self.depth.shape[:2]:
            return
        found, _mask = self._detect(self.depth)
        complete, located = self._score(found)
        payload = {
            "count": len(found),
            "complete": complete,
            "located": located,
            "objects": found,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)
        now = time.monotonic()
        better = self.best is None or located > self.best[0] or (
            located == self.best[0] and complete and not self.best[1])
        if better or (complete and now - self.last_save > 2.0):
            self._save(found, complete, located)
            self.best = (located, complete)
            self.last_save = now
            names = " ".join(f"{it['name']}{'✓' if it['ok'] else '?'}" for it in found)
            self.get_logger().info(
                f"看到 {len(found)} 个，位置对上 {located}/6。{names}")

    def _save(self, found, complete, located):
        os.makedirs(self.output_dir, exist_ok=True)
        items = []
        for it in found:
            x, y, w, h = it["bbox"]
            mark = " 对" if it["ok"] else ""
            items.append((x, y, w, h, it["name"] + mark, it["ok"]))
        canvas = _draw_labels(self.color, items)
        tag = "6/6" if complete and located == 6 else f"{located}/6"
        cv2.putText(canvas, tag, (12, canvas.shape[0] - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imwrite(os.path.join(self.output_dir, "result.png"), canvas)
        path = os.path.join(self.output_dir, "detections.json")
        with open(path, "w", encoding="utf-8") as fh:
            json.dump({"complete": complete, "located": located, "objects": found},
                      fh, ensure_ascii=False, indent=2)


def main(args=None):
    rclpy.init(args=args)
    node = Track1Recognizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
