#!/usr/bin/env python3
"""从正在跑的腕部相机预览里采集基础组 6 类，并写成 YOLO 标注。

不手动画框。仿真知道每个几何体的位姿，把八个角投到图像上就是框。
这是没有真机时的练习数据，不能拿去当决赛那 20 类日用品的权重。

预览要先开着：

  ros2 launch jaka_competition_kit gemini305_preview.launch.py

然后：

  python3 perception/track1_sim/capture_sim.py --samples 24

图片写到 perception/track1_sim/dataset/incoming/<批次>/。
脚本结束时把 6 个几何体放回演示位置。
"""
from __future__ import annotations

import argparse
import math
import random
import subprocess
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from shapes import HOME, SHAPES, Shape


WORLD = "gemini305_preview"
# 手指在画面上沿，对应较小的世界 y。低于这条会和手指叠在一起。
Y_MIN = 0.245
Y_MAX = 0.400
X_MIN = -0.170
X_MAX = 0.170
# 两个中心至少隔开这么多，50 mm 左右的件才不会叠。
MIN_GAP = 0.090


def _package_dir() -> Path:
    return Path(__file__).resolve().parent


def _yaw_quat(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def set_model_pose(name: str, x: float, y: float, z: float, yaw: float) -> None:
    qz, qw = _yaw_quat(yaw)
    req = (
        f'name: "{name}", position: {{x: {x:.5f}, y: {y:.5f}, z: {z:.5f}}}, '
        f"orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}"
    )
    detail = ""
    for _ in range(3):
        result = subprocess.run(
            [
                "ign", "service",
                "-s", f"/world/{WORLD}/set_pose",
                "--reqtype", "ignition.msgs.Pose",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "2000",
                "--req", req,
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode == 0 and "true" in result.stdout.lower():
            return
        detail = (result.stdout + result.stderr).strip()
        time.sleep(0.2)
    raise RuntimeError(f"移动 {name} 失败: {detail}")


def place(shape: Shape, x: float, y: float, yaw: float) -> None:
    set_model_pose(f"shape_{shape.id}", x, y, shape.center_z, yaw)
    set_model_pose(f"plate_{shape.id}", x, y, 0.001, 0.0)


def restore_home() -> None:
    for sid, x, y, yaw in HOME:
        shape = next(s for s in SHAPES if s.id == sid)
        place(shape, x, y, yaw)


def _separated(points: list[tuple[float, float]]) -> bool:
    for i, (x1, y1) in enumerate(points):
        for x2, y2 in points[i + 1:]:
            if math.hypot(x1 - x2, y1 - y2) < MIN_GAP:
                return False
    return True


def sample_layout(rng: random.Random) -> list[tuple[Shape, float, float, float]]:
    """两排三列，每件在格子里抖一点，再随机转一个 yaw。"""
    xs = (-0.120, 0.0, 0.120)
    ys = (0.262, 0.348)
    slots = [(x, y) for y in ys for x in xs]
    rng.shuffle(slots)
    layout = []
    centers = []
    for shape, (sx, sy) in zip(SHAPES, slots):
        for _ in range(40):
            x = min(X_MAX, max(X_MIN, sx + rng.uniform(-0.018, 0.018)))
            y = min(Y_MAX, max(Y_MIN, sy + rng.uniform(-0.012, 0.012)))
            if _separated(centers + [(x, y)]):
                break
        else:
            x, y = sx, sy
        centers.append((x, y))
        layout.append((shape, x, y, rng.uniform(-math.pi, math.pi)))
    return layout


def world_corners(shape: Shape, x: float, y: float, yaw: float) -> list[tuple[float, float, float]]:
    xmin, xmax, ymin, ymax, zmin, zmax = shape.bounds
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for lx in (xmin, xmax):
        for ly in (ymin, ymax):
            for lz in (zmin, zmax):
                pts.append((x + c * lx - s * ly, y + s * lx + c * ly, shape.center_z + lz))
    return pts


def project(points, cam, k) -> list[tuple[float, float]] | None:
    """cam 是 gemini305_link 在 base_link 下的平移。光轴朝世界 −Z。"""
    fx, fy, cx, cy = k
    cam_x, cam_y, cam_z = cam
    uvs = []
    for x, y, z in points:
        z_opt = cam_z - z
        if z_opt < 0.05:
            return None
        x_opt = -(x - cam_x)
        y_opt = y - cam_y
        uvs.append((fx * x_opt / z_opt + cx, fy * y_opt / z_opt + cy))
    return uvs


def yolo_box(uvs, width: int, height: int) -> tuple[float, float, float, float] | None:
    us = [u for u, _ in uvs]
    vs = [v for _, v in uvs]
    x1, x2 = min(us), max(us)
    y1, y2 = min(vs), max(vs)
    if x1 < 2 or y1 < 2 or x2 > width - 3 or y2 > height - 3:
        return None
    bw, bh = x2 - x1, y2 - y1
    if bw < 8 or bh < 8:
        return None
    return ((x1 + x2) / 2 / width, (y1 + y2) / 2 / height, bw / width, bh / height)


def draw_boxes(image, items):
    canvas = image.copy()
    for name, uvs in items:
        us = [int(round(u)) for u, _ in uvs]
        vs = [int(round(v)) for _, v in uvs]
        x1, y1, x2, y2 = min(us), min(vs), max(us), max(vs)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (40, 180, 60), 2)
        cv2.putText(canvas, name, (x1, max(16, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 180, 60), 1, cv2.LINE_AA)
    return canvas


class Camera(Node):
    def __init__(self):
        super().__init__("track1_sim_capture")
        self.bridge = CvBridge()
        self.color = None
        self.color_at = 0.0
        self.k = None
        self.create_subscription(
            Image, "/gemini305/color/image_raw", self._on_color, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, "/gemini305/color/camera_info", self._on_info, qos_profile_sensor_data)

    def _on_color(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.color_at = time.monotonic()

    def _on_info(self, msg):
        self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def wait_fresh(self, after: float, timeout: float = 3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.color is not None and self.k is not None and self.color_at > after:
                return self.color.copy(), self.k
        raise TimeoutError("没有等到新的腕部彩色图。预览仿真是否还在？")


def capture(samples: int, out: Path, seed: int, start: int = 0) -> None:
    rng = random.Random(seed)
    # 续采时先把随机序列快进到上次停下的地方，避免和已有图片同一摆。
    for _ in range(start):
        sample_layout(rng)
    out.mkdir(parents=True, exist_ok=True)
    preview = out / "preview"
    preview.mkdir(exist_ok=True)
    rclpy.init()
    cam = Camera()
    saved = start
    failures = 0
    try:
        cam.wait_fresh(0.0)
        # TF 实测 gemini305_link 在 base 下是 (0, 0.339, 0.289)，光轴朝下。
        camera_xyz = (0.0, 0.339, 0.289)
        while saved < samples:
            try:
                layout = sample_layout(rng)
                for shape, x, y, yaw in layout:
                    place(shape, x, y, yaw)
                time.sleep(0.30)
                mark = time.monotonic()
                image, k = cam.wait_fresh(mark)
                height, width = image.shape[:2]
                lines = []
                drawn = []
                ok = True
                for shape, x, y, yaw in layout:
                    uvs = project(world_corners(shape, x, y, yaw), camera_xyz, k)
                    box = None if uvs is None else yolo_box(uvs, width, height)
                    if box is None:
                        ok = False
                        break
                    cls = next(i for i, s in enumerate(SHAPES) if s.id == shape.id)
                    lines.append(f"{cls} {box[0]:.6f} {box[1]:.6f} {box[2]:.6f} {box[3]:.6f}")
                    drawn.append((shape.id, uvs))
                if not ok:
                    print(f"这一摆有件出了画面，重来（已保存 {saved}）", flush=True)
                    continue
                seq = saved + 1
                image_path = out / f"{seq:04d}.jpg"
                label_path = out / f"{seq:04d}.txt"
                cv2.imwrite(str(image_path), image, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
                label_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
                cv2.imwrite(str(preview / f"{seq:04d}.jpg"), draw_boxes(image, drawn))
                saved += 1
                failures = 0
                print(f"保存 {image_path.name}", flush=True)
            except Exception as exc:  # noqa: BLE001
                failures += 1
                print(f"这一摆失败，重来（已保存 {saved}）: {exc}", flush=True)
                if failures >= 8:
                    raise
    finally:
        try:
            restore_home()
            print("几何体已放回演示位置")
        except Exception as exc:  # noqa: BLE001
            print(f"放回演示位置失败: {exc}")
        cam.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="仿真腕部相机采集基础组 6 类")
    parser.add_argument("--samples", type=int, default=24)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--session", default=None)
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="默认 dataset/incoming/<时间戳>",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="目录里已有图片时接着编号往下采，不覆盖",
    )
    args = parser.parse_args()
    session = args.session or time.strftime("sim-%Y%m%d-%H%M%S")
    out = args.out or (_package_dir() / "dataset" / "incoming" / session)
    if args.samples < 1:
        raise SystemExit("--samples 至少 1")
    start = 0
    if args.resume and out.is_dir():
        start = len(list(out.glob("[0-9][0-9][0-9][0-9].jpg")))
        print(f"续采：已有 {start} 张，目标 {args.samples}", flush=True)
    if start >= args.samples:
        print(f"已经有 {start} 张，不少于目标 {args.samples}: {out}")
        return
    capture(args.samples, out, args.seed, start)
    print(f"完成 {args.samples} 张: {out}", flush=True)


if __name__ == "__main__":
    main()
