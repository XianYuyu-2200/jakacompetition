#!/usr/bin/env python3
"""用训练好的 YOLO11 权重认 20 类, 并映射到四只料盒。

检测框不是抓取点。抓取点以后取框内深度最高的那一片的中位数,
不要用框中心去夹。
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from classes import by_index, load_catalog, package_dir
from vis import BIN_BGR, blit_labels, status_bar
from weights_path import ensure_pretrained


@dataclass(frozen=True)
class Det:
    index: int
    id: str
    name_zh: str
    bin_id: str
    bin_name_zh: str
    conf: float
    xyxy: tuple[float, float, float, float]


def newest_best() -> Path | None:
    candidates = list((package_dir() / "runs").glob("track2*/weights/best.pt"))
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def load_detector(weights: str | Path | None = None):
    """Load a trained checkpoint. None picks the newest runs/track2*/weights/best.pt."""
    from ultralytics import YOLO

    if weights is None:
        path = newest_best()
        if path is None:
            raise SystemExit("没有 best.pt。先采集、划分, 再跑 train.py。")
    else:
        path = Path(weights)
        if not path.is_file():
            raise SystemExit(f"找不到权重: {path}")
    return YOLO(str(path))


def parse_device(value: str):
    if value.lower() == "cpu":
        return "cpu"
    if value.isdigit():
        return int(value)
    return value


def detect_bgr(model, bgr, conf=0.25, imgsz=640, device=0) -> list[Det]:
    catalog = by_index(load_catalog())
    results = model.predict(bgr, conf=conf, imgsz=imgsz, device=device, verbose=False)
    found = []
    if not results or results[0].boxes is None:
        return found
    for box in results[0].boxes:
        class_index = int(box.cls.item())
        sku = catalog.get(class_index)
        if sku is None:
            continue
        xyxy = tuple(float(value) for value in box.xyxy[0].tolist())
        found.append(
            Det(
                index=sku.index,
                id=sku.id,
                name_zh=sku.name_zh,
                bin_id=sku.bin_id,
                bin_name_zh=sku.bin_name_zh,
                conf=float(box.conf.item()),
                xyxy=xyxy,  # type: ignore[arg-type]
            )
        )
    return found


def draw_dets(frame, dets):
    view = frame.copy()
    labels = []
    for det in dets:
        color = BIN_BGR.get(det.bin_id, (220, 220, 220))
        x1, y1, x2, y2 = (int(v) for v in det.xyxy)
        cv2.rectangle(view, (x1, y1), (x2, y2), color, 2)
        labels.append(
            (
                f"{det.conf:.2f} {det.id} {det.name_zh} -> {det.bin_name_zh}",
                x1,
                max(0, y1 - 26),
                color,
                18,
                (0, 0, 0),
            )
        )
    return blit_labels(view, labels)


def resolve_weights(spec: str | None) -> Path:
    if spec:
        path = Path(spec)
        if path.is_file():
            return path.resolve()
        # 允许传入 yolo11n.pt 这种预训练名, 但上场应该用 best.pt。
        if spec.endswith(".pt") and not spec.startswith("yolo11"):
            raise SystemExit(f"找不到权重: {spec}")
        return ensure_pretrained(spec)
    found = newest_best()
    if found is None:
        raise SystemExit("没有训练权重。先跑 train.py。不要拿预训练的 yolo11n.pt 认这 20 类。")
    return found


def main():
    parser = argparse.ArgumentParser(description="看 YOLO11 把工件分到哪只料盒")
    parser.add_argument("--weights", default=None, help="默认用最新一次训练的 best.pt")
    parser.add_argument("--source", default="0", help="相机序号, 或一张图片的路径")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    weights = resolve_weights(args.weights)
    print(f"权重 {weights}")
    model = load_detector(weights)
    device = parse_device(args.device)
    out_dir = package_dir() / "runs" / "predict"
    out_dir.mkdir(parents=True, exist_ok=True)

    source = args.source
    if source.isdigit():
        capture = cv2.VideoCapture(int(source), cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            capture = cv2.VideoCapture(int(source))
        if not capture.isOpened():
            raise SystemExit(f"打不开相机 {source}")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        window = "track2-predict"
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        try:
            while True:
                ok, frame = capture.read()
                if not ok or frame is None:
                    print("读相机失败, 退出。")
                    break
                dets = detect_bgr(model, frame, conf=args.conf, imgsz=args.imgsz, device=device)
                view = draw_dets(frame, dets)
                summary = "  ".join(f"{det.id}:{det.bin_id}" for det in dets) or "没有检出"
                canvas_bar = status_bar(view.shape[1], [summary, "q 退出"])
                cv2.imshow(window, np.vstack([view, canvas_bar]))
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
        finally:
            capture.release()
            cv2.destroyAllWindows()
        return

    image_path = Path(source)
    if not image_path.is_file():
        raise SystemExit(f"找不到图片: {image_path}")
    frame = cv2.imread(str(image_path))
    if frame is None:
        raise SystemExit(f"读不了图片: {image_path}")
    dets = detect_bgr(model, frame, conf=args.conf, imgsz=args.imgsz, device=device)
    for det in dets:
        x1, y1, x2, y2 = det.xyxy
        print(
            f"{det.conf:.3f}  {det.id:<12} {det.name_zh:<8} -> {det.bin_id:<6} {det.bin_name_zh}"
            f"  xyxy=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f})"
        )
    if not dets:
        print("没有检出。")
    stamped = out_dir / f"{image_path.stem}_pred.jpg"
    cv2.imwrite(str(stamped), draw_dets(frame, dets))
    print(f"wrote {stamped}")


if __name__ == "__main__":
    main()
