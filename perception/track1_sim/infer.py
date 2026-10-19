#!/usr/bin/env python3
"""用训好的 best.pt 看一张图，把检测框打到 stdout。

ROS 节点是 Python 3.10，YOLO 在 Python 3.12 的虚拟环境里，所以推理单独跑。
只打印一行 YOLO_JSON=...，别的日志走 stderr。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="对一张图跑基础组 YOLO")
    parser.add_argument("image")
    parser.add_argument("--weights", default="")
    parser.add_argument("--conf", type=float, default=0.40)
    args = parser.parse_args()

    root = Path(__file__).resolve().parent
    weights = Path(args.weights) if args.weights else root / "runs" / "track1" / "weights" / "best.pt"
    if not weights.is_file():
        print(f"没有权重 {weights}", file=sys.stderr)
        raise SystemExit(2)
    if not Path(args.image).is_file():
        print(f"没有图片 {args.image}", file=sys.stderr)
        raise SystemExit(2)

    from ultralytics import YOLO

    model = YOLO(str(weights))
    result = model.predict(source=args.image, conf=args.conf, verbose=False)[0]
    names = result.names
    found = []
    boxes = result.boxes
    if boxes is not None:
        for box in boxes:
            cls_id = int(box.cls)
            found.append({
                "id": str(names[cls_id]),
                "conf": round(float(box.conf), 4),
                "xyxy": [round(float(v), 1) for v in box.xyxy[0].tolist()],
            })
    sys.stdout.write("YOLO_JSON=" + json.dumps({"detections": found}, ensure_ascii=False) + "\n")
    sys.stdout.flush()


if __name__ == "__main__":
    main()
