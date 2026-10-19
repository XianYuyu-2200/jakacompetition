#!/usr/bin/env python3
"""用 YOLO11n 训练赛道二的 20 类检测。

口头说的 YOLOv11 就是 Ultralytics YOLO11。权重文件名是 yolo11n.pt,
没有 yolov11n.pt 这一个文件。默认用 nano: 腕部相机每抓一次认一次,
这个尺寸够快, 不够准先补数据, 不要先换大模型。
"""
from __future__ import annotations

import argparse
from pathlib import Path

from classes import package_dir, write_data_yaml
from weights_path import ensure_pretrained


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def count_images(dataset: Path, split: str) -> int:
    folder = dataset / "images" / split
    if not folder.is_dir():
        return 0
    return sum(1 for path in folder.iterdir() if path.suffix.lower() in IMAGE_EXTS)


def parse_device(value: str):
    if value.lower() == "cpu":
        return "cpu"
    if value.isdigit():
        return int(value)
    return value


def main():
    parser = argparse.ArgumentParser(description="训练赛道二 YOLO11 检测器")
    parser.add_argument("--model", default="yolo11n.pt")
    parser.add_argument("--data", type=Path, default=None, help="默认用 dataset/track2.yaml")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--patience", type=int, default=25)
    parser.add_argument("--no-amp", action="store_true")
    args = parser.parse_args()

    dataset = package_dir() / "dataset"
    train_n = count_images(dataset, "train")
    val_n = count_images(dataset, "val")
    if train_n == 0 or val_n == 0:
        raise SystemExit(
            f"还没有可训练的数据 (train={train_n}, val={val_n})。\n"
            "先在比赛用的那个腕部位姿采集, 再跑 split_dataset.py。\n"
            "不要拿空权重或者只下载的 yolo11n.pt 去上场。"
        )

    data = args.data
    if data is None:
        data = write_data_yaml(dataset / "track2.yaml")
    elif not data.is_file():
        raise SystemExit(f"找不到数据配置: {data}")

    weights = ensure_pretrained(args.model)
    from ultralytics import YOLO

    model = YOLO(str(weights))
    results = model.train(
        data=str(data),
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=args.batch,
        device=parse_device(args.device),
        workers=args.workers,
        patience=args.patience,
        amp=not args.no_amp,
        pretrained=True,
        project=str(package_dir() / "runs"),
        name="track2",
        exist_ok=False,
        seed=0,
        degrees=10.0,
        flipud=0.0,
        fliplr=0.5,
        cache=False,
    )
    save_dir = Path(getattr(results, "save_dir", package_dir() / "runs" / "track2"))
    best = save_dir / "weights" / "best.pt"
    print(f"训练结束。权重: {best}")
    print("下一步: python predict.py --source 0")


if __name__ == "__main__":
    main()
