#!/usr/bin/env python3
"""用仿真采集的基础组图片训练 YOLO11n。

练习用。这 6 类是仿真里的几何体，权重不能拿去认真机上的 20 种日用品。

  perception/track2_yolo/.venv/bin/python perception/track1_sim/train.py
"""
from __future__ import annotations

import argparse
import random
import shutil
from pathlib import Path


def package_dir() -> Path:
    return Path(__file__).resolve().parent


def collect_pairs(incoming: Path) -> list[tuple[Path, Path]]:
    pairs = []
    if not incoming.is_dir():
        return pairs
    for image in sorted(incoming.rglob("*.jpg")):
        if image.parent.name == "preview":
            continue
        label = image.with_suffix(".txt")
        if label.is_file() and label.stat().st_size > 0:
            pairs.append((image, label))
    return pairs


def split_pairs(pairs, val_ratio: float, seed: int):
    items = list(pairs)
    random.Random(seed).shuffle(items)
    if len(items) < 4:
        raise SystemExit(f"至少 4 张才能划分，现在只有 {len(items)} 张。先采集。")
    val_n = max(1, int(round(len(items) * val_ratio)))
    val_n = min(val_n, len(items) - 1)
    return items[val_n:], items[:val_n]


def materialize(pairs, image_dir: Path, label_dir: Path) -> None:
    if image_dir.exists():
        shutil.rmtree(image_dir)
    if label_dir.exists():
        shutil.rmtree(label_dir)
    image_dir.mkdir(parents=True)
    label_dir.mkdir(parents=True)
    for image, label in pairs:
        # 不同批次都从 0001 起名，拼上批次名才不会互相覆盖。
        stem = f"{image.parent.name}-{image.stem}"
        shutil.copy2(image, image_dir / f"{stem}{image.suffix}")
        shutil.copy2(label, label_dir / f"{stem}.txt")


def main():
    parser = argparse.ArgumentParser(description="训练基础组 6 类 YOLO11n")
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    args = parser.parse_args()

    root = package_dir()
    pairs = collect_pairs(root / "dataset" / "incoming")
    train_pairs, val_pairs = split_pairs(pairs, args.val_ratio, args.seed)
    dataset = root / "dataset"
    materialize(train_pairs, dataset / "images" / "train", dataset / "labels" / "train")
    materialize(val_pairs, dataset / "images" / "val", dataset / "labels" / "val")
    print(f"划分 train={len(train_pairs)} val={len(val_pairs)}")

    weights = root.parent / "track2_yolo" / "weights" / "yolo11n.pt"
    if not weights.is_file():
        raise SystemExit(
            f"没有预训练权重 {weights}。把 yolo11n.pt 放到这个路径，不要改文件名。"
        )
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            "这个 Python 里没有 ultralytics。用赛道二的虚拟环境:\n"
            "  perception/track2_yolo/.venv/bin/python perception/track1_sim/train.py"
        ) from exc

    model = YOLO(str(weights))
    data_yaml = dataset / "track1.local.yaml"
    data_yaml.write_text(
        (dataset / "track1.yaml").read_text().replace("path: .", f"path: {dataset.resolve()}"),
        encoding="utf-8",
    )
    model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        project=str(root / "runs"),
        name="track1",
        exist_ok=True,
        pretrained=True,
        seed=args.seed,
        degrees=15.0,
        flipud=0.0,
        fliplr=0.5,
        patience=10,
        # 关掉 AMP 自检。它会临时下载 yolo26n.pt，学校网络卡住时整次训练停住。
        amp=False,
    )
    best = root / "runs" / "track1" / "weights" / "best.pt"
    print(f"训练结束。权重: {best}")


if __name__ == "__main__":
    main()
