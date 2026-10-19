#!/usr/bin/env python3
"""把 incoming/<批次>/ 划成 Ultralytics 的 train / val。

按批次整批划走, 不把同一批连拍随机拆开。连拍几乎是同一帧,
拆开以后 val 会虚高, 上真机就掉。

默认最新的大约 20% 批次做 val, 其余做 train。至少留 1 批训练、1 批验证。
只有一批时拒绝划分; 确实要拆帧再加 --allow-frame-split。
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from classes import load_catalog, package_dir, write_data_yaml


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def assign_sessions(names, val_ratio, explicit):
    """Return (train_names, val_names). names must already be sorted, oldest first."""
    if not names:
        raise SystemExit("incoming/ 里没有任何批次。先用 capture_label.py 拍一批。")
    if explicit:
        missing = [name for name in explicit if name not in names]
        if missing:
            raise SystemExit(f"找不到这些批次: {', '.join(missing)}")
        val = list(explicit)
        train = [name for name in names if name not in set(val)]
        if not train:
            raise SystemExit("不能把每一批都放进 val。")
        if not val:
            raise SystemExit("没有指定 val 批次。")
        return train, val
    if len(names) < 2:
        return list(names), []
    holdout = int(round(len(names) * val_ratio))
    holdout = max(1, min(holdout, len(names) - 1))
    return list(names[:-holdout]), list(names[-holdout:])


def assign_frames(stems, val_ratio):
    if len(stems) < 2:
        raise SystemExit("这一批不足 2 张, 没法拆成 train / val。")
    holdout = int(round(len(stems) * val_ratio))
    holdout = max(1, min(holdout, len(stems) - 1))
    return list(stems[:-holdout]), list(stems[-holdout:])


def session_dirs(incoming: Path):
    if not incoming.is_dir():
        raise SystemExit(f"没有采集目录: {incoming}")
    dirs = [path for path in incoming.iterdir() if path.is_dir()]
    return sorted(dirs, key=lambda path: path.name)


def pairs_in(session: Path):
    images = {}
    labels = {}
    for path in session.iterdir():
        if path.suffix.lower() in IMAGE_EXTS:
            images[path.stem] = path
        elif path.suffix == ".txt":
            labels[path.stem] = path
    stems = sorted(set(images) & set(labels))
    missing = sorted(set(images) - set(labels))
    orphan = sorted(set(labels) - set(images))
    return stems, images, labels, missing, orphan


def parse_label(path: Path, n_class: int):
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        raise ValueError(f"{path} 是空标注")
    lines = []
    counts = [0] * n_class
    for number, raw in enumerate(text.splitlines(), 1):
        parts = raw.split()
        if len(parts) != 5:
            raise ValueError(f"{path}:{number} 不是 class cx cy w h")
        try:
            class_index = int(parts[0])
            coords = [float(value) for value in parts[1:]]
        except ValueError as exc:
            raise ValueError(f"{path}:{number} 有非数字") from exc
        if not 0 <= class_index < n_class:
            raise ValueError(f"{path}:{number} class id {class_index} 超出 0..{n_class - 1}")
        if any(value < 0.0 or value > 1.0 for value in coords):
            raise ValueError(f"{path}:{number} 坐标不在 0..1")
        counts[class_index] += 1
        lines.append(" ".join([str(class_index), *(f"{value:.6f}" for value in coords)]))
    return lines, counts


def reset_split(dataset: Path):
    for kind in ("images", "labels"):
        for split in ("train", "val"):
            path = dataset / kind / split
            if path.exists():
                shutil.rmtree(path)
            path.mkdir(parents=True, exist_ok=True)


def copy_pair(session, stem, image, label_lines, split, dataset):
    ext = image.suffix.lower()
    name = f"{session}__{stem}{ext}"
    shutil.copy2(image, dataset / "images" / split / name)
    (dataset / "labels" / split / f"{session}__{stem}.txt").write_text(
        "\n".join(label_lines) + "\n",
        encoding="utf-8",
    )


def main():
    parser = argparse.ArgumentParser(description="按采集批次划分 train / val")
    parser.add_argument("--incoming", type=Path, default=package_dir() / "dataset" / "incoming")
    parser.add_argument("--dataset", type=Path, default=package_dir() / "dataset")
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument(
        "--val-sessions",
        default="",
        help="逗号分隔的批次名, 指定后不再按比例取最新批次",
    )
    parser.add_argument(
        "--allow-frame-split",
        action="store_true",
        help="只有一批时, 按文件名把末尾一部分帧划进 val。连拍会泄漏, 正式训练别用。",
    )
    args = parser.parse_args()
    if not 0.0 < args.val_ratio < 1.0:
        raise SystemExit("--val-ratio 要在 0 和 1 之间")

    catalog = load_catalog()
    sessions = session_dirs(args.incoming)
    names = [path.name for path in sessions]
    explicit = [part for part in args.val_sessions.split(",") if part]
    train_names, val_names = assign_sessions(names, args.val_ratio, explicit)
    frame_split = False
    if not val_names:
        if not args.allow_frame_split:
            raise SystemExit(
                "现在只有一批。再换一次摆放或光照, 另开一批拍 val。\n"
                "同一批连拍拆开会让验证集虚高。调试可以加 --allow-frame-split。"
            )
        frame_split = True
        print("警告: 只有一批, 按帧划 val。这批数字不能当成比赛成绩。")

    by_name = {path.name: path for path in sessions}
    plan = []
    if frame_split:
        session = by_name[train_names[0]]
        stems, images, labels, missing, orphan = pairs_in(session)
        if missing or orphan:
            print(f"跳过缺对的文件 missing={missing} orphan={orphan}")
        train_stems, val_stems = assign_frames(stems, args.val_ratio)
        plan.append((session, train_stems, images, labels, "train"))
        plan.append((session, val_stems, images, labels, "val"))
    else:
        print(f"train 批次: {', '.join(train_names)}")
        print(f"val   批次: {', '.join(val_names)}")
        for split, chosen in (("train", train_names), ("val", val_names)):
            for name in chosen:
                session = by_name[name]
                stems, images, labels, missing, orphan = pairs_in(session)
                if missing or orphan:
                    print(f"{name}: 跳过缺对的文件 missing={missing} orphan={orphan}")
                plan.append((session, stems, images, labels, split))

    reset_split(args.dataset)
    counts = {"train": [0] * len(catalog), "val": [0] * len(catalog)}
    images_n = {"train": 0, "val": 0}
    for session, stems, images, labels, split in plan:
        for stem in stems:
            try:
                lines, hit = parse_label(labels[stem], len(catalog))
            except ValueError as exc:
                raise SystemExit(str(exc)) from exc
            copy_pair(session.name, stem, images[stem], lines, split, args.dataset)
            images_n[split] += 1
            for index, value in enumerate(hit):
                counts[split][index] += value

    yaml_path = write_data_yaml(args.dataset / "track2.yaml")
    print(f"images train={images_n['train']} val={images_n['val']}")
    print(f"{'id':>2}  {'sku':<12} train  val")
    empty = []
    for sku in catalog:
        train_n = counts["train"][sku.index]
        val_n = counts["val"][sku.index]
        print(f"{sku.index:2d}  {sku.id:<12} {train_n:5d} {val_n:5d}")
        if train_n == 0:
            empty.append(sku.id)
    if empty:
        print("这些类在 train 里一张都没有, 先别训练: " + ", ".join(empty))
    print(f"wrote {yaml_path}")


def _self_test():
    train, val = assign_sessions([f"b{i}" for i in range(5)], 0.2, [])
    assert train == ["b0", "b1", "b2", "b3"] and val == ["b4"], (train, val)
    train, val = assign_sessions(["a", "b"], 0.2, [])
    assert train == ["a"] and val == ["b"], (train, val)
    train, val = assign_sessions(["a", "b", "c"], 0.2, ["b"])
    assert train == ["a", "c"] and val == ["b"], (train, val)
    left, right = assign_frames([f"{i:04d}" for i in range(10)], 0.2)
    assert left == [f"{i:04d}" for i in range(8)] and right == ["0008", "0009"], (left, right)
    print("split ok")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--self-test":
        _self_test()
    else:
        main()
