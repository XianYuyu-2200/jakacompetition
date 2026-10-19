#!/usr/bin/env python3
"""赛道二的 20 类目录。YOLO 的 class id 就是这里的 index。

顺序冻结为 arena.yaml 里的类别顺序, 再按该类内部的工件顺序。
不要按字母排序, 也不要按中文排序。训练出来的权重和这个顺序绑死,
arena.yaml 若被改得对不上, 本模块会直接退出。
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import yaml


# 和 jaka_competition_kit/config/arena.yaml 的 track2.categories 对齐。
# 这是已经发出去的 class id 合同, 不是一份可以随手重排的清单。
EXPECTED_IDS = (
    "bottle",
    "tissue",
    "soap",
    "sponge",
    "mouse",
    "driver",
    "pliers_nose",
    "pliers_cut",
    "tape",
    "brush",
    "brick",
    "duck",
    "car",
    "plush",
    "dino",
    "apple",
    "orange",
    "lemon",
    "pear",
    "kiwi",
)


@dataclass(frozen=True)
class Sku:
    index: int
    id: str
    name_zh: str
    bin_id: str
    bin_name_zh: str
    shape: str
    dims_m: tuple[float, float, float]


def package_dir() -> Path:
    return Path(__file__).resolve().parent


def find_repo(start: Path | None = None) -> Path:
    here = start or package_dir()
    for candidate in (here, *here.parents):
        arena = candidate / "jaka_competition_kit" / "config" / "arena.yaml"
        if arena.is_file():
            return candidate
    raise FileNotFoundError("找不到 jaka_competition_kit/config/arena.yaml")


def default_arena() -> Path:
    return find_repo() / "jaka_competition_kit" / "config" / "arena.yaml"


@lru_cache(maxsize=4)
def load_catalog(arena: str | None = None) -> tuple[Sku, ...]:
    path = Path(arena) if arena else default_arena()
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    track2 = data.get("track2") if isinstance(data, dict) else None
    if not isinstance(track2, dict):
        raise SystemExit(f"{path} 里没有 track2")
    bins = track2.get("bins") or []
    bin_name = {}
    for item in bins:
        bin_name[str(item["id"])] = str(item.get("name") or item["id"])
    catalog = []
    index = 0
    for category in track2.get("categories") or []:
        bin_id = str(category["id"])
        if bin_id not in bin_name:
            raise SystemExit(f"类别 {bin_id} 没有对应的料盒")
        for obj in category.get("objects") or []:
            dims = tuple(float(v) for v in obj["dims"])
            if len(dims) != 3:
                raise SystemExit(f"{obj.get('id')} 的 dims 不是 3 个数")
            catalog.append(
                Sku(
                    index=index,
                    id=str(obj["id"]),
                    name_zh=str(obj["name"]),
                    bin_id=bin_id,
                    bin_name_zh=bin_name[bin_id],
                    shape=str(obj.get("shape") or ""),
                    dims_m=dims,  # type: ignore[arg-type]
                )
            )
            index += 1
    ids = tuple(item.id for item in catalog)
    if ids != EXPECTED_IDS:
        raise SystemExit(
            "arena.yaml 的工件顺序和已冻结的 YOLO class id 不一致。\n"
            f"  yaml:     {ids}\n"
            f"  expected: {EXPECTED_IDS}\n"
            "不要为了消掉这行报错去改 EXPECTED_IDS。已有标注和权重都按旧编号。"
        )
    if len(catalog) != 20:
        raise SystemExit(f"应该是 20 类, 实际 {len(catalog)}")
    return tuple(catalog)


def by_id(catalog: tuple[Sku, ...] | None = None) -> dict[str, Sku]:
    items = catalog if catalog is not None else load_catalog()
    return {item.id: item for item in items}


def by_index(catalog: tuple[Sku, ...] | None = None) -> dict[int, Sku]:
    items = catalog if catalog is not None else load_catalog()
    return {item.index: item for item in items}


def render_data_yaml(dataset_root: Path | None = None) -> str:
    """Ultralytics 数据集 yaml。path 用绝对路径, 避免受启动目录影响。"""
    root = (dataset_root or (package_dir() / "dataset")).resolve()
    lines = [
        "# 赛道二 20 类检测。names 的列表顺序就是 YOLO class id, 不要手改。",
        "# 由 classes.py 从 arena.yaml 生成。",
        f"path: {root.as_posix()}",
        "train: images/train",
        "val: images/val",
        "names:",
    ]
    for sku in load_catalog():
        lines.append(f"  - {sku.id}")
    lines.append("")
    return "\n".join(lines)


def write_data_yaml(path: Path | None = None) -> Path:
    destination = path or (package_dir() / "dataset" / "track2.yaml")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(render_data_yaml(destination.parent), encoding="utf-8")
    return destination


def main():
    parser = argparse.ArgumentParser(description="打印赛道二 20 类并写出 dataset/track2.yaml")
    parser.add_argument("--arena", default=None, help="arena.yaml 路径, 默认用仓库里的那份")
    args = parser.parse_args()
    if args.arena:
        load_catalog.cache_clear()
        catalog = load_catalog(args.arena)
    else:
        catalog = load_catalog()
    print(f"{'id':>2}  {'sku':<12} {'中文':<10} {'bin':<6} 料盒")
    for sku in catalog:
        print(
            f"{sku.index:2d}  {sku.id:<12} {sku.name_zh:<10} {sku.bin_id:<6} {sku.bin_name_zh}"
        )
    written = write_data_yaml()
    print(f"wrote {written}")


if __name__ == "__main__":
    main()
