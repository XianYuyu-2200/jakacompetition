#!/usr/bin/env python3
"""确认这台机器的 CUDA 能跑 YOLO11，并且 20 类编号没漂。

不训练。没有采集图片时也应该能跑过。
预训练权重 yolo11n.pt 在 GitHub 上。下不到时仍用本地的网络结构做一次前向，
用来确认显卡；那种随机初始化不能拿去认工件。
"""
from __future__ import annotations

import shutil
import tempfile
from pathlib import Path

import numpy as np

from classes import load_catalog, package_dir


def local_yolo11n_yaml() -> Path:
    import ultralytics

    source = Path(ultralytics.__file__).resolve().parent / "cfg" / "models" / "11" / "yolo11.yaml"
    if not source.is_file():
        raise FileNotFoundError(f"安装里没有 {source}")
    # 文件名里的 n 决定宽度。yolo11.yaml 本身不带尺度。
    destination = Path(tempfile.mkdtemp(prefix="yolo11n-")) / "yolo11n.yaml"
    shutil.copyfile(source, destination)
    return destination


def main():
    catalog = load_catalog()
    assert [item.id for item in catalog][:5] == [
        "bottle",
        "tissue",
        "soap",
        "sponge",
        "mouse",
    ]
    assert catalog[-1].id == "kiwi" and catalog[-1].bin_id == "fruit"
    print(f"catalog {len(catalog)} classes, last={catalog[-1].id}->{catalog[-1].bin_id}")

    import torch

    print(f"torch {torch.__version__} cuda={torch.cuda.is_available()}")
    if not torch.cuda.is_available():
        raise SystemExit("CUDA 不可用。RTX 5060 Ti 需要带 CUDA 13 的 torch，不要装 CPU 轮子。")
    name = torch.cuda.get_device_name(0)
    major, minor = torch.cuda.get_device_capability(0)
    print(f"gpu {name} sm_{major}{minor}")
    if hasattr(torch.cuda, "get_arch_list"):
        arches = list(torch.cuda.get_arch_list())
        print("arch list:", ", ".join(arches))
        needed = {f"sm_{major}{minor}", f"sm_{major}.{minor}"}
        if not needed.intersection(arches):
            raise SystemExit(
                f"这个 torch 没有 sm_{major}{minor} 的核。当前架构表: {', '.join(arches)}"
            )
    probe = torch.zeros(1, device="cuda") + 1
    torch.cuda.synchronize()
    del probe

    from ultralytics import YOLO

    weights = package_dir() / "weights" / "yolo11n.pt"
    # 不在这里等 GitHub。下权重是训练前的事，超时会拖好几分钟。
    if weights.is_file() and weights.stat().st_size > 100_000:
        model = YOLO(str(weights))
        source = f"weights {weights}"
    else:
        print("yolo11n.pt 不在 weights/，改用本地网络结构做前向。这不是训练好的检测器。")
        model = YOLO(str(local_yolo11n_yaml()))
        source = "local yolo11n.yaml random init"
    blank = np.zeros((640, 640, 3), dtype=np.uint8)
    results = model.predict(blank, imgsz=640, device=0, verbose=False)
    boxes = 0 if results[0].boxes is None else len(results[0].boxes)
    torch.cuda.synchronize()
    print(f"yolo11n forward ok, boxes_on_blank={boxes}, {source}")


if __name__ == "__main__":
    main()
