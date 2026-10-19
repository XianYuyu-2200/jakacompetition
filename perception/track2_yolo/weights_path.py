"""把 yolo11n.pt 下载到本目录的 weights/, 避免掉进仓库根目录。"""
from __future__ import annotations

from pathlib import Path

from classes import package_dir


def ensure_pretrained(spec: str) -> Path:
    requested = Path(spec)
    if requested.is_file():
        return requested.resolve()
    destination = package_dir() / "weights"
    destination.mkdir(parents=True, exist_ok=True)
    target = destination / requested.name
    if target.is_file():
        return target
    from ultralytics.utils.downloads import attempt_download_asset

    url = (
        "https://github.com/ultralytics/assets/releases/download/v8.4.0/"
        + requested.name
    )
    try:
        downloaded = Path(attempt_download_asset(str(target)))
    except Exception as exc:
        raise FileNotFoundError(
            f"没有下载到 {requested.name}。网络到 GitHub 失败时，"
            f"用浏览器把文件放到 {target} 再重试。\n{url}\n{exc}"
        ) from exc
    if target.is_file():
        return target
    if downloaded.is_file():
        return downloaded.resolve()
    raise FileNotFoundError(f"没有找到也没有下载到 {spec}。手动放到 {target}。\n{url}")
