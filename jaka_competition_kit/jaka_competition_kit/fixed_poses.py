"""赛道一固定关节姿态配置。

配置是仿真和真机都能使用的纯 YAML 数据：仿真调出的姿态可以直接复制到真机
配置，再按真机零位、夹爪安装和安全距离重新校准。程序永远不会把二维码真值
``qr_order`` 当成输入；固定姿态只负责把编号映射到已经记录好的动作。
"""
from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

import yaml

JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
PHASES = ("approach", "grasp", "lift", "release", "retreat")


def default_fixed_pose_path() -> str:
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "config", "track1_fixed_poses.yaml")


def validate_joints(values: Iterable[float]) -> list[float]:
    values = list(values)
    if len(values) != 6:
        raise ValueError(f"固定姿态必须有 6 个关节角，收到 {len(values)} 个")
    out = []
    for i, value in enumerate(values, 1):
        value = float(value)
        if not math.isfinite(value) or abs(value) > 2.0 * math.pi + 1e-6:
            raise ValueError(f"joint_{i} 角度超出 [-2π, 2π]：{value}")
        out.append(value)
    return out


def load_fixed_poses(path: Optional[str] = None) -> Dict[str, Any]:
    path = path or default_fixed_pose_path()
    try:
        with open(path, "r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream) or {}
    except FileNotFoundError:
        return {"version": 1, "enabled": False, "objects": {}, "place": {}}
    if not isinstance(raw, dict):
        raise ValueError(f"固定姿态文件必须是 YAML 对象：{path}")
    raw.setdefault("version", 1)
    raw.setdefault("enabled", False)
    raw.setdefault("objects", {})
    raw.setdefault("place", {})
    if not isinstance(raw["objects"], dict) or not isinstance(raw["place"], dict):
        raise ValueError("固定姿态的 objects/place 必须是对象")
    return raw


def pose_for(data: Dict[str, Any], station: int, phase: str) -> Optional[list[float]]:
    entry = data.get("objects", {}).get(str(int(station)), {})
    if not isinstance(entry, dict):
        return None
    value = entry.get(f"{phase}_joints")
    if value is None:
        value = entry.get(phase)
    if value is None:
        return None
    return validate_joints(value)


def place_pose_for(data: Dict[str, Any], phase: str) -> Optional[list[float]]:
    value = data.get("place", {}).get(f"{phase}_joints")
    if value is None:
        value = data.get("place", {}).get(phase)
    return None if value is None else validate_joints(value)


def is_complete(data: Dict[str, Any], stations=range(1, 7)) -> bool:
    if not bool(data.get("enabled", False)):
        return False
    for station in stations:
        if any(pose_for(data, station, phase) is None for phase in ("approach", "grasp", "lift")):
            return False
    return all(place_pose_for(data, phase) is not None for phase in ("approach", "release", "retreat"))


def update_pose(path: str, *, station: Optional[int], phase: str,
                joints: Iterable[float], gripper: Optional[float] = None) -> Dict[str, Any]:
    if phase not in PHASES:
        raise ValueError(f"未知姿态阶段：{phase}")
    data = load_fixed_poses(path)
    values = validate_joints(joints)
    target = data.setdefault("objects", {}).setdefault(str(int(station)), {}) if station else data.setdefault("place", {})
    target[f"{phase}_joints"] = values
    if gripper is not None:
        target[f"{phase}_gripper"] = float(gripper)
    # 只有 6 个工位和放置动作均记录后才自动切换为固定姿态模式。
    data["enabled"] = is_complete(data)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    tmp = Path(path).with_suffix(Path(path).suffix + ".tmp")
    tmp.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")
    tmp.replace(path)
    return data
