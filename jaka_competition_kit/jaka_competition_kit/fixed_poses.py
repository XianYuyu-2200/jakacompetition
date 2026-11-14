"""赛道一固定关节姿态配置的加载、校验和原子更新。"""
from __future__ import annotations

import math
import os
from pathlib import Path
import tempfile
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence

import yaml

SCHEMA_VERSION = 4
MAX_WAYPOINTS = 10
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
PHASES = ("approach", "grasp", "lift", "release", "retreat")
OBJECT_PHASES = ("approach", "grasp", "lift")
PLACE_PHASES = ("approach", "release", "retreat")


def default_fixed_pose_path() -> str:
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "config", "track1_fixed_poses.yaml")


def validate_joints(values: Iterable[float], *,
                    joint_limits: Optional[Mapping[str, Sequence[float]]] = None) -> list[float]:
    values = list(values)
    if len(values) != 6:
        raise ValueError(f"固定姿态必须有 6 个关节角，收到 {len(values)} 个")
    out = []
    for i, value in enumerate(values, 1):
        try:
            value = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"joint_{i} 不是有效数值：{value!r}") from exc
        if not math.isfinite(value):
            raise ValueError(f"joint_{i} 必须是有限数值：{value}")
        if joint_limits and f"joint_{i}" in joint_limits:
            lower, upper = map(float, joint_limits[f"joint_{i}"])
            if not (math.isfinite(lower) and math.isfinite(upper) and lower <= upper):
                raise ValueError(f"joint_{i} 的机器人模型限位无效：[{lower}, {upper}]")
            if value < lower or value > upper:
                raise ValueError(f"joint_{i}={value} 超出机器人模型限位 [{lower}, {upper}]")
        elif abs(value) > 2.0 * math.pi + 1e-6:
            # 仅在机器人模型尚不可用时作宽松防错；运动前仍由 MoveIt 再校验。
            raise ValueError(f"joint_{i} 角度超出默认安全范围 [-2π, 2π]：{value}")
        out.append(value)
    return out



def _validate_waypoints(entry: Mapping[str, Any], phase: str, version: int) -> None:
    key = f"{phase}_via_joints"
    if key not in entry:
        return
    if version < 3:
        raise ValueError("旧版本固定姿态不能包含中间路点")
    points = entry[key]
    if not isinstance(points, list) or len(points) > MAX_WAYPOINTS:
        raise ValueError(f"{key} 必须是最多 {MAX_WAYPOINTS} 个关节姿态的列表")
    for point in points:
        if not isinstance(point, list):
            raise ValueError(f"{key} 中每个路点必须是关节角列表")
        validate_joints(point)


def waypoints_for(entry: Mapping[str, Any], phase: str) -> list[list[float]]:
    _validate_waypoints(entry, phase, SCHEMA_VERSION)
    return [validate_joints(p) for p in entry.get(f"{phase}_via_joints", [])]

def _validate_document(raw: Any, *, allow_legacy: bool = True) -> Dict[str, Any]:
    if not isinstance(raw, dict):
        raise ValueError("固定姿态文件必须是 YAML 对象")
    version = raw.get("version", 1)
    if not isinstance(version, int) or isinstance(version, bool):
        raise ValueError("固定姿态 version 必须是整数")
    if version not in (1, 2, 3, SCHEMA_VERSION) or (version == 1 and not allow_legacy):
        raise ValueError(f"不支持固定姿态配置版本：{version}")
    if version == 1:
        # 旧格式只兼容读取，不假定其校准来源；落盘时升级为 v4。
        raw = dict(raw)
        raw.update(version=SCHEMA_VERSION, enabled=False, backend="simulation",
                   device_profile="jaka_mini_2_gemini305_gripper_sim",
                   joint_names=JOINT_NAMES.copy(), angle_unit="rad",
                   calibration={"source": "legacy-v1", "verified": False,
                                 "requires_recalibration": True})
    if version in (2, 3):
        raw = dict(raw)
        raw["version"] = SCHEMA_VERSION
    if raw.get("backend") not in ("simulation", "hardware"):
        raise ValueError("固定姿态 backend 必须是 simulation 或 hardware")
    if not isinstance(raw.get("device_profile"), str) or not raw["device_profile"].strip():
        raise ValueError("固定姿态必须指定非空 device_profile")
    if raw.get("joint_names") != JOINT_NAMES:
        raise ValueError(f"joint_names 必须严格为 {JOINT_NAMES}")
    if raw.get("angle_unit") != "rad":
        raise ValueError("关节角单位必须为 rad")
    if not isinstance(raw.get("enabled", False), bool):
        raise ValueError("enabled 必须是布尔值")
    objects, place = raw.get("objects", {}), raw.get("place", {})
    if not isinstance(objects, dict) or not isinstance(place, dict):
        raise ValueError("固定姿态的 objects/place 必须是对象")
    unknown = set(objects) - {str(i) for i in range(1, 7)}
    if unknown:
        raise ValueError(f"存在未知工位编号：{', '.join(sorted(map(str, unknown)))}")
    for station, entry in objects.items():
        if not isinstance(entry, dict):
            raise ValueError(f"工位 {station} 的姿态必须是对象")
        if "transfer_via_joints" in entry and version < 4:
            raise ValueError("旧版本固定姿态不能包含工位转场路点")
        _validate_waypoints(entry, "transfer", version)
        for phase in OBJECT_PHASES:
            value = entry.get(f"{phase}_joints", entry.get(phase))
            if value is not None:
                validate_joints(value)
            _validate_waypoints(entry, phase, version)
    for phase in PLACE_PHASES:
        value = place.get(f"{phase}_joints", place.get(phase))
        if value is not None:
            validate_joints(value)
        _validate_waypoints(place, phase, version)
    calibration = raw.get("calibration", {})
    if not isinstance(calibration, dict):
        raise ValueError("calibration 必须是对象")
    result = dict(raw)
    result["objects"], result["place"] = objects, place
    result.setdefault("enabled", False)
    result.setdefault("calibration", {"source": "unknown", "verified": False})
    return result


def load_fixed_poses(path: Optional[str] = None) -> Dict[str, Any]:
    path = path or default_fixed_pose_path()
    try:
        with open(path, "r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
    except FileNotFoundError:
        return new_fixed_pose_document()
    except yaml.YAMLError as exc:
        raise ValueError(f"固定姿态 YAML 格式错误：{exc}") from exc
    if raw is None:
        return new_fixed_pose_document()
    return _validate_document(raw)


def new_fixed_pose_document(backend: str = "simulation",
                           device_profile: str = "jaka_mini_2_gemini305_gripper_sim") -> Dict[str, Any]:
    if backend not in ("simulation", "hardware"):
        raise ValueError("backend 必须是 simulation 或 hardware")
    return {"version": SCHEMA_VERSION, "enabled": False, "backend": backend,
            "device_profile": device_profile, "joint_names": JOINT_NAMES.copy(),
            "angle_unit": "rad", "calibration": {"source": "workbench", "verified": False},
            "objects": {}, "place": {}}


def pose_for(data: Dict[str, Any], station: int, phase: str,
             *, joint_limits=None) -> Optional[list[float]]:
    if phase not in OBJECT_PHASES:
        raise ValueError(f"工位姿态阶段无效：{phase}")
    entry = data.get("objects", {}).get(str(int(station)), {})
    if not isinstance(entry, dict):
        return None
    value = entry.get(f"{phase}_joints", entry.get(phase))
    return None if value is None else validate_joints(value, joint_limits=joint_limits)


def place_pose_for(data: Dict[str, Any], phase: str,
                   *, joint_limits=None) -> Optional[list[float]]:
    if phase not in PLACE_PHASES:
        raise ValueError(f"放置姿态阶段无效：{phase}")
    value = data.get("place", {}).get(f"{phase}_joints", data.get("place", {}).get(phase))
    return None if value is None else validate_joints(value, joint_limits=joint_limits)


def profile_matches(data: Dict[str, Any], *, backend: str, device_profile: str) -> bool:
    """Require the caller to opt into the exact calibrated device profile."""
    return data.get("backend") == backend and data.get("device_profile") == device_profile


def is_complete(data: Dict[str, Any], stations=range(1, 7), *,
                backend: Optional[str] = None,
                device_profile: Optional[str] = None) -> bool:
    if not bool(data.get("enabled", False)):
        return False
    if backend is not None and data.get("backend") != backend:
        return False
    if device_profile is not None and data.get("device_profile") != device_profile:
        return False
    for station in stations:
        if any(pose_for(data, station, phase) is None for phase in OBJECT_PHASES):
            return False
    return all(place_pose_for(data, phase) is not None for phase in PLACE_PHASES)


def update_pose(path: str, *, station: Optional[int], phase: str,
                joints: Iterable[float], gripper: Optional[float] = None,
                backend: str = "simulation", device_profile: str = "jaka_mini_2_gemini305_gripper_sim") -> Dict[str, Any]:
    if phase not in PHASES:
        raise ValueError(f"未知姿态阶段：{phase}")
    if station is not None and int(station) not in range(1, 7):
        raise ValueError(f"工位编号必须为 1..6：{station}")
    values = validate_joints(joints)
    data = load_fixed_poses(path) if Path(path).exists() else new_fixed_pose_document(backend, device_profile)
    if data.get("calibration", {}).get("requires_recalibration"):
        # Do not silently bless poses whose original robot/profile provenance is
        # unknown. The first save starts a clean calibration set.
        data["objects"], data["place"] = {}, {}
        data["calibration"] = {"source": "workbench", "verified": False}
    if data["backend"] != backend or data["device_profile"] != device_profile:
        raise ValueError("不能将不同 backend/device_profile 的姿态混写；请使用独立配置文件")
    target = data["objects"].setdefault(str(int(station)), {}) if station is not None else data["place"]
    target[f"{phase}_joints"] = values
    if gripper is not None:
        gripper = float(gripper)
        if not math.isfinite(gripper):
            raise ValueError("夹爪参数必须是有限数值")
        target[f"{phase}_gripper"] = gripper
    data["enabled"] = is_complete({**data, "enabled": True})
    data.setdefault("calibration", {})["verified"] = False
    return _save_document(path, data)


def _save_document(path: str, data: Dict[str, Any]) -> Dict[str, Any]:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{destination.name}.", suffix=".tmp",
                                    dir=str(destination.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp_name, destination)
    except Exception:
        try:
            os.unlink(tmp_name)
        except OSError:
            pass
        raise
    return data


def update_waypoints(path: str, *, station: Optional[int], phase: str,
                     joints: Optional[Iterable[float]] = None, clear: bool = False,
                     backend: str = "simulation",
                     device_profile: str = "jaka_mini_2_gemini305_gripper_sim") -> Dict[str, Any]:
    """Append a point executed before a phase, or clear that phase's points."""
    allowed = (*OBJECT_PHASES, "transfer") if station is not None else PLACE_PHASES
    if phase not in allowed or (station is not None and int(station) not in range(1, 7)):
        raise ValueError("无效的工位或姿态阶段")
    if clear == (joints is not None):
        raise ValueError("请选择追加路点或清空路点")
    data = load_fixed_poses(path)
    if data["backend"] != backend or data["device_profile"] != device_profile:
        raise ValueError("不能将不同 backend/device_profile 的姿态混写")
    if data.get("calibration", {}).get("requires_recalibration"):
        raise ValueError("旧配置须先重新录入基本姿态")
    target = data["objects"].setdefault(str(int(station)), {}) if station is not None else data["place"]
    key = f"{phase}_via_joints"
    points = waypoints_for(target, phase)
    if clear:
        target.pop(key, None)
    else:
        if len(points) >= MAX_WAYPOINTS:
            raise ValueError(f"每个阶段最多 {MAX_WAYPOINTS} 个中间路点")
        points.append(validate_joints(joints))
        target[key] = points
    data.setdefault("calibration", {})["verified"] = False
    # Changing a route must never implicitly enable a previously disabled configuration.
    return _save_document(path, data)


def import_simulation_poses(source: str, destination: str, *, enable: bool = False):
    """Copy a complete simulation set with explicit opt-in and a unique backup.

    This is configuration provenance checking, not a hardware safety interlock.
    Neither importing nor enabling grants a physical-grasp validation status.
    """
    import shutil
    if not isinstance(enable, bool):
        raise ValueError("enable 必须是布尔值")
    src, dst = Path(source), Path(destination)
    if src.resolve() == dst.resolve():
        raise ValueError("不能将姿态文件导入自身")
    if not src.is_file():
        raise ValueError("仿真姿态来源文件不存在")
    data = load_fixed_poses(str(src))
    if not is_complete({**data, "enabled": True}, backend="simulation",
                       device_profile="jaka_mini_2_gemini305_gripper_sim"):
        raise ValueError("仅可导入完整且 profile 匹配的仿真姿态；不能导入真机姿态")
    if data.get("calibration", {}).get("requires_recalibration"):
        raise ValueError("来源姿态要求重新标定，不能直接导入启用")
    data["enabled"] = enable
    data.setdefault("calibration", {})["verified"] = False
    backup = None
    if dst.exists():
        fd, name = tempfile.mkstemp(prefix=f"{dst.name}.", suffix=".bak", dir=dst.parent)
        os.close(fd)
        backup = Path(name)
        try:
            shutil.copyfile(dst, backup)
        except Exception:
            backup.unlink(missing_ok=True)
            raise
    _save_document(str(dst), data)
    return data, backup
