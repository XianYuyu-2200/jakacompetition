from pathlib import Path

import pytest
import yaml

from jaka_competition_kit.fixed_poses import (
    is_complete, load_fixed_poses, new_fixed_pose_document, pose_for,
    update_pose, validate_joints,
)
from jaka_competition_kit.qr import parse


def test_raw_six_digit_qr_order_is_supported():
    assert parse("432165") == [4, 3, 2, 1, 6, 5]


def test_fixed_pose_template_is_safe_until_calibrated():
    data = load_fixed_poses("jaka_competition_kit/config/track1_fixed_poses.yaml")
    assert data["enabled"] is False
    assert data["backend"] == "simulation"
    assert data["device_profile"] == "jaka_mini_2_gemini305_gripper_sim"
    assert not is_complete(data)
    assert pose_for(data, 1, "grasp") is None


def test_joint_validation_requires_six_finite_values():
    assert validate_joints([0, 1, 2, 3, 4, 5]) == [0, 1, 2, 3, 4, 5]
    with pytest.raises(ValueError, match="6 个"):
        validate_joints([0] * 5)
    with pytest.raises(ValueError, match="有限"):
        validate_joints([0, 1, float("nan"), 3, 4, 5])


def test_joint_validation_uses_robot_model_limits_when_provided():
    limits = {"joint_1": (-1.0, 1.0)}
    assert validate_joints([1, 0, 0, 0, 0, 0], joint_limits=limits)[0] == 1
    with pytest.raises(ValueError, match="机器人模型限位"):
        validate_joints([1.01, 0, 0, 0, 0, 0], joint_limits=limits)


def test_rejects_unknown_schema_and_station(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text("version: 9\n", encoding="utf-8")
    with pytest.raises(ValueError, match="版本"):
        load_fixed_poses(str(path))
    with pytest.raises(ValueError, match="工位"):
        update_pose(str(tmp_path / "poses.yaml"), station=7, phase="grasp", joints=[0] * 6)


def test_atomic_updates_create_profiled_v2_and_remain_disabled_until_complete(tmp_path):
    path = tmp_path / "poses.yaml"
    data = update_pose(str(path), station=1, phase="grasp", joints=[0.1] * 6)
    assert data["version"] == 3
    assert data["backend"] == "simulation"
    assert data["joint_names"] == [f"joint_{i}" for i in range(1, 7)]
    assert data["angle_unit"] == "rad"
    assert not data["enabled"]
    assert not list(tmp_path.glob("*.tmp"))
    loaded = load_fixed_poses(str(path))
    assert pose_for(loaded, 1, "grasp") == [0.1] * 6


def test_simulation_and_hardware_profiles_cannot_be_mixed(tmp_path):
    path = tmp_path / "poses.yaml"
    update_pose(str(path), station=1, phase="grasp", joints=[0] * 6)
    with pytest.raises(ValueError, match="不能将不同"):
        update_pose(str(path), station=1, phase="approach", joints=[0] * 6,
                    backend="hardware", device_profile="jaka_mini2_real")


def test_incomplete_and_complete_pose_sets(tmp_path):
    path = tmp_path / "poses.yaml"
    data = new_fixed_pose_document()
    for station in range(1, 7):
        for phase in ("approach", "grasp", "lift"):
            data = update_pose(str(path), station=station, phase=phase, joints=[0] * 6)
    for phase in ("approach", "release", "retreat"):
        data = update_pose(str(path), station=None, phase=phase, joints=[0] * 6)
    assert is_complete(data)


def test_invalid_stage_and_malformed_yaml_fail_clearly(tmp_path):
    with pytest.raises(ValueError, match="阶段"):
        update_pose(str(tmp_path / "poses.yaml"), station=1, phase="unknown", joints=[0] * 6)
    path = tmp_path / "malformed.yaml"
    path.write_text("version: [unterminated", encoding="utf-8")
    with pytest.raises(ValueError, match="YAML"):
        load_fixed_poses(str(path))


def test_legacy_profile_is_disabled_and_cleared_before_recalibration(tmp_path):
    path = tmp_path / "legacy.yaml"
    path.write_text(yaml.safe_dump({
        "version": 1, "enabled": True,
        "objects": {"1": {"approach_joints": [0.3] * 6,
                            "grasp_joints": [0.4] * 6,
                            "lift_joints": [0.5] * 6}},
        "place": {"approach_joints": [0] * 6,
                 "release_joints": [0] * 6,
                 "retreat_joints": [0] * 6},
    }), encoding="utf-8")
    migrated = load_fixed_poses(str(path))
    assert not migrated["enabled"]
    assert not is_complete({**migrated, "enabled": True})
    saved = update_pose(str(path), station=1, phase="approach", joints=[0.1] * 6)
    assert saved["objects"] == {"1": {"approach_joints": [0.1] * 6}}
    assert saved["calibration"]["source"] == "workbench"


def test_waypoints_migrate_v2_are_ordered_and_disabled_stays_disabled(tmp_path):
    from jaka_competition_kit.fixed_poses import update_waypoints, waypoints_for
    path = tmp_path / 'poses.yaml'
    path.write_text(Path('jaka_competition_kit/config/track1_fixed_poses.yaml').read_text())
    data = update_waypoints(str(path), station=6, phase='approach', joints=[.1] * 6)
    assert data['version'] == 3 and not data['enabled']
    data = update_waypoints(str(path), station=6, phase='approach', joints=[.2] * 6)
    assert waypoints_for(load_fixed_poses(str(path))['objects']['6'], 'approach') == [[.1] * 6, [.2] * 6]
    data = update_waypoints(str(path), station=6, phase='approach', clear=True)
    assert waypoints_for(data['objects']['6'], 'approach') == []
    assert not data['calibration']['verified']


def test_bad_waypoints_and_v2_waypoint_rejected(tmp_path):
    from jaka_competition_kit.fixed_poses import update_waypoints
    path = tmp_path / 'poses.yaml'
    data = new_fixed_pose_document()
    data['objects'] = {'6': {'approach_via_joints': [[0] * 5]}}
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='6 个'):
        load_fixed_poses(str(path))
    data['version'] = 2
    data['objects']['6']['approach_via_joints'] = [[0] * 6]
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='旧版本'):
        load_fixed_poses(str(path))
    with pytest.raises(ValueError):
        update_waypoints(str(tmp_path / 'new.yaml'), station=6, phase='release', joints=[0] * 6)
