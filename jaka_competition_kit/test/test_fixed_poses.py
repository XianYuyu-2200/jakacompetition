from jaka_competition_kit.fixed_poses import (is_complete, load_fixed_poses,
                                               pose_for, validate_joints)
from jaka_competition_kit.qr import parse


def test_raw_six_digit_qr_order_is_supported():
    assert parse("432165") == [4, 3, 2, 1, 6, 5]


def test_fixed_pose_template_is_safe_until_calibrated():
    data = load_fixed_poses("jaka_competition_kit/config/track1_fixed_poses.yaml")
    assert data["enabled"] is False
    assert not is_complete(data)
    assert pose_for(data, 1, "grasp") is None


def test_joint_validation_requires_six_finite_values():
    assert validate_joints([0, 1, 2, 3, 4, 5]) == [0, 1, 2, 3, 4, 5]
