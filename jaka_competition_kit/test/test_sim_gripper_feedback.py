"""Controller acknowledgement is required, but is not a contact-grasp test."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from jaka_competition_kit import gripper


@pytest.fixture
def sim(monkeypatch):
    clock = [0.0]
    node = SimpleNamespace(create_publisher=Mock(return_value=Mock()),
                           create_subscription=Mock(), get_logger=Mock(return_value=Mock()))
    arm = gripper.SimGripper(node, arena=SimpleNamespace(has_gripper=True))
    monkeypatch.setattr(gripper.time, 'monotonic', lambda: clock[0])
    return arm, clock


def sample(arm, left=-.2, right=.2):
    arm._on_joints(SimpleNamespace(name=['gripper_jaw_l_joint', 'gripper_jaw_r_joint'],
                                  position=[left, right]))


def test_command_requires_two_post_settle_samples(sim, monkeypatch):
    arm, clock = sim
    times = []
    def spin(*args, **kw):
        clock[0] += .02
        sample(arm)
        times.append(clock[0])
    monkeypatch.setattr(gripper.rclpy, 'spin_once', spin)
    assert arm._send(.2)
    assert clock[0] >= arm.close_delay + .02
    assert list(arm.pub.publish.call_args.args[0].data) == [-.2, .2]


@pytest.mark.parametrize('feedback', ['missing', 'wrong', 'one', 'cached'])
def test_missing_stalled_or_single_feedback_fails(sim, monkeypatch, feedback):
    arm, clock = sim
    if feedback == 'cached': sample(arm)
    sent = [False]
    def spin(*args, **kw):
        clock[0] += .02
        if feedback == 'wrong': sample(arm, 0., 0.)
        if feedback == 'one' and clock[0] >= .36 and not sent[0]:
            sample(arm)
            sent[0] = True
    monkeypatch.setattr(gripper.rclpy, 'spin_once', spin)
    assert not arm._send(.2)
    arm.node.get_logger().error.assert_called_once()


@pytest.mark.parametrize('names,positions', [
    (['gripper_jaw_l_joint'], [0.]),
    (['gripper_jaw_l_joint', 'gripper_jaw_r_joint'], [0.]),
    (['gripper_jaw_l_joint', 'gripper_jaw_r_joint'], [float('nan'), 0.]),
    (['gripper_jaw_l_joint', 'gripper_jaw_r_joint'], [0., float('inf')]),
    (['gripper_jaw_l_joint', 'gripper_jaw_l_joint', 'gripper_jaw_r_joint'], [0., 0., 0.]),
])
def test_invalid_joint_feedback_ignored(sim, names, positions):
    arm, _ = sim
    arm._on_joints(SimpleNamespace(name=names, position=positions))
    assert arm._feedback_seq == 0
    assert arm._feedback is None


def test_nonfinite_command_never_published(sim):
    arm, _ = sim
    assert not arm._send(float('nan'))
    arm.pub.publish.assert_not_called()


def test_failed_command_does_not_claim_open_or_closed(sim, monkeypatch):
    arm, _ = sim
    monkeypatch.setattr(arm, '_send', lambda _: False)
    monkeypatch.setattr(arm, '_angle_for', lambda _: .2)
    assert not arm.close(.04)
    assert arm.is_closed is None
    assert not arm.open()
    assert arm.is_closed is None
