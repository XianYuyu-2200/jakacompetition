"""Regression tests for the reference node's scene/start waiting policy.

Run with ROS 2 and the workspace sourced, then:
    python3 -m pytest -q jaka_competition_kit/test/test_reference_waits.py

A fake clock and spin callback keep the tests deterministic without a ROS node.
"""
from types import SimpleNamespace

import pytest

from jaka_competition_kit import reference


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.spins = 0

    def monotonic(self):
        return self.now


@pytest.fixture
def clock(monkeypatch):
    clock = FakeClock()
    # Replace this module's reference to time, not the process-wide clock.
    monkeypatch.setattr(reference, "time", clock)
    monkeypatch.setattr(reference.rclpy, "ok", lambda: True)
    return clock


def wait(node, kind, timeout=None):
    if kind == "scene":
        return reference.ReferenceBase.wait_for_scene(node, timeout=timeout)
    return reference.ReferenceBase.wait_for_state(node, "running", timeout=timeout)


def ready(node, kind):
    if kind == "scene":
        node.scene = {"track": 1}
    else:
        node.run_state = "running"


def unexpected_spin(*args, **kwargs):
    pytest.fail("ready or shut-down wait must not spin")


@pytest.mark.parametrize("kind", ["scene", "state"])
def test_ready_returns_without_spinning(kind, clock, monkeypatch):
    node = SimpleNamespace(scene={}, run_state="idle")
    ready(node, kind)
    monkeypatch.setattr(reference.rclpy, "spin_once", unexpected_spin)

    assert wait(node, kind)
    assert clock.spins == 0


@pytest.mark.parametrize("kind,step,ready_after", [
    ("scene", 25.0, 75.0),  # Greater than the old 60-second timeout.
    ("state", 50.0, 200.0),  # Greater than the old 180-second timeout.
])
def test_default_wait_has_no_implicit_deadline(kind, step, ready_after, clock, monkeypatch):
    node = SimpleNamespace(scene={}, run_state="idle")

    def spin(actual_node, timeout_sec):
        assert actual_node is node and timeout_sec == 0.1
        clock.spins += 1
        clock.now += step
        assert clock.spins <= 10, "wait did not observe the ready callback"
        if clock.now >= ready_after:
            ready(node, kind)

    monkeypatch.setattr(reference.rclpy, "spin_once", spin)

    assert wait(node, kind)
    assert clock.now == ready_after


@pytest.mark.parametrize("kind", ["scene", "state"])
def test_explicit_timeout_is_respected(kind, clock, monkeypatch):
    node = SimpleNamespace(scene={}, run_state="idle")

    def spin(actual_node, timeout_sec):
        clock.spins += 1
        clock.now += 1.0
        assert clock.spins <= 3, "explicit deadline was not respected"

    monkeypatch.setattr(reference.rclpy, "spin_once", spin)

    assert not wait(node, kind, timeout=3.0)
    assert clock.spins == 3


@pytest.mark.parametrize("kind", ["scene", "state"])
def test_shutdown_interrupts_wait(kind, clock, monkeypatch):
    node = SimpleNamespace(scene={}, run_state="idle")
    monkeypatch.setattr(reference.rclpy, "ok", lambda: False)
    monkeypatch.setattr(reference.rclpy, "spin_once", unexpected_spin)

    assert not wait(node, kind)


@pytest.mark.parametrize("kind", ["scene", "state"])
def test_callback_at_deadline_is_not_lost(kind, clock, monkeypatch):
    node = SimpleNamespace(scene={}, run_state="idle")

    def spin(actual_node, timeout_sec):
        clock.spins += 1
        clock.now += 1.0
        assert clock.spins <= 2
        if clock.now == 2.0:
            ready(node, kind)

    monkeypatch.setattr(reference.rclpy, "spin_once", spin)

    assert wait(node, kind, timeout=2.0)
    assert clock.spins == 2
