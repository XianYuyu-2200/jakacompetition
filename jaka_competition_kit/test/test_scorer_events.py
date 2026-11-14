"""Malformed event telemetry must neither crash the scorer nor mutate results."""
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from jaka_competition_kit.scorer import Scorer


@pytest.fixture
def scorer():
    # Use real callback/handlers without ROS publishers or a running robot.
    node = SimpleNamespace(
        state='running', holding=None, scene={'objects': [
            {'id': 'wp_1', 'xy': [.1, .2], 'station': 1}],
            'bin': {'center': [.3, .2], 'size': [.2, .2]}},
        rules={'tolerance': {'grasp_xy_m': .045, 'bin_xy_m': .02}},
        track=1, correct_order=[1, 2, 3, 4, 5, 6], order_ptr=0,
        collected=[], dropped=[], out_of_order=[], misplaced=[],
        _log=Mock(), get_logger=lambda: Mock(), elapsed=lambda: 1.0, _finish=Mock())
    for name in ('_on_event', '_event_pose', '_object_truth', '_handle_grasp',
                 '_handle_release', '_bin_containing', '_all_collected'):
        setattr(node, name, getattr(Scorer, name).__get__(node))
    return node


def event(node, kind, **kw):
    node._on_event(SimpleNamespace(data=json.dumps({'event': kind, 'object_id': 'wp_1', **kw})))


@pytest.mark.parametrize('pose', [None, [], [0], [0, 0], [0, 0, 0, 0], 'xyz', {},
    ['.1', .2, 0], [True, .2, 0], [float('nan'), .2, 0],
    [.1, float('inf'), 0], [.1, .2, float('-inf')], [.1, .2, 10**400]])
@pytest.mark.parametrize('kind', ['grasp', 'release'])
def test_invalid_pose_leaves_holding_and_score_unchanged(scorer, pose, kind):
    scorer.holding = 'wp_1' if kind == 'release' else None
    before = scorer.holding
    event(scorer, kind, pose=pose)
    assert scorer.holding == before
    assert not scorer.collected and not scorer.dropped
    assert scorer.order_ptr == 0


@pytest.mark.parametrize('kind', ['grasp', 'release'])
def test_missing_pose_rejected(scorer, kind):
    scorer.holding = 'wp_1'
    event(scorer, kind)
    assert scorer.holding == 'wp_1' and not scorer.collected


@pytest.mark.parametrize('data', ['[]', 'null', '42', '"grasp"', '{broken'])
def test_non_object_json_does_not_crash(scorer, data):
    scorer._on_event(SimpleNamespace(data=data))
    assert scorer.holding is None and not scorer.collected


def test_valid_grasp_and_release_still_score(scorer):
    event(scorer, 'grasp', pose=[.1, .2, .1])
    assert scorer.holding == 'wp_1'
    event(scorer, 'release', pose=[.3, .2, .1])
    assert scorer.holding is None
    assert [o['station'] for o in scorer.collected] == [1]
    assert scorer.order_ptr == 1 and not scorer.dropped
