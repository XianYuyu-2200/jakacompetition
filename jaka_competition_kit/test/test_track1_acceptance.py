"""实际二维码图片解码和基础组失败中止回归。"""
from types import SimpleNamespace

import pytest

from jaka_competition_kit import qr, reference
from jaka_competition_kit.ref_track1 import Track1Reference


class Logger:
    def __init__(self):
        self.messages = []

    def __getattr__(self, name):
        return lambda message: self.messages.append((name, message))


@pytest.mark.parametrize("order", [[4, 3, 2, 1, 6, 5], [1, 2, 3, 4, 5, 6],
                                  [6, 5, 4, 3, 2, 1], [2, 4, 6, 1, 3, 5]])
def test_image_to_station_order(tmp_path, order):
    path = str(tmp_path / 'code.png')
    qr.render_png(qr.payload(order), path)
    logger = Logger()
    objects = [{'station': n} for n in range(1, 7)]
    node = SimpleNamespace(scene={'qr_png': path, 'objects': objects, 'qr_order': [1]*6},
                           get_logger=lambda: logger)
    assert [o['station'] for o in Track1Reference.plan_order(node)] == order


@pytest.mark.parametrize('stations', [[1, 2, 3, 4, 5], [1, 2, 3, 4, 5, 5],
                                      [1, 2, 3, 4, 5, 6, 6]])
def test_incomplete_or_duplicate_stations_stop(tmp_path, stations):
    path = str(tmp_path / 'code.png')
    qr.render_png(qr.payload([4, 3, 2, 1, 6, 5]), path)
    logger = Logger()
    node = SimpleNamespace(scene={'qr_png': path, 'objects': [{'station': n} for n in stations]},
                           get_logger=lambda: logger)
    assert Track1Reference.plan_order(node) == []


def test_fixed_failure_stops_without_retry_or_next_station(monkeypatch):
    monkeypatch.setattr(reference, 'is_complete', lambda *a, **kw: True)
    calls = []
    arm = SimpleNamespace(last_failure='motion', fixed_pick_and_place=lambda obj, *a: calls.append(obj['station']) or False)
    logger = Logger()
    objects = [{'station': n, 'xy': [0.1, 0.2]} for n in [4, 3, 2, 1, 6, 5]]
    node = SimpleNamespace(get_logger=lambda: logger, wait_for_scene=lambda: True,
                           wait_for_state=lambda _: True, get_parameter=lambda _: SimpleNamespace(value=False),
                           plan_order=lambda: objects, scene={'track': 1}, fixed_poses={},
                           fixed_poses_backend='simulation', fixed_poses_device_profile='sim',
                           arena=SimpleNamespace(track1_bin=object()), arm=arm, done=False)
    reference.ReferenceBase.run(node)
    assert calls == [4]
    assert node.done
    assert not node.succeeded
    assert any('本轮停止' in message for _, message in logger.messages)


def test_grasp_does_not_attach_or_score_when_close_fails():
    from jaka_competition_kit.executor import Executor
    logger = Logger()
    emitted = []
    arm = SimpleNamespace(gripper=SimpleNamespace(close=lambda **kw: False),
                          node=SimpleNamespace(get_logger=lambda: logger),
                          emit=lambda *a, **kw: emitted.append(a))
    assert not Executor.grasp(arm, 'wp_1', .04, (0, 0, 0))
    assert not emitted


def test_release_does_not_score_when_open_or_scene_release_fails():
    from jaka_competition_kit.executor import Executor
    logger = Logger()
    emitted, releases = [], []
    arm = SimpleNamespace(gripper=SimpleNamespace(open=lambda: False),
                          node=SimpleNamespace(get_logger=lambda: logger),
                          scene=SimpleNamespace(release_into_bin=lambda *a: releases.append(a) or False),
                          arena=SimpleNamespace(tcp_link='tcp'),
                          emit=lambda *a, **kw: emitted.append(a))
    assert not Executor.release(arm, 'wp_1', None)
    assert not releases and not emitted
    arm.gripper.open = lambda: True
    assert not Executor.release(arm, 'wp_1', None)
    assert len(releases) == 1 and not emitted


def test_fixed_selfcheck_warns_that_plans_are_independent(monkeypatch):
    monkeypatch.setattr(reference, 'is_complete', lambda *a, **kw: True)
    logger = Logger()
    obj = {'station': 1, 'xy': [0.1, 0.2]}
    pose = [0.0] * 6
    config = {'objects': {'1': {p + '_joints': pose for p in ('approach', 'grasp', 'lift')}},
              'place': {p + '_joints': pose for p in ('approach', 'release', 'retreat')}}
    calls = []
    node = SimpleNamespace(get_logger=lambda: logger, wait_for_scene=lambda: True,
                           get_parameter=lambda _: SimpleNamespace(value=True),
                           plan_order=lambda: [obj], scene={'track': 1}, fixed_poses=config,
                           fixed_poses_backend='simulation', fixed_poses_device_profile='sim',
                           arena=SimpleNamespace(track1_bin=object()),
                           backend=SimpleNamespace(goto_joints=lambda j, plan_only: calls.append(plan_only) or SimpleNamespace(ok=True)),
                           done=False)
    reference.ReferenceBase.run(node)
    assert calls == [True] * 6
    assert any('不验证点间连续轨迹' in msg for _, msg in logger.messages)
    assert any('非连续流程验收' in msg for _, msg in logger.messages)


def test_waypoint_execution_order_and_failure_stop():
    from jaka_competition_kit.executor import Executor
    pose = lambda n: [float(n)] * 6
    entry = {'approach_via_joints': [pose(1), pose(2)], 'approach_joints': pose(3),
             'grasp_joints': pose(4), 'lift_joints': pose(5), 'transfer_via_joints': [pose(5.5)]}
    place = {'approach_via_joints': [pose(6)], 'approach_joints': pose(7),
             'release_joints': pose(8), 'retreat_joints': pose(9)}
    poses = {'objects': {'6': entry}, 'place': place}
    obj = {'station': 6, 'id': 'wp_6', 'xy': (0, 0), 'z': 0, 'dims': [0.04] * 3}
    calls = []
    def goto(j):
        calls.append(j[0])
        return SimpleNamespace(ok=True)
    arm = SimpleNamespace(backend=SimpleNamespace(goto_joints=goto), node=SimpleNamespace(get_logger=Logger),
                          tcp_pose=lambda: ([0, 0, 0], None), grasp=lambda *a, **kw: True,
                          release=lambda *a: True, last_failure='')
    assert Executor.fixed_pick_and_place(arm, obj, None, poses)
    assert calls == [1, 2, 3, 4, 5, 5.5, 6, 7, 8, 9]
    calls.clear()
    def fail(j):
        calls.append(j[0])
        return SimpleNamespace(ok=j[0] != 2)
    arm.backend.goto_joints = fail
    assert not Executor.fixed_pick_and_place(arm, obj, None, poses)
    assert calls == [1, 2] and arm.last_failure == 'motion'


def test_station_transfer_failure_stops_before_place():
    from jaka_competition_kit.executor import Executor
    pose = lambda n: [float(n)] * 6
    poses = {'objects': {'6': {'approach_joints': pose(1), 'grasp_joints': pose(2),
                               'lift_joints': pose(3), 'transfer_via_joints': [pose(4), pose(5)]}},
             'place': {'approach_joints': pose(6), 'release_joints': pose(7), 'retreat_joints': pose(8)}}
    obj = {'station': 6, 'id': 'wp_6', 'xy': (0, 0), 'z': 0, 'dims': [0.04] * 3}
    calls = []
    def goto(j):
        calls.append(j[0])
        return SimpleNamespace(ok=j[0] != 4)
    releases = []
    arm = SimpleNamespace(backend=SimpleNamespace(goto_joints=goto), node=SimpleNamespace(get_logger=Logger),
                          tcp_pose=lambda: ([0, 0, 0], None), grasp=lambda *a, **kw: True,
                          release=lambda *a: releases.append(True), last_failure='')
    assert not Executor.fixed_pick_and_place(arm, obj, None, poses)
    assert calls == [1, 2, 3, 4] and arm.last_failure == 'motion' and not releases


@pytest.mark.parametrize('success', [False, True])
def test_cli_exit_status_reports_failure_and_cleans_up(monkeypatch, success):
    from unittest.mock import Mock
    node = SimpleNamespace(run=Mock(), done=True, succeeded=success,
                           scene_cli=SimpleNamespace(close=Mock()), destroy_node=Mock())
    shutdown = Mock()
    monkeypatch.setattr(reference.rclpy, 'try_shutdown', shutdown)
    assert reference.spin(node) == (0 if success else 2)
    node.scene_cli.close.assert_called_once()
    node.destroy_node.assert_called_once()
    shutdown.assert_called_once()


def test_cli_interrupt_is_not_success(monkeypatch):
    from unittest.mock import Mock
    node = SimpleNamespace(run=Mock(side_effect=KeyboardInterrupt),
                           scene_cli=SimpleNamespace(close=Mock()), destroy_node=Mock())
    monkeypatch.setattr(reference.rclpy, 'try_shutdown', Mock())
    assert reference.spin(node) == 130
    node.destroy_node.assert_called_once()


@pytest.mark.parametrize('track,fixed_ready', [(1, False), (2, False), (2, True)])
def test_verify_only_never_falls_back_to_executing_motion(monkeypatch, track, fixed_ready):
    from unittest.mock import Mock
    monkeypatch.setattr(reference, 'is_complete', lambda *a, **kw: fixed_ready)
    arm = SimpleNamespace(pick_and_place=Mock(), fixed_pick_and_place=Mock())
    node = SimpleNamespace(get_logger=lambda: Logger(), wait_for_scene=lambda: True,
                           wait_for_state=Mock(), get_parameter=lambda _: SimpleNamespace(value=True),
                           plan_order=lambda: [{'station': 1, 'xy': [0., 0.]}],
                           scene={'track': track}, fixed_poses={},
                           fixed_poses_backend='simulation', fixed_poses_device_profile='sim',
                           arm=arm, done=True, succeeded=True)
    reference.ReferenceBase.run(node)
    arm.pick_and_place.assert_not_called()
    arm.fixed_pick_and_place.assert_not_called()
    node.wait_for_state.assert_not_called()
    assert not node.succeeded and not node.done
