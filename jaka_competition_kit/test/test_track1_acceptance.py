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
