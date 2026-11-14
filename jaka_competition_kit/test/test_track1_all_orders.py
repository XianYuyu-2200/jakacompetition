"""All 720 image orders through reference + fixed-stage dispatch, not physics.

The production QR decoder and fixed-pose executor run here. Motion, TF and
attachment are deterministic doubles; this is not 720 Gazebo trajectory tests.
"""
from itertools import permutations
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from jaka_competition_kit import qr, reference
from jaka_competition_kit.executor import Executor
from jaka_competition_kit.fixed_poses import load_fixed_poses, waypoints_for
from jaka_competition_kit.ref_track1 import Track1Reference

CONFIG = Path(__file__).resolve().parents[1] / 'config/track1_simulation_example.yaml'


@pytest.fixture(scope='module')
def poses():
    data = load_fixed_poses(str(CONFIG))
    data['enabled'] = True  # In-memory copy only; no simulator/hardware command.
    return data


def expected_trace(poses, order):
    trace = []
    for station in order:
        entry = poses['objects'][str(station)]
        for phase in ('approach', 'grasp', 'lift', 'transfer'):
            trace.extend(('move', tuple(j)) for j in waypoints_for(entry, phase))
            if phase != 'transfer':
                trace.append(('move', tuple(entry[phase + '_joints'])))
            if phase == 'grasp':
                trace.append(('grasp', f'wp_{station}'))
        for phase in ('approach', 'release', 'retreat'):
            trace.extend(('move', tuple(j)) for j in waypoints_for(poses['place'], phase))
            trace.append(('move', tuple(poses['place'][phase + '_joints'])))
            if phase == 'release':
                trace.append(('release', f'wp_{station}'))
    return trace


def run_reference(poses, image_path, fail_at=None, tf=None):
    calls = []
    def action(kind, value):
        calls.append((kind, value))
        return len(calls) - 1 != fail_at
    arm = SimpleNamespace(
        backend=SimpleNamespace(goto_joints=lambda j: SimpleNamespace(ok=action('move', tuple(j)))),
        node=SimpleNamespace(get_logger=lambda: Mock()),
        tcp_pose=tf or (lambda: ([0., 0., 0.], [0., 0., 0., 1.])),
        grasp=lambda oid, *a, **kw: action('grasp', oid),
        release=lambda oid, *a: action('release', oid), last_failure='')
    arm.fixed_pick_and_place = Executor.fixed_pick_and_place.__get__(arm)
    objects = [{'station': n, 'id': f'wp_{n}', 'xy': [.1, .2], 'z': .03,
                'dims': [.04] * 3} for n in range(6, 0, -1)]
    node = SimpleNamespace(
        get_logger=lambda: Mock(), wait_for_scene=lambda: True, wait_for_state=lambda _: True,
        get_parameter=lambda _: SimpleNamespace(value=False),
        scene={'track': 1, 'qr_png': str(image_path), 'objects': objects,
               'qr_order': [0] * 6},  # Poison truth: it must not choose the order.
        fixed_poses=poses, fixed_poses_backend=poses['backend'],
        fixed_poses_device_profile=poses['device_profile'],
        arena=SimpleNamespace(track1_bin=object()), arm=arm)
    node.plan_order = lambda: Track1Reference.plan_order(node)
    reference.ReferenceBase.run(node)
    return node, calls


@pytest.mark.parametrize('order', list(permutations(range(1, 7))),
                         ids=lambda order: ''.join(map(str, order)))
def test_all_image_orders_dispatch_exact_stages(tmp_path, poses, order):
    path = str(tmp_path / 'qr.png')
    for text in (qr.payload(order), ''.join(map(str, order))):
        qr.render_png(text, path)
        node, calls = run_reference(poses, path)
        assert node.done and node.succeeded
        assert calls == expected_trace(poses, order)


FAULT_ORDER = [4, 3, 2, 1, 6, 5]
# One fault at every movement (including via points), grasp and release across
# all six stations. No automatic retry, recovery or subsequent station allowed.
FAULT_TRACE = expected_trace(load_fixed_poses(str(CONFIG)), FAULT_ORDER)


@pytest.mark.parametrize('fail_at', range(len(FAULT_TRACE)))
def test_every_action_failure_stops_the_whole_round(tmp_path, poses, fail_at):
    path = str(tmp_path / 'qr.png')
    qr.render_png(qr.payload(FAULT_ORDER), path)
    node, calls = run_reference(poses, path, fail_at)
    assert node.done and not node.succeeded
    assert calls == FAULT_TRACE[:fail_at + 1]
    expected = 'motion' if FAULT_TRACE[fail_at][0] == 'move' else FAULT_TRACE[fail_at][0]
    assert node.arm.last_failure == expected


@pytest.mark.parametrize('phase_index', range(36))
def test_missing_tf_at_each_phase_stops_before_next_action(tmp_path, poses, phase_index):
    path = str(tmp_path / 'qr.png')
    qr.render_png(qr.payload(FAULT_ORDER), path)
    reads = []
    def tf():
        reads.append(True)
        if len(reads) - 1 == phase_index:
            raise RuntimeError('injected missing TF')
        return [0., 0., 0.], [0., 0., 0., 1.]
    node, calls = run_reference(poses, path, tf=tf)
    # Independently locate the phase-ending move (via points don't read TF).
    prefix = []
    phase = 0
    for station in FAULT_ORDER:
        for entry, names in ((poses['objects'][str(station)], ('approach', 'grasp', 'lift', 'transfer')),
                             (poses['place'], ('approach', 'release', 'retreat'))):
            for name in names:
                prefix.extend(('move', tuple(j)) for j in waypoints_for(entry, name))
                if name == 'transfer':
                    continue
                prefix.append(('move', tuple(entry[name + '_joints'])))
                if phase == phase_index:
                    assert calls == prefix
                    assert node.done and not node.succeeded
                    assert node.arm.last_failure == 'feedback'
                    assert len(reads) == phase_index + 1
                    return
                phase += 1
                if name in ('grasp', 'release'):
                    prefix.append((name, f'wp_{station}'))
    pytest.fail('phase index not found')


@pytest.mark.parametrize('xyz', [[], [0, 0], [0, 0, 0, 0], [float('nan'), 0, 0],
                                [0, float('inf'), 0]])
def test_invalid_tf_stops_before_grasp(tmp_path, poses, xyz):
    path = str(tmp_path / 'qr.png')
    qr.render_png(qr.payload(FAULT_ORDER), path)
    node, calls = run_reference(poses, path, tf=lambda: (xyz, None))
    assert node.done and not node.succeeded and node.arm.last_failure == 'feedback'
    assert all(kind == 'move' for kind, _ in calls)
