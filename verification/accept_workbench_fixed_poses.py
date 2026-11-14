#!/usr/bin/env python3
"""Explicit opt-in, isolated Gazebo replay; never a hardware/contact acceptance.

Run from the repository with /usr/bin/python3. The source pose file is never
changed. Only a temporary copy is enabled, and it is disabled again on exit.
The output directory must not already exist. No existing simulation is stopped.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'jaka_competition_kit')]
import yaml
from jaka_competition_kit.fixed_poses import is_complete, load_fixed_poses
from workbench.ros_bridge import PREFIX
from workbench.runtime import ProcessManager, ros_command



# Used only after the isolated runner's six releases leave the fingers open.
# Executor must reject before any TF, scene attachment or scoring operation.
STALL_PROBE = """
import rclpy
from types import SimpleNamespace
from jaka_competition_kit.arena import load_arena
from jaka_competition_kit.gripper import SimGripper
from jaka_competition_kit.executor import Executor
rclpy.init()
node = rclpy.create_node('acceptance_gripper_stall')
try:
    gripper = SimGripper(node, arena=load_arena())
    arm = SimpleNamespace(gripper=gripper, node=node)
    result = Executor.grasp(arm, 'acceptance_no_attach', .04, (0., 0., 0.))
    if result or gripper.is_closed is not None:
        raise RuntimeError('inactive controller was accepted as a successful grasp')
    print('PASS: inactive simulated gripper rejected before attachment or scoring')
finally:
    node.destroy_node()
    rclpy.shutdown()
"""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--poses', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--domain', type=int, default=94)
    parser.add_argument('--seeds', type=int, nargs='+', default=[246135, 1, 2, 3])
    parser.add_argument('--velocity', type=float, default=.5)
    parser.add_argument('--check-gripper-stall', action='store_true',
                        help='deactivate only this isolated simulated gripper controller to test rejection')
    args = parser.parse_args(argv)
    if not 1 <= args.domain <= 100 or not .1 <= args.velocity <= .5:
        parser.error('domain must be 1..100, velocity must be .1.. .5')
    if len(set(args.seeds)) != len(args.seeds):
        parser.error('seeds must be distinct to keep score evidence unambiguous')
    source, out = args.poses.resolve(), args.output.resolve()
    data = load_fixed_poses(str(source))
    if not source.is_file() or not is_complete({**data, 'enabled': True},
            backend='simulation', device_profile='jaka_mini_2_gemini305_gripper_sim'):
        parser.error('requires complete simulation poses with the exact simulation profile')
    if data.get('calibration', {}).get('requires_recalibration'):
        parser.error('legacy or unknown calibration provenance cannot be replayed')
    out.mkdir(parents=True, exist_ok=False)
    active = out / 'active-test-only.yaml'
    data['enabled'] = False
    data.setdefault('calibration', {})['verified'] = False
    active.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
    partition = f'jaka_acceptance_{args.domain}_{os.getpid()}'
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
               GZ_PARTITION=partition, IGN_PARTITION=partition, JAKA_GRIPPER='1',
               PYTHONPATH=f'{ROOT}:{ROOT}/jaka_competition_kit:' + os.environ.get('PYTHONPATH', ''))
    code_files = [
        'jaka_competition_kit/jaka_competition_kit/reference.py',
        'jaka_competition_kit/jaka_competition_kit/ref_track1.py',
        'jaka_competition_kit/jaka_competition_kit/qr.py',
        'jaka_competition_kit/jaka_competition_kit/scorer.py',
        'jaka_competition_kit/config/arena.yaml',
        'jaka_competition_kit/config/rules.yaml',
        'jaka_competition_kit/jaka_competition_kit/gripper.py',
        'jaka_competition_kit/jaka_competition_kit/executor.py',
        'verification/accept_workbench_fixed_poses.py',
        'verification/sim_controller_switch.py',
    ]
    report = {'code_sha256': {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                              for name in code_files}, 'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'source': str(source), 'domain': args.domain, 'partition': partition,
              'velocity': args.velocity, 'physical_contact_validated': False, 'rounds': []}
    manager = ProcessManager(out / 'logs')
    texts, exits = {}, {}
    state = {'status': None, 'images': 0}

    def launch(name, cmd):
        texts[name] = []
        manager.launch(name, ros_command(ROOT, cmd), ROOT, env)

    def pump():
        for event in manager.poll():
            name = event['name']
            if event['kind'] == 'exited':
                exits[name] = event['returncode']
            if event['kind'] != 'output':
                continue
            text = event['text']
            if name != 'monitor':
                texts.setdefault(name, []).append(text)
            if text.startswith(PREFIX):
                payload = json.loads(text[len(PREFIX):])
                if payload['type'] == 'status':
                    state['status'] = payload
                elif payload['type'] == 'image':
                    state['images'] += 1
        time.sleep(.1)

    def until(predicate, timeout):
        end = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= end:
                raise TimeoutError(f'condition timeout ({timeout}s)')
            pump()
            if 'stack' in exits:
                raise RuntimeError('isolated stack exited; inspect stack log')

    def run(name, cmd, timeout=25):
        launch(name, cmd)
        until(lambda: name in exits, timeout)
        text = '\n'.join(texts[name])
        (out / (name + '.log')).write_text(text)
        if exits[name] != 0:
            raise RuntimeError(f'{name}: exit {exits[name]}: {text[-500:]}')
        return text

    def service(name, endpoint):
        text = run(name, ['ros2', 'service', 'call', '/competition/' + endpoint,
                          'std_srvs/srv/Trigger', '{}'])
        if 'success=True' not in text:
            raise RuntimeError(text)
        return text

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    previous_term = signal.signal(signal.SIGTERM, interrupt)
    try:
        existing = run('preflight', ['ros2', 'node', 'list', '--no-daemon'], 15)
        if existing.strip():
            raise RuntimeError('ROS domain is not empty: ' + existing)
        data['enabled'] = True
        active.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
        launch('stack', ['ros2', 'launch', 'jaka_competition_kit', 'workbench.launch.py',
                         'backend:=gazebo', 'use_rviz:=false', 'output_dir:=' + str(out)])
        launch('monitor', ['/usr/bin/python3', '-m', 'workbench.ros_bridge', 'monitor',
                           '--mode', 'basic', '--camera'])
        until(lambda: state['status'] and state['status']['ready'], 100)
        for seed in args.seeds:
            text = run(f'param-{seed}', ['ros2', 'param', 'set', '--no-daemon', '--spin-time', '2',
                                         '/competition_scene_generator', 'seed', str(seed)])
            if 'Set parameter successful' not in text:
                raise RuntimeError('seed change was not acknowledged: ' + text)
            service(f'generate-{seed}', 'generate')
            name = f'ref-{seed}'
            images_before = state['images']
            launch(name, ['/usr/bin/python3', '-m', 'jaka_competition_kit.ref_track1',
                          '--ros-args', '-p', 'gripper:=sim', '-p', f'vel_scale:={args.velocity}',
                          '-p', 'fixed_poses_config:=' + str(active),
                          '-p', 'allow_cartesian_fallback:=false'])
            until(lambda: name in exits or any('等待裁判发令' in t for t in texts[name]), 35)
            if name in exits:
                raise RuntimeError('reference exited before start: ' + name)
            service(f'start-{seed}', 'start')
            until(lambda: name in exits, 200)
            text = '\n'.join(texts[name])
            (out / (name + '.log')).write_text(text)
            scores = [json.loads(p.read_text()) for p in out.glob('score_*.json')]
            score = next((s for s in scores if s['seed'] == seed), None)
            frames = state['images'] - images_before
            passed = (exits[name] == 0 and '参考实现结束: 6/6' in text
                      and text.count('使用已记录固定关节姿态') == 6 and frames > 0
                      and score is not None and score['collected_count'] == 6
                      and not score['out_of_order'] and not score['invalid']
                      and [o['station'] for o in score['collected']] == score['generated_order'])
            summary = [line for line in text.splitlines() if '二维码内容' in line or '参考实现结束' in line]
            report['rounds'].append({'seed': seed, 'passed': passed, 'summary': summary,
                                     'camera_frames_during_round': frames, 'score': score})
            print(seed, passed, summary, flush=True)
            if not passed:
                raise RuntimeError(f'round {seed} failed; no automatic retry')
        if args.check_gripper_stall:
            # No arm motion. Deactivate the controller owned by this isolated
            # stack, request a different finger position, then restore it.
            def switch(target):
                text = run('gripper-' + target, ['/usr/bin/python3', '-c',
                    'from verification.sim_controller_switch import main; '
                    f'main({target!r})'], timeout=40)
                report['gripper_switch_' + target] = json.loads(text.strip().splitlines()[-1])
            try:
                switch('inactive')
                run('gripper-stall', ['/usr/bin/python3', '-c', STALL_PROBE])
                report['gripper_stall_rejected'] = True
            finally:
                # A lost reply does not mean the request had no effect. Always
                # restore using state readback, including deactivate failures.
                switch('active')
        report['passed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = repr(exc)
        report['passed'] = False
    finally:
        data['enabled'] = False
        try:
            active.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
        finally:
            manager.close()
            signal.signal(signal.SIGTERM, previous_term)
        report['image_messages'] = state['images']
        report['last_status'] = state['status']
        report['remaining_owned_jobs'] = manager.active_names()
        if report['remaining_owned_jobs']:
            report['passed'] = False
        (out / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print('REPORT', out / 'report.json', flush=True)
    return 0 if report.get('passed') else 2


if __name__ == '__main__':
    raise SystemExit(main())
