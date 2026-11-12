"""No ROS, graphical session, or installed Qt is needed for these tests.

Run: python3 -m unittest discover -s workbench/test -p test_runtime.py -v
"""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

from workbench import runtime


class FilesystemTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='workbench-runtime-')
        self.root = Path(self.tmp.name)
        self.addCleanup(self.tmp.cleanup)

    def make_repo(self):
        (self.root / 'jaka_competition_kit').mkdir()
        (self.root / 'jaka_ros2').mkdir()
        return self.root

    def test_discover_from_directory_and_file(self):
        root = self.make_repo()
        nested = root / '目录' / 'nested'
        nested.mkdir(parents=True)
        source = nested / 'test.py'
        source.touch()
        self.assertEqual(runtime.discover_root(nested), root)
        self.assertEqual(runtime.discover_root(source), root)

    def test_discover_module_default(self):
        root = runtime.discover_root()
        self.assertTrue((root / 'jaka_ros2').is_dir())

    def test_discover_failure(self):
        with self.assertRaisesRegex(RuntimeError, '仓库根目录'):
            runtime.discover_root(self.root)

    def test_environment_missing_files_and_display_warning(self):
        with mock.patch.dict(os.environ, {}, clear=True), \
             mock.patch.object(runtime, '_ROS_SETUP', self.root / 'missing-ros'), \
             mock.patch.object(runtime, '_ROS_EXECUTABLE', self.root / 'missing-bin'), \
             mock.patch.object(runtime.importlib.util, 'find_spec', return_value=None):
            checks = runtime.check_environment(self.root)
        self.assertTrue(any(item['level'] == 'error' for item in checks))
        self.assertTrue(any(item['name'] == 'PyQt5' and item['level'] == 'error' for item in checks))
        self.assertTrue(any(item['name'] == '图形会话 DISPLAY' and item['level'] == 'warning'
                            for item in checks))
        self.assertTrue(all(set(item) == {'name', 'level', 'detail'} for item in checks))

    def test_environment_present_files_are_not_ready(self):
        paths = [
            'ros-setup', 'ros2-bin', 'jaka_ros2/install/setup.bash',
            'jaka_competition_kit/package.xml',
            'jaka_competition_kit/launch/round.launch.py',
            'jaka_competition_kit/launch/workbench.launch.py',
            'jaka_competition_kit/config/workbench.rviz',
            'jaka_ros2/src/wheeltec_gripper/package.xml',
            'jaka_ros2/src/wheeltec_gripper/config/gripper.yaml',
            'jaka_ros2/src/wheeltec_gripper/urdf/wheeltec_gripper.urdf.xacro',
            'jaka_competition_kit/jaka_competition_kit/reference.py',
            'jaka_competition_kit/jaka_competition_kit/executor.py',
            'jaka_competition_kit/jaka_competition_kit/scorer.py',
            'jaka_competition_kit/config/arena.yaml',
            'jaka_competition_kit/config/rules.yaml',
        ]
        for relative in paths:
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
            path.chmod(0o755)
        with mock.patch.object(runtime, '_ROS_SETUP', self.root / 'ros-setup'), \
             mock.patch.object(runtime, '_ROS_EXECUTABLE', self.root / 'ros2-bin'), \
             mock.patch.object(runtime.importlib.util, 'find_spec', return_value=object()), \
             mock.patch.dict(os.environ, {'DISPLAY': ':123'}), \
             mock.patch.object(runtime.subprocess, 'Popen', side_effect=AssertionError('must not spawn')):
            checks = runtime.check_environment(self.root)
        self.assertFalse(any(item['level'] == 'error' for item in checks))
        self.assertTrue(any(item['name'] == '运行就绪状态' and item['level'] == 'warning'
                            for item in checks))

    def test_environment_checks_workbench_and_gripper_sources(self):
        expected = (
            'jaka_competition_kit/launch/workbench.launch.py',
            'jaka_competition_kit/config/workbench.rviz',
            'jaka_ros2/src/wheeltec_gripper/package.xml',
            'jaka_ros2/src/wheeltec_gripper/config/gripper.yaml',
            'jaka_ros2/src/wheeltec_gripper/urdf/wheeltec_gripper.urdf.xacro',
        )
        checks = runtime.check_environment(self.root)
        by_detail = {item['detail']: item for item in checks}
        for relative in expected:
            with self.subTest(relative=relative):
                detail = f'缺失或不可访问：{self.root / relative}'
                self.assertEqual(by_detail[detail]['level'], 'error')

    def test_recognition_checks_scene_and_classifier_dependencies(self):
        with mock.patch.object(runtime.shutil, 'which', return_value=None):
            checks = runtime.check_environment(self.root, mode='recognition')
        by_name = {item['name']: item for item in checks}
        for name in ('Gazebo Fortress', '基础组识别启动文件', '基础组几何体分类器',
                     '识别场景关节配置'):
            self.assertEqual(by_name[name]['level'], 'error')

    def test_vision_checks_all_missing_dependencies(self):
        with mock.patch.object(runtime.shutil, 'which', return_value=None):
            checks = runtime.check_environment(self.root, mode='vision')
        by_name = {item['name']: item for item in checks}
        for name in ('Gazebo ign', '视觉启动文件', '基础组视觉模型', 'YOLO 虚拟环境 Python'):
            self.assertEqual(by_name[name]['level'], 'error')

    def test_invalid_mode(self):
        checks = runtime.check_environment(self.root, mode='invalid')
        self.assertTrue(any(item['name'] == '运行模式' and item['level'] == 'error' for item in checks))

    def test_ros_command_safely_quotes_paths_arguments_and_executes(self):
        root = self.root / "空 格 ' $(touch INJECTED) ;"
        setup = root / 'jaka_ros2/install/setup.bash'
        setup.parent.mkdir(parents=True)
        setup.write_text('export WORKBENCH_OVERLAY=overlay\n')
        ros_setup = self.root / "ROS ' ; 中文.bash"
        ros_setup.write_text('export WORKBENCH_BASE=base\n')
        script = self.root / 'capture.py'
        script.write_text('import json,os,sys\nprint(json.dumps([sys.argv[1:],os.environ["WORKBENCH_BASE"],os.environ["WORKBENCH_OVERLAY"]]))\n')
        args = ['', "引号 ' \"", '$(touch INJECTED)', '; touch INJECTED', 'line\nbreak', 'a b']
        with mock.patch.object(runtime, '_ROS_SETUP', ros_setup):
            command = runtime.ros_command(root, [sys.executable, str(script), *args])
        self.assertEqual(command[:3], ['/bin/bash', '--noprofile', '--norc'])
        result = subprocess.run(command, cwd=self.root, text=True, capture_output=True, check=True)
        self.assertEqual(json.loads(result.stdout), [args, 'base', 'overlay'])
        self.assertFalse((self.root / 'INJECTED').exists())

    def test_ros_command_stops_on_source_failure(self):
        with mock.patch.object(runtime, '_ROS_SETUP', self.root / 'nonexistent'):
            command = runtime.ros_command(self.root, ['touch', str(self.root / 'should-not-exist')])
        result = subprocess.run(command, capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.root / 'should-not-exist').exists())

    def test_ros_command_rejects_shell_string(self):
        for argv in ('echo unsafe', [], ''):
            with self.assertRaises(ValueError):
                runtime.ros_command(self.root, argv)


@unittest.skipUnless(sys.platform.startswith('linux'), 'Linux /proc required')
class ConflictTests(unittest.TestCase):
    def test_matches_ros_and_native_but_not_shell_or_python_payload(self):
        yes = [
            ['/opt/ros/humble/bin/ros2', 'launch', 'jaka_competition_kit', 'round.launch.py'],
            ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'jaka_competition_kit', 'workbench.launch.py'],
            ['ros2', 'launch', 'jaka_competition_kit', 'gemini305_preview.launch.py'],
            ['ros2', 'run', 'jaka_competition_kit', 'scorer'],
            ['/opt/ros/humble/lib/moveit_ros_move_group/move_group'],
            ['/usr/bin/python3', '/pkg/jaka_competition_kit/scorer'],
            ['ros2_control_node'], ['competition_scorer'], ['scene_generator'],
            ['ign', 'gazebo', '-r'],
        ]
        no = [
            ['bash', '-c', 'ros2 launch jaka_competition_kit round.launch.py'],
            [sys.executable, '-c', 'print("ign gazebo")'],
            ['grep', 'move_group'],
            ['ros2', 'launch', 'other', 'other.launch.py'],
            ['ros2', 'run', 'jaka_competition_kit', 'arena_check'],
            ['ign', 'topic', '-l'], [],
        ]
        for argv in yes:
            with self.subTest(argv=argv):
                self.assertTrue(runtime._simulation_argv(argv))
        for argv in no:
            with self.subTest(argv=argv):
                self.assertFalse(runtime._simulation_argv(argv))

    def test_launch_package_absolute_relative_and_bare_file_forms(self):
        for filename in ('round.launch.py', 'gemini305_preview.launch.py', 'workbench.launch.py'):
            for target in (["jaka_competition_kit", filename],
                           [f"/abs/path/{filename}"],
                           [f"../relative/{filename}"],
                           [f"./relative/{filename}"], [filename]):
                for suffix in ([], ['seed:=246135', 'use_rviz:=true']):
                    for prefix in (['ros2'], ['/usr/bin/python3', '/opt/ros/humble/bin/ros2']):
                        argv = [*prefix, 'launch', *target, *suffix]
                        with self.subTest(argv=argv):
                            self.assertTrue(runtime._simulation_argv(argv))

    def test_launch_assignments_are_not_filenames(self):
        no = [
            ['ros2', 'launch', 'jaka_competition_kit', 'seed:=workbench.launch.py'],
            ['ros2', 'launch', 'jaka_competition_kit', 'seed:=/abs/workbench.launch.py'],
            ['ros2', 'launch', 'seed:=/abs/workbench.launch.py'],
            ['ros2', 'launch', '/abs/other.launch.py', 'seed:=/abs/workbench.launch.py'],
            ['ros2', 'launch', './other.launch.py', 'workbench.launch.py'],
            ['ros2', 'launch', '/abs/other.launch.py', '/abs/workbench.launch.py'],
            ['ros2', 'launch', 'jaka_competition_kit', '--name=workbench.launch.py'],
            ['ros2', 'launch', 'jaka_competition_kit', 'other.launch.py', 'seed:=workbench.launch.py'],
            ['ros2', 'launch', '/abs/other.launch.py', 'seed:=246135'],
            ['ros2', 'launch', 'jaka_competition_kit'],
        ]
        for argv in no:
            with self.subTest(argv=argv):
                self.assertFalse(runtime._simulation_argv(argv))

    def test_excludes_ancestors_and_returns_pid_and_command(self):
        own = os.getpid()
        parent = os.getppid()
        other = max(own, parent) + 1000
        snapshot = {
            own: runtime._Proc(own, parent, own, own, 10, 'R'),
            parent: runtime._Proc(parent, 1, parent, parent, 9, 'S'),
            other: runtime._Proc(other, 1, other, other, 11, 'S'),
        }
        command = b'/opt/ros/humble/bin/ros2\0launch\0jaka_competition_kit\0round.launch.py\0'
        with mock.patch.object(runtime, '_process_snapshot', return_value=snapshot), \
             mock.patch.object(Path, 'read_bytes', return_value=command), \
             mock.patch.object(runtime.os, 'kill', side_effect=AssertionError('must not signal')):
            hits = runtime.simulation_conflicts()
        self.assertEqual(len(hits), 1)
        self.assertEqual(hits[0]['pid'], other)
        self.assertIn('round.launch.py', hits[0]['command'])

    def test_proc_stat_parses_name_with_parentheses(self):
        # Field 22 = starttime; comm must not shift the stat positions.
        tail = ['S', '3', '4', '5'] + ['0'] * 15 + ['12345'] + ['0'] * 10
        with mock.patch.object(Path, 'read_text', return_value='9 (name with ) and spaces) ' + ' '.join(tail)):
            proc = runtime._read_proc(9)
        self.assertEqual((proc.pid, proc.ppid, proc.pgrp, proc.session, proc.start), (9, 3, 4, 5, 12345))


@unittest.skipUnless(sys.platform.startswith('linux'), 'Linux process management required')
class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='workbench-process-')
        self.root = Path(self.tmp.name)
        self.manager = runtime.ProcessManager(self.root / 'logs')
        # Short grace periods keep the suite quick; production defaults are longer.
        self.manager.INT_TIMEOUT = 0.2
        self.manager.TERM_TIMEOUT = 0.2
        self.events = []
        self.serial = 0
        self.addCleanup(self.tmp.cleanup)
        self.addCleanup(self.manager.close)

    def script(self, source):
        self.serial += 1
        path = self.root / f'job{self.serial}.py'
        path.write_text(source, encoding='utf-8')
        return [sys.executable, '-u', str(path)]

    def drain_until(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.events.extend(self.manager.poll())
            if predicate():
                return
            time.sleep(0.02)
        self.fail(f'condition timed out; events={self.events!r}')

    def exited(self, name):
        return any(e['kind'] == 'exited' and e['name'] == name for e in self.events)

    def output(self, text):
        return any(e['kind'] == 'output' and e['text'] == text for e in self.events)

    def test_output_stderr_persisted_and_exit_once(self):
        self.manager.launch('sample', self.script('import sys\nprint("hello 中文")\nprint("error line", file=sys.stderr)\nsys.exit(7)\n'), self.root)
        self.drain_until(lambda: self.exited('sample'))
        self.assertTrue(self.output('hello 中文'))
        self.assertTrue(self.output('error line'))
        self.assertEqual([e['returncode'] for e in self.events if e['kind'] == 'exited'], [7])
        for _ in range(5):
            self.events.extend(self.manager.poll())
        self.assertEqual(sum(e['kind'] == 'exited' for e in self.events), 1)
        self.assertEqual(self.manager.active_names(), [])
        self.assertFalse(self.manager.alive('sample'))
        logs = list((self.root / 'logs').glob('*.log'))
        self.assertEqual(len(logs), 1)
        self.assertIn('hello 中文', logs[0].read_text())
        self.assertIn('error line', logs[0].read_text())

    def test_service_result_burst_and_unterminated_line_precede_exit(self):
        success = 'WB_JSON:' + json.dumps({'kind': 'service', 'success': True})
        for newline in (False, True):
            name = f'service-{newline}'
            source = ('import sys\n'
                      'for i in range(200): print("log-"+str(i))\n'
                      f'sys.stdout.write({success!r} + {chr(10) if newline else ""!r})\n')
            self.manager.launch(name, self.script(source), self.root)
            self.drain_until(lambda: self.exited(name))
            events = [event for event in self.events if event['name'] == name]
            self.assertEqual(len([e for e in events if e['kind'] == 'output']), 201)
            self.assertEqual(events[-2]['text'], success)
            self.assertEqual(events[-2]['kind'], 'output')
            self.assertEqual(events[-1]['kind'], 'exited')
            self.assertEqual(events[-1]['returncode'], 0)

    def test_exit_waits_until_reader_queues_service_result(self):
        success = 'WB_JSON:' + json.dumps({'kind': 'service', 'success': True})
        blocked = threading.Event()
        release = threading.Event()
        self.addCleanup(release.set)
        original_put = self.manager._events.put

        def delayed_put(event, *args, **kwargs):
            if event['kind'] == 'output' and event['text'] == success:
                blocked.set()
                release.wait(timeout=3)
            return original_put(event, *args, **kwargs)

        with mock.patch.object(self.manager._events, 'put', side_effect=delayed_put):
            self.manager.launch('reader-gated', self.script(f'print({success!r})\n'), self.root)
            self.assertTrue(blocked.wait(timeout=2))
            # The OS process is already finished, but the reader has NOT yet
            # enqueued its result. poll must not synthesize exited early.
            self.manager._jobs['reader-gated'].process.wait(timeout=2)
            self.events.extend(self.manager.poll())
            self.assertFalse(self.exited('reader-gated'))
            release.set()
            self.drain_until(lambda: self.exited('reader-gated'))
        events = [event for event in self.events if event['name'] == 'reader-gated']
        self.assertEqual([event['kind'] for event in events], ['output', 'exited'])
        self.assertEqual(events[0]['text'], success)

    def test_environment_is_merged_without_mutating_parent(self):
        key = 'WORKBENCH_RUNTIME_TEST_VALUE'
        before = dict(os.environ)
        script = 'import os\nprint(os.environ["WORKBENCH_RUNTIME_TEST_VALUE"])\nprint("has-path="+str("PATH" in os.environ))\n'
        self.manager.launch('env', self.script(script), self.root, {key: 'isolated'})
        self.drain_until(lambda: self.exited('env'))
        self.assertTrue(self.output('isolated'))
        self.assertTrue(self.output('has-path=True'))
        self.assertEqual(dict(os.environ), before)

    def test_duplicate_live_name_and_reuse_finished_name(self):
        self.manager.launch('same', self.script('import time\nprint("ready")\ntime.sleep(60)\n'), self.root)
        self.drain_until(lambda: self.output('ready'))
        with self.assertRaises(RuntimeError):
            self.manager.launch('same', [sys.executable, '--version'], self.root)
        self.assertEqual(self.manager.active_names(), ['same'])
        self.manager.stop('same')
        self.drain_until(lambda: self.exited('same'))
        self.manager.launch('same', self.script('print("again")\n'), self.root)
        self.drain_until(lambda: self.output('again'))
        self.drain_until(lambda: sum(e['kind'] == 'exited' for e in self.events) == 2)
        self.assertEqual(len(list((self.root / 'logs').glob('same-*.log'))), 2)

    def test_normal_stop_is_nonblocking_and_graceful(self):
        source = ('import signal,time,sys\n'
                  'def stop(sig, frame):\n print("stopped"); sys.exit(0)\n'
                  'signal.signal(signal.SIGINT, stop)\nprint("ready")\ntime.sleep(60)\n')
        self.manager.launch('normal', self.script(source), self.root)
        self.drain_until(lambda: self.output('ready'))
        start = time.monotonic()
        self.manager.stop('normal')
        self.assertLess(time.monotonic() - start, 0.25)
        self.drain_until(lambda: self.exited('normal'))
        self.assertTrue(self.output('stopped'))
        self.assertEqual([e['returncode'] for e in self.events if e['kind'] == 'exited'], [0])

    def test_ignored_interrupt_and_terminate_are_killed(self):
        source = ('import signal,time\n'
                  'signal.signal(signal.SIGINT, signal.SIG_IGN)\n'
                  'signal.signal(signal.SIGTERM, signal.SIG_IGN)\n'
                  'print("ready")\ntime.sleep(60)\n')
        self.manager.launch('stubborn', self.script(source), self.root)
        self.drain_until(lambda: self.output('ready'))
        self.manager.stop('stubborn')
        self.drain_until(lambda: self.exited('stubborn'))
        self.assertEqual([e['returncode'] for e in self.events if e['kind'] == 'exited'], [-signal.SIGKILL])

    def test_signal_terminate_before_kill(self):
        source = ('import signal,time,sys\n'
                  'signal.signal(signal.SIGINT, signal.SIG_IGN)\n'
                  'def stop(sig, frame):\n print("term"); sys.exit(0)\n'
                  'signal.signal(signal.SIGTERM, stop)\nprint("ready")\ntime.sleep(60)\n')
        self.manager.launch('term', self.script(source), self.root)
        self.drain_until(lambda: self.output('ready'))
        self.manager.stop('term')
        self.drain_until(lambda: self.exited('term'))
        self.assertTrue(self.output('term'))
        self.assertEqual([e['returncode'] for e in self.events if e['kind'] == 'exited'], [0])

    def test_children_in_normal_and_detached_groups_cleaned(self):
        child = self.script('import signal,time\nsignal.signal(signal.SIGINT,signal.SIG_IGN)\nsignal.signal(signal.SIGTERM,signal.SIG_IGN)\nprint("child-ready")\ntime.sleep(60)\n')
        for detached in (False, True):
            with self.subTest(detached=detached):
                name = f'children-{detached}'
                source = ('import subprocess,time\n'
                          f'p=subprocess.Popen({child!r}, start_new_session={detached!r})\n'
                          'print("child-pid="+str(p.pid))\ntime.sleep(60)\n')
                self.manager.launch(name, self.script(source), self.root)
                self.drain_until(lambda: any(e['kind'] == 'output' and e['name'] == name and e['text'] == 'child-ready' for e in self.events))
                pid_event = next(e for e in self.events if e['name'] == name and e['text'].startswith('child-pid='))
                pid = int(pid_event['text'].split('=')[1])
                self.manager.stop(name)
                self.drain_until(lambda: self.exited(name))
                proc = runtime._read_proc(pid)
                self.assertTrue(proc is None or proc.state == 'Z')

    def test_parent_exit_does_not_leave_observed_detached_child(self):
        child = self.script('import time\nprint("child-ready")\ntime.sleep(60)\n')
        parent = ('import subprocess,time\n'
                  f'p=subprocess.Popen({child!r}, start_new_session=True)\n'
                  'print("child-pid="+str(p.pid))\ntime.sleep(0.25)\n')
        self.manager.launch('orphan', self.script(parent), self.root)
        self.drain_until(lambda: self.exited('orphan'))
        pid = int(next(e['text'].split('=')[1] for e in self.events if e['text'].startswith('child-pid=')))
        proc = runtime._read_proc(pid)
        self.assertTrue(proc is None or proc.state == 'Z')

    def test_immediate_parent_exit_recovers_detached_child_marker(self):
        child = self.script('import time\nprint("child-ready")\ntime.sleep(60)\n')
        # Parent exits before the periodic observer can see ancestry. Recovery
        # must use the inherited job marker, not process-name/global matching.
        parent = ('import subprocess\n'
                  f'p=subprocess.Popen({child!r}, start_new_session=True)\n'
                  'print("child-pid="+str(p.pid))\n')
        self.manager.launch('fast-orphan', self.script(parent), self.root)
        self.drain_until(lambda: self.exited('fast-orphan'))
        pid = int(next(e['text'].split('=')[1] for e in self.events if e['text'].startswith('child-pid=')))
        proc = runtime._read_proc(pid)
        self.assertTrue(proc is None or proc.state == 'Z')

    def test_unrelated_process_is_not_stopped(self):
        unrelated = subprocess.Popen(self.script('import time\ntime.sleep(60)\n'), start_new_session=True)
        try:
            self.manager.launch('owned', self.script('import time\nprint("ready")\ntime.sleep(60)\n'), self.root)
            self.drain_until(lambda: self.output('ready'))
            self.manager.stop_all()
            self.drain_until(lambda: self.exited('owned'))
            self.assertIsNone(unrelated.poll())
        finally:
            unrelated.kill()
            unrelated.wait(timeout=2)

    def test_pid_reuse_is_not_signalled(self):
        job = runtime._Job('fake', mock.Mock(), self.root / 'none', mock.Mock(), owned={999999: 123})
        replacement = runtime._Proc(999999, 1, 999999, 999999, 456, 'S')
        with mock.patch.object(runtime, '_read_proc', return_value=replacement), \
             mock.patch.object(runtime.os, 'kill') as kill:
            self.manager._signal(job, signal.SIGKILL)
        kill.assert_not_called()

    def test_close_is_bounded_and_preserves_events(self):
        source = ('import signal,time\n'
                  'signal.signal(signal.SIGINT,signal.SIG_IGN)\n'
                  'signal.signal(signal.SIGTERM,signal.SIG_IGN)\n'
                  'print("ready")\ntime.sleep(60)\n')
        self.manager.launch('close', self.script(source), self.root)
        self.drain_until(lambda: self.output('ready'))
        start = time.monotonic()
        self.manager.close(timeout=1.0)
        self.assertLess(time.monotonic() - start, 1.25)
        self.events.extend(self.manager.poll())
        self.assertTrue(self.exited('close'))
        self.assertEqual(self.manager.active_names(), [])
        self.assertFalse(self.manager._observer.is_alive())
        with self.assertRaises(RuntimeError):
            self.manager.launch('late', [sys.executable, '--version'], self.root)
        self.manager.close()

    def test_failed_launch_closes_log_resources(self):
        with self.assertRaises(FileNotFoundError):
            self.manager.launch('bad', ['/definitely/not/a/command'], self.root)
        self.assertEqual(self.manager.active_names(), [])
        self.assertEqual(self.manager.poll(), [])

    def test_unknown_stop_is_harmless(self):
        self.manager.stop('does-not-exist')
        self.assertFalse(self.manager.alive('does-not-exist'))
        self.assertEqual(self.manager.poll(), [])


if __name__ == '__main__':
    unittest.main()
