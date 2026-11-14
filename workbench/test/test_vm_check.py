"""VM diagnostics are bounded/read-only and must not claim simulation acceptance."""
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from workbench import vm_check, runtime
from workbench.__main__ import main


@pytest.mark.parametrize('version,level', [('2.1', 'error'), ('3.2', 'error'),
    ('3.3', 'warning'), ('4.1', 'warning'), ('4.3', 'ok'), ('4.6', 'ok')])
def test_gl_version(version, level):
    result = {'returncode': 0, 'stdout': 'OpenGL core profile version string: ' + version}
    assert vm_check.graphics_checks(result)[0]['level'] == level


def test_core_profile_preferred_to_legacy():
    result = {'returncode': 0, 'stdout': 'OpenGL version string: 2.1\n'
              'OpenGL core profile version string: 4.5 (Core Profile) Mesa'}
    assert vm_check.graphics_checks(result)[0]['level'] == 'ok'


def test_software_rendering_is_explicit_warning():
    result = {'returncode': 0, 'stdout': 'OpenGL version string: 4.5\n'
              'OpenGL renderer string: llvmpipe (LLVM 15)'}
    assert vm_check.graphics_checks(result)[1]['level'] == 'warning'


@pytest.mark.parametrize('code,text,level', [(1, '', 'error'), (None, '', 'error'),
                                           (0, 'unknown output', 'warning')])
def test_unusable_or_unknown_gl(code, text, level):
    assert vm_check.graphics_checks({'returncode': code, 'stdout': text})[0]['level'] == level


@pytest.mark.parametrize('failure', [FileNotFoundError('missing'),
    subprocess.TimeoutExpired(['glxinfo', '-B'], 8)])
def test_probe_failure_bounded(monkeypatch, failure):
    runner = Mock(side_effect=failure)
    monkeypatch.setattr(vm_check.subprocess, 'run', runner)
    assert vm_check.probe(['glxinfo', '-B'])['returncode'] is None
    assert runner.call_args.kwargs['timeout'] == 8
    assert not runner.call_args.kwargs.get('shell', False)


def test_probe_limits_output_and_does_not_change_environment(monkeypatch):
    monkeypatch.setenv('LC_ALL', 'original')
    runner = Mock(return_value=SimpleNamespace(returncode=0, stdout='x'*20000, stderr='e'*3000))
    monkeypatch.setattr(vm_check.subprocess, 'run', runner)
    result = vm_check.probe(['literal;not-shell'])
    assert len(result['stdout']) == 16000 and len(result['stderr']) == 2000
    assert vm_check.os.environ['LC_ALL'] == 'original'
    assert runner.call_args.args[0] == ['literal;not-shell']


def mock_system(monkeypatch, tmp_path):
    monkeypatch.setattr(vm_check.platform, 'freedesktop_os_release',
                        lambda: {'ID': 'ubuntu', 'VERSION_ID': '22.04', 'PRETTY_NAME': 'Ubuntu 22.04'})
    monkeypatch.setattr(vm_check.platform, 'machine', lambda: 'x86_64')
    monkeypatch.setattr(vm_check.os, 'cpu_count', lambda: 4)
    monkeypatch.setattr(vm_check.shutil, 'disk_usage', lambda _: SimpleNamespace(free=25*vm_check.GIB))
    monkeypatch.delenv('WAYLAND_DISPLAY', raising=False)
    monkeypatch.setenv('DISPLAY', ':12')


def test_no_display_never_launches_graphics_probe(monkeypatch, tmp_path):
    mock_system(monkeypatch, tmp_path)
    monkeypatch.delenv('DISPLAY')
    monkeypatch.setattr(vm_check.shutil, 'which', lambda name: '/usr/bin/glxinfo' if name == 'glxinfo' else None)
    runner = Mock(side_effect=AssertionError('must not launch'))
    monkeypatch.setattr(vm_check, 'probe', runner)
    report = vm_check.collect_vm_report(tmp_path)
    runner.assert_not_called()
    assert any(c['level'] == 'error' and c['name'] == 'X11 图形会话' for c in report['checks'])
    assert report['simulation_acceptance'] == report['hardware_acceptance'] == 'not_run'


def test_guest_probe_missing_tools_and_wrong_target(monkeypatch, tmp_path):
    mock_system(monkeypatch, tmp_path)
    monkeypatch.setattr(vm_check.platform, 'machine', lambda: 'aarch64')
    monkeypatch.setattr(vm_check.platform, 'freedesktop_os_release', lambda: {'ID': 'ubuntu', 'VERSION_ID': '24.04'})
    monkeypatch.setattr(vm_check.shutil, 'which', lambda _: None)
    report = vm_check.collect_vm_report(tmp_path)
    levels = {c['name']: c['level'] for c in report['checks']}
    assert levels['教学镜像系统'] == levels['教学镜像架构'] == levels['OpenGL 检查工具'] == 'error'


def test_successful_probe_still_not_simulation_acceptance(monkeypatch, tmp_path):
    mock_system(monkeypatch, tmp_path)
    monkeypatch.setattr(vm_check.shutil, 'which', lambda name: '/usr/bin/' + name)
    runner = Mock(return_value={'returncode': 0, 'stdout': 'OpenGL core profile version string: 4.6', 'stderr': ''})
    monkeypatch.setattr(vm_check, 'probe', runner)
    report = vm_check.collect_vm_report(tmp_path)
    assert report['simulation_acceptance'] == 'not_run'
    commands = [call.args[0] for call in runner.call_args_list]
    assert all(Path(c[0]).name in ('systemd-detect-virt', 'glxinfo', 'dpkg-query', 'git') for c in commands)
    assert not any('ros2' in ' '.join(c) for c in commands)


@pytest.mark.parametrize('vm_errors,expected', [(False, 0), (True, 1)])
def test_cli_vm_check_no_qt_or_simulation(monkeypatch, tmp_path, capsys, vm_errors, expected):
    monkeypatch.setattr(runtime, 'discover_root', lambda *a: tmp_path)
    monkeypatch.setattr(runtime, 'check_environment', lambda *a: [])
    monkeypatch.setattr(runtime, 'simulation_conflicts', lambda: [])
    monkeypatch.setattr(vm_check, 'collect_vm_report', lambda _: {
        'checks': [{'name': 'probe', 'level': 'error' if vm_errors else 'ok', 'detail': 'test'}],
        'simulation_acceptance': 'not_run'})
    assert main(['--vm-check']) == expected
    assert json.loads(capsys.readouterr().out)['vm']['simulation_acceptance'] == 'not_run'


def test_plain_check_stays_metadata_only(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(runtime, 'discover_root', lambda *a: tmp_path)
    monkeypatch.setattr(runtime, 'check_environment', lambda *a: [])
    monkeypatch.setattr(runtime, 'simulation_conflicts', lambda: [])
    probe = Mock(side_effect=AssertionError('unexpected subprocess'))
    monkeypatch.setattr(vm_check, 'collect_vm_report', probe)
    assert main(['--check']) == 0
    probe.assert_not_called()


@pytest.mark.parametrize('args,code', [([], 0), (['--plan'], 0), (['--invalid'], 2),
                                      (['--apply', 'extra'], 2)])
def test_provisioning_default_never_executes_installation(tmp_path, args, code):
    import shutil
    source = Path(__file__).resolve().parents[2] / 'setup/prepare-student-vm.sh'
    setup = tmp_path / 'setup'
    setup.mkdir()
    shutil.copy2(source, setup / source.name)
    # Deliberately omit installer/build/launcher. A plan must work without them.
    result = subprocess.run(['bash', str(setup / source.name), *args], capture_output=True,
                            text=True, timeout=5)
    assert result.returncode == code
    assert not (tmp_path / 'jaka_ros2').exists()


def test_missing_python_dependencies_are_errors(monkeypatch, tmp_path):
    mock_system(monkeypatch, tmp_path)
    monkeypatch.setattr(vm_check.shutil, 'which', lambda _: None)
    monkeypatch.setattr(vm_check.importlib.util, 'find_spec', lambda _: None)
    report = vm_check.collect_vm_report(tmp_path)
    missing = [c for c in report['checks'] if c['name'].startswith('教学 Python 依赖 ')]
    assert len(missing) == 6 and all(c['level'] == 'error' for c in missing)


def test_graphics_overrides_are_reported_not_modified(monkeypatch, tmp_path):
    mock_system(monkeypatch, tmp_path)
    monkeypatch.setattr(vm_check.shutil, 'which', lambda _: None)
    monkeypatch.setenv('MESA_GL_VERSION_OVERRIDE', '4.6')
    report = vm_check.collect_vm_report(tmp_path)
    assert any(c['name'] == '图形环境覆盖' and c['level'] == 'warning' for c in report['checks'])
    assert vm_check.os.environ['MESA_GL_VERSION_OVERRIDE'] == '4.6'
