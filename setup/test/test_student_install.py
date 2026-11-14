"""No sudo, networking, APT, build or robot commands are used in these tests."""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tarfile
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[2]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


si = load('student_install', ROOT / 'setup/student_install.py')
pkg = load('student_package', ROOT / 'setup/package-student.py')


@pytest.mark.parametrize('args, code', [([], 0), (['--plan'], 0), (['--help'], 0),
                                      (['--invalid'], 2), (['--plan', '--install'], 2)])
def test_entrypoint_plan_never_installs(tmp_path, args, code):
    env = {'HOME': str(tmp_path), 'PATH': '/usr/bin:/bin'}
    result = subprocess.run(['bash', str(ROOT / 'setup/install-student.sh'), *args],
                            env=env, text=True, capture_output=True, timeout=5)
    assert result.returncode == code
    assert list(tmp_path.iterdir()) == []


def host(monkeypatch):
    monkeypatch.setattr(si.os, 'geteuid', lambda: 1000)
    monkeypatch.setattr(si.platform, 'freedesktop_os_release', lambda: {'ID': 'ubuntu', 'VERSION_ID': '22.04'})
    monkeypatch.setattr(si.platform, 'machine', lambda: 'x86_64')
    monkeypatch.setattr(si.shutil, 'disk_usage', lambda _: SimpleNamespace(free=30 * 1024 ** 3))
    monkeypatch.setattr(si.shutil, 'which', lambda _: '/usr/bin/sudo')


def test_supported_host(monkeypatch):
    host(monkeypatch)
    si.validate_host(ROOT)


@pytest.mark.parametrize('kind, message', [('root', '普通账户'), ('arch', '仅支持'),
                                          ('os', '仅支持'), ('disk', '空间不足'), ('sudo', '缺少 sudo')])
def test_host_fail_closed(monkeypatch, kind, message):
    host(monkeypatch)
    if kind == 'root':
        monkeypatch.setattr(si.os, 'geteuid', lambda: 0)
    elif kind == 'arch':
        monkeypatch.setattr(si.platform, 'machine', lambda: 'aarch64')
    elif kind == 'os':
        monkeypatch.setattr(si.platform, 'freedesktop_os_release', lambda: {'ID': 'ubuntu', 'VERSION_ID': '24.04'})
    elif kind == 'disk':
        monkeypatch.setattr(si.shutil, 'disk_usage', lambda _: SimpleNamespace(free=1))
    else:
        monkeypatch.setattr(si.shutil, 'which', lambda _: None)
    with pytest.raises(RuntimeError, match=message):
        si.validate_host(ROOT)


def test_hash_validation(tmp_path):
    p = tmp_path / 'source.deb'
    p.write_bytes(b'not a real deb')
    si.verify_sha256(p, hashlib.sha256(p.read_bytes()).hexdigest())
    with pytest.raises(RuntimeError, match='SHA-256'):
        si.verify_sha256(p, '0' * 64)


def key_lines(primary, subkey='other'):
    return f'pub:any\nfpr:::::::::{primary}:\nsub:any\nfpr:::::::::{subkey}:\n'


def test_key_validation_checks_primary_not_subkey():
    si.verify_gazebo_key(key_lines(si.GAZEBO_FINGERPRINT))
    for value in ('', key_lines('evil', si.GAZEBO_FINGERPRINT),
                  key_lines(si.GAZEBO_FINGERPRINT) + key_lines('evil')):
        with pytest.raises(RuntimeError, match='密钥指纹'):
            si.verify_gazebo_key(value)


def test_existing_sources_and_comments(tmp_path):
    (tmp_path / 'sources.list.d').mkdir()
    (tmp_path / 'sources.list').write_text('# packages.ros.org/ros2/ubuntu\n')
    assert not si.source_exists('packages.ros.org/ros2/ubuntu', tmp_path)
    (tmp_path / 'sources.list.d/ros.sources').write_text('Types: deb\nURIs: https://packages.ros.org/ros2/ubuntu\n')
    assert si.source_exists('packages.ros.org/ros2/ubuntu', tmp_path)


def installer(tmp_path):
    logs = tmp_path / 'logs'
    logs.mkdir()
    return si.Installer(ROOT, logs)


def test_environment_does_not_inherit_overlay(monkeypatch, tmp_path):
    for key in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'ROS_DISTRO', 'VIRTUAL_ENV'):
        monkeypatch.setenv(key, 'bad-environment')
    monkeypatch.setenv('DISPLAY', ':42')
    i = installer(tmp_path)
    assert i.env['ROS_DISTRO'] == 'humble'
    assert i.env['DISPLAY'] == ':42'
    assert 'PYTHONPATH' not in i.env
    assert 'AMENT_PREFIX_PATH' not in i.env
    assert i.env['MAKEFLAGS'] == '-j2'
    i.log.close()


def test_install_order_no_global_patch_or_launch(monkeypatch, tmp_path):
    i = installer(tmp_path)
    calls = []
    def run(args, **kw):
        calls.append([str(x) for x in args])
        return SimpleNamespace(stdout='{}', returncode=0)
    monkeypatch.setattr(i, 'run', run)
    monkeypatch.setattr(i, 'configure_sources', lambda _: calls.append(['sources']))
    i.install()
    flat = '\n'.join(' '.join(c) for c in calls)
    assert flat.index('sources') < flat.index('ros-humble-desktop') < flat.index('install-deps.sh')
    assert flat.index('build.sh --student') < flat.index('install-workbench-desktop.sh') < flat.index('--vm-check')
    assert 'patch-moveit' not in flat and 'ros2 launch' not in flat
    assert 'apt-get -o Acquire::Retries=3 upgrade' not in flat
    assert (i.log_dir / 'vm-preflight.json').read_text() == '{}'
    i.log.close()


def test_apt_failure_never_builds(monkeypatch, tmp_path):
    i = installer(tmp_path)
    calls = []
    def run(args, **kw):
        calls.append(args)
        raise RuntimeError('APT failed')
    monkeypatch.setattr(i, 'run', run)
    with pytest.raises(RuntimeError, match='APT failed'):
        i.install()
    assert len(calls) == 1
    i.log.close()


def test_bad_ros_download_never_installed(monkeypatch, tmp_path):
    i = installer(tmp_path)
    monkeypatch.setattr(si, 'source_exists', lambda _: False)
    monkeypatch.setattr(i, 'download', lambda url, path: path.write_bytes(b'bad download'))
    run = Mock()
    monkeypatch.setattr(i, 'run', run)
    with pytest.raises(RuntimeError, match='SHA-256'):
        i.configure_sources(tmp_path)
    run.assert_not_called()
    i.log.close()


def test_existing_sources_not_overwritten(monkeypatch, tmp_path):
    i = installer(tmp_path)
    monkeypatch.setattr(si, 'source_exists', lambda _: True)
    run, download = Mock(), Mock()
    monkeypatch.setattr(i, 'run', run)
    monkeypatch.setattr(i, 'download', download)
    i.configure_sources(tmp_path)
    run.assert_not_called()
    download.assert_not_called()
    i.log.close()


def test_preflight_failure_not_success(monkeypatch, tmp_path):
    i = installer(tmp_path)
    def run(args, **kw):
        return SimpleNamespace(stdout='{"error":"no DISPLAY"}', returncode=int('--vm-check' in args))
    monkeypatch.setattr(i, 'run', run)
    monkeypatch.setattr(i, 'configure_sources', lambda _: None)
    with pytest.raises(RuntimeError, match='预检未通过'):
        i.install()
    assert json.loads((i.log_dir / 'vm-preflight.json').read_text())['error'] == 'no DISPLAY'
    i.log.close()


@pytest.mark.parametrize('name', ['jaka_ros2/build/x', '.git/config', 'workbench/__pycache__/x.pyc',
                                  'perception/track1_sim/runs/weights.pt', 'runs/private.json',
                                  'setup/../../secret', '/setup/foo'])
def test_distribution_excludes_developer_data(name):
    assert not pkg.selected(name)


def test_distribution_source_list():
    names = pkg.collect(ROOT)
    assert 'workbench/theme.py' in names
    assert 'workbench/test/test_ui_layout.py' in names
    assert 'setup/install-student.sh' in names
    assert 'jaka_ros2/src/jaka_competition_kit' in names
    assert 'jaka_competition_kit/config/track1_simulation_example.yaml' in names
    assert all(pkg.selected(name) for name in names)


def test_archive_manifest_links_hash_and_no_overwrite(monkeypatch, tmp_path):
    root = tmp_path / 'src'
    root.mkdir()
    (root / 'example.py').write_text('print(123)\n')
    (root / 'alias.py').symlink_to('example.py')
    monkeypatch.setattr(pkg, 'collect', lambda _: ['example.py', 'alias.py'])
    monkeypatch.setattr(pkg.subprocess, 'check_output', lambda *a, **kw: 'abc123\n' if kw.get('text') else b' M example.py')
    archive = pkg.write_package(root, tmp_path / 'out', 'test-candidate')
    with tarfile.open(archive) as tar:
        assert tar.getmember('jaka-student/alias.py').issym()
        assert tar.getmember('jaka-student/alias.py').linkname == 'example.py'
        manifest = json.load(tar.extractfile('jaka-student/PACKAGE-MANIFEST.json'))
        assert manifest['student_vm_acceptance'] == 'not_run'
        assert manifest['working_tree_modified']
        assert manifest['files']['example.py']['sha256'] == hashlib.sha256(b'print(123)\n').hexdigest()
        assert 'bash setup/install-student.sh --install' in tar.extractfile('jaka-student/INSTALL.txt').read().decode()
    assert archive.with_name(archive.name + '.sha256').read_text().startswith(hashlib.sha256(archive.read_bytes()).hexdigest())
    with pytest.raises(ValueError, match='覆盖'):
        pkg.write_package(root, archive.parent, 'test-candidate')


def test_external_symlink_rejected(monkeypatch, tmp_path):
    (tmp_path / 'setup').mkdir()
    (tmp_path / 'setup/foo').symlink_to('/etc/passwd')
    monkeypatch.setattr(pkg, 'ADDITIONS', set())
    monkeypatch.setattr(pkg.subprocess, 'check_output', lambda *a, **kw: b'setup/foo\0')
    with pytest.raises(ValueError, match='符号链接'):
        pkg.collect(tmp_path)
