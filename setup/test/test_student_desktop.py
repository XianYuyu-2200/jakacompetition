"""Exercise desktop parsing with GIO and a fake installer, never APT/sudo."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('student_desktop', ROOT / 'setup/student_install_desktop.py')
desktop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(desktop)


def test_distributed_launcher_matches_generator():
    path = ROOT / desktop.LAUNCHER
    assert path.read_text() == desktop.desktop_content()
    assert os.access(path, os.X_OK)
    assert 'Terminal=true' in path.read_text()
    assert '/home/ktz' not in path.read_text()


@pytest.mark.parametrize('code', [0, 1, 17, -15])
def test_install_result_preserved(tmp_path, monkeypatch, capsys, code):
    (tmp_path / 'setup').mkdir()
    (tmp_path / 'setup/install-student.sh').touch()
    runner = Mock(return_value=SimpleNamespace(returncode=code))
    monkeypatch.setattr(desktop.subprocess, 'run', runner)
    assert desktop.install(tmp_path) == (code if code >= 0 else 128 - code)
    runner.assert_called_once_with(['/bin/bash', str(tmp_path / 'setup/install-student.sh'), '--install'],
                                   cwd=tmp_path, check=False)
    out = capsys.readouterr().out
    assert ('安装和预检已完成' in out) == (code == 0)
    assert ('安装未完成（退出码' in out) == (code != 0)


def test_missing_script_does_not_run(tmp_path, monkeypatch):
    runner = Mock(side_effect=AssertionError('must not launch'))
    monkeypatch.setattr(desktop.subprocess, 'run', runner)
    assert desktop.install(tmp_path) == 1
    runner.assert_not_called()


def fake_payload(root, capture):
    (root / 'setup').mkdir(parents=True)
    (root / 'setup/student_install_desktop.py').write_text(
        'from pathlib import Path\n'
        'def install():\n'
        f'    Path({str(capture)!r}).write_text({str(root)!r})\n'
        '    return 0\n')


@pytest.mark.parametrize('copied', [False, True])
def test_actual_gio_exec_round_trip(tmp_path, copied):
    probe = subprocess.run(['/usr/bin/python3', '-c', 'from gi.repository import Gio'], capture_output=True)
    if probe.returncode:
        pytest.skip('GIO unavailable')
    home = tmp_path / 'home'
    home.mkdir()
    root = home / 'jaka-student' if copied else tmp_path / '中文 dir " $` touch INJECTED ` % stuff'
    capture = tmp_path / 'capture'
    fake_payload(root, capture)
    parent = home / 'Desktop' if copied else root
    parent.mkdir(exist_ok=True)
    launcher = parent / desktop.LAUNCHER
    # Only disable opening a terminal for this parser/execution test; payload is fake.
    launcher.write_text(desktop.desktop_content().replace('Terminal=true', 'Terminal=false'))
    code = '''from gi.repository import Gio
import sys
app = Gio.DesktopAppInfo.new_from_filename(sys.argv[1])
assert app is not None
assert app.launch([], None)
'''
    result = subprocess.run(['/usr/bin/python3', '-c', code, str(launcher)],
                            env=dict(os.environ, HOME=str(home)), cwd=tmp_path,
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    for _ in range(100):
        if capture.exists():
            break
        time.sleep(.02)
    assert capture.read_text() == str(root)
    assert not list(tmp_path.rglob('INJECTED'))


@pytest.mark.parametrize('uri', [False, True])
def test_bootstrap_local_uri_or_path(tmp_path, uri):
    root = tmp_path / '学生 % data'
    capture = tmp_path / 'capture'
    fake_payload(root, capture)
    launcher = root / desktop.LAUNCHER
    argument = launcher.as_uri() if uri else str(launcher)
    result = subprocess.run(['/usr/bin/python3', '-c', desktop.BOOTSTRAP, argument],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stdout + result.stderr
    assert capture.read_text() == str(root)


def test_bootstrap_missing_payload_errors(tmp_path):
    result = subprocess.run(['/usr/bin/python3', '-c', desktop.BOOTSTRAP,
                             str(tmp_path / desktop.LAUNCHER)],
                            env=dict(os.environ, HOME=str(tmp_path)),
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 1
    assert '找不到安装包' in result.stdout


def test_bootstrap_failure_keeps_terminal_open(monkeypatch, tmp_path, capsys):
    # Exercise the actual terminal branch without needing a GUI or PTY.
    import builtins
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.setattr('sys.argv', ['desktop', str(tmp_path / desktop.LAUNCHER)])
    monkeypatch.setattr('sys.stdin', SimpleNamespace(isatty=lambda: True))
    pause = Mock(return_value='')
    monkeypatch.setattr(builtins, 'input', pause)
    with pytest.raises(SystemExit) as result:
        exec(desktop.BOOTSTRAP, {})
    assert result.value.code == 1
    pause.assert_called_once()
    assert '按回车' in pause.call_args.args[0]
