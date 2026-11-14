"""Distribution helpers; only temporary HOME/XDG paths and fake interpreters.

Run: /usr/bin/python3 -m unittest discover -s workbench/test -p test_distribution.py -v
"""
from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import types
import textwrap
import time
import unittest
from unittest import mock

REPO = Path(__file__).resolve().parents[2]
DESKTOP_NAME = "jaka-competition-workbench.desktop"


class DistributionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="workbench-distribution-")
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.root = self.base / '项目 空格 " $HOME $(touch INJECTED) `echo bad` %f \\路径'
        (self.root / "setup").mkdir(parents=True)
        for filename in ("run-workbench.sh", "install-workbench-desktop.sh"):
            target = self.root / "setup" / filename
            shutil.copy2(REPO / "setup" / filename, target)
            target.chmod(0o755)
        self.home = self.base / "临时 home"
        self.home.mkdir()
        self.desktop = self.home / "桌面 空格"
        self.desktop.mkdir()
        self.bin = self.base / "test-bin"
        self.bin.mkdir()
        self.capture = self.base / "capture.json"
        self.env = os.environ.copy()
        self.env.update({"HOME": str(self.home), "XDG_DATA_HOME": str(self.home / "应用数据"),
                         "PATH": str(self.bin) + ":" + os.environ.get("PATH", "/usr/bin:/bin"),
                         "WB_CAPTURE": str(self.capture), "WB_DESKTOP": str(self.desktop)})
        self.env.pop("WORKBENCH_PYTHON", None)
        self.env.pop("PYTHONPATH", None)
        xdg = self.bin / "xdg-user-dir"
        xdg.write_text('#!/bin/sh\nprintf "%s\\n" "$WB_DESKTOP"\n', encoding="utf-8")
        xdg.chmod(0o755)
        self.python = self.bin / "fake python"
        self.python.write_text(textwrap.dedent('''\
            #!/usr/bin/python3
            import json, os, pathlib, sys
            if sys.argv[1:2] == ["-c"]:
                sys.exit(int(os.environ.get("WB_NO_QT", "0")))
            pathlib.Path(os.environ["WB_CAPTURE"]).write_text(json.dumps({
                "cwd": os.getcwd(), "args": sys.argv[1:]
            }))
            sys.exit(int(os.environ.get("WB_EXIT", "0")))
            '''), encoding="utf-8")
        self.python.chmod(0o755)

    def invoke(self, name, *args, env=None):
        return subprocess.run(["bash", str(self.root / "setup" / name), *args],
                              cwd=self.base, env=env or self.env, text=True,
                              capture_output=True, timeout=15)

    def install(self):
        result = self.invoke("install-workbench-desktop.sh")
        self.assertEqual(result.returncode, 0, result.stderr)
        return Path(self.env["XDG_DATA_HOME"]) / "applications" / DESKTOP_NAME

    def test_launcher_cwd_arguments_and_exit_code(self):
        self.env.update(WORKBENCH_PYTHON=str(self.python), WB_EXIT="17")
        args = ["--root", "目录 带空格", "--data-dir", "$(touch INJECTED)",
                "--mode", "basic", "--demo", 'quote"; & %f']
        result = self.invoke("run-workbench.sh", *args)
        self.assertEqual(result.returncode, 17, result.stderr)
        data = json.loads(self.capture.read_text())
        self.assertEqual(data, {"cwd": str(self.root), "args": ["-m", "workbench", *args]})
        self.assertFalse((self.root / "INJECTED").exists())
        self.assertFalse((self.base / "INJECTED").exists())

    def test_check_and_help_do_not_require_qt(self):
        self.env.update(WORKBENCH_PYTHON=str(self.python), WB_NO_QT="1")
        for option in ("--check", "--vm-check", "--help"):
            result = self.invoke("run-workbench.sh", option)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(self.capture.read_text())["args"],
                             ["-m", "workbench", option])

    def test_missing_qt_gives_manual_instruction(self):
        self.env.update(WORKBENCH_PYTHON=str(self.python), WB_NO_QT="1")
        result = self.invoke("run-workbench.sh")
        self.assertEqual(result.returncode, 1)
        self.assertIn("sudo apt install python3-pyqt5", result.stderr)
        self.assertFalse(self.capture.exists())

    def test_missing_python(self):
        self.env["WORKBENCH_PYTHON"] = str(self.base / "missing-python")
        result = self.invoke("run-workbench.sh", "--check")
        self.assertEqual(result.returncode, 127)
        self.assertIn("找不到 Python", result.stderr)

    def test_installer_user_only_and_repeatable(self):
        entry = self.install()
        first = entry.read_bytes()
        self.assertEqual(first, (self.desktop / DESKTOP_NAME).read_bytes())
        self.assertTrue(entry.stat().st_mode & 0o111)
        self.assertIn("Terminal=false", entry.read_text())
        self.assertIn("Name=基础组学习工作台", entry.read_text())
        self.assertEqual(self.install().read_bytes(), first)
        self.assertFalse(self.capture.exists())

    def test_does_not_create_missing_desktop_directory(self):
        self.desktop.rmdir()
        self.assertTrue(self.install().is_file())
        self.assertFalse(self.desktop.exists())

    def test_no_desktop_file_in_home_when_desktop_disabled(self):
        self.env["WB_DESKTOP"] = str(self.home)
        self.install()
        self.assertFalse((self.home / DESKTOP_NAME).exists())

    def test_refuses_unrelated_existing_entry(self):
        entry = Path(self.env["XDG_DATA_HOME"]) / "applications" / DESKTOP_NAME
        entry.parent.mkdir(parents=True)
        entry.write_text("[Desktop Entry]\nName=Other software\n", encoding="utf-8")
        result = self.invoke("install-workbench-desktop.sh")
        self.assertEqual(result.returncode, 1)
        self.assertIn("Other software", entry.read_text())

    def test_refuses_symlink(self):
        entry = Path(self.env["XDG_DATA_HOME"]) / "applications" / DESKTOP_NAME
        entry.parent.mkdir(parents=True)
        original = self.base / "do-not-change"
        original.write_text("keep", encoding="utf-8")
        entry.symlink_to(original)
        result = self.invoke("install-workbench-desktop.sh")
        self.assertEqual(result.returncode, 1)
        self.assertEqual(original.read_text(), "keep")

    def test_rejects_relative_xdg_path(self):
        self.env["XDG_DATA_HOME"] = "relative-location"
        result = self.invoke("install-workbench-desktop.sh")
        self.assertEqual(result.returncode, 1)
        self.assertFalse((self.base / "relative-location").exists())

    def test_desktop_exec_roundtrip_with_gio(self):
        # GIO parses the actual Desktop Entry grammar, not shell/shlex syntax.
        probe = subprocess.run(["/usr/bin/python3", "-c", "from gi.repository import Gio"],
                               capture_output=True)
        if probe.returncode:
            self.skipTest("optional python3-gi unavailable; other tests need no GI")
        entry = self.install()
        self.env["WORKBENCH_PYTHON"] = str(self.python)
        script = '''from gi.repository import Gio
import sys
app = Gio.DesktopAppInfo.new_from_filename(sys.argv[1])
assert app is not None, "invalid desktop file"
assert app.launch([], None)
'''
        result = subprocess.run(["/usr/bin/python3", "-c", script, str(entry)],
                                env=self.env, cwd=self.base, capture_output=True,
                                text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        for _ in range(100):
            if self.capture.exists():
                break
            time.sleep(0.02)
        self.assertTrue(self.capture.exists(), "desktop launcher did not execute")
        self.assertEqual(json.loads(self.capture.read_text()),
                         {"cwd": str(self.root), "args": ["-m", "workbench"]})
        self.assertFalse((self.root / "INJECTED").exists())
        self.assertFalse((self.base / "INJECTED").exists())


class StudentExampleTests(unittest.TestCase):
    def load_module(self, name, filename):
        spec = importlib.util.spec_from_file_location(name, REPO / filename)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def setUp(self):
        qr = self.load_module("wb_test_real_qr", "jaka_competition_kit/jaka_competition_kit/qr.py")
        self.decode = mock.Mock(return_value="JAKA1-246135")
        qr.decode_image = self.decode
        self.base = type("Track1Reference", (), {"run": lambda self: "inherited-wait-flow"})
        reference = types.ModuleType("jaka_competition_kit.reference")
        reference.spin = mock.Mock()
        track = types.ModuleType("jaka_competition_kit.ref_track1")
        track.Track1Reference = self.base
        with mock.patch.dict("sys.modules", {
            "rclpy": types.ModuleType("rclpy"),
            "jaka_competition_kit": types.ModuleType("jaka_competition_kit"),
            "jaka_competition_kit.qr": qr,
            "jaka_competition_kit.reference": reference,
            "jaka_competition_kit.ref_track1": track,
        }):
            module = self.load_module("wb_test_student", "workbench/examples/student_track1.py")
        self.student = module.StudentTrack1()
        self.logger = mock.Mock()
        self.student.get_logger = lambda: self.logger
        # Deliberately omit qr_order and qr_payload: they must not be required.
        self.student.scene = {"qr_png": "/fake/training-qr.png",
                              "objects": [{"station": n} for n in range(1, 7)]}

    def test_student_decodes_image_and_keeps_correct_order(self):
        result = self.student.plan_order()
        self.assertEqual([obj["station"] for obj in result], [2, 4, 6, 1, 3, 5])
        self.decode.assert_called_once_with("/fake/training-qr.png")
        self.assertIs(self.student.__class__.run, self.base.run)

    def test_student_invalid_qr_does_not_move(self):
        self.decode.return_value = "JAKA1-111111"
        self.assertEqual(self.student.plan_order(), [])
        self.logger.error.assert_called_once()

    def test_student_missing_station_does_not_move(self):
        self.student.scene["objects"].pop()
        self.assertEqual(self.student.plan_order(), [])
        self.logger.error.assert_called_once()

    def test_student_missing_png_does_not_read_truth(self):
        self.student.scene.update(qr_png="", qr_order=[1, 2, 3, 4, 5, 6], qr_payload="JAKA1-123456")
        self.assertEqual(self.student.plan_order(), [])
        self.decode.assert_not_called()

    def test_pure_python_exercise(self):
        exercise = self.load_module("wb_test_order_exercise", "workbench/examples/qr_order_exercise.py")
        self.assertEqual(exercise.parse_order(" JAKA1-246135 "), [2, 4, 6, 1, 3, 5])
        for payload in ("JAKA1-111111", "JAKA1-12345", "BAD-246135", "JAKA1-１２３４５６"):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                exercise.parse_order(payload)


if __name__ == "__main__":
    unittest.main()
