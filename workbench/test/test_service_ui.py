"""Service/UI race regressions using offscreen Qt, never ROS or real jobs.

Run: /usr/bin/python3 -m unittest discover -s workbench/test -p test_service_ui.py -v
"""
from __future__ import annotations

import json
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import MagicMock, patch

from PyQt5.QtWidgets import QApplication, QMessageBox

from workbench.app import WorkbenchWindow
from workbench.ros_bridge import PREFIX
from workbench.runtime import ProcessManager


class ServiceUIRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.qt = QApplication.instance() or QApplication([])
        cls.root = Path(__file__).resolve().parents[2]

    def setUp(self):
        self.directory = TemporaryDirectory(prefix="workbench-ui-service-")
        self.addCleanup(self.directory.cleanup)
        self.data_dir = Path(self.directory.name)
        # Construct in demo mode: no automatic asynchronous checks or ROS jobs.
        self.window = WorkbenchWindow(self.root, self.data_dir, demo=True)
        self.window.timer.stop()
        # Dispose of the real manager's idle observer before substituting it.
        self.window.manager.close(timeout=1.0)
        self.jobs = {"stack": True, "program": False, "service": False}
        self.events = []
        self.manager = MagicMock(spec=ProcessManager)
        self.manager.alive.side_effect = lambda name: self.jobs.get(name, False)
        self.manager.active_names.side_effect = lambda: [name for name, alive in self.jobs.items() if alive]
        self.manager.poll.side_effect = self._poll
        self.manager.launch.side_effect = self._launch
        self.window.manager = self.manager
        self.window.demo = False
        self.window.ready = True
        self.window.run_state = "armed"
        self.window.scene = {"seed": 246135, "object_count": 6}
        self.student_path = self.data_dir / "programs" / "trusted student.py"
        self.student_path.write_text("# trusted local test program; never executed\n", encoding="utf-8")
        self.window.program_path.setText(str(self.student_path))
        self.window._update_buttons()
        self.addCleanup(self._close_window)

    def _close_window(self):
        # Avoid closeEvent's interactive prompt and never signal real processes.
        self.window.shutdown()
        self.window.demo = True
        self.window.close()
        self.window.deleteLater()

    def _poll(self):
        pending, self.events = self.events, []
        return pending

    def _launch(self, name, *_args, **_kwargs):
        self.jobs[name] = True
        return 123456

    def _service_result(self, name, success=True, returncode=0):
        self.events += [
            {"kind": "output", "name": "service",
             "text": PREFIX + json.dumps({"type": "service", "name": name,
                                            "success": success, "message": "test response"})},
            {"kind": "exited", "name": "service", "returncode": returncode},
        ]

    def _launched_services(self):
        names = []
        for call in self.manager.launch.call_args_list:
            if call.args[0] == "service":
                command = call.args[1]
                names.append(command[command.index("--name") + 1])
        return names

    def test_dead_service_is_busy_until_exited_consumed_and_reset_chain_not_lost(self):
        self.window.reset_scene()
        self.assertEqual(self._launched_services(), ["run-reset"])
        self.assertEqual(self.window._service_queue, ["scene-reset"])
        # OS process is already dead; queued output/exited has not reached GUI.
        self.jobs["service"] = False
        self.window._update_buttons()
        self.assertTrue(self.window._service_busy())
        for button in (self.window.reset_button, self.window.generate_button,
                       self.window.reference_button, self.window.student_button,
                       self.window.start_button):
            self.assertFalse(button.isEnabled())
        self.window._service("generate")
        self.window.reset_scene()
        self.window._tick()  # No exited event yet: must retain the queued step.
        self.assertEqual(self._launched_services(), ["run-reset"])
        self.assertEqual(self.window._service_queue, ["scene-reset"])
        self._service_result("run-reset")
        self.window._tick()
        self.assertEqual(self._launched_services(), ["run-reset", "scene-reset"])
        self.assertEqual(self.window._service_queue, [])
        self.assertTrue(self.window._service_busy())
        self.jobs["service"] = False
        self._service_result("scene-reset")
        self.window._tick()
        self.assertFalse(self.window._service_busy())
        self.assertEqual(self.window.run_state, "idle")

    def test_stop_queues_halt_when_dead_service_exit_is_still_pending(self):
        self.window._service("generate")
        self.jobs["service"] = False
        self.window.stop_round()
        self.assertEqual(self._launched_services(), ["generate"])
        self.assertEqual(self.window._service_queue, ["halt"])
        self._service_result("generate")
        self.window._tick()
        self.assertEqual(self._launched_services(), ["generate", "halt"])
        self.assertEqual(self.window._service_queue, [])
        self.assertTrue(self.window._service_busy())

    def test_service_result_requires_matching_job_source_and_requested_name(self):
        self.window._service("run-reset")
        correct = {"type": "service", "name": "run-reset", "success": True}
        self.window._handle_protocol("program", correct)
        self.assertFalse(self.window._service_success)
        self.window._handle_protocol("monitor", correct)
        self.assertFalse(self.window._service_success)
        self.window._handle_protocol("service", {**correct, "name": "generate"})
        self.assertFalse(self.window._service_success)
        self.window._handle_protocol("service", correct)
        self.assertTrue(self.window._service_success)

    def test_failed_launch_releases_service_inflight_flag(self):
        self.manager.launch.side_effect = OSError("test cannot launch")
        self.window._service("generate")
        self.assertFalse(self.window._service_busy())
        self.assertIn("test cannot launch", self.window.log_view.toPlainText())

    def test_manual_start_without_program_is_disabled_and_does_not_call_service(self):
        self.window._update_buttons()
        self.assertFalse(self.window.start_button.isEnabled())
        with patch.object(self.window, "_service") as service:
            self.window.start_round()
        service.assert_not_called()
        self.manager.launch.assert_not_called()

    def test_manual_start_with_program_calls_start_service(self):
        self.jobs["program"] = True
        self.window._update_buttons()
        self.assertTrue(self.window.start_button.isEnabled())
        with patch.object(self.window, "_service") as service:
            self.window.start_round()
        service.assert_called_once_with("start")

    def test_student_does_not_launch_if_confirmation_loses_ready(self):
        self._assert_student_confirmation_rechecked(lambda: setattr(self.window, "ready", False))

    def test_student_does_not_launch_if_confirmation_starts_closing(self):
        self._assert_student_confirmation_rechecked(lambda: setattr(self.window, "_closing_stack", True))

    def test_student_does_not_launch_if_another_program_starts_during_confirmation(self):
        self._assert_student_confirmation_rechecked(lambda: self.jobs.update(program=True))

    def test_student_does_not_launch_if_round_changes_during_confirmation(self):
        self._assert_student_confirmation_rechecked(lambda: setattr(self.window, "run_state", "running"))

    def _assert_student_confirmation_rechecked(self, mutate):
        def confirm(*_args, **_kwargs):
            # Simulate monitor/timer events processed by the Qt modal event loop.
            mutate()
            return QMessageBox.Yes
        with patch("workbench.app.QMessageBox.question", side_effect=confirm) as question:
            self.window.run_student()
        question.assert_called_once()
        self.manager.launch.assert_not_called()

    def test_student_confirmation_unchanged_still_launches(self):
        with patch("workbench.app.QMessageBox.question", return_value=QMessageBox.Yes):
            self.window.run_student()
        self.manager.launch.assert_called_once()
        call = self.manager.launch.call_args
        self.assertEqual(call.args[0], "program")
        self.assertIn(str(self.student_path), call.args[1])

    def _run_program(self, kind):
        if kind == "reference":
            self.window.run_reference()
        else:
            with patch("workbench.app.QMessageBox.question", return_value=QMessageBox.Yes):
                self.window.run_student()

    def _assert_dead_program_stays_inflight_until_exit(self, kind):
        self._run_program(kind)
        self.assertTrue(self.window._program_inflight)
        self.jobs["program"] = False
        self.window._update_buttons()
        self.assertTrue(self.window._runner_active())
        for button in (self.window.reference_button, self.window.student_button,
                       self.window.generate_button, self.window.reset_button):
            self.assertFalse(button.isEnabled())
        # Direct slot calls must also reject reuse; disabling buttons is not enough.
        self.window.run_reference()
        with patch("workbench.app.QMessageBox.question", return_value=QMessageBox.Yes) as question:
            self.window.run_student()
        question.assert_not_called()
        self.assertEqual(self.manager.launch.call_count, 1)
        self.window._tick()  # No exited consumed yet: pending lifecycle persists.
        self.assertTrue(self.window._program_inflight)
        self.assertEqual(self.window._auto_start_reference, kind == "reference")
        self.events.append({"kind": "exited", "name": "program", "returncode": 0})
        self.window._tick()
        self.assertFalse(self.window._program_inflight)
        self.assertFalse(self.window._runner_active())
        self.assertFalse(self.window._auto_start_reference)
        self.assertTrue(self.window.reference_button.isEnabled())
        self.assertTrue(self.window.student_button.isEnabled())
        # Only now reuse the slot: no old exited event may cancel the new handshake.
        self.window.run_reference()
        self.assertEqual(self.manager.launch.call_count, 2)
        self.assertTrue(self.window._program_inflight)
        self.assertTrue(self.window._auto_start_reference)
        self.window._tick()
        self.assertTrue(self.window._auto_start_reference)
        self.window._handle_protocol("program", {"type": "program-ready"})
        self.assertEqual(self._launched_services(), ["start"])

    def test_dead_reference_blocks_reuse_until_exit_then_new_handshake_survives(self):
        self._assert_dead_program_stays_inflight_until_exit("reference")

    def test_dead_student_blocks_reuse_until_exit_then_new_handshake_survives(self):
        self._assert_dead_program_stays_inflight_until_exit("student")

    def _assert_program_marked_inflight_before_launch(self, kind):
        def launch(name, *args, **kwargs):
            self.assertEqual(name, "program")
            self.assertTrue(self.window._program_inflight)
            return self._launch(name, *args, **kwargs)
        self.manager.launch.side_effect = launch
        self._run_program(kind)
        self.manager.launch.assert_called_once()
        self.assertTrue(self.window._program_inflight)

    def test_reference_sets_program_inflight_before_launch(self):
        self._assert_program_marked_inflight_before_launch("reference")

    def test_student_sets_program_inflight_before_launch(self):
        self._assert_program_marked_inflight_before_launch("student")

    def _assert_program_launch_failure_releases_inflight(self, kind):
        def launch_failure(*_args, **_kwargs):
            self.assertTrue(self.window._program_inflight)
            raise OSError("test program launch failure")
        self.manager.launch.side_effect = launch_failure
        self._run_program(kind)
        self.manager.launch.assert_called_once()
        self.assertFalse(self.window._program_inflight)
        self.assertFalse(self.window._runner_active())
        self.assertFalse(self.window._auto_start_reference)
        self.assertIn("test program launch failure", self.window.log_view.toPlainText())

    def test_reference_launch_failure_releases_program_inflight(self):
        self._assert_program_launch_failure_releases_inflight("reference")

    def test_student_launch_failure_releases_program_inflight(self):
        self._assert_program_launch_failure_releases_inflight("student")

    def test_student_rechecks_program_inflight_after_confirmation(self):
        # A second program can itself die before the dialog returns; its pending
        # exited event must still prevent this program from reusing the slot.
        self._assert_student_confirmation_rechecked(
            lambda: setattr(self.window, "_program_inflight", True))

    def test_new_program_copy_failure_is_reported_without_changing_selected_path(self):
        selected = self.window.program_path.text()
        with patch("workbench.app.shutil.copyfile", side_effect=PermissionError("test read-only directory")):
            self.window.new_program()
        self.assertEqual(self.window.program_path.text(), selected)
        self.assertIn("test read-only directory", self.window.log_view.toPlainText())

    def test_demo_result_write_failure_does_not_escape_timer_callback(self):
        self.window.demo = True
        self.window._demo_stack = True
        self.window._demo_running = True
        self.window._demo_step = 5
        epoch = self.window._demo_epoch
        with patch.object(Path, "write_text", side_effect=OSError("test disk full")):
            self.window._demo_advance(epoch)
        self.assertFalse(self.window._demo_running)
        self.assertEqual(self.window.run_state, "finished")
        self.assertTrue(self.window.score["demo"])
        self.assertIn("test disk full", self.window.log_view.toPlainText())


if __name__ == "__main__":
    unittest.main()
