"""离屏 Qt 工作台回归；不启动 ROS，不触碰现有仿真。"""
import json
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication, QMessageBox

from workbench.app import WorkbenchWindow
from workbench.ros_bridge import readiness, PREFIX


class AppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.qt = QApplication.instance() or QApplication([])
        cls.root = Path(__file__).resolve().parents[2]

    def setUp(self):
        self.directory = TemporaryDirectory()
        self.window = WorkbenchWindow(self.root, Path(self.directory.name), demo=True)
        self.window.show()
        self.qt.processEvents()

    def tearDown(self):
        self.window.shutdown()
        self.window.close()
        self.qt.processEvents()
        self.directory.cleanup()

    def test_waypoint_editor_records_order_and_refreshes_station(self):
        self.window.pose_station.setValue(6)
        for value in (0.1, 0.2):
            for spin in self.window.joint_spins:
                spin.setValue(value)
            self.window._append_waypoint()
        self.assertIn("2 个", self.window.waypoint_status.text())
        self.assertIn("0.1000", self.window.waypoint_status.toolTip())
        self.window.pose_station.setValue(5)
        self.assertIn("0 个", self.window.waypoint_status.text())
        self.window.pose_station.setValue(6)
        self.assertIn("2 个", self.window.waypoint_status.text())
        with patch('workbench.app.QMessageBox.question', return_value=QMessageBox.Yes):
            self.window._clear_waypoints()
        self.assertIn("0 个", self.window.waypoint_status.text())

    def test_station_transfer_waypoint_does_not_create_fixed_pose(self):
        self.window.pose_station.setValue(6)
        self.window.pose_phase.setCurrentIndex(self.window.pose_phase.findData("object:transfer"))
        self.window._record_pose()
        self.assertIn("追加", self.window.pose_status.text())
        self.window._append_waypoint()
        import yaml
        data = yaml.safe_load(self.window._pose_path().read_text(encoding="utf-8"))
        self.assertEqual(len(data["objects"]["6"]["transfer_via_joints"]), 1)
        self.assertNotIn("transfer_joints", data["objects"]["6"])

    def test_initial_state_disables_motion(self):
        self.assertTrue(self.window.launch_button.isEnabled())
        self.assertFalse(self.window.reference_button.isEnabled())
        self.assertFalse(self.window.start_button.isEnabled())
        self.assertFalse(self.window.generate_button.isEnabled())
        self.assertIn("不启动 ROS", self.window.notice.text())

    def test_demo_controls_and_reset(self):
        self.window.start_environment()
        self.assertTrue(self.window.generate_button.isEnabled())
        self.assertFalse(self.window.mode_combo.isEnabled())
        self.window.generate()
        self.assertEqual(self.window.run_state, "armed")
        self.assertTrue(self.window.reference_button.isEnabled())
        self.window.run_reference()
        self.assertFalse(self.window.generate_button.isEnabled())
        self.assertTrue(self.window.stop_round_button.isEnabled())
        self.window.stop_round()
        self.assertEqual(self.window.run_state, "finished")
        self.window.reset_scene()
        self.assertFalse(self.window.scene)
        self.assertEqual(self.window.run_state, "idle")
        self.window.stop_environment()
        self.assertFalse(self.window.generate_button.isEnabled())
        self.assertTrue(self.window.mode_combo.isEnabled())

    def test_pose_record_uses_private_versioned_simulation_config(self):
        for index, spin in enumerate(self.window.joint_spins):
            spin.setValue(index * 0.1)
        self.window._record_pose()
        pose_path = Path(self.directory.name) / "config" / "track1_fixed_poses.yaml"
        self.assertTrue(pose_path.exists())
        import yaml
        data = yaml.safe_load(pose_path.read_text(encoding="utf-8"))
        self.assertEqual(data["version"], 4)
        self.assertEqual(data["backend"], "simulation")
        self.assertEqual(data["device_profile"], "jaka_mini_2_gemini305_gripper_sim")
        self.assertFalse(data["enabled"])
        self.assertEqual(len(data["objects"]["1"]["approach_joints"]), 6)
        # Calibration stays in the team's data folder and does not mutate the
        # checked-in starter template.
        self.assertEqual(yaml.safe_load(self.window._pose_template_path().read_text())["objects"], {})

    def test_demo_records_are_explicitly_labelled(self):
        self.window.start_environment()
        self.window.generate()
        self.window.run_reference()
        epoch = self.window._demo_epoch
        for _ in range(6):
            self.window._demo_advance(epoch)
        paths = list((Path(self.directory.name) / "results").glob("*.json"))
        self.assertEqual(len(paths), 1)
        self.assertTrue(json.loads(paths[0].read_text())["demo"])
        self.assertIn("DEMO", self.window.results_table.item(0, 1).text())

    def test_stale_demo_callback_cannot_complete_next_round(self):
        self.window.start_environment()
        self.window.generate()
        self.window.run_reference()
        old_epoch = self.window._demo_epoch
        self.window.stop_round()
        self.window.reset_scene()
        self.window.generate()
        self.window.run_reference()
        self.window._demo_advance(old_epoch)
        self.assertEqual(self.window._demo_step, 0)

    def test_student_copy_preserves_original(self):
        source = self.root / "workbench/examples/student_track1.py"
        before = source.read_bytes()
        self.window.new_program()
        dest = Path(self.window.program_path.text())
        self.assertTrue(dest.is_file())
        self.assertNotEqual(source, dest)
        self.assertEqual(before, dest.read_bytes())
        dest.write_text("# my exercise\n")
        self.assertEqual(before, source.read_bytes())
        self.window.start_environment()
        self.window.reset_scene()
        self.assertTrue(dest.is_file())

    def test_recognition_mode_embeds_results_and_starts_existing_recognizer_launch(self):
        index = self.window.mode_combo.findData("recognition")
        self.assertGreaterEqual(index, 0)
        self.window.mode_combo.setCurrentIndex(index)
        self.assertTrue(self.window.recognition_status.isVisible())
        self.window._show_recognition({"count": 2, "located": 1,
                                       "objects": [{"name": "正方体"}, {"name": "圆柱体"}]})
        self.assertIn("2/6", self.window.recognition_status.text())
        self.assertIn("正方体", self.window.recognition_status.text())

        self.window.demo = False
        with patch.object(self.window.manager, "launch") as launch:
            self.window._launch_stack()
        command = launch.call_args_list[0].args[1]
        self.assertIn("gemini305_preview.launch.py", command)
        self.assertIn("learn:=false", command)
        monitor_command = launch.call_args_list[1].args[1]
        self.assertIn("--recognition", monitor_command)

    def test_vision_disables_rule_controls(self):
        self.window.mode_combo.setCurrentIndex(1)
        self.window.start_environment()
        self.assertFalse(self.window.generate_button.isEnabled())
        self.assertFalse(self.window.reference_button.isEnabled())
        self.assertFalse(self.window.start_button.isEnabled())
        self.assertFalse(self.window.student_button.isEnabled())
        self.assertTrue(self.window.stop_stack_button.isEnabled())

    def test_log_strips_terminal_escape_sequences(self):
        self.window._log("\033[31merror\033[0m")
        self.assertIn("error", self.window.log_view.toPlainText())
        self.assertNotIn("\033[", self.window.log_view.toPlainText())

    def test_bad_result_file_does_not_break_records(self):
        results = Path(self.directory.name) / "results"
        (results / "bad.json").write_text("{bad")
        (results / "wrong.json").write_text("[]")
        self.window._refresh_results()
        self.assertEqual(self.window.results_table.rowCount(), 0)

    def test_conflict_prevents_launch_and_never_stops_external(self):
        self.window.demo = False
        self.window._start_after_check = True
        with patch.object(self.window, "_launch_stack") as launch, patch.object(self.window.manager, "stop_all") as stop:
            self.window._checked({"checks": [], "conflicts": [{"pid": 1234, "command": "external sim"}]})
            launch.assert_not_called()
            stop.assert_not_called()
        self.assertIn("外部", self.window.notice.text())
        self.assertIn("PID 1234", self.window.notice.text())
        self.assertIn("Ctrl+C", self.window.notice.text())
        self.assertIn("external sim", self.window.log_view.toPlainText())
        self.assertIn("仅关闭 Gazebo/RViz", self.window.log_view.toPlainText())

    def test_retry_after_external_conflict_removed_launches_normally(self):
        self.window.demo = False
        with patch.object(self.window, "_launch_stack") as launch, patch.object(self.window.manager, "stop_all") as stop:
            self.window._start_after_check = True
            self.window._checked({"checks": [], "conflicts": [{"pid": 1234, "command": "external sim"}]})
            launch.assert_not_called()
            self.window._start_after_check = True
            self.window._checked({"checks": [], "conflicts": []})
            launch.assert_called_once_with()
            stop.assert_not_called()

    def test_missing_dependencies_prevent_launch(self):
        self.window.demo = False
        self.window._start_after_check = True
        with patch.object(self.window, "_launch_stack") as launch:
            self.window._checked({"checks": [{"name": "ROS", "level": "error", "detail": "missing"}], "conflicts": []})
            launch.assert_not_called()

    def test_program_ready_only_starts_armed_healthy_round(self):
        self.window.demo = False
        self.window._auto_start_reference = True
        self.window.ready = True
        self.window.run_state = "armed"
        with patch.object(self.window, "_service") as service:
            self.window._handle_protocol("program", {"type": "program-ready"})
            service.assert_called_once_with("start")
        self.window._auto_start_reference = True
        self.window.ready = False
        with patch.object(self.window, "_service") as service, patch.object(self.window.manager, "stop") as stop:
            self.window._handle_protocol("program", {"type": "program-ready"})
            service.assert_not_called()
            stop.assert_called_once_with("program")

    def test_malformed_monitor_json_is_logged_not_raised(self):
        self.window.demo = False
        events = [{"kind": "output", "name": "monitor", "text": PREFIX + "not-json"}]
        with patch.object(self.window.manager, "poll", return_value=events):
            self.window._tick()
        self.assertIn("无法解析", self.window.log_view.toPlainText())

    def test_gazebo_embed_checks_qt_backend_not_desktop_session_type(self):
        self.window.demo = False
        with patch.object(self.window, "_has_stack", return_value=True), \
             patch.object(QApplication, "platformName", return_value="wayland"):
            self.window._refresh_gazebo_view()
        self.assertIn("wayland", self.window.gazebo_placeholder.text())

    def test_navigation_and_missing_program(self):
        for i in range(4):
            self.window._navigate(i)
            self.assertEqual(self.window.pages.currentIndex(), i)
            self.assertTrue(self.window.nav_buttons[i].isChecked())
        self.window.program_path.setText("/definitely/missing.py")
        self.window._load_preview()
        self.assertIn("无法预览", self.window.code_preview.toPlainText())


class ReadinessTests(unittest.TestCase):
    def test_no_process_only_ready(self):
        checks = readiness("basic", {})
        self.assertFalse(all(c["level"] == "ok" for c in checks))

    def test_missing_gripper_blocks_ready(self):
        facts = dict(arm_joints=True, finger_joints=False, controllers=True,
                     moveit=True, planning_scene=True, competition=True)
        self.assertFalse(all(c["level"] == "ok" for c in readiness("basic", facts)))
        facts["finger_joints"] = True
        self.assertTrue(all(c["level"] == "ok" for c in readiness("basic", facts)))

    def test_basic_gazebo_requires_camera_when_requested(self):
        facts = dict(arm_joints=True, finger_joints=True, controllers=True,
                     moveit=True, planning_scene=True, competition=True,
                     images=False)
        self.assertFalse(all(c["level"] == "ok"
                             for c in readiness("basic", facts, camera_required=True)))
        facts["images"] = True
        self.assertTrue(all(c["level"] == "ok"
                            for c in readiness("basic", facts, camera_required=True)))

    def test_recognition_readiness_requires_camera_and_detection(self):
        facts = {"clock": True, "images": True, "detections": False}
        self.assertFalse(all(c["level"] == "ok" for c in readiness("recognition", facts)))
        facts["detections"] = True
        self.assertTrue(all(c["level"] == "ok" for c in readiness("recognition", facts)))

    def test_vision_requires_clock_and_both_images(self):
        facts = dict(arm_joints=True, finger_joints=True, controllers=True,
                     moveit=True, planning_scene=True, clock=True, images=False)
        self.assertFalse(all(c["level"] == "ok" for c in readiness("vision", facts)))
        facts["images"] = True
        self.assertTrue(all(c["level"] == "ok" for c in readiness("vision", facts)))


if __name__ == "__main__":
    unittest.main()
