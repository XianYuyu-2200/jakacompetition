#!/usr/bin/env python3
import subprocess
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parent
CORE_TEST = ROOT / "waypoint_recorder_core_test.cpp"
CORE_BINARY = ROOT / "waypoint_recorder_core_test"
SOURCE = ROOT / "waypoint_recorder.cpp"


class WaypointRecorderContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not CORE_TEST.is_file():
            return
        result = subprocess.run(
            ["g++", "-std=c++17", "-Wall", "-Wextra", "-pedantic", str(CORE_TEST), "-o", str(CORE_BINARY)],
            text=True,
            capture_output=True,
            check=False,
        )
        if result.returncode != 0:
            raise RuntimeError(result.stderr)

    def test_stability_and_median(self):
        self.assertTrue(CORE_TEST.is_file(), "core test is missing")
        self.assertTrue(CORE_BINARY.is_file(), "core test binary is missing")
        result = subprocess.run([str(CORE_BINARY), "stable"], text=True, capture_output=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_rejects_joint_and_tcp_jitter(self):
        self.assertTrue(CORE_TEST.is_file(), "core test is missing")
        self.assertTrue(CORE_BINARY.is_file(), "core test binary is missing")
        for mode in ("joint_jitter", "tcp_jitter", "read_failure"):
            result = subprocess.run([str(CORE_BINARY), mode], text=True, capture_output=True, check=False)
            self.assertEqual(result.returncode, 0, f"{mode}: {result.stderr}")

    def test_yaml_round_trip_shape_and_overwrite_policy(self):
        self.assertTrue(CORE_TEST.is_file(), "core test is missing")
        self.assertTrue(CORE_BINARY.is_file(), "core test binary is missing")
        result = subprocess.run([str(CORE_BINARY), "yaml_policy"], text=True, capture_output=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        document = yaml.safe_load(result.stdout)
        self.assertEqual(document["metadata"]["robot_ip"], "192.168.0.102")
        self.assertFalse(document["metadata"]["hand_state_available"])
        self.assertEqual(document["waypoints"]["P1"]["name"], "cup_below_pregrasp")
        self.assertEqual(document["waypoints"]["P3"]["name"], "cup_safe_lower")
        self.assertIsNone(document["waypoints"]["P1"]["hand_position"])
        self.assertFalse(document["waypoints"]["P1"]["confirmed"])
        self.assertEqual(len(document["waypoints"]["P1"]["joint_position_rad"]), 6)

    def test_production_source_is_read_only(self):
        self.assertTrue(SOURCE.is_file(), "waypoint_recorder.cpp is missing")
        source = SOURCE.read_text(encoding="utf-8")
        forbidden = (
            "power_on(", "power_off(", "enable_robot(", "disable_robot(",
            "servo_j(", "servo_move_enable(", "joint_move(", "linear_move(",
            "circular_move(", "motion_abort(", "set_digital_output(",
            "set_analog_output(", "send_tio_rs_command(",
        )
        for call in forbidden:
            self.assertNotIn(call, source, f"unsafe call present: {call}")
        for required in (
            "login_in(", "get_actual_joint_position(", "get_actual_tcp_position(", "login_out("
        ):
            self.assertIn(required, source, f"required call missing: {required}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
