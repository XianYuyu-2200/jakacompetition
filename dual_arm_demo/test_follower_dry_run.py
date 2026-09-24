#!/usr/bin/env python3
import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent
CORE_HEADER = ROOT / "dry_run_core.hpp"
CORE_TEST_SOURCE = ROOT / "dry_run_core_test.cpp"
CORE_TEST_BINARY = ROOT / "dry_run_core_test"
FOLLOWER_SOURCE = ROOT / "tracking_arm_dry_run.cpp"
FOLLOWER_BINARY = ROOT / "tracking_arm_dry_run"


class FollowerDryRunContractTest(unittest.TestCase):
    def test_core_mapping_and_watchdog(self):
        for path in (CORE_HEADER, CORE_TEST_SOURCE, CORE_TEST_BINARY):
            self.assertTrue(path.is_file(), f"missing: {path.name}")

        result = subprocess.run(
            [str(CORE_TEST_BINARY)],
            text=True,
            capture_output=True,
            timeout=5,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("DRY_RUN_CORE_TEST_OK", result.stdout)

        core_source = CORE_HEADER.read_text(encoding="utf-8")
        self.assertIn("class ReadinessGate", core_source)
        self.assertIn("required_consecutive_samples", core_source)

    def test_follower_program_is_readonly(self):
        self.assertTrue(FOLLOWER_SOURCE.is_file(), "missing tracking_arm_dry_run.cpp")
        self.assertTrue(FOLLOWER_BINARY.is_file(), "missing tracking_arm_dry_run binary")

        source = FOLLOWER_SOURCE.read_text(encoding="utf-8")
        forbidden = (
            "power_on(",
            "enable_robot(",
            "drag_mode_enable(",
            "servo_move_enable(",
            "servo_j(",
            "joint_move(",
            "linear_move(",
        )
        for call in forbidden:
            self.assertNotIn(call, source, f"unsafe call present: {call}")

        for required in (
            "login_in(",
            "get_robot_status_simple(",
            "get_actual_joint_position(",
            "login_out(",
        ):
            self.assertIn(required, source, f"missing read-only call: {required}")

        usage = subprocess.run(
            [str(FOLLOWER_BINARY)],
            text=True,
            capture_output=True,
            timeout=5,
            check=False,
        )
        self.assertEqual(usage.returncode, 64)
        self.assertIn("Usage:", usage.stderr)

        self.assertIn("operator_ready", source)
        self.assertIn("tracking_ready", source)
        self.assertIn("[csv_path]", source)
        self.assertIn("sequence,rx_monotonic_ns,receive_interval_ms", source)
        self.assertIn("max_receive_interval_ms=", source)
        self.assertIn("max_tracking_joint_call_ms=", source)
        self.assertIn("max_tracking_status_call_ms=", source)
        self.assertIn("StatusPollScheduler tracking_status_poll(25", source)
        self.assertIn("std::thread receiver_thread", source)
        self.assertIn("LatestPacketMailbox", source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
