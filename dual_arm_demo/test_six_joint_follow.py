#!/usr/bin/env python3
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "six_joint_follow_test.cpp"
SCRIPT = ROOT / "run_six_joint_follow_test.sh"
BINARY = ROOT / "six_joint_follow_test"


class SixJointFollowContractTest(unittest.TestCase):
    def test_six_joint_relative_mapping_and_safety_contract(self):
        self.assertTrue(SOURCE.is_file())
        source = SOURCE.read_text(encoding="utf-8")

        for forbidden in (
            "power_on(",
            "enable_robot(",
            "drag_mode_enable(",
        ):
            self.assertNotIn(forbidden, source)

        for required in (
            '"START_SIX_JOINTS"',
            "servo_move_enable(TRUE)",
            "servo_j(&command, MoveMode::ABS, 1)",
            "motion_abort()",
            "servo_move_enable(FALSE)",
            "jaka_six_joint::map_relative(",
            "for (int joint = 0; joint < 6; ++joint)",
            'stop_reason="',
            "active_duration_sec=",
            "avg_latency_ms=",
            "max_latency_ms=",
            "avg_error_deg=[",
            "max_error_deg=[",
        ):
            self.assertIn(required, source)

    def test_script_stops_operator_after_follower(self):
        self.assertTrue(SCRIPT.is_file())
        script = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("START_SIX_JOINTS", script)
        self.assertIn('"$OPERATOR_IP" "$IPC_SOCKET" "0" &', script)
        wait_follower = script.index('wait "$FOLLOWER_PID"')
        kill_operator = script.index('kill "$OPERATOR_PID"', wait_follower)
        wait_operator = script.index('wait "$OPERATOR_PID"', kill_operator)
        self.assertLess(wait_follower, kill_operator)
        self.assertLess(kill_operator, wait_operator)

    def test_binary_rejects_missing_start_token(self):
        self.assertTrue(BINARY.is_file())
        result = subprocess.run(
            [str(BINARY)], text=True, capture_output=True, timeout=5, check=False
        )
        self.assertEqual(result.returncode, 64)
        self.assertIn("Usage:", result.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
