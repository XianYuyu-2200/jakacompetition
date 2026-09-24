#!/usr/bin/env python3
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "j1_follow_test.cpp"
SCRIPT = ROOT / "run_j1_follow_test.sh"


class J1FollowContractTest(unittest.TestCase):
    def test_minimal_j1_follow_contract(self):
        source = SOURCE.read_text(encoding="utf-8")
        self.assertNotIn("power_on(", source)
        self.assertNotIn("enable_robot(", source)
        self.assertNotIn("drag_mode_enable(", source)
        self.assertIn("servo_move_enable(TRUE)", source)
        self.assertIn("servo_j(&command, MoveMode::ABS, 1)", source)
        self.assertIn("motion_abort()", source)
        self.assertIn("servo_move_enable(FALSE)", source)
        self.assertIn('"START_J1"', source)
        self.assertIn("packet.position[0] - operator_zero[0]", source)
        self.assertIn("tracking_zero[0] + (packet.position[0]", source)
        self.assertIn("for (int joint = 1; joint < 6; ++joint)", source)
        self.assertIn("latency_ms", source)
        self.assertIn("error_deg", source)
        self.assertIn('stop_reason="', source)
        self.assertIn("active_duration_sec=", source)

    def test_script_stops_operator_when_follower_finishes(self):
        script = SCRIPT.read_text(encoding="utf-8")
        wait_follower = script.index('wait "$FOLLOWER_PID"')
        kill_operator = script.index('kill "$OPERATOR_PID"', wait_follower)
        wait_operator = script.index('wait "$OPERATOR_PID"', kill_operator)
        self.assertLess(wait_follower, kill_operator)
        self.assertLess(kill_operator, wait_operator)
        self.assertIn('"$OPERATOR_IP" "$IPC_SOCKET" "0" &', script)


if __name__ == "__main__":
    unittest.main(verbosity=2)
