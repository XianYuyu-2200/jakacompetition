#!/usr/bin/env python3
import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "single_readonly.cpp"
BINARY = ROOT / "single_readonly"


class SingleReadonlyContractTest(unittest.TestCase):
    def test_readonly_program_contract(self):
        self.assertTrue(SOURCE.is_file(), "single_readonly.cpp is missing")

        source = SOURCE.read_text(encoding="utf-8")
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
            "get_joint_position(",
            "get_actual_joint_position(",
            "login_out(",
        ):
            self.assertIn(required, source, f"required call missing: {required}")

        self.assertTrue(BINARY.is_file(), "single_readonly binary is missing")
        result = subprocess.run(
            [str(BINARY)],
            text=True,
            capture_output=True,
            timeout=5,
            check=False,
        )
        self.assertEqual(result.returncode, 64)
        self.assertIn("Usage:", result.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
