#!/usr/bin/env python3
import os
import socket
import struct
import subprocess
import tempfile
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent
LEADER_SOURCE = ROOT / "operator_joint_publisher.cpp"
LISTENER_SOURCE = ROOT / "ipc_joint_listener.cpp"
PROTOCOL_HEADER = ROOT / "joint_sample_ipc.hpp"
LEADER_BINARY = ROOT / "operator_joint_publisher"
LISTENER_BINARY = ROOT / "ipc_joint_listener"

MAGIC = 0x31414B4A
VERSION = 2
PACKET_SIZE = 80
PACKET = struct.Struct("<IHHQQ6diBBBB")


class IpcReadonlyContractTest(unittest.TestCase):
    def test_sources_and_binaries_are_readonly(self):
        for path in (LEADER_SOURCE, LISTENER_SOURCE, PROTOCOL_HEADER):
            self.assertTrue(path.is_file(), f"missing source: {path.name}")

        forbidden = (
            "power_on(",
            "enable_robot(",
            "drag_mode_enable(",
            "servo_move_enable(",
            "servo_j(",
            "joint_move(",
            "linear_move(",
        )
        combined = LEADER_SOURCE.read_text() + LISTENER_SOURCE.read_text()
        for call in forbidden:
            self.assertNotIn(call, combined, f"unsafe call present: {call}")

        listener_source = LISTENER_SOURCE.read_text()
        self.assertNotIn("JAKAZuRobot", listener_source)
        self.assertNotIn("libjakaAPI", listener_source)

        for binary in (LEADER_BINARY, LISTENER_BINARY):
            self.assertTrue(binary.is_file(), f"missing binary: {binary.name}")

        usage = subprocess.run(
            [str(LISTENER_BINARY)],
            text=True,
            capture_output=True,
            timeout=5,
            check=False,
        )
        self.assertEqual(usage.returncode, 64)
        self.assertIn("Usage:", usage.stderr)

        linked = subprocess.run(
            ["ldd", str(LISTENER_BINARY)],
            text=True,
            capture_output=True,
            timeout=5,
            check=True,
        ).stdout
        self.assertNotIn("libjakaAPI", linked)

        operator_source = LEADER_SOURCE.read_text(encoding="utf-8")
        self.assertNotIn("sdk_code = cached_status_ret", operator_source)
        self.assertNotIn("sdk_code = cached_drag_ret", operator_source)

    def test_listener_receives_samples_and_faults_when_stale(self):
        with tempfile.TemporaryDirectory(prefix="jaka_ipc_test_") as tmp:
            socket_path = os.path.join(tmp, "leader.sock")
            proc = subprocess.Popen(
                [str(LISTENER_BINARY), socket_path, "0.55"],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            try:
                deadline = time.monotonic() + 2.0
                while not os.path.exists(socket_path):
                    self.assertIsNone(proc.poll(), "listener exited before bind")
                    if time.monotonic() > deadline:
                        self.fail("listener socket was not created")
                    time.sleep(0.005)

                sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
                try:
                    start_ns = time.monotonic_ns()
                    for sequence in range(40):
                        now_ns = time.monotonic_ns()
                        packet = PACKET.pack(
                            MAGIC,
                            VERSION,
                            PACKET_SIZE,
                            sequence,
                            now_ns,
                            0.1,
                            0.2,
                            0.3,
                            0.4,
                            0.5,
                            0.6,
                            0,
                            1,
                            1,
                            1,
                            0,
                        )
                        sender.sendto(packet, socket_path)
                        target_ns = start_ns + (sequence + 1) * 8_000_000
                        delay = (target_ns - time.monotonic_ns()) / 1e9
                        if delay > 0:
                            time.sleep(delay)
                finally:
                    sender.close()

                output, _ = proc.communicate(timeout=3)
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.wait(timeout=2)

            self.assertEqual(proc.returncode, 0, output)
            self.assertIn("state=FRESH", output)
            self.assertIn("state=HOLD", output)
            self.assertIn("state=FAULT", output)
            self.assertIn("received=40", output)
            self.assertIn("valid=40", output)
            self.assertIn("invalid=0", output)
            self.assertIn("sequence_gaps=0", output)

    def test_listener_latches_fault_on_sdk_error_packet(self):
        with tempfile.TemporaryDirectory(prefix="jaka_ipc_sdk_error_") as tmp:
            socket_path = os.path.join(tmp, "leader.sock")
            proc = subprocess.Popen(
                [str(LISTENER_BINARY), socket_path, "0.25"],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            try:
                deadline = time.monotonic() + 2.0
                while not os.path.exists(socket_path):
                    self.assertIsNone(proc.poll(), "listener exited before bind")
                    if time.monotonic() > deadline:
                        self.fail("listener socket was not created")
                    time.sleep(0.005)

                sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
                try:
                    packet = PACKET.pack(
                        MAGIC,
                        VERSION,
                        PACKET_SIZE,
                        0,
                        time.monotonic_ns(),
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        -3,
                        1,
                        1,
                        1,
                        0,
                    )
                    sender.sendto(packet, socket_path)
                finally:
                    sender.close()

                output, _ = proc.communicate(timeout=3)
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.wait(timeout=2)

            self.assertEqual(proc.returncode, 0, output)
            self.assertIn("state=FAULT", output)
            self.assertIn("reason=SDK_ERROR", output)
            self.assertIn("sdk_errors=1", output)


if __name__ == "__main__":
    unittest.main(verbosity=2)
