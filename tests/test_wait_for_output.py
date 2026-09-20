#!/usr/bin/env python3
import json
import os
import subprocess
import sys
import tempfile
import time
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WAITER = os.path.join(ROOT, "scripts", "wait_for_output.py")


class WaitForOutputTests(unittest.TestCase):
    def run_waiter(self, path, *arguments):
        return subprocess.run(
            [sys.executable, WAITER, "--file", path, *arguments],
            check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=3)

    def test_literal_split_across_appends(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "output")
            open(path, "wb").close()
            writer = subprocess.Popen([
                sys.executable, "-c",
                "import sys,time\n"
                "p=sys.argv[1]\n"
                "f=open(p,'ab',buffering=0)\n"
                "f.write(b'boot RE')\n"
                "time.sleep(.05)\n"
                "f.write(b'ADY\\n')\n"
                "time.sleep(.05)\n",
                path,
            ])
            try:
                result = self.run_waiter(
                    path, "--contains", "READY", "--pid", str(writer.pid),
                    "--timeout", "1")
            finally:
                writer.wait(timeout=2)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout, b"")

    def test_regex_capture_is_printed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "output")
            with open(path, "wb") as output:
                output.write(b"firmware port 80 -> host 127.0.0.1:43127\n")
            result = self.run_waiter(
                path, "--regex", r"firmware port 80 .*:([0-9]+)",
                "--group", "1", "--timeout", "1")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout, b"43127\n")

    def test_sandbox_uart_is_decoded_incrementally(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "events")
            with open(path, "wb") as output:
                for byte in b"noise v1.16.0 prompt":
                    output.write((json.dumps({"t": "uart", "u": 0,
                                              "b": byte}) + "\n").encode())
            result = self.run_waiter(
                path, "--sandbox-uart", "--contains", "v1.16.0",
                "--timeout", "1")
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_timeout_is_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "output")
            open(path, "wb").close()
            before = time.monotonic()
            result = self.run_waiter(
                path, "--contains", "never", "--timeout", "0.08")
            elapsed = time.monotonic() - before
            self.assertEqual(result.returncode, 1)
            self.assertIn(b"timed out", result.stderr)
            self.assertLess(elapsed, 1.0)


if __name__ == "__main__":
    unittest.main()
