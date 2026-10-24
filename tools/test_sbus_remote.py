#!/usr/bin/env python3
"""Compile and run the host regression test for the SBUS startup gate."""

from __future__ import annotations

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TEST_SOURCE = ROOT / "tools" / "test_sbus_remote.c"
MODULE_SOURCE = ROOT / "App" / "Src" / "sbus_remote.c"
INCLUDE = ROOT / "App" / "Inc"


def _compiler() -> str | None:
    for candidate in ("cc", "clang", "gcc"):
        if shutil.which(candidate) is not None:
            return candidate
    return None


class SbusRemoteStartupTest(unittest.TestCase):
    def test_startup_gate(self) -> None:
        compiler = _compiler()
        if compiler is None:
            self.skipTest("no host C compiler available")
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "test_sbus_remote"
            build = subprocess.run(
                [
                    compiler,
                    "-std=c11",
                    "-O2",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-DSBUS_REMOTE_HOST_TEST",
                    f"-I{INCLUDE}",
                    "-o",
                    str(binary),
                    str(TEST_SOURCE),
                    str(MODULE_SOURCE),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(
                build.returncode, 0, f"compile failed:\n{build.stderr}"
            )
            run = subprocess.run(
                [str(binary)], capture_output=True, text=True, check=False
            )
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            self.assertIn("0 failure(s)", run.stdout)


if __name__ == "__main__":
    unittest.main()
