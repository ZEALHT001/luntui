#!/usr/bin/env python3
"""Tests for the JLinkRTTLogger-based capture analyser."""

from __future__ import annotations

import os
import socket
import subprocess
import tempfile
import unittest
import unittest.mock
from pathlib import Path

from tools.rtt_logger_capture import (
    GdbServerRttReader,
    check_ports_free,
    coverage_summary,
    longest_segment_indices,
    parse_log,
)


def _listening_socket() -> tuple[socket.socket, int]:
    """Bind an ephemeral port and start listening so it reads as occupied."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    return server, server.getsockname()[1]


def _free_port() -> int:
    server, port = _listening_socket()
    server.close()
    return port


class CoverageTest(unittest.TestCase):
    def test_perfect_stream_is_full_coverage(self) -> None:
        times = [index * 20.0 for index in range(51)]
        summary = coverage_summary(times, 20.0)
        self.assertEqual(summary["samples"], 51)
        self.assertAlmostEqual(summary["coverage_pct"], 100.0)
        self.assertEqual(summary["estimated_missing_frames"], 0)

    def test_every_other_frame_is_half_coverage(self) -> None:
        times = [index * 40.0 for index in range(26)]
        summary = coverage_summary(times, 20.0)
        self.assertAlmostEqual(summary["coverage_pct"], 50.98, places=1)
        self.assertEqual(summary["estimated_missing_frames"], 25)
        self.assertEqual(summary["interval_histogram"], {"40": 25})

    def test_one_tick_quantisation_is_not_loss(self) -> None:
        # HAL_GetTick makes a 20 ms request land on 21 ms ticks.
        times = [index * 21.0 for index in range(100)]
        summary = coverage_summary(times, 20.0)
        self.assertAlmostEqual(summary["coverage_pct"], 100.0)
        self.assertEqual(summary["estimated_missing_frames"], 0)

    def test_broken_timestamp_does_not_create_fake_gap(self) -> None:
        times = [0.0, 20.0, 40.0, 0.089, 60.0]
        summary = coverage_summary(times, 20.0)
        self.assertEqual(summary["invalid_samples"], 1)
        self.assertEqual(summary["max_period_ms"], 20.0)


class ParseLogTest(unittest.TestCase):
    def test_counts_prefixes_and_parses_vmcvoa(self) -> None:
        # Full-width VMCVOFA frame: 136 fields, mcu_time_ms lives at index 99.
        def frame(mcu_time_ms: int) -> str:
            fields = ["0"] * 136
            fields[99] = str(mcu_time_ms)
            return "VMCVOFA," + ",".join(fields)

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "raw.log"
            path.write_text(
                "SEGGER J-Link header\n"
                f"{frame(1000)}\n"
                "VMCFAST,1000,1,2\n"
                f"{frame(1020)}\n"
                "\n",
                encoding="utf-8",
            )
            parsed = parse_log(path)
        self.assertEqual(parsed["total_lines"], 4)  # blank lines are skipped
        self.assertEqual(parsed["vmcfast_lines"], 1)
        self.assertEqual(parsed["vmcvoa_lines"], 2)
        self.assertEqual(parsed["other_lines"], 1)  # the J-Link header

    def test_truncated_vmcvoa_line_is_not_a_sample(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "raw.log"
            path.write_text("VMCVOFA,1000,1,2,3\n", encoding="utf-8")
            parsed = parse_log(path)
        self.assertEqual(parsed["vmcvoa_lines"], 0)
        self.assertEqual(parsed["other_lines"], 1)


class SegmentTest(unittest.TestCase):
    def test_keeps_longest_run_across_stale_prefix(self) -> None:
        # Stale pre-attach frames, then a 26 s gap, then the live capture.
        stale = [index * 21.0 for index in range(74)]
        live = [stale[-1] + 26000.0 + index * 21.0 for index in range(2795)]
        start, end, sessions = longest_segment_indices(stale + live)
        self.assertEqual(sessions, 2)
        self.assertEqual(start, 74)
        self.assertEqual(end, 74 + 2795)

    def test_single_segment_is_returned_whole(self) -> None:
        times = [index * 20.0 for index in range(50)]
        start, end, sessions = longest_segment_indices(times)
        self.assertEqual((start, end, sessions), (0, 50, 1))

    def test_empty_input(self) -> None:
        self.assertEqual(longest_segment_indices([]), (0, 0, 0))


class StartupGuardTest(unittest.TestCase):
    """The capture must not squat on ports, even when it fails to start."""

    def test_occupied_port_is_reported_with_its_owner(self) -> None:
        server, port = _listening_socket()
        self.addCleanup(server.close)
        with self.assertRaises(RuntimeError) as ctx:
            check_ports_free(port, _free_port())
        message = str(ctx.exception)
        self.assertIn(str(port), message)
        # Our own test process is the listener, so the PID must be resolvable
        # on any host with lsof; tolerate hosts without it.
        self.assertIn("already in use", message)

    def test_free_ports_pass(self) -> None:
        check_ports_free(_free_port(), _free_port())

    def test_silent_target_releases_the_server(self) -> None:
        """The "no RTT telemetry" path must not leak its GDB server.

        This is the lockout that followed the first failed capture: the server
        started, the target emitted nothing, ``wait_for_telemetry`` raised, and
        the reader was never returned to the caller - so nothing ever closed
        it.  Port 2331 stayed bound and every later attempt failed.
        """
        gdb_port, rtt_port = _free_port(), _free_port()
        # A stand-in for JLinkGDBServerCLExe: it binds both ports and then
        # stays silent, exactly like a relay attached to a halted core.
        script = Path(tempfile.mkdtemp()) / "silent_server.py"
        script.write_text(
            "#!/usr/bin/env python3\n"
            "import os, socket, time\n"
            "listeners = []\n"
            "for port in (int(os.environ['FAKE_GDB_PORT']),\n"
            "             int(os.environ['FAKE_RTT_PORT'])):\n"
            "    listener = socket.socket()\n"
            "    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)\n"
            "    listener.bind(('127.0.0.1', port))\n"
            "    listener.listen(5)\n"
            "    listeners.append(listener)\n"
            "while True:\n"
            "    for listener in listeners:\n"
            "        try:\n"
            "            listener.accept()\n"
            "        except OSError:\n"
            "            pass\n"
            "    time.sleep(0.05)\n",
            encoding="utf-8",
        )
        script.chmod(0o755)

        spawned: list[subprocess.Popen[bytes]] = []
        real_popen = subprocess.Popen

        def recording_popen(*args, **kwargs):
            proc = real_popen(*args, **kwargs)
            spawned.append(proc)
            return proc

        env = {
            "FAKE_GDB_PORT": str(gdb_port),
            "FAKE_RTT_PORT": str(rtt_port),
        }
        with unittest.mock.patch.dict(os.environ, env):
            with unittest.mock.patch.object(subprocess, "Popen", recording_popen):
                with self.assertRaises(TimeoutError) as ctx:
                    GdbServerRttReader(
                        server=str(script),
                        gdb="/usr/bin/true",
                        gdb_port=gdb_port,
                        rtt_port=rtt_port,
                        connect_timeout=1.0,
                    )
        self.assertIn("no RTT telemetry", str(ctx.exception))
        self.assertTrue(spawned, "the stand-in server should have been spawned")
        for proc in spawned:
            self.assertIsNotNone(
                proc.poll(),
                "a process spawned by the failed start is still running",
            )
        # The ports must be usable again right away.
        check_ports_free(gdb_port, rtt_port)


if __name__ == "__main__":
    unittest.main()
