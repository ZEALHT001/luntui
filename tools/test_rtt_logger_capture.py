#!/usr/bin/env python3
"""Tests for the JLinkRTTLogger-based capture analyser."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from tools.rtt_logger_capture import (
    coverage_summary,
    longest_segment_indices,
    parse_log,
)


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


if __name__ == "__main__":
    unittest.main()
