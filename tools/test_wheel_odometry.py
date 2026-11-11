#!/usr/bin/env python3
"""Tests for the wheel-odometry drift report."""

from __future__ import annotations

import csv
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools import wheel_odometry as module  # noqa: E402


def write_capture(path: Path, rows: list[tuple[float, float, float]]) -> Path:
    """rows: (mcu_time_ms, left_raw, right_raw)."""
    path.mkdir(parents=True, exist_ok=True)
    target = path / "kalman_samples.csv"
    with target.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["mcu_time_ms", "kf_wheel_left_raw", "kf_wheel_right_raw"])
        for stamp, left, right in rows:
            writer.writerow([stamp, left, right])
    return target


class SignTests(unittest.TestCase):
    def test_right_minus_left_is_the_body_direction(self) -> None:
        """The right motor is mirrored, so the sum cancels and the difference
        is the body-forward speed."""
        rows = module.load_rows(
            write_capture(
                Path(tempfile.mkdtemp()) / "cap",
                [(0.0, 0.5, -0.5), (20.0, 0.5, -0.5)],
            )
        )
        _t, velocity, _x, gap = module.integrate(rows, 1.0)
        self.assertAlmostEqual(velocity[0], -0.5)
        self.assertAlmostEqual(gap[0], 0.0)

    def test_motion_that_cancels_in_the_sum_still_registers(self) -> None:
        rows = module.load_rows(
            write_capture(
                Path(tempfile.mkdtemp()) / "cap",
                [(0.0, 0.3, -0.3), (20.0, 0.3, -0.3)],
            )
        )
        _t, velocity, _x, _gap = module.integrate(rows, 1.0)
        self.assertGreater(abs(velocity[0]), 0.1)


class IntegrationTests(unittest.TestCase):
    def test_constant_speed_integrates_to_distance(self) -> None:
        # 0.1 m/s for one second at 50 Hz, in the mirrored convention.
        rows = [(index * 20.0, 0.1, -0.1) for index in range(51)]
        loaded = module.load_rows(write_capture(Path(tempfile.mkdtemp()) / "c", rows))
        _t, _v, position, _gap = module.integrate(loaded, 1.0)
        # v = (R - L) / 2 = -0.1 m/s, so the displacement is -0.1 m.
        self.assertAlmostEqual(position[-1], -0.1, places=3)

    def test_scale_is_applied(self) -> None:
        rows = [(index * 20.0, 0.1, -0.1) for index in range(51)]
        loaded = module.load_rows(write_capture(Path(tempfile.mkdtemp()) / "c", rows))
        _t, _v, position, _gap = module.integrate(loaded, 0.5)
        self.assertAlmostEqual(position[-1], -0.05, places=3)

    def test_gap_does_not_fabricate_distance(self) -> None:
        rows = [(0.0, 0.05, -0.05), (20.0, 0.05, -0.05), (5000.0, 0.05, -0.05)]
        loaded = module.load_rows(write_capture(Path(tempfile.mkdtemp()) / "c", rows))
        _t, _v, position, _gap = module.integrate(loaded, 1.0)
        # Only the first 20 ms step is real; the 4.98 s gap must be skipped.
        self.assertLess(abs(position[-1]), 0.01)


class StaleBufferTests(unittest.TestCase):
    def test_prefix_before_the_last_jump_is_dropped(self) -> None:
        rows = [
            {"mcu_time_ms": "10"},
            {"mcu_time_ms": "30"},
            {"mcu_time_ms": "100000"},
            {"mcu_time_ms": "100020"},
        ]
        kept = module.drop_stale_prefix(rows)
        self.assertEqual(len(kept), 2)
        self.assertEqual(kept[0]["mcu_time_ms"], "100000")


if __name__ == "__main__":
    unittest.main()
