#!/usr/bin/env python3
"""Tests for the roll-compensation diagnostic."""

from __future__ import annotations

import unittest

from tools.roll_comp_report import LEG_LENGTH_MIN, analyse, verdict


def _rows(offset: float, left: float, right: float, roll: float,
          count: int = 100, measured_left: float | None = None,
          measured_right: float | None = None,
          load_left: float = 11.0, load_right: float = 11.0,
          air_left: float = 0.0, air_right: float = 0.0) -> list[dict[str, str]]:
    rows = []
    for i in range(count):
        row = {
            "mcu_time_ms": str(1000 + i * 20),
            "roll": str(roll),
            "roll_comp_offset": str(offset),
            "leg_target_left": str(left),
            "leg_target_right": str(right),
            # Default: the legs track their targets exactly.
            "leg_length_left": str(left if measured_left is None else measured_left),
            "leg_length_right": str(right if measured_right is None else measured_right),
            "airborne_side_torque_left": str(load_left),
            "airborne_side_torque_right": str(load_right),
            "airborne_side_active_left": str(air_left),
            "airborne_side_active_right": str(air_right),
        }
        rows.append(row)
    return rows


class AnalyseTest(unittest.TestCase):
    def test_missing_channel_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            analyse([{"mcu_time_ms": "1", "roll": "0"}])

    def test_clip_is_detected(self) -> None:
        # Offset asks for +-80 mm, but the retract side is pinned on MIN, so
        # the delivered differential is short of the request near the 18 cm floor.
        rows = _rows(offset=0.08, left=0.25, right=LEG_LENGTH_MIN, roll=0.2,
                     measured_left=0.25, measured_right=LEG_LENGTH_MIN)
        result = analyse(rows)
        self.assertAlmostEqual(float(result["want_diff_max"]), 0.16, places=6)
        self.assertAlmostEqual(float(result["commanded_diff_max"]), 0.07, places=6)
        self.assertEqual(int(result["clipped_total_frames"]), len(rows))

    def test_no_clip_when_stroke_allows_it(self) -> None:
        rows = _rows(offset=0.05, left=0.33, right=0.23, roll=0.1)
        result = analyse(rows)
        self.assertEqual(int(result["clipped_total_frames"]), 0)
        self.assertAlmostEqual(
            float(result["want_diff_max"]),
            float(result["commanded_diff_max"]),
            places=6,
        )

    def test_untracked_legs_are_detected(self) -> None:
        """Commanded well, achieved badly - the low-leg-length failure mode."""
        rows = _rows(
            offset=0.048, left=0.218, right=0.121, roll=0.1227,
            measured_left=0.229, measured_right=0.193,
        )
        result = analyse(rows)
        self.assertAlmostEqual(
            float(result["commanded_diff_max"]), 0.097, places=6
        )
        self.assertAlmostEqual(
            float(result["achieved_diff_max"]), 0.036, places=6
        )


class VerdictTest(unittest.TestCase):
    def test_zero_offset_reads_as_not_responding(self) -> None:
        lines = verdict(analyse(_rows(0.0, 0.28, 0.28, 0.15)))
        self.assertIn("NOT RESPONDING", lines[0])

    def test_clipped_capture_says_so(self) -> None:
        rows = _rows(0.08, 0.25, LEG_LENGTH_MIN, roll=0.2,
                     measured_left=0.25, measured_right=LEG_LENGTH_MIN)
        lines = verdict(analyse(rows))
        joined = " ".join(lines)
        self.assertIn("CLIPPED", joined)
        self.assertIn("LIMIT CONTACT", joined)

    def test_healthy_capture_reports_no_clip(self) -> None:
        lines = verdict(analyse(_rows(0.05, 0.33, 0.23, roll=0.1)))
        joined = " ".join(lines)
        self.assertIn("NOT CLIPPED", joined)
        self.assertIn("NO LIMIT CONTACT", joined)
        self.assertIn("TRACKED", joined)

    def test_untracked_capture_is_called_out(self) -> None:
        """The compensator is fine; the legs are not following."""
        lines = verdict(analyse(_rows(
            0.048, 0.218, 0.121, roll=0.1227,
            measured_left=0.229, measured_right=0.193,
        )))
        joined = " ".join(lines)
        self.assertIn("RESPONDING", joined)
        self.assertIn("NOT CLIPPED", joined)
        self.assertIn("NOT TRACKED", joined)

    def test_lifted_leg_is_reported_from_its_load_latch(self) -> None:
        """The contact flag is Shanhai FN, not leg length.

        The right leg was lifted, so it could not resist the length loop at all;
        that is why a commanded retraction produced a measured extension.
        """
        lines = verdict(analyse(_rows(
            0.048, 0.218, 0.121, roll=0.1227,
            measured_left=0.229, measured_right=0.213,
            load_left=14.9, load_right=1.4, air_left=0.0, air_right=1.0,
        )))
        joined = " ".join(lines)
        self.assertIn("LEG OFF THE GROUND", joined)
        self.assertIn("RIGHT", joined)

    def test_both_legs_loaded_is_reported(self) -> None:
        lines = verdict(analyse(_rows(0.05, 0.33, 0.23, roll=0.1)))
        joined = " ".join(lines)
        self.assertIn("BOTH LEGS LOADED", joined)

if __name__ == "__main__":
    unittest.main()
