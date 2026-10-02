#!/usr/bin/env python3
"""Regression test for the joint zero-offset calibration.

The joint model angles are reconstructed from the motor's raw position with

    model = REF_MODEL + DIR * (raw - REF_RAW)      (wrapped to +-pi)

so REF_RAW is a zero offset.  These constants are hardware specific: they were
captured with both legs vertical and fully extended, and the right leg was
re-zeroed on 2026-10-02 after it started reporting a 0.4442 m leg length -
past the 0.385 m mechanical stop - while the left leg reported 0.3892 m.

This test pins the calibration to the raw positions recorded at that pose.  If
it fails after a deliberate recalibration, re-derive the reference with

    REF_RAW = raw - (target - REF_MODEL)/DIR

and update the expected values here from the new acceptance run.
"""

from __future__ import annotations

import math
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "App" / "Inc" / "vmc_probe.h"


def _macro(name: str) -> float:
    source = HEADER.read_text(encoding="utf-8")
    match = re.search(rf"^#define\s+{name}\s+(-?[0-9.]+)f?", source, re.MULTILINE)
    if match is None:
        raise AssertionError(f"macro {name} not found as a numeric literal")
    return float(match.group(1))


def _wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def joint_model_angle(raw: float, raw_ref: float, model_ref: float, direction: float) -> float:
    return _wrap(model_ref + direction * (raw - raw_ref))


class JointCalibrationTest(unittest.TestCase):
    def test_right_leg_reference_matches_the_left_leg_pose(self) -> None:
        """At the 2026-10-02 pose both legs agreed after re-zeroing."""
        model_phi1 = _macro("VMC_REF_MODEL_PHI1")
        model_phi4 = _macro("VMC_REF_MODEL_PHI4")

        # Raw positions the right motors reported while both legs were held
        # vertical and fully extended.
        raw_right_back = 0.416762352
        raw_right_ahead = -1.85072899

        phi1 = joint_model_angle(
            raw_right_back,
            _macro("VMC_REF_RAW_RIGHT_BACK"),
            model_phi1,
            _macro("VMC_DIR_RIGHT_BACK"),
        )
        phi4 = joint_model_angle(
            raw_right_ahead,
            _macro("VMC_REF_RAW_RIGHT_AHEAD"),
            model_phi4,
            _macro("VMC_DIR_RIGHT_AHEAD"),
        )

        # The left leg's reading at the same pose is the ground truth.
        self.assertAlmostEqual(phi1, 2.180948734, places=5)
        self.assertAlmostEqual(phi4, 0.941273689, places=5)

    def test_leg_directions_are_mirrored(self) -> None:
        """A rebuilt linkage may need DIR flipped; it must not silently change."""
        self.assertEqual(_macro("VMC_DIR_LEFT_BACK"), -1.0)
        self.assertEqual(_macro("VMC_DIR_LEFT_AHEAD"), -1.0)
        self.assertEqual(_macro("VMC_DIR_RIGHT_BACK"), 1.0)
        self.assertEqual(_macro("VMC_DIR_RIGHT_AHEAD"), 1.0)


if __name__ == "__main__":
    unittest.main()
