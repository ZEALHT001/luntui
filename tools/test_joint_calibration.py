#!/usr/bin/env python3
"""Regression test for the joint zero-offset calibration.

The joint model angles are reconstructed from the motor's raw position with

    model = REF_MODEL + DIR * (raw - REF_RAW)      (wrapped to +-pi)

so REF_RAW is a zero offset, not a gain.

The important finding recorded here is negative: the right_ahead motor (id 2)
does not report a stable absolute position across power cycles.  At the same
physical pose it read -2.5378, -1.8507 and -2.5519 rad on three power-ups.
The steps, 39.4 and 40.2 degrees, match one motor-rotor revolution through the
~9:1 reduction (360/9 = 40.0 deg), so the multi-turn reference is rebuilt at
power-on and the joint angle is only known modulo ~40 degrees.

Re-calibrating REF_RAW from a single reading is therefore a trap: it bakes in
whichever turn came up and breaks again on the next power-on.  That is exactly
what happened on 2026-10-02 - a "fix" solved against the -1.8507 reading put
the leg 40 deg out once the motor came back up as -2.5519.

This test pins the long-standing constants and checks that they track the left
leg to a sane tolerance for the power-up state we measured.
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
        """The shipped references put the right leg next to the left one.

        Raw positions below were read with both legs vertical and fully
        extended.  The right_ahead value is from the power-up that agrees with
        the shipped constant; the deviation that remains is hand placement.
        """
        model_phi1 = _macro("VMC_REF_MODEL_PHI1")
        model_phi4 = _macro("VMC_REF_MODEL_PHI4")

        # Readings from the power-up state that matches the shipped constants.
        raw_right_back = 0.414092064
        raw_right_ahead = -2.55188084

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

        # The left leg at the same pose is the reference.
        self.assertAlmostEqual(phi1, 2.180948734, delta=0.02)
        self.assertAlmostEqual(phi4, 0.941273689, delta=0.03)

    def test_reference_is_not_the_anomalous_power_up_state(self) -> None:
        """Guard against re-introducing the 2026-10-02 mis-calibration.

        Solving REF_RAW against the -1.8507 reading produced -1.85989916.  That
        value is only correct while the motor happens to come up in that turn;
        with the normal -2.5519 reading it puts the leg 40 deg out.
        """
        raw_right_ahead = -2.55188084
        model_phi4 = _macro("VMC_REF_MODEL_PHI4")
        phi4 = joint_model_angle(
            raw_right_ahead,
            _macro("VMC_REF_RAW_RIGHT_AHEAD"),
            model_phi4,
            _macro("VMC_DIR_RIGHT_AHEAD"),
        )
        # A 40 degree error is ~0.70 rad; the shipped constant keeps this small.
        self.assertLess(abs(phi4 - 0.941273689), 0.05)

    def test_leg_directions_are_mirrored(self) -> None:
        """A rebuilt linkage may need DIR flipped; it must not silently change."""
        self.assertEqual(_macro("VMC_DIR_LEFT_BACK"), -1.0)
        self.assertEqual(_macro("VMC_DIR_LEFT_AHEAD"), -1.0)
        self.assertEqual(_macro("VMC_DIR_RIGHT_BACK"), 1.0)
        self.assertEqual(_macro("VMC_DIR_RIGHT_AHEAD"), 1.0)


if __name__ == "__main__":
    unittest.main()
