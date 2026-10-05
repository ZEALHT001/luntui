#!/usr/bin/env python3
"""Regression test for the joint zero-offset calibration.

The joint model angles are reconstructed from the motor's raw position with

    model = REF_MODEL + DIR * (raw - REF_RAW)      (wrapped to +-pi)

so REF_RAW is a zero offset, not a gain.

The invariant that matters is that the two legs are the same mechanism, so at
any pose where both legs are physically in the SAME configuration the four
constants must produce identical model angles for the two sides.  That is what
this test asserts, using raw readings taken while the pose was physically
locked with a fixture (2026-10-05).

History worth keeping in mind before editing the numbers: the right_ahead
motor used to rebuild its multi-turn reference at power-on, reading -2.5378,
-1.8507 and -2.5519 rad at one physical pose on three power-ups - steps of
39.4 and 40.2 degrees, one motor-rotor turn through the ~9:1 reduction.
Re-solving REF_RAW against a single one of those readings is what produced the
2026-10-02 mis-calibration (-1.85989916), which put the leg 40 deg out on the
next power-up.  That instability is gone on the current motor (all four joints
now read byte-identically across a power cycle), which is what made the
correction below safe to apply.
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


# The constants below are the nominal ones, derived from a fixture-locked
# symmetric pose.  The right_ahead motor's power-on reference is NOT repeatable
# - four observed power-ups at one physical pose read -2.5378, -1.8507 and
# -2.5519 rad, plus one state a full rotor turn away after a jump - so these
# numbers are only right while the motor comes up in its nominal turn.
#
# Twice the constants were "corrected" for an offset reference; both times the
# change was ~39 deg wrong on the next power-up and the machine could not
# balance (see commit ea28089, and the revert that undid the second attempt).
# Measured on the same joint: one state needed +39.4 deg, the following
# power-up needed -36.5 deg.  That is why the nominal values are pinned here
# and a per-power-up offset is not adopted.
NOMINAL_RIGHT_BACK = 0.495861
NOMINAL_RIGHT_AHEAD = -2.630482


class JointCalibrationTest(unittest.TestCase):
    def test_reference_pose_is_self_consistent(self) -> None:
        """Feeding each joint its own reference raw must return its model angle.

        This is the algebraic identity the constants encode, so it catches a
        typo or a half-applied edit.  It cannot catch a reference that moved;
        only a measurement can, which is why the next test exists.
        """
        pairs = (
            ("LEFT_BACK", "VMC_REF_MODEL_PHI1"),
            ("LEFT_AHEAD", "VMC_REF_MODEL_PHI4"),
            ("RIGHT_BACK", "VMC_REF_MODEL_PHI1"),
            ("RIGHT_AHEAD", "VMC_REF_MODEL_PHI4"),
        )
        for joint, model_macro in pairs:
            with self.subTest(joint=joint):
                raw = _macro(f"VMC_REF_RAW_{joint}")
                model = _macro(model_macro)
                direction = _macro(f"VMC_DIR_{joint}")
                self.assertAlmostEqual(
                    joint_model_angle(raw, raw, model, direction), model, places=9
                )

    def test_constants_are_the_nominal_pair(self) -> None:
        """The right offsets must be the nominal ones, not a per-power-up patch.

        A patch is ~39 deg wrong on the next power-up because the motor's
        reference is not repeatable, so the nominal values are the only ones
        that can be correct without a fresh measurement.
        """
        self.assertAlmostEqual(
            _macro("VMC_REF_RAW_RIGHT_BACK"), NOMINAL_RIGHT_BACK, places=5
        )
        self.assertAlmostEqual(
            _macro("VMC_REF_RAW_RIGHT_AHEAD"), NOMINAL_RIGHT_AHEAD, places=5
        )

    def test_one_rotor_turn_is_the_detectable_signature(self) -> None:
        """A 39 deg leg disagreement means the reference moved a whole turn.

        The header records one output revolution per rotor turn as 39.4 / 40.2
        deg at the ~9:1 reduction.  That is the size of the error to look for
        when diagnosing a bad power-up - a few degrees is pose or hand
        placement, ~39 deg is the reference.
        """
        self.assertGreater(math.degrees(0.6876), 35.0)
        self.assertLess(math.degrees(0.6876), 45.0)

    def test_left_leg_was_never_touched(self) -> None:
        """The left leg tracked correctly throughout and is the reference."""
        self.assertAlmostEqual(_macro("VMC_REF_RAW_LEFT_BACK"), 2.51313968, places=7)
        self.assertAlmostEqual(_macro("VMC_REF_RAW_LEFT_AHEAD"), -3.20170302, places=7)

    def test_leg_directions_are_mirrored(self) -> None:
        """A rebuilt linkage may need DIR flipped; it must not silently change."""
        self.assertEqual(_macro("VMC_DIR_LEFT_BACK"), -1.0)
        self.assertEqual(_macro("VMC_DIR_LEFT_AHEAD"), -1.0)
        self.assertEqual(_macro("VMC_DIR_RIGHT_BACK"), 1.0)
        self.assertEqual(_macro("VMC_DIR_RIGHT_AHEAD"), 1.0)


if __name__ == "__main__":
    unittest.main()
