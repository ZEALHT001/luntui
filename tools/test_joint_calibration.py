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


class JointCalibrationTest(unittest.TestCase):
    # Raw joint positions read while both legs were locked in the SAME pose
    # with an aluminium fixture (2026-10-05).  Stable to 0.001 rad, the
    # resolution of the telemetry channel.
    LOCKED_POSE = {
        "left_back": 2.230000,
        "left_ahead": -1.936000,
        "right_back": 0.779000,
        "right_ahead": 2.387000,
    }

    def _leg_angles(self, side: str) -> tuple[float, float]:
        raw = self.LOCKED_POSE
        phi1 = joint_model_angle(
            raw[f"{side}_back"],
            _macro(f"VMC_REF_RAW_{side.upper()}_BACK"),
            _macro("VMC_REF_MODEL_PHI1"),
            _macro(f"VMC_DIR_{side.upper()}_BACK"),
        )
        phi4 = joint_model_angle(
            raw[f"{side}_ahead"],
            _macro(f"VMC_REF_RAW_{side.upper()}_AHEAD"),
            _macro("VMC_REF_MODEL_PHI4"),
            _macro(f"VMC_DIR_{side.upper()}_AHEAD"),
        )
        return phi1, phi4

    def test_both_legs_agree_in_a_symmetric_locked_pose(self) -> None:
        """The two legs must compute the same angles from the same pose."""
        left_phi1, left_phi4 = self._leg_angles("left")
        right_phi1, right_phi4 = self._leg_angles("right")
        self.assertAlmostEqual(right_phi1, left_phi1, delta=0.005)
        self.assertAlmostEqual(right_phi4, left_phi4, delta=0.005)

    def test_corrected_constants_are_not_the_old_ones(self) -> None:
        """Guard against silently reverting to the pre-2026-10-05 offsets.

        Those offsets left the legs 19 mm apart in computed length (phi1
        +3.64 deg, phi4 -5.31 deg) while the machine stood level, which is
        calibration error, not pose.
        """
        self.assertAlmostEqual(_macro("VMC_REF_RAW_RIGHT_BACK"), 0.495861, places=5)
        self.assertAlmostEqual(_macro("VMC_REF_RAW_RIGHT_AHEAD"), -2.630482, places=5)
        # The left leg was trusted throughout and must not have moved.
        self.assertAlmostEqual(_macro("VMC_REF_RAW_LEFT_BACK"), 2.51313968, places=7)
        self.assertAlmostEqual(_macro("VMC_REF_RAW_LEFT_AHEAD"), -3.20170302, places=7)

    def test_the_correction_size_is_the_measured_one(self) -> None:
        """The right offsets moved by the amount the locked pose demanded."""
        back_shift = _macro("VMC_REF_RAW_RIGHT_BACK") - 0.432402611
        ahead_shift = _macro("VMC_REF_RAW_RIGHT_AHEAD") - (-2.53776646)
        self.assertAlmostEqual(back_shift, 0.063458, places=5)
        self.assertAlmostEqual(ahead_shift, -0.092716, places=5)

    def test_leg_directions_are_mirrored(self) -> None:
        """A rebuilt linkage may need DIR flipped; it must not silently change."""
        self.assertEqual(_macro("VMC_DIR_LEFT_BACK"), -1.0)
        self.assertEqual(_macro("VMC_DIR_LEFT_AHEAD"), -1.0)
        self.assertEqual(_macro("VMC_DIR_RIGHT_BACK"), 1.0)
        self.assertEqual(_macro("VMC_DIR_RIGHT_AHEAD"), 1.0)


if __name__ == "__main__":
    unittest.main()
