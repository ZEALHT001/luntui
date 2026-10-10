#!/usr/bin/env python3
"""The body-motion filter's acceleration input must be the fore-aft axis.

The HI12's accelerometer channels are not the body frame's.  Measured on the
vehicle: gravity sits on channel 2 at rest (so channel 2 is vertical), but it
moves onto channel 1 when the body PITCHES and onto channel 0 when it ROLLS.
Channel 1 is therefore fore-aft, and a hand-push recording fixes the sign -
corr(delta wheel velocity, integral of ay_lin) = -0.999 against +0.019 for
ax_lin.

dd_x is not a display value: it is the acceleration measurement of the
body-motion Kalman filter, and it is the body-acceleration term in condition C
of the slip detector (|wheel acceleration| - |body acceleration|).  Feeding
either of those a sideways signal is silent - the filter still runs, the slip
metric still produces numbers - which is why the axis is pinned by a test
rather than left to the comment above the assignment.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "App" / "Src" / "vmc_probe.c"


def _dd_x_assignment() -> str:
    """The expression assigned to motion.dd_x."""
    source = SOURCE.read_text(encoding="utf-8")
    match = re.search(
        r"vmc_probe_state\.motion\.dd_x\s*=\s*(.+?);", source, re.DOTALL
    )
    if match is None:
        raise AssertionError("motion.dd_x is never assigned")
    return " ".join(match.group(1).split())


class ForeAftAccelerationTest(unittest.TestCase):
    def test_dd_x_comes_from_the_fore_aft_channel(self) -> None:
        """Channel 1, not channel 0."""
        expression = _dd_x_assignment()
        self.assertIn("body.ay_lin", expression)
        self.assertNotIn("body.ax_lin", expression)

    def test_dd_x_is_negated(self) -> None:
        """The IMU's fore-aft channel reads opposite to body-forward.

        Without the sign the filter is driven the wrong way, which is worse
        than the original bug: it would fight the wheel measurement instead of
        ignoring it.
        """
        expression = _dd_x_assignment()
        self.assertTrue(
            expression.startswith("-"),
            f"dd_x is not negated: {expression}",
        )

    def test_the_axis_choice_is_documented_with_its_evidence(self) -> None:
        """The next reader must not 'fix' this back to ax_lin."""
        source = SOURCE.read_text(encoding="utf-8")
        start = source.index("fore-aft linear acceleration in m/s2")
        comment = source[start - 200 : start + 2000]
        self.assertIn("PITCHES", comment)
        self.assertIn("ROLLS", comment)
        self.assertIn("ay_lin", comment)


if __name__ == "__main__":
    unittest.main()
