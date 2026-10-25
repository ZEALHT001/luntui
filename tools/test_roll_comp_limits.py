#!/usr/bin/env python3
"""Pin the roll-compensation limit to the physical stroke.

The compensator drives the two targets symmetrically about the preset,
``left = clamp(T + offset)`` and ``right = clamp(T - offset)``, so the offset
can only be as large as the smaller of the two remaining strokes.  These tests
hold the constant, both clamp sites and the documented per-preset headroom
together, because the failure mode is silent: an offset larger than the stroke
is not rejected, it is quietly clipped per leg, and while it is being clipped
the pair stops levelling and starts raising the machine instead.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "App" / "Inc" / "vmc_probe.h"
SOURCE = ROOT / "App" / "Src" / "vmc_probe.c"
RTT = ROOT / "App" / "Src" / "vmc_lqr_rtt.c"


def _macro(name: str) -> float:
    source = HEADER.read_text(encoding="utf-8")
    match = re.search(rf"^#define\s+{name}\s+([0-9.]+)f?", source, re.MULTILINE)
    if match is None:
        raise AssertionError(f"macro {name} not found as a numeric literal")
    return float(match.group(1))


class RollCompLimitTest(unittest.TestCase):
    def test_ceiling_is_half_the_stroke(self) -> None:
        """The only offset that can never be clipped is half the travel."""
        stroke = _macro("VMC_LEG_LENGTH_MAX") - _macro("VMC_LEG_LENGTH_MIN")
        self.assertAlmostEqual(
            _macro("VMC_ROLL_LENGTH_COMP_LIMIT_MAX"), stroke / 2.0
        )

    def test_default_uses_the_whole_available_authority(self) -> None:
        """The power-on value must be the physical limit, not a fraction."""
        self.assertAlmostEqual(
            _macro("VMC_ROLL_LENGTH_COMP_LIMIT_DEFAULT"),
            _macro("VMC_ROLL_LENGTH_COMP_LIMIT_MAX"),
        )

    def test_ceiling_is_a_plain_literal(self) -> None:
        """The tuner and its tests read these tokens as numbers.

        Writing the ceiling as ``(MAX - MIN) / 2`` would be the same value in
        C but would break every parser that reads the header.
        """
        source = HEADER.read_text(encoding="utf-8")
        for name in (
            "VMC_ROLL_LENGTH_COMP_LIMIT_DEFAULT",
            "VMC_ROLL_LENGTH_COMP_LIMIT_MAX",
        ):
            with self.subTest(name=name):
                match = re.search(
                    rf"^#define\s+{name}\s+(.+)$", source, re.MULTILINE
                )
                self.assertIsNotNone(match)
                self.assertRegex(match.group(1).strip(), r"^[0-9.]+f$")

    def test_both_clamp_sites_use_the_ceiling(self) -> None:
        """The stance clamp and the RTT setter must share one limit.

        They used to hold their own copies, so raising one left the other
        still capping the runtime value at the old number.
        """
        for path, label in ((SOURCE, "stance clamp"), (RTT, "RTT setter")):
            with self.subTest(site=label):
                source = path.read_text(encoding="utf-8")
                self.assertTrue(
                    "VMC_ROLL_LENGTH_COMP_LIMIT_MAX" in source,
                    f"{path.name} does not clamp roll_comp_limit to "
                    "VMC_ROLL_LENGTH_COMP_LIMIT_MAX",
                )

    def test_no_bare_roll_comp_ceiling_remains(self) -> None:
        """A hard-coded 0.10 ceiling is exactly the drift this test prevents."""
        for path in (SOURCE, RTT):
            source = path.read_text(encoding="utf-8")
            for line in source.splitlines():
                if "roll_comp_limit" not in line and "clamp_float" not in line:
                    continue
                with self.subTest(path=path.name, line=line.strip()):
                    self.assertNotIn("0.10", line)

    def test_short_preset_is_limited_by_the_stroke_not_the_constant(self) -> None:
        """Documented symmetric headroom: SHORT 0, MID 90, LONG 25 mm.

        This is why raising the constant alone cannot fix the platform case -
        at SHORT the retract side has nothing left.  At LONG the EXTEND side
        is the binding one, so this is a min() over both, not a retract-only
        number.
        """
        leg_min = _macro("VMC_LEG_LENGTH_MIN")
        leg_max = _macro("VMC_LEG_LENGTH_MAX")
        expected = {
            "VMC_LEG_TARGET_SHORT": 0.000,
            "VMC_LEG_TARGET_MID": 0.090,
            "VMC_LEG_TARGET_LONG": 0.025,
        }
        for name, headroom in expected.items():
            with self.subTest(preset=name):
                target = _macro(name)
                self.assertAlmostEqual(
                    min(target - leg_min, leg_max - target), headroom, places=6
                )

    def test_min_keeps_a_margin_above_the_measured_stop(self) -> None:
        """The software floor is 10 mm above the measured mechanical stop.

        It was 0.120, which let every length target be driven 40 mm past the
        real end stop: a capture showed a target pinned at 0.120 while the leg
        sat at 0.161 +-1 mm for 40 s.  The stop measures 0.160 m.
        """
        self.assertAlmostEqual(_macro("VMC_LEG_LENGTH_MIN"), 0.170)

    def test_no_preset_sits_below_the_mechanical_stop(self) -> None:
        """A preset under MIN would be clipped the instant it is applied."""
        leg_min = _macro("VMC_LEG_LENGTH_MIN")
        for name in (
            "VMC_LEG_TARGET_SHORT",
            "VMC_LEG_TARGET_MID",
            "VMC_LEG_TARGET_LONG",
        ):
            with self.subTest(preset=name):
                self.assertGreaterEqual(_macro(name), leg_min)

    def test_constant_never_exceeds_what_a_preset_can_use_at_the_centre(self) -> None:
        """Sanity: the ceiling is the global max over all presets."""
        leg_min = _macro("VMC_LEG_LENGTH_MIN")
        leg_max = _macro("VMC_LEG_LENGTH_MAX")
        centre = (leg_min + leg_max) / 2.0
        self.assertAlmostEqual(
            min(centre - leg_min, leg_max - centre),
            _macro("VMC_ROLL_LENGTH_COMP_LIMIT_MAX"),
        )


if __name__ == "__main__":
    unittest.main()
