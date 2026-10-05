#!/usr/bin/env python3
"""Regression test for the jump stage timing defaults.

The jump state machine advances a stage when EITHER the stage has timed out OR
the leg has reached that stage's target length, so the timeouts are upper
bounds rather than delays.  That made them look harmless to shorten - but they
still have to outlast the legs' physical travel.  Every one of them was once
left at 1 ms, which collapsed the whole 0.18 -> 0.25 -> 0.18 sequence into
about 3 ms: the machine never extended, so jumping was silently impossible
with no error reported anywhere.

These tests keep the values in a range where a jump can actually happen.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "App" / "Inc" / "vmc_probe.h"

# Stage timeouts as the jump feature was written, plus the phase durations the
# state machine actually depends on.
STAGE_TIMEOUTS = {
    "VMC_JUMP_COMPRESS_TIMEOUT_MS_DEFAULT": 180,
    "VMC_JUMP_EXTEND_TIMEOUT_MS_DEFAULT": 120,
    "VMC_JUMP_RETRACT_TIMEOUT_MS_DEFAULT": 140,
    "VMC_JUMP_LAND_TIMEOUT_MS_DEFAULT": 600,
    "VMC_JUMP_COOLDOWN_MS_DEFAULT": 1500,
    "VMC_JUMP_LAND_CONFIRM_MS_DEFAULT": 80,
}


def _macro(name: str) -> float:
    source = HEADER.read_text(encoding="utf-8")
    match = re.search(
        rf"^#define\s+{name}\s+(-?[0-9.]+)[fUu]*", source, re.MULTILINE
    )
    if match is None:
        raise AssertionError(f"macro {name} not found in vmc_probe.h")
    return float(match.group(1))


class JumpDefaultsTest(unittest.TestCase):
    def test_stage_timeouts_allow_real_leg_travel(self) -> None:
        for name, expected in STAGE_TIMEOUTS.items():
            with self.subTest(macro=name):
                self.assertAlmostEqual(_macro(name), expected)

    def test_no_stage_timeout_is_degenerate(self) -> None:
        """A 1 ms stage makes the jump finish before the legs move."""
        for name in STAGE_TIMEOUTS:
            with self.subTest(macro=name):
                self.assertGreaterEqual(
                    _macro(name),
                    50.0,
                    f"{name} is too short for the stage to complete",
                )

    def test_total_timeout_leaves_room_for_every_stage(self) -> None:
        """The overall budget must exceed the sum of the stage timeouts."""
        total = _macro("VMC_JUMP_TOTAL_TIMEOUT_MS_DEFAULT")
        stages = sum(
            _macro(name)
            for name in (
                "VMC_JUMP_COMPRESS_TIMEOUT_MS_DEFAULT",
                "VMC_JUMP_EXTEND_TIMEOUT_MS_DEFAULT",
                "VMC_JUMP_RETRACT_TIMEOUT_MS_DEFAULT",
                "VMC_JUMP_LAND_TIMEOUT_MS_DEFAULT",
            )
        )
        self.assertGreater(total, stages)

    def test_stage_timeouts_survive_the_firmware_clamp(self) -> None:
        """The firmware clamps each stage timeout to [1, 2000] ms.

        Anything at the ceiling would be silently truncated, so the useful
        values have to sit inside it.
        """
        for name in STAGE_TIMEOUTS:
            with self.subTest(macro=name):
                self.assertLessEqual(_macro(name), 2000.0)

    def test_jump_is_enabled_by_default(self) -> None:
        self.assertEqual(_macro("VMC_JUMP_ENABLE_DEFAULT"), 1.0)

    def test_jump_can_lift_the_machine(self) -> None:
        """The jump force must exceed the machine's weight, with margin.

        About 20 kg means ~200 N just to stand: a 100 N per-leg ceiling left
        no net upward force at all, so the legs extended and the robot never
        left the ground.  Also note the jump force is clamped by the general
        leg-force ceiling, so raising only one of the two does nothing.
        """
        per_leg = _macro("VMC_JUMP_FORCE_MAX_DEFAULT")
        ceiling = _macro("VMC_LEG_FORCE_MAX_DEFAULT")
        weight = 20.0 * 9.81
        self.assertGreaterEqual(per_leg, 300.0, "jump force is too small to lift off")
        self.assertGreater(
            ceiling,
            per_leg,
            "the general leg-force ceiling would clip the jump to below its own limit",
        )
        # Net lift, as a multiple of g.
        net_g = (2.0 * per_leg - weight) / weight
        self.assertGreater(net_g, 1.0, f"only {net_g:.2f} g of net lift")


if __name__ == "__main__":
    unittest.main()
