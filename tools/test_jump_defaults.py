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
    # Raised from the original 120 so the higher KP below has time to act.
    "VMC_JUMP_EXTEND_TIMEOUT_MS_DEFAULT": 160,
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

    def test_airborne_latch_is_not_single_frame(self) -> None:
        """One telemetry frame must not be able to declare the robot airborne.

        Both times were 1 ms, so a single wheel-speed spike or a single
        dropped torque sample cut the wheels mid-drive.  A landing impact
        produces exactly those spikes, which is why this looked like the robot
        randomly dying after a jump.
        """
        confirm = _macro("VMC_AIRBORNE_CONFIRM_MS_DEFAULT")
        release = _macro("VMC_AIRBORNE_RELEASE_MS_DEFAULT")
        self.assertGreaterEqual(confirm, 50.0)
        self.assertGreaterEqual(release, 100.0)
        self.assertGreater(release, confirm)

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

    def test_jump_length_gain_has_torque_headroom(self) -> None:
        """The length gain drives joint torque, which the motors cap.

        A position PD commands force proportional to the remaining error, so
        the gain - not the force limit - is what sets how hard the legs push.
        The measured peak was 36 N.m per joint against a 54 N.m motor limit at
        Kp = 3000, which leaves about 1.4x.  Keep the gain inside that.
        """
        kp = _macro("VMC_JUMP_LENGTH_KP_DEFAULT")
        self.assertGreater(kp, 3000.0, "the stock gain could not launch the robot")
        self.assertLessEqual(
            kp,
            4500.0,
            "beyond this the legs hit the 54 N.m joint limit and clip",
        )
        # The force cap must stay under the joint-limit equivalent (~390 N).
        self.assertLessEqual(_macro("VMC_JUMP_FORCE_MAX_DEFAULT"), 400.0)


SOURCE = ROOT / "App" / "Src" / "vmc_probe.c"


def _function_body(source: str, name: str) -> str:
    """Body of the *definition* of name, skipping the forward declaration."""
    pattern = re.compile(
        rf"^static\s+\w+\s+{re.escape(name)}\s*\([^;{{]*\)\s*\{{",
        re.MULTILINE,
    )
    match = pattern.search(source)
    if match is None:
        raise AssertionError(f"definition of {name} not found")
    body_start = match.end() - 1
    depth = 0
    for index in range(body_start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[body_start:index + 1]
    raise AssertionError(f"unterminated body for {name}")


class MovingJumpTest(unittest.TestCase):
    """A jump must be startable while the machine is already driving.

    The jump itself was never speed-gated; what made it stationary-only was
    that the wheel output was blanked for the WHOLE jump.  That also kills the
    balance loop through the 300 ms of COMPRESS and EXTEND, which the robot
    spends on the ground, so a moving jump tips over before it can leave.
    """

    def test_moving_jump_is_enabled_by_default(self) -> None:
        self.assertEqual(_macro("VMC_JUMP_MOVING_ENABLE_DEFAULT"), 1.0)

    def test_wheel_blanking_does_not_cover_the_ground_phases(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        body = _function_body(source, "vmc_jump_blocks_wheel_output")
        # The default (moving-enabled) path must key off the airborne-only
        # predicate.  vmc_jump_is_active() may appear, but only inside the
        # legacy branch guarded by jump_moving_enable == 0.
        self.assertIn("vmc_jump_joint_lqr_active()", body)
        self.assertIn("vmc_control.jump_moving_enable == 0U", body)
        legacy = body.index("vmc_control.jump_moving_enable == 0U")
        # Whatever the legacy guard returns must come before the default
        # return, and the default return must be the airborne-only one.
        tail = body[body.rindex("return vmc_jump_joint_lqr_active();"):]
        self.assertTrue(tail.startswith("return vmc_jump_joint_lqr_active();"))
        self.assertGreater(len(tail), 0)
        self.assertLess(legacy, body.rindex("return vmc_jump_joint_lqr_active();"))

    def test_the_leg_path_still_sees_every_stage(self) -> None:
        """The leg torque path and the airborne detector keep the wide gate."""
        source = SOURCE.read_text(encoding="utf-8")
        wide = _function_body(source, "vmc_jump_is_active_or_aborted")
        self.assertIn("vmc_jump_is_active()", wide)
        # Exactly the two consumers that need the wide gate: the airborne
        # detector and the leg-torque (Tp) path.
        self.assertEqual(source.count("vmc_jump_is_active_or_aborted()"), 2)

    def test_an_aborted_jump_still_returns_the_wheels(self) -> None:
        """An aborted jump must not leave the wheels dead.

        The wheel gate used to include the "stage == ABORT and S1 is still up"
        latch.  Since the only way out of ABORT is moving S1 to the middle, an
        abort the pilot never acknowledged held the wheels off indefinitely -
        no balance loop, and the machine fell over.  Jump aborts are ordinary
        (a landing that tips past the attitude limit, one missed CAN frame),
        so that failure was intermittent.
        """
        source = SOURCE.read_text(encoding="utf-8")
        body = _function_body(source, "vmc_jump_blocks_wheel_output")
        self.assertNotIn(
            "VMC_JUMP_STAGE_ABORT",
            body,
            "the abort latch must not gate the wheel output",
        )
        self.assertNotIn(
            "RC_SW_UP",
            body,
            "the wheel output must not depend on the jump switch position",
        )

    def test_the_wide_gate_still_carries_the_abort_latch(self) -> None:
        """The leg target and the detector legitimately keep the latch."""
        source = SOURCE.read_text(encoding="utf-8")
        wide = _function_body(source, "vmc_jump_is_active_or_aborted")
        self.assertIn("VMC_JUMP_STAGE_ABORT", wide)

    def test_every_jump_exit_clears_the_airborne_latch(self) -> None:
        """A finished jump must hand the wheels back on BOTH exit paths.

        The LAND phase ends either when leg torque confirms contact or when it
        times out.  Only the first path used to clear the airborne latch, so a
        jump that ended on the timeout left airborne_active set and the wheels
        cut until the normal detector released it - which needs 500 ms of
        sustained leg load and never arrives if the landing is soft.
        """
        source = SOURCE.read_text(encoding="utf-8")
        body = _function_body(source, "vmc_jump_finish")
        self.assertIn(
            "vmc_clear_airborne_latch()",
            body,
            "vmc_jump_finish must clear the latch so both exits behave alike",
        )


if __name__ == "__main__":
    unittest.main()
