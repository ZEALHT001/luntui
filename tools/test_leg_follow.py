#!/usr/bin/env python3
"""Regression tests for the ground-following leg extension.

The behaviour under test: a leg that has stopped carrying the machine lets its
length target chase OUTWARD, so the support feedforward pushes that wheel back
down to the ground.  Only when the leg reaches VMC_LEG_LENGTH_MAX is further
body lift actual flight, which is what the airborne detector is for.

Without this the loop holds the channel-9 target, treats the lift as a position error,
and pulls the wheel UP toward the body - so the leg stops following long before
the stroke limit.

These are source-level checks, in the same style as test_jump_defaults.py: the
firmware has no host-buildable control layer, so the properties are asserted
against the shipped source.
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
    match = re.search(
        rf"^#define\s+{name}\s+(-?[0-9.]+)[fUu]*", source, re.MULTILINE
    )
    if match is None:
        raise AssertionError(f"macro {name} not found in vmc_probe.h")
    return float(match.group(1))


def _body(source: str, name: str) -> str:
    """Body of the *definition* of name, skipping any forward declaration."""
    pattern = re.compile(
        rf"^(?:static\s+)?\w+\s+{re.escape(name)}\s*\([^;{{]*\)\s*\{{",
        re.MULTILINE,
    )
    match = pattern.search(source)
    if match is None:
        raise AssertionError(f"definition of {name} not found")
    start = match.end() - 1
    depth = 0
    for index in range(start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start:index + 1]
    raise AssertionError(f"unterminated body for {name}")


def _has(haystack: str, needle: str, why: str) -> None:
    """Assert membership without dumping the whole function body on failure.

    unittest's own assertIn message quotes the entire haystack, which for a
    control-loop body is hundreds of lines of noise around a one-line failure.
    """
    if needle not in haystack:
        raise AssertionError(f"{why}\n  missing: {needle}")


class DefaultsTest(unittest.TestCase):
    def test_enable_is_a_flag(self) -> None:
        """The macro is only the power-on default; the web UI writes it.

        Pinning the exact value here meant every tuning session broke the
        suite, exactly like the jump-gate thresholds.  What actually has to
        hold is that it is a 0/1 flag - and that it stays individually
        overridable at runtime, which the RTT test below covers.
        """
        self.assertIn(_macro("VMC_LEG_FOLLOW_ENABLE_DEFAULT"), (0.0, 1.0))

    def test_scope_is_a_flag(self) -> None:
        self.assertIn(
            _macro("VMC_LEG_FOLLOW_MACHINE_WIDE_DEFAULT"), (0.0, 1.0)
        )

    def test_rates_are_positive_and_bounded(self) -> None:
        self.assertGreater(_macro("VMC_LEG_FOLLOW_RATE_DEFAULT"), 0.0)
        self.assertLess(_macro("VMC_LEG_FOLLOW_RATE_DEFAULT"), 1.0)
        self.assertGreater(_macro("VMC_LEG_FOLLOW_RETURN_RATE_DEFAULT"), 0.0)
        self.assertLess(_macro("VMC_LEG_FOLLOW_RETURN_RATE_DEFAULT"), 1.0)

class ControlPathTest(unittest.TestCase):
    def test_control_update_reads_the_runtime_values(self) -> None:
        """Reading the macros would make the web controls silently inert."""
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        for key in (
            "vmc_control.leg_follow_enable",
            "vmc_control.leg_follow_machine_wide",
            "vmc_control.leg_follow_rate",
            "vmc_control.leg_follow_return_rate",
        ):
            with self.subTest(key=key):
                _has(body, key, "the follow offsets are not applied to the target")

    def test_follow_is_gated_per_side_on_the_load_latch(self) -> None:
        """Only the leg that lost the ground may extend."""
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "VMC_AIRBORNE_SIDE_LEFT", "left side is not distinguished")
        _has(body, "VMC_AIRBORNE_SIDE_RIGHT", "right side is not distinguished")
        _has(body, "airborne_side_active", "the per-side load latch is unused")

    def test_fixed_scope_survives_only_under_auto_scope_off(self) -> None:
        """The old fixed per-leg / whole-machine choice is legacy.

        Auto scope now always extends the low side, so leg_follow_machine_wide
        is only read when auto_scope is 0 - kept so a contrast run can restore
        the previous behaviour without a reflash.
        """
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "leg_follow_machine_wide != 0U",
             "the fixed whole-machine scope can no longer be selected")
        _has(body, "follow_left = vmc_control.leg_follow_episode_active;",
             "the fixed whole-machine branch lost the left leg")
        _has(body, "follow_right = vmc_control.leg_follow_episode_active;",
             "the fixed whole-machine branch lost the right leg")
        _has(body, "follow_left = left_unloaded;",
             "the fixed per-leg branch lost the left leg")
        _has(body, "follow_right = right_unloaded;",
             "the fixed per-leg branch lost the right leg")

    def test_each_leg_is_capped_by_its_own_remaining_stroke(self) -> None:
        """A leg near full extension must stop there, not be dragged further."""
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "VMC_LEG_LENGTH_MAX - left_target_length",
             "the left leg is not capped by its own remaining stroke")
        _has(body, "VMC_LEG_LENGTH_MAX - right_target_length",
             "the right leg is not capped by its own remaining stroke")

    def test_offset_cannot_exceed_the_stroke(self) -> None:
        """The whole point is that extension stops at VMC_LEG_LENGTH_MAX.

        If the offset were unbounded the target clamp would still hold the
        command down, but the stored offset would keep growing and the leg
        would not return correctly once it reloaded.
        """
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "VMC_LEG_LENGTH_MAX - left_target_length",
             "the left offset is not bounded by the stroke")
        _has(body, "VMC_LEG_LENGTH_MAX - right_target_length",
             "the right offset is not bounded by the stroke")

    def test_follow_is_suspended_for_jump_and_recovery(self) -> None:
        """A jump and the self-righting sequence own the leg outright."""
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "vmc_getup_is_active() == 0U", "follow is not gated on recovery")
        _has(body, "jump_override == 0U", "follow is not gated on a jump")

    def test_offsets_reset_when_the_feature_stops_applying(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        # The disabled path must zero the state, not merely stop growing it.
        self.assertGreaterEqual(body.count("leg_follow_offset_left = 0.0f"), 1)
        self.assertGreaterEqual(body.count("leg_follow_offset_right = 0.0f"), 1)

    def test_init_restores_the_defaults(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_init")
        for key in (
            "leg_follow_enable",
            "leg_follow_machine_wide",
            "leg_follow_auto_scope",
            "leg_follow_roll_deadband",
            "leg_follow_episode_timeout_ms",
            "leg_follow_scope_confirm_ms",
            "leg_follow_scope_resolved",
            "leg_follow_low_side",
            "leg_follow_candidate_side",
            "leg_follow_candidate_ms",
            "leg_follow_episode_ms",
            "leg_follow_rate",
            "leg_follow_return_rate",
            "leg_follow_offset_left",
            "leg_follow_offset_right",
        ):
            with self.subTest(key=key):
                self.assertTrue(
                    f"vmc_control.{key} =" in body,
                    f"vmc_control_init does not initialise {key}",
                )

    def test_every_key_is_tunable_over_rtt(self) -> None:
        source = RTT.read_text(encoding="utf-8")
        for key in (
            "leg_follow_enable",
            "leg_follow_machine_wide",
            "leg_follow_auto_scope",
            "leg_follow_roll_deadband",
            "leg_follow_episode_timeout_ms",
            "leg_follow_rate",
            "leg_follow_return_rate",
        ):
            with self.subTest(key=key):
                self.assertTrue(
                    f'"{key}"' in source,
                    f"{key} is not parsed from the RTT payload",
                )

    def test_tuner_exposes_every_key(self) -> None:
        from tools.lqr_tuner.server import (
            BALANCE_PARAM_DEFAULT_MACROS,
            _firmware_balance_params,
        )

        resolved = _firmware_balance_params()
        for key in (
            "leg_follow_enable",
            "leg_follow_machine_wide",
            "leg_follow_auto_scope",
            "leg_follow_roll_deadband",
            "leg_follow_rate",
            "leg_follow_return_rate",
        ):
            with self.subTest(key=key):
                self.assertIn(key, BALANCE_PARAM_DEFAULT_MACROS)
                self.assertIn(key, resolved)
        self.assertAlmostEqual(
            resolved["leg_follow_rate"], _macro("VMC_LEG_FOLLOW_RATE_DEFAULT")
        )


class LowSideTest(unittest.TestCase):
    """The follow feature extends the LOW side, settled over a confirm window.

    The first version classified "which side unloaded, relative to roll" and
    chose per-leg or whole-machine from it.  That rule cannot work: pushing one
    side off a platform unloaded the PLATFORM side on the way through (its
    torque dipped under the 2.0 N.m set threshold for 120 ms), and that side is
    in the high half whenever the other one has gone down.  So the drop came
    out as "the high side unloaded, therefore the machine was lifted" and both
    legs extended - the machine ran both legs to their stroke limit at 38.7 deg
    of roll without reaching anything.

    The action is now the same in every case - extend the low side - so there is
    no lift/drop classifier left to get wrong.
    """

    def setUp(self) -> None:
        self.body = _body(SOURCE.read_text(encoding="utf-8"),
                          "vmc_leg_follow_low_side_from_roll")
        self.loop = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")

    def test_positive_roll_means_the_left_side_is_low(self) -> None:
        """body.roll is positive when the RIGHT side is high."""
        _has(self.body, "roll > deadband", "the positive-roll case is gone")
        _has(self.body, "return VMC_AIRBORNE_SIDE_LEFT;",
             "positive roll does not select the left side as the low one")

    def test_negative_roll_means_the_right_side_is_low(self) -> None:
        _has(self.body, "roll < -deadband", "the negative-roll case is gone")
        _has(self.body, "return VMC_AIRBORNE_SIDE_RIGHT;",
             "negative roll does not select the right side as the low one")

    def test_level_reports_no_side(self) -> None:
        """Inside the deadband the direction cannot be read."""
        _has(self.body, "return VMC_LEG_FOLLOW_SIDE_NONE;",
             "the level case does not report undecided")

    def test_no_lift_drop_classifier_remains(self) -> None:
        """The rule that could not be made to work must be gone.

        If a per-leg/whole-machine decision based on which side unloaded comes
        back, the platform case regresses to extending the platform side.
        """
        source = SOURCE.read_text(encoding="utf-8")
        self.assertNotIn("vmc_leg_follow_scope_from_roll", source)
        self.assertNotIn("leg_follow_episode_side", source)

    def test_a_single_frame_is_not_believed(self) -> None:
        """The classification has to hold for the confirmation window.

        This is the whole fix: the platform-side transient lasted 120 ms, so
        believing the first frame extended the wrong leg.
        """
        _has(self.loop, "leg_follow_candidate_ms", "there is no confirm timer")
        _has(self.loop, "scope_confirm_ms", "no confirmation window is applied")
        _has(self.loop,
             "vmc_control.leg_follow_candidate_ms >= scope_confirm_ms",
             "the candidate is used before the window elapses")
        _has(self.loop, "leg_follow_candidate_side = low_side_now;",
             "a changed candidate does not restart the clock")

    def test_a_new_episode_inherits_nothing(self) -> None:
        """A stale decision must not leak into the next episode."""
        start = self.loop.index("Fresh episode")
        window = self.loop[start:start + 600]
        _has(window, "leg_follow_scope_resolved = 0U;",
             "a new episode inherits the previous settlement")
        _has(window, "leg_follow_low_side = VMC_LEG_FOLLOW_SIDE_NONE;",
             "a new episode inherits the previous low side")

    def test_the_settled_side_is_held_for_the_episode(self) -> None:
        """The body swings through level while this feature pushes it."""
        _has(self.loop, "leg_follow_scope_resolved != 0U",
             "the loop does not check whether the side is already settled")
        _has(self.loop, "do not reconsider", "the hold is not documented")
        # Once settled the candidate machinery is cleared, so nothing can
        # re-open the decision mid-episode.
        _has(self.loop, "leg_follow_candidate_ms = 0U;",
             "the pending timer is not cleared once settled")

    def test_extend_is_the_low_side_in_both_scopes(self) -> None:
        """Auto scope drives the low side; the fixed scope is legacy only."""
        _has(self.loop, "leg_follow_auto_scope == 0U",
             "the fixed-scope fallback is gone")
        _has(self.loop, "leg_follow_low_side == VMC_AIRBORNE_SIDE_LEFT",
             "the left leg is not driven by the low side")
        _has(self.loop, "leg_follow_low_side == VMC_AIRBORNE_SIDE_RIGHT",
             "the right leg is not driven by the low side")

    def test_undecided_still_reaches_a_side_that_is_unloaded_and_low(self) -> None:
        """Conservative while undecided: never push the HIGH side."""
        _has(self.loop, "(left_unloaded != 0U && left_low != 0U)",
             "the undecided left condition is not both unloaded and low")
        _has(self.loop, "(right_unloaded != 0U && right_low != 0U)",
             "the undecided right condition is not both unloaded and low")

    def test_episode_outlives_the_latch(self) -> None:
        """Follow must not undo itself.

        Extending the leg raises joint torque, which trips the same load latch
        the follow waits on: measured, extend ~14 mm, release, unwind 102 mm,
        repeat - it never accumulated enough reach to touch anything.
        """
        _has(self.loop, "body_level != 0U",
             "the episode ends without waiting for the body to level")
        _has(self.loop, "leg_follow_episode_timeout_ms",
             "there is no timeout for an episode that never levels")
        _has(self.loop, "leg_follow_episode_ms", "the episode age is not tracked")

    def test_scope_is_published_to_telemetry(self) -> None:
        """A recording must show which leg was pushing."""
        source = RTT.read_text(encoding="utf-8")
        _has(source, "leg_follow_low_side", "the low side is not published")
        _has(source, "leg_follow_scope_resolved", "the settled flag is not published")
        _has(source, "leg_follow_candidate_side",
             "the pending candidate is not published")

    def test_scope_keys_are_tunable_over_rtt(self) -> None:
        source = RTT.read_text(encoding="utf-8")
        for key in ("leg_follow_auto_scope", "leg_follow_roll_deadband",
                    "leg_follow_episode_timeout_ms",
                    "leg_follow_scope_confirm_ms"):
            with self.subTest(key=key):
                _has(source, f'"{key}"', f"{key} is not parsed from RTT")


if __name__ == "__main__":
    unittest.main()
