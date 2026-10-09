#!/usr/bin/env python3
"""Regression tests for the ground-following leg extension.

The behaviour under test: a leg that has stopped carrying the machine lets its
length target chase OUTWARD, so the support feedforward pushes that wheel back
down to the ground.  Only when the leg reaches VMC_LEG_LENGTH_MAX is further
body lift actual flight, which is what the airborne detector is for.

Without this the loop holds the S2 preset, treats the lift as a position error,
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

    def test_whole_machine_scope_extends_both_legs(self) -> None:
        """Whole-machine scope hands the same decision to both legs.

        The scope is no longer read straight from leg_follow_machine_wide -
        it is either forced to that value by auto_scope = 0, or resolved from
        the roll direction - but the hand-off to both legs is unchanged.
        """
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "follow_left = vmc_control.leg_follow_episode_active;",
             "whole-machine scope must hand the episode state to the left leg")
        _has(body, "follow_right = vmc_control.leg_follow_episode_active;",
             "whole-machine scope must hand the episode state to the right leg")
        # The per-leg path survives, one branch per remembered side.
        _has(body, "leg_follow_episode_side == VMC_AIRBORNE_SIDE_LEFT",
             "the per-leg left branch is missing")
        _has(body, "leg_follow_episode_side == VMC_AIRBORNE_SIDE_RIGHT",
             "the per-leg right branch is missing")

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
            "leg_follow_scope_resolved",
            "leg_follow_episode_side",
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


class AutoScopeTest(unittest.TestCase):
    """The follow scope is chosen from the roll direction, per episode.

    A single fixed scope cannot be right: "one side unloaded" covers two
    opposite situations.  Lifting one side on flat ground needs BOTH legs to
    extend so the body rises with it; one wheel going over an edge needs ONLY
    that leg to reach down, because lengthening the grounded leg levers the
    body further over the edge.

    Both are in the recordings, with the same side unloaded and opposite roll
    signs, so the sign is the usable discriminator.
    """

    def setUp(self) -> None:
        self.body = _body(SOURCE.read_text(encoding="utf-8"),
                          "vmc_leg_follow_scope_from_roll")

    def test_positive_roll_for_the_unloaded_side_extends_both(self) -> None:
        """Unloaded side HIGH => lifted => whole machine."""
        _has(self.body, "signed_roll > deadband",
             "the lifted case is not tested against the deadband")
        _has(self.body, "return 1U;", "the lifted case does not return whole-machine")

    def test_negative_roll_for_the_unloaded_side_extends_only_it(self) -> None:
        """Unloaded side LOW => over an edge => that leg only."""
        _has(self.body, "signed_roll < -deadband",
             "the dropped case is not tested against the deadband")
        _has(self.body, "return 0U;", "the dropped case does not return per-leg")

    def test_right_uses_roll_and_left_uses_minus_roll(self) -> None:
        """body.roll is positive when the RIGHT side is high.

        Getting this backwards would invert both cases, so the mapping is
        asserted rather than left to a comment.
        """
        _has(self.body, "signed_roll = (right_unloaded != 0U) ? roll : -roll;",
             "the roll sign is not mapped to the correct side")

    def test_deadband_holds_the_previous_episode_decision(self) -> None:
        """Near level is ambiguous, so report "undecided" rather than a guess."""
        _has(self.body, "return 2U;", "the deadband does not report undecided")

    def test_both_sides_unloaded_is_whole_machine(self) -> None:
        """A genuine flight has no grounded leg to lever against."""
        _has(self.body, "left_unloaded != 0U && right_unloaded != 0U",
             "the both-sides-latched case is not handled")

    def test_decision_is_latched_per_episode(self) -> None:
        """Re-deciding every cycle would flip the scope through level."""
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "leg_follow_scope_resolved == 0U",
             "the scope is not held once decided")
        _has(body, "vmc_leg_follow_scope_from_roll",
             "the scope is not derived from the roll sign")
        _has(body, "leg_follow_episode_active = 0U;",
             "an episode never ends")

    def test_a_new_episode_inherits_nothing(self) -> None:
        """A brief noise latch must not poison the next real lift.

        Measured: a 0.2 s spurious latch left the scope on per-leg, and the
        next real lift - whose own first latch landed at roll +0.047, inside
        the deadband - inherited that stale answer and extended one leg.
        """
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        start = body.index("Fresh episode")
        window = body[start:start + 700]
        _has(window, "leg_follow_scope_resolved = 0U;",
             "a new episode inherits the previous scope resolution")
        _has(window, "leg_follow_scope_machine_wide = 0U;",
             "a new episode inherits the previous scope")

    def test_fixed_scope_is_still_reachable(self) -> None:
        """auto_scope = 0 must fall back to the old fixed behaviour."""
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "leg_follow_auto_scope == 0U",
             "the fixed-scope fallback is gone")
        _has(body, "leg_follow_machine_wide != 0U",
             "the fixed-scope value is no longer read")

    def test_scope_is_settled_after_the_latch_lets_go(self) -> None:
        """The decision must not depend on the instantaneous latch.

        On a lift the wheel leaves the ground BEFORE the body has tilted, and
        the latch drops again as soon as the legs carry load - so the sign has
        to be evaluated against the REMEMBERED side while the episode is live.
        """
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "leg_follow_episode_side ==",
             "the decision ignores which side unloaded")
        _has(body, "VMC_LEG_FOLLOW_SIDE_NONE",
             "the both-sides-at-once case is not recognised")

    def test_episode_outlives_the_latch(self) -> None:
        """Follow must not undo itself.

        Extending the leg raises joint torque, which trips the same load latch
        the follow waits on.  Measured: extend ~14 mm, release, unwind 102 mm,
        repeat - it never accumulated enough reach to touch anything.
        """
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        # The episode only ends once the body is level (or the timeout fires).
        _has(body, "body_level != 0U",
             "the episode ends without waiting for the body to level")
        _has(body, "leg_follow_episode_timeout_ms",
             "there is no timeout for an episode that never levels")
        _has(body, "leg_follow_episode_ms",
             "the episode age is not tracked")

    def test_reach_continues_while_the_side_is_still_low(self) -> None:
        """Per-leg scope keeps reaching after the latch lets go."""
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "left_low", "the left side-low condition is missing")
        _has(body, "right_low", "the right side-low condition is missing")
        _has(body, "(right_unloaded != 0U || right_low != 0U)",
             "the right leg stops reaching when its latch lets go")
        _has(body, "(left_unloaded != 0U || left_low != 0U)",
             "the left leg stops reaching when its latch lets go")

    def test_scope_is_published_to_telemetry(self) -> None:
        """A recording must show which mode was active, not just what was asked."""
        source = RTT.read_text(encoding="utf-8")
        _has(source, "leg_follow_scope_machine_wide",
             "the resolved scope is not published")
        _has(source, "leg_follow_scope_resolved",
             "the settled flag is not published")

    def test_scope_keys_are_tunable_over_rtt(self) -> None:
        source = RTT.read_text(encoding="utf-8")
        for key in ("leg_follow_auto_scope", "leg_follow_roll_deadband",
                    "leg_follow_episode_timeout_ms"):
            with self.subTest(key=key):
                _has(source, f'"{key}"', f"{key} is not parsed from RTT")


if __name__ == "__main__":
    unittest.main()
