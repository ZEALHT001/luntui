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
                self.assertIn(key, body)

    def test_follow_is_gated_per_side_on_the_load_latch(self) -> None:
        """Only the leg that lost the ground may extend."""
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        self.assertIn("VMC_AIRBORNE_SIDE_LEFT", body)
        self.assertIn("VMC_AIRBORNE_SIDE_RIGHT", body)
        self.assertIn("airborne_side_active", body)

    def test_whole_machine_scope_extends_both_legs(self) -> None:
        """With the machine-wide switch on, either side unloading extends both.

        The per-leg branch must still exist so the driving case can be
        restored without a reflash.
        """
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        self.assertIn("leg_follow_machine_wide", body)
        # Both sides OR-ed into one decision, and that decision handed to both.
        self.assertIn("follow_left = (left_unloaded != 0U || right_unloaded != 0U)",
                      body)
        self.assertIn("follow_right = follow_left;", body)
        # The per-leg path survives.
        self.assertIn("follow_left = left_unloaded;", body)
        self.assertIn("follow_right = right_unloaded;", body)

    def test_each_leg_is_capped_by_its_own_remaining_stroke(self) -> None:
        """A leg near full extension must stop there, not be dragged further."""
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        self.assertIn("VMC_LEG_LENGTH_MAX - left_target_length", body)
        self.assertIn("VMC_LEG_LENGTH_MAX - right_target_length", body)

    def test_offset_cannot_exceed_the_stroke(self) -> None:
        """The whole point is that extension stops at VMC_LEG_LENGTH_MAX.

        If the offset were unbounded the target clamp would still hold the
        command down, but the stored offset would keep growing and the leg
        would not return correctly once it reloaded.
        """
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        self.assertIn("VMC_LEG_LENGTH_MAX - left_target_length", body)
        self.assertIn("VMC_LEG_LENGTH_MAX - right_target_length", body)

    def test_follow_is_suspended_for_jump_and_recovery(self) -> None:
        """A jump and the self-righting sequence own the leg outright."""
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        self.assertIn("vmc_getup_is_active() == 0U", body)
        self.assertIn("jump_override == 0U", body)

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
            "leg_follow_rate",
            "leg_follow_return_rate",
            "leg_follow_offset_left",
            "leg_follow_offset_right",
        ):
            with self.subTest(key=key):
                self.assertIn(f"vmc_control.{key} =", body)

    def test_every_key_is_tunable_over_rtt(self) -> None:
        source = RTT.read_text(encoding="utf-8")
        for key in (
            "leg_follow_enable",
            "leg_follow_machine_wide",
            "leg_follow_rate",
            "leg_follow_return_rate",
        ):
            with self.subTest(key=key):
                self.assertIn(f'"{key}"', source)

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
                          "vmc_leg_follow_resolve_scope")

    def test_positive_roll_for_the_unloaded_side_extends_both(self) -> None:
        """Unloaded side HIGH => lifted => whole machine."""
        self.assertIn("signed_roll > deadband", self.body)
        self.assertIn("return 1U;", self.body)

    def test_negative_roll_for_the_unloaded_side_extends_only_it(self) -> None:
        """Unloaded side LOW => over an edge => that leg only."""
        self.assertIn("signed_roll < -deadband", self.body)
        self.assertIn("return 0U;", self.body)

    def test_right_uses_roll_and_left_uses_minus_roll(self) -> None:
        """body.roll is positive when the RIGHT side is high.

        Getting this backwards would invert both cases, so the mapping is
        asserted rather than left to a comment.
        """
        self.assertIn("signed_roll = (right_unloaded != 0U) ? roll : -roll;",
                      self.body)

    def test_deadband_holds_the_previous_episode_decision(self) -> None:
        """Near level is genuinely ambiguous, so do not flip mid-lift."""
        self.assertIn("have_previous", self.body)
        self.assertIn("previous_machine_wide", self.body)

    def test_no_history_defaults_to_per_leg(self) -> None:
        """The safe default: never lever the grounded leg on the first episode.

        Held at the level of the final fallback, which must not be 1U.
        """
        tail = self.body.rstrip().rstrip("}")
        self.assertIn("return (have_previous != 0U) ? previous_machine_wide : 0U;",
                      tail)

    def test_both_sides_unloaded_is_whole_machine(self) -> None:
        """A genuine flight has no grounded leg to lever against."""
        self.assertIn("left_unloaded != 0U && right_unloaded != 0U", self.body)

    def test_fixed_scope_is_still_reachable(self) -> None:
        """auto_scope = 0 must fall back to the old fixed behaviour."""
        self.assertIn("auto_scope == 0U", self.body)
        self.assertIn("fixed_machine_wide", self.body)

    def test_decision_is_latched_per_episode(self) -> None:
        """Re-deciding every cycle would flip the scope through level."""
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        self.assertIn("leg_follow_episode_active == 0U", body)
        self.assertIn("leg_follow_scope_machine_wide = vmc_leg_follow_resolve_scope",
                      body)
        self.assertIn("leg_follow_episode_active = 0U;", body)

    def test_history_bit_is_not_the_episode_flag(self) -> None:
        """The deadband fallback needs the previous EPISODE's answer.

        On the rising edge the episode flag is still 0 by construction, so
        reusing it as "have previous" would make the fallback dead code.  The
        two flags have to be separate.
        """
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        self.assertIn("vmc_control.leg_follow_scope_valid,", body)
        self.assertNotIn("vmc_control.leg_follow_episode_active,\n"
                         "          vmc_control.leg_follow_scope_machine_wide);",
                         body)
        self.assertIn("vmc_control.leg_follow_scope_valid = 1U;", body)

    def test_scope_is_published_to_telemetry(self) -> None:
        """A recording must show which mode was active, not just what was asked."""
        source = RTT.read_text(encoding="utf-8")
        self.assertIn("leg_follow_scope_machine_wide", source)

    def test_scope_keys_are_tunable_over_rtt(self) -> None:
        source = RTT.read_text(encoding="utf-8")
        for key in ("leg_follow_auto_scope", "leg_follow_roll_deadband"):
            with self.subTest(key=key):
                self.assertIn(f'"{key}"', source)


if __name__ == "__main__":
    unittest.main()
