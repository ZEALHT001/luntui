#!/usr/bin/env python3
"""Regression tests for the slip LQR weight switching.

Ported from the Nanhang sentry wheel-leg firmware (开源参考/南航/SENTRY_LEG),
whose response to a slip is:

    Fitting_K[i][2] = Fitting_K[i][3] = 0;      // drop the YAW columns
    Fitting_K[i][4] *= 2;  Fitting_K[i][6] *= 2; // double the LEG ANGLE columns

Only the posture half transfers to this firmware: its LQR states are
theta, dtheta, x, dx, phi, dphi - there is no yaw state, and heading is held by
a separate yaw_hold loop.  These tests pin what was actually implemented and,
just as importantly, what was deliberately left out.
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
    if needle not in haystack:
        raise AssertionError(f"{why}\n  missing: {needle}")


def _hasnt(haystack: str, needle: str, why: str) -> None:
    """Like _has, but for absence - and without dumping the whole body."""
    if needle in haystack:
        raise AssertionError(f"{why}\n  unexpected: {needle}")


class DefaultsTest(unittest.TestCase):
    def test_on_by_default_after_the_comparison_run(self) -> None:
        self.assertEqual(_macro("VMC_SLIP_LQR_ENABLE_DEFAULT"), 1.0)

    def test_faithful_scales(self) -> None:
        """Reference doubles the leg ANGLE and leaves the rate alone."""
        self.assertAlmostEqual(_macro("VMC_SLIP_LQR_THETA_SCALE_DEFAULT"), 2.0)
        self.assertAlmostEqual(_macro("VMC_SLIP_LQR_DTHETA_SCALE_DEFAULT"), 1.0)

    def test_current_scale_matches_the_reference(self) -> None:
        self.assertAlmostEqual(
            _macro("VMC_SLIP_WHEEL_CURRENT_SCALE_DEFAULT"), 0.6
        )

    def test_the_balancing_authority_cost_is_documented(self) -> None:
        """This is the term that fights the machine; say so where it is set.

        The wheel current IS the balancing authority on a wheel-leg balancer,
        so cutting it gives up authority to arrest a pitch excursion exactly
        while the machine is slipping.  A reader adjusting the number has to
        know that before they do.
        """
        header = HEADER.read_text(encoding="utf-8")
        start = header.index("Slip current scaling")
        comment = header[start : start + 3000]
        self.assertIn("balancing authority", comment)
        self.assertIn("1.0", comment)

    def test_the_yaw_gap_is_documented(self) -> None:
        """The next reader must not think the yaw half was forgotten.

        The reference zeroes yaw columns; this LQR has no yaw state.  That is a
        deliberate omission and has to stay written down, because a reader who
        knows the reference will otherwise assume it was missed.
        """
        header = HEADER.read_text(encoding="utf-8")
        start = header.index("Slip LQR weight switching")
        comment = header[start : start + 4000]
        self.assertIn("DOES NOT TRANSFER", comment)
        self.assertIn("yaw", comment)
        # And the earlier wrong claim must be corrected in place rather than
        # silently dropped.
        self.assertIn("was wrong", comment)


class ApplicationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.body = _body(SOURCE.read_text(encoding="utf-8"),
                          "vmc_balance_update")

    def test_the_scale_multiplies_the_error_not_the_gain(self) -> None:
        """K is rewritten every cycle by vmc_update_lqr_gain().

        Scaling the K entries in place would either be wiped next cycle or, if
        applied after the update, accumulate.  Scaling the error is stateless.
        """
        _has(self.body, "left_lqr_theta_error * left_theta_scale",
             "the left angle term is not scaled through the error")
        _has(self.body, "right_lqr_theta_error * right_theta_scale",
             "the right angle term is not scaled through the error")
        _has(self.body, "left_lqr_d_theta_limited * left_dtheta_scale",
             "the left rate term is not scaled through the error")
        _has(self.body, "right_lqr_d_theta_limited * right_dtheta_scale",
             "the right rate term is not scaled through the error")
        # No in-place mutation of the gain arrays anywhere in the LQR calc.
        _hasnt(self.body, "left_k[0] *=",
               "the gain array is mutated in place instead of the error")
        _hasnt(self.body, "left_k[1] *=",
               "the gain array is mutated in place instead of the error")
        _hasnt(self.body, "right_k[0] *=",
               "the gain array is mutated in place instead of the error")

    def test_both_outputs_are_scaled(self) -> None:
        """The reference scales the wheel row AND the joint row."""
        _has(self.body, "vmc_probe_state.lqr.left_k[0] * (left_lqr_theta_error * left_theta_scale)",
             "the left wheel row is not scaled")
        _has(self.body, "vmc_probe_state.lqr.left_k[1] * (left_lqr_theta_error * left_theta_scale)",
             "the left virtual-leg row is not scaled")
        _has(self.body, "vmc_probe_state.lqr.right_k[1] *",
             "the right virtual-leg row is not scaled")

    def test_each_side_uses_its_own_latch(self) -> None:
        """The reference is per-side; one slipping wheel must not rescale both."""
        _has(self.body, "body_motion_kf_slip_left", "the left latch is not used")
        _has(self.body, "body_motion_kf_slip_right", "the right latch is not used")
        _has(self.body, "left_slipping", "there is no left-side decision")
        _has(self.body, "right_slipping", "there is no right-side decision")

    def test_scale_is_one_when_not_slipping(self) -> None:
        """A scale of 1.0 is the identity, so the disabled path is exact."""
        self.assertGreaterEqual(self.body.count("? slip_theta_scale : 1.0"), 1)
        self.assertGreaterEqual(self.body.count("? slip_dtheta_scale : 1.0"), 1)

    def test_scale_is_bounded(self) -> None:
        """A malformed runtime value must not be able to blow up the gains."""
        _has(self.body, "clamp_double(vmc_balance.slip_lqr_theta_scale, 0.0, 10.0)",
             "the angle scale is not clamped")
        _has(self.body, "clamp_double(vmc_balance.slip_lqr_dtheta_scale, 0.0, 10.0)",
             "the rate scale is not clamped")

    def test_disabled_and_while_jumping_is_inert(self) -> None:
        """A jump owns the joints; do not fight it with a slip rescale."""
        _has(self.body, "vmc_balance.slip_lqr_enable != 0U",
             "there is no master enable")
        _has(self.body, "vmc_control.enabled != 0U",
             "the response is not gated on the controller being enabled")
        _has(self.body, "vmc_jump_joint_lqr_active() == 0U",
             "the response is not gated against the jump joint controller")

    def test_applied_scale_is_published(self) -> None:
        """A recording must be able to prove the response fired."""
        _has(self.body, "slip_lqr_applied_left",
             "the applied left scale is not published")
        source = RTT.read_text(encoding="utf-8")
        _has(source, "slip_lqr_applied_left", "the scale is not in telemetry")
        _has(source, "slip_lqr_applied_right", "the scale is not in telemetry")

    def test_applied_current_scale_is_published(self) -> None:
        output_body = _body(SOURCE.read_text(encoding="utf-8"),
                            "vmc_balance_apply_wheel_output")
        _has(output_body, "slip_curr_applied_left",
             "the applied left current scale is not published")
        source = RTT.read_text(encoding="utf-8")
        _has(source, "slip_curr_applied_left",
             "the current scale is not in telemetry")

    def test_current_scale_is_gated_by_the_same_master_switch(self) -> None:
        """slip_lqr_enable is the master switch for the whole response."""
        output_body = _body(SOURCE.read_text(encoding="utf-8"),
                            "vmc_balance_apply_wheel_output")
        # Not just declared - actually consulted by both sides' conditions.
        # Declaring `slip_curr_enabled` and then not using it would leave the
        # reduction permanently on, which is the failure this guards.
        _has(output_body, "slip_curr_enabled != 0U && body_motion_kf_slip_left",
             "the left current reduction does not consult the master switch")
        _has(output_body, "slip_curr_enabled != 0U && body_motion_kf_slip_right",
             "the right current reduction does not consult the master switch")
        _has(output_body, "clamp_double(vmc_balance.slip_wheel_current_scale, 0.0, 1.0)",
             "the current scale is not clamped")

    def test_current_is_scaled_after_the_final_clamp(self) -> None:
        """The reference scales the LIMITED final current, not the raw demand.

        Scaling before the clamp would let the clamp undo it; scaling after
        matches `joint_m[i]->A = joint_m[i]->A * 0.6f` in the reference, which
        runs on the value that was just limited.
        """
        output_body = _body(SOURCE.read_text(encoding="utf-8"),
                            "vmc_balance_apply_wheel_output")
        clamp_at = output_body.index("vmc_wheel_control.right_current =")
        scale_at = output_body.index("slip_curr_applied_left")
        self.assertLess(clamp_at, scale_at,
                        "the current is scaled before it is assembled/limited")

    def test_every_key_is_tunable_over_rtt(self) -> None:
        source = RTT.read_text(encoding="utf-8")
        for key in (
            "slip_lqr_enable",
            "slip_lqr_theta_scale",
            "slip_lqr_dtheta_scale",
            "slip_wheel_current_scale",
        ):
            with self.subTest(key=key):
                _has(source, f'"{key}"', f"{key} is not parsed from RTT")


if __name__ == "__main__":
    unittest.main()
