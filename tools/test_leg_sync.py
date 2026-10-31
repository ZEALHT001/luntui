#!/usr/bin/env python3
"""Regression tests for the leg-synchronisation (anti-splay) loop.

The behaviour under test: the left and right virtual legs are commanded from
the same theta target, so nothing in the balance model ties
(left_theta - right_theta) to zero.  Body roll is common-mode and every other
feedback term is blind to a pure left/right difference, which lets the legs
splay apart - worst at long leg lengths, where the gravity de-stabilising term
is largest, and with the wheels unloaded.

The fix is a differential term on the joint torque:

    e     = left_theta_error - right_theta_error
    Tp    = clamp(kp * e + kd * edot, +/- limit)
    left -= Tp ; right += Tp

These are source-level checks, in the same style as test_leg_follow.py: the
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
SERVER = ROOT / "tools" / "lqr_tuner" / "server.py"
WEB_INDEX = ROOT / "tools" / "lqr_tuner" / "web" / "index.html"
WEB_MAIN = ROOT / "tools" / "lqr_tuner" / "web" / "src" / "main.js"


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
    if needle not in haystack:
        raise AssertionError(f"{why}\n  missing: {needle}")


class DefaultsTest(unittest.TestCase):
    def test_leg_knob_minimum_is_eighteen_centimetres(self) -> None:
        self.assertEqual(_macro("VMC_LEG_KNOB_MIN_DEFAULT"), 0.180)
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_remote_leg_length_target")
        _has(body, "VMC_LEG_KNOB_MIN_DEFAULT", "the knob target does not use its 18 cm endpoint")
        self.assertNotIn("(double)VMC_LEG_TARGET_SHORT +", body)

    def test_enable_is_a_flag(self) -> None:
        """The macro is only the power-on default; the web UI writes it."""
        self.assertIn(_macro("VMC_LEG_SYNC_ENABLE_DEFAULT"), (0.0, 1.0))

    def test_gains_are_positive_and_bounded(self) -> None:
        self.assertGreater(_macro("VMC_LEG_SYNC_KP_DEFAULT"), 0.0)
        self.assertLess(_macro("VMC_LEG_SYNC_KP_DEFAULT"), 500.0)
        self.assertGreaterEqual(_macro("VMC_LEG_SYNC_KD_DEFAULT"), 0.0)
        self.assertLess(_macro("VMC_LEG_SYNC_KD_DEFAULT"), 50.0)

    def test_limit_stays_inside_the_virtual_leg_torque_range(self) -> None:
        """Tp is the virtual-leg torque, so its ceiling is VMC_LEG_TP_MAX."""
        limit = _macro("VMC_LEG_SYNC_TORQUE_LIMIT_DEFAULT")
        self.assertGreater(limit, 0.0)
        self.assertLessEqual(limit, _macro("VMC_LEG_TP_MAX"))


class ControlPathTest(unittest.TestCase):
    def test_knob_gear_thresholds_match_the_old_presets(self) -> None:
        """Discrete gear logic now derives from the continuous channel 9 knob."""
        gear = _body(SOURCE.read_text(encoding="utf-8"), "vmc_remote_leg_length_gear")
        for needle in (
            "VMC_LEG_GEAR_THRESHOLD_TOL",
            "VMC_LEG_TARGET_LONG",
            "VMC_LEG_TARGET_MID",
            "return VMC_LEG_GEAR_HIGH",
            "return VMC_LEG_GEAR_MID",
            "return VMC_LEG_GEAR_LOW",
        ):
            with self.subTest(needle=needle):
                _has(gear, needle, "the knob gear mapping is incomplete")

    def test_error_is_the_left_minus_right_difference(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_balance_update")
        _has(
            body,
            "left_lqr_theta_error_raw - right_lqr_theta_error_raw",
            "the sync error is not the left/right theta difference",
        )

    def test_rate_term_uses_the_same_difference(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_balance_update")
        _has(
            body,
            "left_lqr_d_theta - right_lqr_d_theta",
            "the sync damping is not the left/right theta-rate difference",
        )

    def test_term_is_applied_with_opposite_signs(self) -> None:
        """A differential term must push the two legs in opposite directions.

        Applied with the same sign it would be a common-mode torque, i.e. a
        second balance loop instead of an anti-splay loop.
        """
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        left = body.find("update_leg_length_control(&vmc_probe_state.left_leg")
        right = body.find("update_leg_length_control(&vmc_probe_state.right_leg")
        self.assertGreaterEqual(left, 0)
        self.assertGreater(right, left)
        left_call = body[left:right]
        right_call = body[right:]
        _has(
            left_call,
            "-vmc_balance.leg_sync_torque,",
            "the left leg does not take the negated sync torque",
        )
        _has(
            right_call,
            "vmc_balance.leg_sync_torque,",
            "the right leg does not take the sync torque",
        )
        self.assertNotIn(
            "-vmc_balance.leg_sync_torque",
            right_call,
            "the right leg took the negated term as well, cancelling the pair",
        )

    def test_applied_in_the_joint_torque_path_not_only_the_lqr_tp(self) -> None:
        """It has to survive tp_output_enable = 0.

        When the balance loop is switched back to the phi0 PD the LQR Tp
        command is not used at all, so folding the sync term into
        left/right_tp_raw would make it silently inert.  It is summed into the
        same phi0_torque_total that both paths feed instead.
        """
        body = _body(SOURCE.read_text(encoding="utf-8"), "update_leg_length_control")
        _has(
            body,
            "leg_sync_torque",
            "the joint-torque path ignores the sync term",
        )
        _has(
            body,
            "phi0_torque + lqr_tp_torque + leg_sync_torque",
            "the sync term is not summed into the joint-torque command",
        )

    def test_only_active_with_the_high_leg_knob_band(self) -> None:
        """The channel-9 knob must be at or above the old high target.

        The stair length override keeps the legs near 0.18 m after a climb
        while channel 9 may still command a long target; that latched override
        must remain a separate exclusion.
        """
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_balance_update")
        _has(body, "vmc_remote_leg_length_gear() == VMC_LEG_GEAR_HIGH",
             "the high leg-knob band is not gated")
        self.assertNotIn("remote_control.s[0]", body, "leg sync still depends on the old S2")
        _has(body, "stair_length_override_active == 0U",
             "the latched short-leg stair state is not excluded")
        _has(body, "leg_sync_high_gear != 0U", "the high-gear flag is not used")

    def test_inert_while_getup_or_jump_owns_the_legs(self) -> None:
        """Those sequences drive both legs from their own tuned profiles."""
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_balance_update")
        _has(body, "vmc_getup_is_active()", "the sync loop runs during get-up")
        _has(
            body,
            "vmc_jump_is_active_or_aborted()",
            "the sync loop runs during a jump",
        )

    def test_torque_is_clamped_to_its_own_limit(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_balance_update")
        _has(
            body,
            "leg_sync_torque_limit",
            "the sync term is not bounded by leg_sync_torque_limit",
        )


class RuntimeTuningTest(unittest.TestCase):
    def test_rtt_exposes_every_knob(self) -> None:
        """The loop has to be tunable without a reflash."""
        source = RTT.read_text(encoding="utf-8")
        for key in (
            "leg_sync_enable",
            "leg_sync_kp",
            "leg_sync_kd",
            "leg_sync_torque_limit",
        ):
            with self.subTest(key=key):
                _has(source, f'"{key}"', f"{key} is not accepted over RTT")

    def test_web_ui_exposes_every_knob(self) -> None:
        index = WEB_INDEX.read_text(encoding="utf-8")
        main = WEB_MAIN.read_text(encoding="utf-8")
        for element, key in (
            ("legSyncEnable", "leg_sync_enable"),
            ("legSyncKp", "leg_sync_kp"),
            ("legSyncKd", "leg_sync_kd"),
            ("legSyncTorqueLimit", "leg_sync_torque_limit"),
        ):
            with self.subTest(key=key):
                _has(index, f'id="{element}"', f"{element} input is missing")
                _has(main, f'"{key}"', f"{key} is not sent by the page")

    def test_server_defaults_match_the_firmware_header(self) -> None:
        source = SERVER.read_text(encoding="utf-8")
        for key, macro in (
            ("leg_sync_enable", "VMC_LEG_SYNC_ENABLE_DEFAULT"),
            ("leg_sync_kp", "VMC_LEG_SYNC_KP_DEFAULT"),
            ("leg_sync_kd", "VMC_LEG_SYNC_KD_DEFAULT"),
            ("leg_sync_torque_limit", "VMC_LEG_SYNC_TORQUE_LIMIT_DEFAULT"),
        ):
            with self.subTest(key=key):
                _has(
                    source,
                    f'"{key}": ("{macro}"',
                    f"{key} is not imported from {macro}",
                )


class TelemetryTest(unittest.TestCase):
    def test_error_and_torque_are_published(self) -> None:
        """left_theta/right_theta alone cannot prove the loop fired."""
        source = RTT.read_text(encoding="utf-8")
        _has(source, "leg_sync_error", "leg_sync_error is not in telemetry")
        _has(source, "leg_sync_torque", "leg_sync_torque is not in telemetry")


if __name__ == "__main__":
    unittest.main()
