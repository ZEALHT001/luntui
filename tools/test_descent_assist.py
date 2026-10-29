#!/usr/bin/env python3
"""Regression tests for automatic platform descent assist.

The action extends both legs while the machine is airborne, confirms touchdown
from measured joint load, restores wheel output immediately, and ramps the legs
back to the length captured at lift-off.  These source-level checks match the
existing stair and leg-sync tests because the control layer is not host
buildable.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "App" / "Inc" / "vmc_probe.h"
SOURCE = ROOT / "App" / "Src" / "vmc_probe.c"
RTT = ROOT / "App" / "Src" / "vmc_lqr_rtt.c"
CAPTURE = ROOT / "tools" / "rtt_kalman_capture.py"
SERVER = ROOT / "tools" / "lqr_tuner" / "server.py"
WEB_INDEX = ROOT / "tools" / "lqr_tuner" / "web" / "index.html"
WEB_MAIN = ROOT / "tools" / "lqr_tuner" / "web" / "src" / "main.js"


def _macro(name: str) -> float:
    source = HEADER.read_text(encoding="utf-8")
    match = re.search(
        rf"^#define\s+{name}\s+(-?[0-9.]+)[fUu]*", source, re.MULTILINE
    )
    if match is None:
        raise AssertionError(f"macro {name} not found")
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


class DefaultsTest(unittest.TestCase):
    def test_extend_and_rates_are_inside_the_leg_workspace(self) -> None:
        leg_min = _macro("VMC_LEG_LENGTH_MIN")
        leg_max = _macro("VMC_LEG_LENGTH_MAX")
        extend = _macro("VMC_DESCENT_EXTEND_LENGTH_DEFAULT")
        self.assertGreaterEqual(extend, leg_min)
        self.assertLessEqual(extend, leg_max)
        self.assertGreater(_macro("VMC_DESCENT_EXTEND_RATE_DEFAULT"), 0.0)
        self.assertGreater(_macro("VMC_DESCENT_RETURN_RATE_DEFAULT"), 0.0)

    def test_contact_uses_a_short_confirmation_not_the_generic_release(self) -> None:
        self.assertGreater(_macro("VMC_DESCENT_CONTACT_TORQUE_DEFAULT"), 0.0)
        self.assertLessEqual(_macro("VMC_DESCENT_CONTACT_CONFIRM_MS_DEFAULT"), 50.0)
        self.assertLess(
            _macro("VMC_DESCENT_CONTACT_CONFIRM_MS_DEFAULT"),
            _macro("VMC_AIRBORNE_RELEASE_MS_DEFAULT"),
        )

    def test_descent_profile_has_no_constant_airborne_support_force(self) -> None:
        self.assertEqual(_macro("VMC_DESCENT_SUPPORT_FORCE_DEFAULT"), 0.0)
        self.assertGreater(_macro("VMC_DESCENT_LENGTH_KP_DEFAULT"), 0.0)
        self.assertGreater(_macro("VMC_DESCENT_LENGTH_KD_DEFAULT"), 0.0)


class StateMachineTest(unittest.TestCase):
    def test_start_requires_high_knob_or_stair_locked_stance(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_descent_start_is_allowed")
        for needle in (
            "remote_control_stair_enabled()",
            "vmc_remote_leg_length_gear() == VMC_LEG_GEAR_HIGH",
            "stair_length_override_active",
            "airborne_active",
            "airborne_candidate_ms",
            "motion.kf_dx",
            "feedback_timeout_active",
        ):
            with self.subTest(needle=needle):
                _has(body, needle, "the descent entry gate is incomplete")

    def test_start_is_excluded_from_jump_getup_and_stair(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_descent_start_is_allowed")
        for needle in (
            "vmc_getup_is_active()",
            "vmc_jump_is_active_or_aborted()",
            "vmc_stair_is_active()",
        ):
            with self.subTest(needle=needle):
                _has(body, needle, "another leg owner can overlap descent")

    def test_air_extension_captures_the_pre_drop_length_and_ramps(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_descent_update")
        for needle in (
            "descent_entry_length = (float)leg_length_avg",
            "VMC_DESCENT_MIN_EXTEND_DELTA",
            "descent_extend_target",
            "descent_length_command",
            "vmc_step_towards",
        ):
            with self.subTest(needle=needle):
                _has(body, needle, "the airborne extension profile is incomplete")

    def test_touchdown_requires_torque_and_confirmation(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_descent_update")
        for needle in (
            "vmc_average_joint_torque_abs()",
            "descent_contact_torque",
            "descent_contact_confirm_ms",
            "VMC_DESCENT_STAGE_CUSHION",
            "descent_contact_detected = 1U",
        ):
            with self.subTest(needle=needle):
                _has(body, needle, "touchdown detection is incomplete")

    def test_cushion_returns_to_the_captured_length(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_descent_update")
        _has(body, "VMC_DESCENT_STAGE_CUSHION", "the cushion stage is missing")
        _has(body, "vmc_control.descent_entry_length", "entry length is not restored")
        _has(body, "descent_return_rate", "the return stroke is not rate-limited")

    def test_only_the_airborne_stage_blocks_wheels(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_descent_blocks_wheel_output")
        _has(body, "VMC_DESCENT_STAGE_EXTEND", "the airborne wheel gate is missing")
        if "CUSHION" in body:
            raise AssertionError("the cushion stage must allow wheel balance to resume")


class IntegrationTest(unittest.TestCase):
    def test_airborne_detector_yields_to_the_descent_owner(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_airborne_state")
        _has(body, "vmc_descent_is_active() == 0U",
             "the generic latch can re-close the wheels during descent")

    def test_descent_target_precedes_the_latched_short_stair_length(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        extend_target = body.index("vmc_descent_target_length()")
        stair_target = body.index("vmc_stair_target_length()")
        self.assertLess(extend_target, stair_target)
        _has(body, "descent_override != 0U", "the descent target is not selected")

    def test_descent_uses_its_own_length_profile(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        for needle in (
            "descent_force_profile",
            "descent_kp_length",
            "descent_kd_length",
            "descent_max_force",
            "descent_support_force",
        ):
            with self.subTest(needle=needle):
                _has(body, needle, "the descent leg profile is incomplete")

    def test_wheel_output_obeys_the_descent_gate(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        for name in ("vmc_safety_allows_output", "vmc_wheel_control_update"):
            body = _body(source, name)
            _has(body, "vmc_descent_blocks_wheel_output()",
                 f"{name} ignores the descent wheel gate")

    def test_auxiliary_loops_are_disabled_during_descent(self) -> None:
        control = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        balance = _body(SOURCE.read_text(encoding="utf-8"), "vmc_balance_update")
        _has(control, "descent_override == 0U", "roll/follow can run during descent")
        _has(control, "(descent_override != 0U)", "the descent roll offset is not cleared")
        _has(balance, "vmc_descent_is_active() == 0U",
             "anti-splay can run during descent")

    def test_jump_and_stair_cannot_start_during_descent(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        for name in ("vmc_jump_start_is_allowed", "vmc_stair_start_is_allowed"):
            body = _body(source, name)
            _has(body, "vmc_descent_is_active() == 0U",
                 f"{name} does not exclude descent")

    def test_control_loop_runs_descent_after_airborne_update(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_probe_task")
        airborne = body.index("vmc_update_airborne_state();")
        descent = body.index("vmc_descent_update();")
        stair = body.index("vmc_stair_update();")
        self.assertLess(airborne, descent)
        self.assertLess(descent, stair)


class TuningAndTelemetryTest(unittest.TestCase):
    def test_rtt_accepts_every_descent_knob(self) -> None:
        source = RTT.read_text(encoding="utf-8")
        for key in (
            "descent_enable",
            "descent_extend_length",
            "descent_extend_rate",
            "descent_return_rate",
            "descent_kp_length",
            "descent_kd_length",
            "descent_max_force",
            "descent_support_force",
            "descent_start_speed",
            "descent_start_confirm_ms",
            "descent_contact_torque",
            "descent_contact_confirm_ms",
            "descent_air_timeout_ms",
            "descent_cushion_timeout_ms",
            "descent_cooldown_ms",
        ):
            with self.subTest(key=key):
                _has(source, f'"{key}"', f"{key} is not accepted over RTT")

    def test_web_ui_and_server_expose_the_action(self) -> None:
        index = WEB_INDEX.read_text(encoding="utf-8")
        main = WEB_MAIN.read_text(encoding="utf-8")
        server = SERVER.read_text(encoding="utf-8")
        for element, key in (
            ("descentEnable", "descent_enable"),
            ("descentExtendLength", "descent_extend_length"),
            ("descentContactTorque", "descent_contact_torque"),
            ("descentContactConfirmMs", "descent_contact_confirm_ms"),
        ):
            with self.subTest(key=key):
                _has(index, f'id="{element}"', f"{element} input is missing")
                _has(main, f'"{key}"', f"{key} is not sent by the page")
        _has(server, '"descent_enable": ("VMC_DESCENT_ENABLE_DEFAULT"',
             "the server does not import the descent default")

    def test_telemetry_and_capture_publish_the_state_machine(self) -> None:
        source = RTT.read_text(encoding="utf-8")
        for key in (
            "descent_stage",
            "descent_abort_reason",
            "descent_contact_detected",
            "descent_entry_length",
            "descent_extend_target",
            "descent_length_command",
            "descent_contact_torque_measured",
            "descent_contact_ms",
            "descent_trigger_count",
            "descent_abort_count",
        ):
            with self.subTest(key=key):
                _has(source, key, f"{key} is not in telemetry")
        _has(source, "VMC_LQR_RTT_SCHEMA_VERSION 22U", "the schema was not bumped")
        capture = CAPTURE.read_text(encoding="utf-8")
        _has(capture, "field_count=187", "the schema-22 capture profile is missing")
        _has(capture, '"descent_stage": 177', "the descent telemetry indices are wrong")
        _has(capture, '"descent_abort_count": 186',
             "the final descent telemetry index is wrong")


if __name__ == "__main__":
    unittest.main()
