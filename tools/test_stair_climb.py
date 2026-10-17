#!/usr/bin/env python3
"""Regression tests for the automatic stair climb.

Operator procedure: select the HIGHEST leg-length gear on S2 and drive into a
step below the body.  The wheels jam, the LQR keeps demanding torque, the legs
are pushed back, and both per-leg impact tests agree.  The state machine then
retracts the legs, ramps the balance theta target forward, holds, and stays on
the short legs until S2 is moved again.

These are source-level checks, in the same style as test_leg_sync.py: the
firmware has no host-buildable control layer, so the shipped source is the
contract under test.
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
    def test_retract_and_ready_lengths_are_inside_the_stroke(self) -> None:
        leg_min = _macro("VMC_LEG_LENGTH_MIN")
        leg_max = _macro("VMC_LEG_LENGTH_MAX")
        retract = _macro("VMC_STAIR_RETRACT_LENGTH_DEFAULT")
        ready = _macro("VMC_STAIR_READY_LENGTH_DEFAULT")
        self.assertGreaterEqual(retract, leg_min)
        self.assertLess(retract, ready)
        self.assertLessEqual(ready, leg_max)

    def test_phi0_target_is_the_requested_2_5_rad_pose(self) -> None:
        self.assertAlmostEqual(_macro("VMC_STAIR_PHI0_TARGET_DEFAULT"), 2.5)
        self.assertGreater(_macro("VMC_STAIR_PHI0_RAMP_RATE_DEFAULT"), 0.0)
        self.assertGreater(_macro("VMC_STAIR_PHI0_KP_DEFAULT"), 0.0)
        self.assertGreater(_macro("VMC_STAIR_PHI0_KD_DEFAULT"), 0.0)

    def test_timing_and_impact_defaults_are_sane(self) -> None:
        for name in (
            "VMC_STAIR_READY_CONFIRM_MS_DEFAULT",
            "VMC_STAIR_SPEED_CONFIRM_MS_DEFAULT",
            "VMC_STAIR_PREIMPACT_CONFIRM_MS_DEFAULT",
            "VMC_STAIR_IMPACT_CONFIRM_MS_DEFAULT",
            "VMC_STAIR_HOLD_MS_DEFAULT",
            "VMC_STAIR_TIMEOUT_MS_DEFAULT",
        ):
            with self.subTest(name=name):
                self.assertGreater(_macro(name), 0.0)
        self.assertGreater(_macro("VMC_STAIR_IMPACT_TORQUE_THRESHOLD_DEFAULT"), 0.0)
        self.assertGreater(_macro("VMC_STAIR_IMPACT_THETA_THRESHOLD_DEFAULT"), 0.0)


class DetectionTest(unittest.TestCase):
    def test_only_the_highest_s2_gear_can_start(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_start_is_allowed")
        _has(body, "remote_control.s[0] == RC_SW_UP", "S2 high gear is not required")
        _has(body, "vmc_getup_is_active()", "get-up is not excluded")
        _has(body, "vmc_jump_is_active_or_aborted()", "jump is not excluded")
        _has(body, "airborne_active", "airborne is not excluded")
        _has(body, "feedback_timeout_active", "feedback timeout is not excluded")

    def test_impact_needs_both_torque_and_leg_angle_on_both_sides(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_update")
        for needle in (
            "left_wheel_torque_raw",
            "right_wheel_torque_raw",
            "motion.left_theta",
            "motion.right_theta",
            "stair_left_impact",
            "stair_right_impact",
        ):
            with self.subTest(needle=needle):
                _has(body, needle, "impact test is incomplete")
        _has(body, "left_impact != 0U &&", "the left impact flag is not required")
        _has(body, "right_impact != 0U", "the right impact flag is not required")

    def test_impact_requires_the_preimpact_arm_and_forward_command(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_update")
        _has(body, "vmc_stair_preimpact_armed != 0U", "the pre-impact arm is ignored")
        _has(body, "forward_command != 0U", "a forward command is not required")

    def test_zero_torque_threshold_falls_back_to_the_speed_drop_test(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_update")
        _has(body, "impact_torque_threshold > 0.0f", "the A/B switch is missing")
        _has(body, "impact_diff_threshold", "the legacy speed-drop fallback is missing")

    def test_zero_preimpact_diff_disables_the_raw_accel_gate(self) -> None:
        """Raw wheel-speed acceleration is too noisy at 450 Hz to gate on.

        The first capture showed both torque+theta impact tests firing while
        the pre-impact arm stayed at zero: quantization spikes in the raw
        wheel-speed derivative almost never left 30 ms clean.  Zero disables
        that gate and lets the already-confirmed wheel-speed window arm it;
        a non-zero value restores the stricter check for A/B runs.
        """
        self.assertAlmostEqual(_macro("VMC_STAIR_PREIMPACT_DIFF_MAX_DEFAULT"), 0.0)
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_update")
        _has(body, "preimpact_accel_gate == 0U", "the zero-disables path is missing")


class StateMachineTest(unittest.TestCase):
    def test_retract_exits_on_the_target_or_its_own_timeout(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_update")
        _has(body, "VMC_STAIR_STAGE_RETRACT", "RETRACT is missing")
        _has(body, "measured_leg_avg <= retract_length + VMC_STAIR_LENGTH_READY_TOL",
             "RETRACT does not recognise the leg reaching the target")
        _has(body, "stair_stage_ms >= retract_timeout_ms",
             "RETRACT has no timeout fallback when the step blocks the stroke")

    def test_hold_uses_its_own_hold_time(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_update")
        _has(body, "VMC_STAIR_STAGE_HOLD", "HOLD is missing")
        _has(body, "stair_stage_ms >= hold_ms", "HOLD does not use stair_hold_ms")

    def test_finish_keeps_a_length_override_until_s2_moves(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_enter_stage")
        _has(body, "stair_length_override_active = 1U", "the length override is not latched")
        remote = _body(
            SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_target_length"
        )
        _has(remote, "vmc_stair_is_active()", "an active action can release the lock")
        _has(remote, "stair_length_override_remote_position",
             "the override does not remember the triggering gear")


class ControlIntegrationTest(unittest.TestCase):
    def test_stair_owns_the_target_and_should_skip_the_ramp(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "target_length = vmc_stair_retract_target_length();",
             "the stair target is not selected")
        _has(body, "jump_override != 0U || stair_override != 0U",
             "the stair target still runs through the normal ramp")

    def test_roll_compensation_follow_and_sync_are_disabled_during_stair(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "stair_override == 0U", "roll/follow still runs during stair")
        _has(body, "vmc_control.stair_length_override_active == 0U",
             "the override does not gate the auxiliary loops")
        balance = _body(SOURCE.read_text(encoding="utf-8"), "vmc_balance_update")
        _has(balance, "vmc_stair_is_active() == 0U", "anti-splay runs during stair")

    def test_stair_uses_its_own_length_gains(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        for needle in (
            "stair_force_profile",
            "stair_kp_length",
            "stair_kd_length",
            "stair_max_force",
            "stair_support_force",
        ):
            with self.subTest(needle=needle):
                _has(body, needle, "the stair leg profile is incomplete")

    def test_stair_drives_phi0_pd_instead_of_lqr_tp(self) -> None:
        control = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(control, "vmc_control.stair_phi0_ramped",
             "the stair phi0 target is not applied")
        _has(control, "left_target_phi0 = vmc_control.stair_phi0_ramped",
             "the left leg is not commanded to the stair phi0 pose")
        _has(control, "right_target_phi0 = vmc_control.stair_phi0_ramped",
             "the right leg is not commanded to the stair phi0 pose")
        _has(control, "vmc_control.stair_phi0_kp",
             "the stair phi0 KP is not applied")
        _has(control, "vmc_control.stair_phi0_kd",
             "the stair phi0 KD is not applied")
        _has(control, "VMC_STAIR_STAGE_HOLD",
             "HOLD does not switch from the climbing pose to recovery")
        _has(control, "vmc_control.target_phi0",
             "HOLD does not return phi0 to the normal standing target")
        leg_control = _body(
            SOURCE.read_text(encoding="utf-8"), "update_leg_length_control"
        )
        _has(leg_control, "vmc_stair_is_active() == 0U",
             "LQR Tp still overrides the stair phi0 PD")

    def test_state_machine_runs_before_balance_and_control(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_probe_task")
        stair = body.index("vmc_stair_update();")
        self.assertLess(stair, body.index("vmc_balance_update();"))
        self.assertLess(stair, body.index("vmc_control_update();"))


class RuntimeTuningTest(unittest.TestCase):
    def test_rtt_accepts_every_stair_key(self) -> None:
        source = RTT.read_text(encoding="utf-8")
        for key in (
            "stair_enable",
            "stair_retract_length",
            "stair_ready_length",
            "stair_ready_confirm_ms",
            "stair_speed_confirm_ms",
            "stair_preimpact_confirm_ms",
            "stair_impact_confirm_ms",
            "stair_impact_torque_threshold",
            "stair_impact_theta_threshold",
            "stair_phi0_target",
            "stair_phi0_ramp_rate",
            "stair_phi0_kp",
            "stair_phi0_kd",
            "stair_retract_timeout_ms",
            "stair_hold_ms",
            "stair_timeout_ms",
            "stair_cooldown_ms",
        ):
            with self.subTest(key=key):
                _has(source, f'"{key}"', f"{key} is not accepted over RTT")

    def test_web_ui_and_server_expose_the_feature(self) -> None:
        index = WEB_INDEX.read_text(encoding="utf-8")
        main = WEB_MAIN.read_text(encoding="utf-8")
        server = SERVER.read_text(encoding="utf-8")
        _has(index, 'id="stairEnable"', "the stair enable input is missing")
        _has(main, '"stair_enable"', "the page does not send stair_enable")
        _has(server, '"stair_enable": ("VMC_STAIR_ENABLE_DEFAULT"',
             "the server does not import the stair default")

    def test_telemetry_publishes_stage_and_impact_flags(self) -> None:
        source = RTT.read_text(encoding="utf-8")
        for key in (
            "stair_stage",
            "stair_abort_reason",
            "stair_impact_detected",
            "stair_trigger_count",
            "stair_speed_abs",
            "stair_left_impact",
            "stair_right_impact",
        ):
            with self.subTest(key=key):
                _has(source, key, f"{key} is not in telemetry")
        telemetry = _body(source, "vmc_lqr_rtt_telemetry_update")
        for needle, why in (
            ("remote_control_source", "the remote source is not published"),
            ("remote_control.s[1]", "S1 is not published"),
            ("remote_control.s[0]", "S2 is not published"),
        ):
            _has(telemetry, needle, why)
        capture = CAPTURE.read_text(encoding="utf-8")
        _has(capture, 'field_count=177', "the schema-21 capture profile is missing")
        _has(capture, '"stair_stage": 157', "the stair telemetry indices are wrong")
        _has(capture, '"stair_preimpact_ms": 172', "the confirmation counters are missing")
        _has(capture, '"remote_s1": 175', "the remote switch indices are wrong")


if __name__ == "__main__":
    unittest.main()
