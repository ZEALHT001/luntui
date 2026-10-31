#!/usr/bin/env python3
"""Regression tests for the manual stair climb.

Operator procedure: drive the robot up to the step, then move SBUS channel 7
from disabled to enabled.  The rising edge triggers one retract/roll/hold
sequence; the switch must be returned to disabled before another sequence can
start.  No wheel-speed, wheel-torque or leg-angle impact detector participates.

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
    def test_both_pose_lengths_are_inside_the_stroke(self) -> None:
        leg_min = _macro("VMC_LEG_LENGTH_MIN")
        leg_max = _macro("VMC_LEG_LENGTH_MAX")
        lift = _macro("VMC_STAIR_LIFT_LENGTH_DEFAULT")
        retract = _macro("VMC_STAIR_RETRACT_LENGTH_DEFAULT")
        retract_min = _macro("VMC_STAIR_RETRACT_LENGTH_MIN")
        self.assertGreaterEqual(lift, leg_min)
        self.assertLessEqual(lift, leg_max)
        self.assertGreater(lift, retract)
        self.assertGreaterEqual(retract, leg_min)
        self.assertLessEqual(retract, leg_max)
        self.assertAlmostEqual(retract_min, 0.18)
        self.assertGreaterEqual(retract, retract_min)

    def test_two_theta_poses_match_shanhai(self) -> None:
        self.assertAlmostEqual(_macro("VMC_STAIR_LIFT_THETA_DEFAULT"), 1.70)
        self.assertGreater(
            _macro("VMC_STAIR_LIFT_THETA_DEFAULT"),
            3.14159265 / 2.0,
            "the lift angle is below the demonstrated peak",
        )
        self.assertAlmostEqual(
            _macro("VMC_STAIR_RETRACT_THETA_DEFAULT"),
            3.14159265 / 3.0,
        )
        self.assertGreater(
            _macro("VMC_STAIR_POSE_LENGTH_TOL_DEFAULT"), 0.0
        )
        self.assertGreater(
            _macro("VMC_STAIR_POSE_THETA_TOL_DEFAULT"), 0.0
        )
        self.assertGreater(_macro("VMC_STAIR_PHI0_RAMP_RATE_DEFAULT"), 0.0)
        self.assertAlmostEqual(
            _macro("VMC_STAIR_LIFT_PHI0_RAMP_RATE_DEFAULT"),
            _macro("VMC_STAIR_PHI0_RAMP_RATE_DEFAULT"),
            msg="LIFT should use the normal phi0 ramp again",
        )
        self.assertGreater(_macro("VMC_STAIR_PHI0_KP_DEFAULT"), 0.0)
        self.assertGreater(_macro("VMC_STAIR_PHI0_KD_DEFAULT"), 0.0)

    def test_action_timing_defaults_are_sane(self) -> None:
        for name in (
            "VMC_STAIR_RETRACT_TIMEOUT_MS_DEFAULT",
            "VMC_STAIR_POSE_CONFIRM_MS_DEFAULT",
            "VMC_STAIR_HOLD_MS_DEFAULT",
            "VMC_STAIR_RECOVER_MS_DEFAULT",
            "VMC_STAIR_RELEASE_RAMP_MS_DEFAULT",
            "VMC_STAIR_TIMEOUT_MS_DEFAULT",
            "VMC_STAIR_COOLDOWN_MS_DEFAULT",
        ):
            with self.subTest(name=name):
                self.assertGreater(_macro(name), 0.0)
        self.assertGreater(_macro("VMC_STAIR_LENGTH_RAMP_RATE_DEFAULT"), 0.0)
        ramp_time = (
            _macro("VMC_STAIR_LIFT_LENGTH_DEFAULT")
            - _macro("VMC_STAIR_RETRACT_LENGTH_DEFAULT")
        ) / _macro("VMC_STAIR_LENGTH_RAMP_RATE_DEFAULT")
        self.assertGreaterEqual(
            _macro("VMC_STAIR_RETRACT_TIMEOUT_MS_DEFAULT") / 1000.0,
            ramp_time,
            "the retract timeout expires before the length ramp finishes",
        )


class ManualTriggerTest(unittest.TestCase):
    def test_rising_edge_and_safety_gates_are_required(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_start_is_allowed")
        _has(body, "remote_control_stair_enabled()", "stair channel is not required")
        _has(body, "vmc_getup_is_active()", "get-up is not excluded")
        _has(body, "vmc_jump_is_active_or_aborted()", "jump is not excluded")
        _has(body, "airborne_active", "airborne is not excluded")
        _has(body, "feedback_timeout_active", "feedback timeout is not excluded")
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_update")
        _has(body, "remote_frame_ready", "stale or missing RC frames can trigger")
        _has(body, "stair_switch_enabled", "the stair switch is not sampled")
        _has(body, "remote_control_channel(RC_STAIR_ENABLE)",
             "the physical switch level is not sampled")
        _has(body, "trigger_edge", "the action is not rising-edge triggered")
        _has(body, "vmc_stair_manual_armed = 1U",
             "returning the switch to disabled does not re-arm the action")
        _has(body, "vmc_stair_manual_armed = 0U",
             "one switch edge can trigger more than one action")

    def test_impact_detection_no_longer_controls_the_action(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_update")
        for forbidden in (
            "left_wheel_torque_raw",
            "right_wheel_torque_raw",
            "motion.left_theta",
            "motion.right_theta",
            "stair_ready_detected != 0U",
            "stair_fast_detected != 0U",
            "stair_impact_confirmed_ms",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(
                    forbidden,
                    body,
                    "the manual trigger still depends on an impact detector",
                )


class StateMachineTest(unittest.TestCase):
    def test_lift_and_retract_advance_only_after_pose_is_confirmed(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_update")
        _has(body, "VMC_STAIR_STAGE_LIFT", "LIFT is missing")
        _has(body, "VMC_STAIR_STAGE_RETRACT", "RETRACT is missing")
        _has(body, "vmc_stair_pose_ready(target_length, target_theta)",
             "the stage does not wait for the actual leg pose")
        ready = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_pose_ready")
        _has(ready, "vmc_rad_format", "pose checks are not wrap-safe")
        _has(body, "stair_pose_confirm_elapsed_ms >= pose_confirm_ms",
             "the pose is not debounced")
        self.assertNotIn(
            "stair_lift_hold",
            body,
            "LIFT still has an independent hold timer",
        )
        _has(body, "VMC_STAIR_STAGE_SETTLE",
             "RETRACT does not hand off to settle")

    def test_settle_keeps_the_climb_pose_before_recovery(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_update")
        _has(body, "VMC_STAIR_STAGE_SETTLE", "SETTLE is missing")
        _has(body, "stair_stage_ms >= hold_ms", "SETTLE does not use stair_hold_ms")
        _has(body, "VMC_STAIR_STAGE_RECOVER",
             "SETTLE does not hand off to a separate recovery stage")
        _has(body, "stair_stage_ms >= recover_ms",
             "RECOVER does not use stair_recover_ms")

    def test_finish_keeps_a_length_override_until_the_knob_moves(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_enter_stage")
        _has(body, "stair_length_override_active = 1U", "the length override is not latched")
        remote = _body(
            SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_target_length"
        )
        _has(remote, "vmc_stair_is_active()", "an active action can release the lock")
        _has(remote, "stair_length_override_remote_position",
             "the override does not remember the triggering knob position")
        _has(remote, "VMC_STAIR_LENGTH_UNLOCK_DELTA_DEFAULT",
             "small knob noise can release the short-leg lock")
        _has(remote, "VMC_STAIR_REMOTE_SWITCH_CONFIRM_MS_DEFAULT",
             "the stair lock release is not debounced")

    def test_finish_blends_wheels_and_lqr_back_in(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        release = _body(source, "vmc_stair_release_scale")
        _has(release, "VMC_STAIR_STAGE_FINISHED",
             "the release ramp is not tied to the finished stage")
        _has(release, "VMC_STAIR_RELEASE_RAMP_MS_DEFAULT",
             "the post-stair release is not rate-limited")

        leg_control = _body(source, "update_leg_length_control")
        _has(leg_control, "vmc_stair_release_scale()",
             "LQR Tp returns without blending")
        wheel_control = _body(source, "vmc_wheel_control_update")
        _has(wheel_control, "vmc_stair_release_scale()",
             "wheel current returns without blending")

    def test_stair_pose_does_not_auto_start_getup(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_getup_state")
        _has(body, "pose_getup_allowed",
             "pose-triggered get-up is not gated by the stair action")
        self.assertGreaterEqual(
            body.count("pose_getup_allowed != 0U"),
            3,
            "body fan, leg recovery or direct shrink can still steal LIFT",
        )

        abort = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_abort_reason")
        _has(abort, "VMC_STAIR_ABORT_ATTITUDE",
             "the stair action has no route for a real attitude failure")
        _has(abort, "VMC_GETUP_START_PITCH_RAD",
             "a real pitch fall cannot abort the stair action")
        _has(abort, "VMC_GETUP_BODY_FAN_START_ROLL_RAD",
             "a real roll fall cannot abort the stair action")


class ControlIntegrationTest(unittest.TestCase):
    def test_stair_target_uses_a_dedicated_length_ramp(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "target_length = vmc_stair_target_length();",
             "the stair target is not selected")
        _has(body, "descent_override != 0U", "descent is missing from the direct-ramp gate")
        _has(body, "vmc_control.stair_length_ramp_rate",
             "the stair target does not use its dedicated ramp")
        _has(body, "ramped_target_length =",
             "the stair target bypasses the common length ramp")
        target = _body(SOURCE.read_text(encoding="utf-8"), "vmc_stair_target_length")
        _has(target, "VMC_STAIR_RETRACT_LENGTH_MIN",
             "the short-leg lock can be configured below 0.18 m")

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
            "stair_lift_support_force",
            "stair_retract_support_force",
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
        _has(control, "vmc_control.stair_lift_theta",
             "LIFT does not use the 90-degree theta target")
        _has(control, "vmc_control.stair_lift_phi0_ramp_rate",
             "LIFT does not use its faster phi0 ramp")
        _has(control, "vmc_control.stair_retract_theta",
             "RETRACT does not use the 60-degree theta target")
        _has(control, "vmc_probe_state.body.pitch",
             "theta is not converted back to an absolute phi0 target")
        _has(control, "VMC_STAIR_STAGE_RECOVER",
             "SETTLE has no separate rate-limited recovery stage")
        _has(control, "vmc_control.target_phi0",
             "RECOVER does not return phi0 to the normal standing target")
        leg_control = _body(
            SOURCE.read_text(encoding="utf-8"), "update_leg_length_control"
        )
        _has(leg_control, "vmc_stair_is_active() == 0U",
             "LQR Tp still overrides the stair phi0 PD")
        _has(leg_control,
             "vmc_rad_format((float)(target_phi0 - leg->phi0))",
             "the stair phi0 error is not wrap-safe")
        wheel_control = _body(
            SOURCE.read_text(encoding="utf-8"), "vmc_wheel_control_update"
        )
        _has(wheel_control, "vmc_stair_blocks_wheel_output() == 0U",
             "wheel output is not disabled during the leg-only climb")

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
            "stair_lift_length",
            "stair_retract_length",
            "stair_lift_theta",
            "stair_retract_theta",
            "stair_pose_length_tol",
            "stair_pose_theta_tol",
            "stair_pose_confirm_ms",
            "stair_lift_phi0_ramp_rate",
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
            "stair_recover_ms",
            "stair_timeout_ms",
            "stair_cooldown_ms",
            "stair_length_ramp_rate",
            "stair_lift_support_force",
            "stair_retract_support_force",
        ):
            with self.subTest(key=key):
                _has(source, f'"{key}"', f"{key} is not accepted over RTT")

    def test_web_ui_and_server_expose_the_feature(self) -> None:
        index = WEB_INDEX.read_text(encoding="utf-8")
        main = WEB_MAIN.read_text(encoding="utf-8")
        server = SERVER.read_text(encoding="utf-8")
        _has(index, 'id="stairEnable"', "the stair enable input is missing")
        _has(index, "手动上台阶", "the stair UI is not marked as manual")
        _has(index, 'id="stairLengthRampRate"',
             "the stair length ramp input is missing")
        _has(index, 'id="stairRecoverMs"',
             "the stair recovery time input is missing")
        for new_id in (
            "stairLiftLength",
            "stairLiftTheta",
            "stairRetractTheta",
            "stairPoseLengthTol",
            "stairPoseThetaTol",
            "stairPoseConfirmMs",
            "stairLiftPhi0RampRate",
            "stairLiftSupportForce",
            "stairRetractSupportForce",
        ):
            with self.subTest(new_id=new_id):
                _has(
                    index,
                    f'id="{new_id}"',
                    "a two-stage stair tuning input is missing",
                )
        for obsolete_id in (
            "stairSpeedThreshold",
            "stairReadyLength",
            "stairImpactConfirmMs",
            "stairImpactTorqueThreshold",
            "stairImpactThetaThreshold",
            "stairPhi0Target",
            "stairSupportForce",
            "stairLiftHoldMs",
        ):
            with self.subTest(obsolete_id=obsolete_id):
                self.assertNotIn(
                    obsolete_id,
                    index,
                    "an obsolete impact-detection control is still exposed",
                )
                self.assertNotIn(
                    f'"{obsolete_id}"',
                    main,
                    "an obsolete impact-detection control is still mapped",
                )
        _has(main, '"stair_enable"', "the page does not send stair_enable")
        _has(main, '"stair_length_ramp_rate"',
             "the page does not send the stair length ramp")
        _has(main, '"stair_recover_ms"',
             "the page does not send the stair recovery time")
        for new_key in (
            "stair_lift_length",
            "stair_lift_theta",
            "stair_retract_theta",
            "stair_pose_length_tol",
            "stair_pose_theta_tol",
            "stair_pose_confirm_ms",
            "stair_lift_phi0_ramp_rate",
            "stair_lift_support_force",
            "stair_retract_support_force",
        ):
            with self.subTest(new_key=new_key):
                _has(
                    main,
                    f'"{new_key}"',
                    "the page does not send a two-stage stair key",
                )
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
            ("remote_control.s[1]", "legacy master switch is not published"),
            ("remote_control.s[0]", "legacy leg switch is not published"),
        ):
            _has(telemetry, needle, why)
        capture = CAPTURE.read_text(encoding="utf-8")
        _has(capture, 'field_count=177', "the schema-21 capture profile is missing")
        _has(capture, '"stair_stage": 157', "the stair telemetry indices are wrong")
        _has(capture, '"stair_preimpact_ms": 172', "the confirmation counters are missing")
        _has(capture, '"remote_s1": 175', "the remote switch indices are wrong")


if __name__ == "__main__":
    unittest.main()
