#!/usr/bin/env python3
"""Regression tests for the low-shock self-righting sequence.

The normal stand-up path must not hand a fallen chassis straight to the LQR
wheel and joint loops.  These source-level checks lock in the Shanhai-style
body-inversion and per-leg fold preconditions plus the staged handoff: brace,
turn, retract, upright, then READY.  They also verify that the get-up profile
has its own weak gains and torque ceiling.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "App" / "Inc" / "vmc_probe.h"
SOURCE = ROOT / "App" / "Src" / "vmc_probe.c"
BOARD_COMM = ROOT / "App" / "Src" / "board_comm.c"


def _macro(name: str) -> float:
    source = HEADER.read_text(encoding="utf-8")
    match = re.search(
        rf"^#define\s+{name}\s+(-?[0-9.]+)[fFUu]*", source, re.MULTILINE
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


class ProfileTest(unittest.TestCase):
    def test_getup_has_a_bounded_profile_instead_of_the_normal_stance_loop(self) -> None:
        self.assertLess(
            _macro("VMC_GETUP_PHI0_LEAD_RAD") * _macro("VMC_GETUP_PHI0_KP"),
            0.2 * _macro("VMC_LEG_PHI0_KP_DEFAULT"),
            "the turn command is as stiff as the old 0.2 rad step",
        )
        self.assertGreater(_macro("VMC_GETUP_PHI0_KD"), _macro("VMC_LEG_PHI0_KD_DEFAULT"))
        self.assertLess(
            _macro("VMC_GETUP_PHI0_TORQUE_MAX"),
            _macro("VMC_LEG_PHI0_TORQUE_MAX_DEFAULT"),
        )
        self.assertLess(_macro("VMC_GETUP_LENGTH_KP"), _macro("VMC_LEG_LENGTH_KP_DEFAULT"))
        self.assertLess(
            _macro("VMC_GETUP_FORCE_MAX"), _macro("VMC_LEG_FORCE_MAX_DEFAULT")
        )

    def test_getup_motion_is_rate_limited_and_requires_settling(self) -> None:
        for name in (
            "VMC_GETUP_PHI0_LEAD_RAD",
            "VMC_GETUP_LENGTH_RAMP_RATE",
            "VMC_GETUP_UPRIGHT_PHI0_RATE_RADPS",
            "VMC_GETUP_BRACE_MS",
            "VMC_GETUP_POSE_CONFIRM_MS",
            "VMC_GETUP_LENGTH_CONFIRM_MS",
            "VMC_GETUP_READY_CONFIRM_MS",
            "VMC_GETUP_TIMEOUT_MS",
        ):
            with self.subTest(name=name):
                self.assertGreater(_macro(name), 0.0)

    def test_upright_accepts_the_measured_supported_pitch(self) -> None:
        self.assertGreaterEqual(
            _macro("VMC_GETUP_UPRIGHT_PITCH_RAD"),
            0.35,
            "the verified supported chassis settles near 0.30 rad pitch",
        )

    def test_body_and_leg_recovery_preconditions_have_distinct_targets(self) -> None:
        self.assertAlmostEqual(_macro("VMC_GETUP_BODY_POS_TARGET_RAD"), -3 * 3.14159265 / 4)
        self.assertAlmostEqual(_macro("VMC_GETUP_BODY_NEG_TARGET_RAD"), -3.14159265 / 2)
        self.assertGreater(
            _macro("VMC_GETUP_FAN_PHI0_LEAD_RAD"),
            _macro("VMC_GETUP_PHI0_LEAD_RAD"),
            "the folded-leg escape is not stronger than the normal turn",
        )
        self.assertLess(
            _macro("VMC_GETUP_FAN_PHI0_KD"),
            _macro("VMC_GETUP_PHI0_KD"),
            "folded-leg escape is too heavily damped",
        )
        self.assertGreater(_macro("VMC_GETUP_BODY_ALIGN_CONFIRM_MS"), 0.0)

    def test_body_fan_uses_shanhai_speed_and_timeout_scale(self) -> None:
        self.assertGreaterEqual(_macro("VMC_GETUP_BODY_FAN_SPEED_RADPS"), 3.0)
        self.assertEqual(_macro("VMC_GETUP_BODY_FAN_SPEED_KP"), 10.0)
        self.assertEqual(_macro("VMC_GETUP_BODY_FAN_SPEED_DEADBAND_RADPS"), 0.05)
        self.assertEqual(_macro("VMC_GETUP_BODY_FAN_SPEED_TORQUE_MAX"), 15.0)
        self.assertEqual(_macro("VMC_GETUP_BODY_FAN_TORQUE_MAX"), 19.0)
        self.assertEqual(_macro("VMC_GETUP_BODY_FAN_PHI0_KP"), 0.0)
        self.assertEqual(_macro("VMC_GETUP_BODY_FAN_PHI0_KD"), 0.0)
        self.assertGreater(
            _macro("VMC_GETUP_BODY_FAN_TIMEOUT_MS"),
            _macro("VMC_GETUP_TIMEOUT_MS"),
            "the slow inverted windmill inherits the normal 10 s timeout",
        )
        self.assertLessEqual(_macro("VMC_GETUP_BODY_FAN_EXIT_RAD"), 0.2)

    def test_body_fan_synchronizes_both_legs_before_spinning(self) -> None:
        self.assertLessEqual(
            _macro("VMC_GETUP_LEG_BAD_POS_MIN_RAD"),
            1.80,
            "the measured 1.942 rad folded leg is misclassified as normal",
        )
        self.assertGreater(
            _macro("VMC_GETUP_BODY_FAN_SYNC_LENGTH_MIN"), 0.30
        )
        self.assertGreater(
            _macro("VMC_GETUP_BODY_FAN_SYNC_LENGTH_TOL_M"), 0.0
        )
        self.assertGreater(
            _macro("VMC_GETUP_BODY_FAN_SYNC_LENGTH_RATE"), 0.0
        )
        self.assertLessEqual(
            _macro("VMC_GETUP_BODY_FAN_SYNC_THETA_TOL_RAD"), 0.15
        )
        self.assertLessEqual(
            _macro("VMC_GETUP_BODY_FAN_SYNC_EARLY_PHASE_TOL_RAD"), 0.15
        )
        self.assertEqual(
            _macro("VMC_GETUP_BODY_FAN_SYNC_RATE_RADPS"),
            2.0,
        )
        self.assertGreater(_macro("VMC_GETUP_BODY_FAN_SYNC_PHASE_KP"), 0.0)
        self.assertGreater(_macro("VMC_GETUP_BODY_FAN_SYNC_TORQUE_FF"), 0.0)
        self.assertLess(
            _macro("VMC_GETUP_BODY_FAN_SYNC_TORQUE_FF"),
            _macro("VMC_GETUP_BODY_FAN_SYNC_TORQUE_MAX"),
        )
        self.assertGreaterEqual(
            _macro("VMC_GETUP_BODY_FAN_SYNC_TORQUE_MAX"), 30.0
        )
        self.assertEqual(
            _macro("VMC_GETUP_BODY_FAN_SYNC_SUPPORT_FORCE"), 0.0
        )
        self.assertGreater(_macro("VMC_GETUP_BODY_FAN_SYNC_CONFIRM_MS"), 0.0)

    def test_sync_breakaway_feedforward_exits_above_low_speed(self) -> None:
        full_speed = _macro(
            "VMC_GETUP_BODY_FAN_SYNC_BREAKAWAY_FULL_SPEED_RADPS"
        )
        exit_speed = _macro(
            "VMC_GETUP_BODY_FAN_SYNC_BREAKAWAY_EXIT_SPEED_RADPS"
        )
        self.assertGreaterEqual(full_speed, 0.15)
        self.assertLessEqual(full_speed, 0.25)
        self.assertGreater(exit_speed, full_speed)
        self.assertLessEqual(exit_speed, 0.25)
        self.assertGreater(
            _macro("VMC_GETUP_BODY_FAN_SYNC_TORQUE_FF_RISE_RATE"), 0.0
        )
        self.assertGreater(
            _macro("VMC_GETUP_BODY_FAN_SYNC_TORQUE_FF_FALL_RATE"),
            _macro("VMC_GETUP_BODY_FAN_SYNC_TORQUE_FF_RISE_RATE"),
            "the sync feedforward does not release faster than it builds",
        )
        self.assertGreater(
            _macro("VMC_GETUP_BODY_FAN_TORQUE_FF_RISE_RATE"), 0.0
        )
        self.assertGreater(
            _macro("VMC_GETUP_BODY_FAN_TORQUE_FF_FALL_RATE"),
            _macro("VMC_GETUP_BODY_FAN_TORQUE_FF_RISE_RATE"),
            "the fan feedforward does not release faster than it builds",
        )

        source = SOURCE.read_text(encoding="utf-8")
        feedforward = _body(
            source,
            "vmc_getup_body_fan_sync_torque_feedforward",
        )
        for needle in (
            "fabs(measured_speed)",
            "VMC_GETUP_BODY_FAN_SYNC_BREAKAWAY_FULL_SPEED_RADPS",
            "VMC_GETUP_BODY_FAN_SYNC_BREAKAWAY_EXIT_SPEED_RADPS",
            "breakaway_scale = 1.0;",
            "breakaway_scale = 0.0;",
            "breakaway_scale;",
        ):
            with self.subTest(needle=needle):
                _has(
                    feedforward,
                    needle,
                    "the synchronization feedforward is not speed gated",
                )
        slew = _body(source, "vmc_getup_phi0_feedforward_slew")
        for needle in ("rise_rate", "fall_rate", "vmc_step_towards"):
            with self.subTest(needle=needle):
                _has(
                    slew,
                    needle,
                    "the get-up feedforward has no asymmetric slew limit",
                )

    def test_body_fan_has_a_direction_preserving_deadpoint_escape(self) -> None:
        self.assertGreater(
            _macro("VMC_GETUP_BODY_FAN_STALL_STARTUP_MS"), 0.0
        )
        self.assertGreater(
            _macro("VMC_GETUP_BODY_FAN_STALL_CONFIRM_MS"), 0.0
        )
        self.assertGreater(
            _macro("VMC_GETUP_BODY_FAN_STALL_RATE_RADPS"), 0.0
        )
        self.assertGreater(
            _macro("VMC_GETUP_BODY_FAN_STALL_ROLL_RATE_RADPS"), 0.0
        )

    def test_shallow_fold_starts_in_the_lower_pose(self) -> None:
        self.assertAlmostEqual(
            _macro("VMC_GETUP_PARTIAL_LOWER_LENGTH"),
            _macro("VMC_LEG_TARGET_SHORT"),
        )
        self.assertLess(
            _macro("VMC_GETUP_PARTIAL_ENTRY_LENGTH_MAX"),
            _macro("VMC_GETUP_SAFE_LENGTH_MAX") + 0.02,
            "the shallow-fold detector accepts an ordinary extended stance",
        )
        self.assertGreater(
            _macro("VMC_GETUP_PARTIAL_CONFIRM_MS"), 0.0
        )
        self.assertAlmostEqual(
            _macro("VMC_GETUP_PARTIAL_CONFIRM_MS"), 1.0
        )
        self.assertAlmostEqual(
            _macro("VMC_GETUP_PARTIAL_PHI0_KP"), 50.0
        )
        self.assertAlmostEqual(
            _macro("VMC_GETUP_READY_CONFIRM_MS"), 1.0
        )
    def test_shallow_fold_uses_position_pd_for_theta_alignment(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        leg_control = _body(source, "update_leg_length_control")
        for needle in (
            "kp_phi0 * phi0_error",
            "-kd_phi0 * leg->d_phi0",
            "phi0_torque_raw",
        ):
            with self.subTest(needle=needle):
                _has(
                    leg_control,
                    needle,
                    "the get-up phi0 PD path is incomplete",
                )
        self.assertNotIn(
            "vmc_getup_lqr_signal_theta",
            source,
            "shallow-fold alignment still contains an LQR theta pre-stage",
        )

        control = _body(source, "vmc_control_update")
        for needle in (
            "VMC_PHI0_VERTICAL_RAD",
            "vmc_angle_nearest_to",
            "VMC_GETUP_PARTIAL_PHI0_KP",
            "getup_partial_active",
        ):
            with self.subTest(needle=needle):
                _has(
                    control,
                    needle,
                    "FOLDED_LOWER does not target theta=0 through PD",
                )

class StateMachineTest(unittest.TestCase):
    def test_every_entry_requires_a_safe_knob_length(self) -> None:
        self.assertAlmostEqual(_macro("VMC_GETUP_SAFE_LENGTH_MAX"), 0.220)
        source = SOURCE.read_text(encoding="utf-8")
        ready = _body(source, "vmc_getup_safe_length_ready")
        self.assertIn("VMC_GETUP_SAFE_LENGTH_MAX", ready)
        self.assertIn("remote_control_source != RC_SOURCE_SBUS", ready)
        self.assertIn("RC_LOST_TIMEOUT_MS", ready)
        self.assertIn("vmc_remote_leg_length_target()", ready)

        brace = _body(source, "vmc_getup_enter_brace")
        self.assertIn("vmc_getup_safe_length_ready() == 0U", brace)
        self.assertIn("return 0U;", brace)

    def test_entry_starts_with_brace_instead_of_turn(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_getup_state")
        _has(body, "VMC_GETUP_STATE_BRACE", "the recovery entry has no brace stage")
        _has(body, "vmc_getup_enter_brace", "the recovery entry bypasses the brace helper")
        _has(body, "vmc_getup_next_precondition_state()",
             "the brace exit does not classify the fall type")

    def test_turn_waits_for_a_confirmed_pose_before_retracting(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_getup_state")
        _has(body, "vmc_getup_pose_ready()", "TURN does not check the recovery pose")
        _has(body, "VMC_GETUP_POSE_CONFIRM_MS", "TURN can change stage on one frame")
        _has(body, "VMC_GETUP_STATE_SHRINK", "TURN never hands off to retraction")

    def test_length_settle_goes_to_upright_not_ready(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_getup_state")
        _has(body, "VMC_GETUP_STATE_UPRIGHT", "the final posture stage is missing")
        _has(body, "VMC_GETUP_LENGTH_CONFIRM_MS", "retraction readiness is not debounced")
        _has(body, "vmc_getup_upright_ready()", "UPRIGHT has no readiness check")
        _has(body, "vmc_control.getup_lqr_release_pending = 0U;",
             "LQR release is not reserved for the end of the sequence")
        _has(body, "VMC_GETUP_SHRINK_THETA_TARGET_RAD",
             "the shrink stage locks theta and cannot retract through the linkage")
        _has(body, "VMC_GETUP_SHRINK_THETA_RATE_RADPS",
             "the shrink-stage theta recovery is not rate-limited")

    def test_timeout_requires_a_rearm(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_getup_abort")
        _has(body, "vmc_fall_rearm_required = 1U", "abort can silently restart")
        _has(body, "clear_leg_command", "abort does not clear joint commands")
        _has(body, "wheel_output_enable = 0U", "abort does not close wheel output")

    def test_full_inversion_uses_two_direction_specific_stages(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        body_mode = _body(source, "vmc_getup_body_mode")
        _has(body_mode, "VMC_GETUP_BODY_START_PITCH_RAD",
             "the full-inversion threshold is missing")
        _has(body_mode, "return 1;", "positive-pitch inversion is not detected")
        _has(body_mode, "return -1;", "negative-pitch inversion is not detected")

        align = _body(source, "vmc_getup_body_align_direction")
        _has(align, "VMC_GETUP_BODY_POS_TARGET_RAD",
             "positive inversion does not use the -135 deg target")
        _has(align, "VMC_GETUP_BODY_NEG_TARGET_RAD",
             "negative inversion does not use the -90 deg target")

        state = _body(source, "vmc_update_getup_state")
        _has(state, "VMC_GETUP_STATE_BODY_ALIGN",
             "the full-inversion alignment state is absent")
        _has(state, "VMC_GETUP_STATE_BODY_FLIP",
             "the full-inversion flip state is absent")
        _has(state, "getup_body_flip_direction",
             "the flip direction is not latched from the initial pitch")
        _has(state, "VMC_GETUP_BODY_EXIT_PITCH_RAD",
             "the body flip does not use Shanhai's 10 deg exit")
        _has(state, "right_at_target == 0U",
             "the left-side alignment does not wait for the other leg")
        _has(state, "left_at_target == 0U",
             "the right-side alignment does not wait for the other leg")

    def test_roll_dominant_inversion_uses_body_fan(self) -> None:
        self.assertLessEqual(
            _macro("VMC_GETUP_BODY_FAN_START_ROLL_RAD"),
            1.90,
            "the fan entry leaves out the measured 1.989 rad folded-leg pose",
        )
        source = SOURCE.read_text(encoding="utf-8")
        required = _body(source, "vmc_getup_body_fan_is_required")
        _has(required, "VMC_GETUP_BODY_FAN_START_ROLL_RAD",
             "roll-dominant inversion is not detected")

        state = _body(source, "vmc_update_getup_state")
        _has(state, "VMC_GETUP_STATE_BODY_FAN",
             "the windmill state is unreachable")
        _has(state, "VMC_GETUP_BODY_FAN_EXIT_RAD",
             "the windmill has no level-body exit window")
        _has(state, "VMC_GETUP_BODY_FAN_CONFIRM_MS",
             "the windmill exit is not debounced")
        _has(state, "VMC_GETUP_BODY_FAN_TIMEOUT_MS",
             "the windmill uses the normal short get-up timeout")

        precondition = _body(source, "vmc_getup_next_precondition_state")
        fan = precondition.index("VMC_GETUP_STATE_BODY_FAN")
        turn = precondition.index("VMC_GETUP_STATE_TURN")
        self.assertLess(fan, turn, "windmill classification is shadowed by TURN")

    def test_body_fan_requires_matching_leg_state_first(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        state = _body(source, "vmc_update_getup_state")
        ready = _body(source, "vmc_getup_body_fan_sync_ready")
        for needle in (
            "VMC_GETUP_BODY_FAN_SYNC_LENGTH_MIN",
            "VMC_GETUP_BODY_FAN_SYNC_LENGTH_TOL_M",
            "VMC_GETUP_BODY_FAN_SYNC_THETA_TOL_RAD",
        ):
            with self.subTest(needle=needle):
                _has(ready, needle, "the windmill synchronization gate is incomplete")

        target = _body(source, "vmc_getup_body_fan_sync_targets")
        _has(target, "getup_body_fan_sync_fold_side",
             "the synchronization does not latch which leg is folded")
        _has(target, "getup_body_fan_sync_phi0_target",
             "the selected leader phi0 is not latched")
        _has(target, "leader_side = (left_phase >= right_phase) ? 1U : 2U",
             "the windmill direction does not choose the leader")
        _has(target, "leader_theta = (leader_side == 1U) ? left_theta : right_theta",
             "the selected phase leader is not used")
        _has(target, "*left_target = leader_theta;",
             "the left leg does not follow the selected leader")
        _has(target, "*right_target = leader_theta;",
             "the right leg does not follow the selected leader")
        _has(target, "left_phase", "the windmill direction does not choose the leader")
        _has(target, "right_phase", "the windmill direction does not choose the leader")

        precondition = _body(source, "vmc_getup_next_precondition_state")
        _has(
            precondition,
            "getup_body_fan_sync_complete",
            "BODY_FAN can bypass the synchronization stage",
        )
        _has(
            precondition,
            "VMC_GETUP_STATE_BODY_FAN_SYNC",
            "the unsynchronized windmill state is unreachable",
        )
        _has(
            state,
            "getup_body_fan_sync_complete = 1U;",
            "a completed synchronization does not enable BODY_FAN",
        )
        _has(
            state,
            "getup_body_fan_escape_active = 1U;",
            "a stalled windmill cannot request the forward deadpoint escape",
        )
        _has(
            state,
            "vmc_getup_enter_stage(VMC_GETUP_STATE_BODY_FAN_SYNC);",
            "the deadpoint escape bypasses synchronization",
        )

        escape = _body(source, "vmc_getup_body_fan_sync_targets")
        _has(
            escape,
            "VMC_GETUP_BODY_FAN_ENTRY_THETA_RAD",
            "the escape target is not the verified windmill entry phase",
        )
        _has(
            escape,
            "vmc_angle_forward_to",
            "the escape can rotate the folded leg backwards through the deadpoint",
        )
        _has(
            state,
            "if (vmc_getup_body_fan_sync_ready() != 0U)",
            "BODY_FAN_SYNC can exit without completing leg synchronization",
        )
        self.assertNotIn(
            "fan_required == 0U && fan_escape_can_finish != 0U",
            state,
            "body attitude still bypasses synchronization",
        )
        _has(
            ready,
            "vmc_probe_state.left_leg.phi0",
            "synchronization readiness still relies on body-relative theta",
        )
        _has(
            ready,
            "getup_body_fan_sync_prelift_active",
            "theta synchronization can start before the folded leg is extended",
        )
        _has(
            state,
            "getup_body_fan_sync_targets_ready",
            "BODY_FAN_SYNC does not wait for the lift-first preparation",
        )
        _has(
            state,
            "getup_body_fan_sync_initial_phase_close",
            "the initial close-phase pose does not get an early fan handoff",
        )
        _has(
            state,
            "vmc_getup_enter_stage(VMC_GETUP_STATE_BODY_FAN);",
            "a ready short-leg pose cannot hand off directly to BODY_FAN",
        )
        _has(
            state,
            "getup_body_fan_sync_prelift_confirm_ms",
            "the lift-first confirmation shares the synchronization timer",
        )
        _has(
            source,
            "while the length loop extends the folded leg",
            "the folded leg is not held while its length is extended",
        )
        _has(
            source,
            "getup_body_fan_sync_phi0_target",
            "BODY_FAN_SYNC never latches the healthy-leg phi0",
        )
        _has(
            source,
            "getup_sync_torque_profile",
            "BODY_FAN_SYNC still uses only position control",
        )
        _has(
            source,
            "follower_delta",
            "the follower leg has no phase-error velocity target",
        )
        _has(
            source,
            "vmc_getup_body_fan_sync_torque_feedforward",
            "the synchronization stage has no breakaway torque feedforward",
        )
        _has(
            target,
            "getup_body_fan_sync_leader_side",
            "the synchronization leader is not latched for torque control",
        )

    def test_single_leg_fold_is_detected_from_theta_and_recovered_independently(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        folded = _body(source, "vmc_getup_theta_recover_is_required")
        _has(folded, "vmc_getup_theta_is_in_bad_zone",
             "single-leg fold detection does not use theta")
        state = _body(source, "vmc_update_getup_state")
        _has(state, "VMC_GETUP_LEG_RECOVER_TARGET_RAD",
             "the folded leg does not drive to the verified recovery angle")
        _has(state, "getup_left_done", "the left leg cannot finish independently")
        _has(state, "getup_right_done", "the right leg cannot finish independently")

    def test_other_side_pose_skips_leg_fan_and_shrinks_directly(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        ready = _body(source, "vmc_getup_legs_ready_to_shrink")
        _has(ready, "vmc_getup_theta_is_on_shrink_side",
             "the shrink-side window is missing")
        recover = _body(source, "vmc_getup_leg_recover_is_required")
        _has(recover, "vmc_getup_legs_ready_to_shrink() == 0U",
             "leg recovery ignores an already-correct opposite pose")
        direct = _body(source, "vmc_getup_direct_shrink_is_required")
        _has(direct, "vmc_getup_theta_is_on_shrink_side",
             "the direct-shrink entry ignores the physical leg side")
        _has(
            direct,
            "vmc_getup_theta_is_on_shrink_side(vmc_probe_state.motion.left_theta)",
            "the left leg is not considered independently",
        )
        _has(
            direct,
            "vmc_getup_theta_is_on_shrink_side(vmc_probe_state.motion.right_theta)",
            "the right leg is not considered independently",
        )
        _has(direct, "VMC_GETUP_STAND_LENGTH",
             "the direct-shrink entry ignores the standing length")
        state = _body(source, "vmc_update_getup_state")
        _has(state, "vmc_getup_direct_shrink_is_required()",
             "the startup path bypasses direct shrink and enables LQR")

    def test_shallow_fold_entry_is_narrow_and_one_shot(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        entry = _body(source, "vmc_getup_partial_folded_is_required")
        for needle in (
            "VMC_GETUP_PARTIAL_PITCH_MIN_RAD",
            "VMC_GETUP_START_PITCH_RAD",
            "VMC_GETUP_PARTIAL_ROLL_MAX_RAD",
            "VMC_GETUP_PARTIAL_ENTRY_LENGTH_MAX",
            "VMC_GETUP_PARTIAL_ENTRY_THETA_MIN_RAD",
            "VMC_GETUP_PARTIAL_ENTRY_THETA_MAX_RAD",
            "left_theta * right_theta > 0.0",
        ):
            with self.subTest(needle=needle):
                _has(
                    entry,
                    needle,
                    "the shallow-fold detector is too broad",
                )

        start = _body(source, "vmc_getup_enter_partial_fold")
        for needle in (
            "getup_partial_armed == 0U",
            "vmc_getup_safe_length_ready() == 0U",
            "VMC_GETUP_STATE_FOLDED_LOWER",
            "getup_partial_armed = 0U",
            "getup_partial_active = 1U",
            "wheel_output_enable = 0U",
        ):
            with self.subTest(needle=needle):
                _has(
                    start,
                    needle,
                    "the shallow-fold entry does not latch a safe wheel-off start",
                )

        state = _body(source, "vmc_update_getup_state")
        self.assertLess(
            state.index("partial_folded != 0U"),
            state.index("vmc_getup_direct_shrink_is_required()"),
            "the generic direct-shrink branch shadows the shallow-fold path",
        )
        _has(
            state,
            "getup_partial_armed = 1U;",
            "turning S1 off does not re-arm shallow-fold recovery",
        )
        _has(
            state,
            "partial_folded == 0U",
            "a normal upright enable does not consume shallow-fold recovery",
        )

        active = _body(source, "vmc_getup_is_active")
        for state_name in ("FOLDED_LOWER",):
            with self.subTest(state=state_name):
                _has(
                    active,
                    f"VMC_GETUP_STATE_{state_name}",
                    "the new get-up stage can release wheel output",
                )

    def test_shallow_fold_lower_hands_to_upright_after_length_is_reached(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        state = _body(source, "vmc_update_getup_state")
        self.assertNotIn(
            "if (vmc_control.getup_state == VMC_GETUP_STATE_FOLDED_RAISE)",
            state,
            "FOLDED_RAISE is still executed",
        )
        lower_start = state.index(
            "if (vmc_control.getup_state == VMC_GETUP_STATE_FOLDED_LOWER)",
        )
        upright_start = state.index(
            "if (vmc_control.getup_state == VMC_GETUP_STATE_UPRIGHT)",
            lower_start,
        )
        lower_stage = state[lower_start:upright_start]
        for needle in (
            "VMC_GETUP_LENGTH_READY_TOL",
            "VMC_GETUP_STATE_UPRIGHT",
            "getup_upright_phi0_left",
        ):
            with self.subTest(needle=needle):
                _has(
                    lower_stage,
                    needle,
                    "FOLDED_LOWER can enter LQR before settling",
                )

        upright_stage = state[upright_start:]
        for needle in (
            "vmc_getup_upright_ready",
            "VMC_GETUP_READY_CONFIRM_MS",
            "wheel_output_enable = 1U",
        ):
            with self.subTest(needle=needle):
                _has(
                    upright_stage,
                    needle,
                    "UPRIGHT does not hard-switch to LQR after settling",
                )

        control = _body(source, "vmc_control_update")
        for needle in (
            "VMC_GETUP_PARTIAL_LOWER_LENGTH",
            "VMC_PHI0_VERTICAL_RAD",
            "vmc_angle_nearest_to",
        ):
            with self.subTest(needle=needle):
                _has(
                    control,
                    needle,
                    "the shallow-fold length/posture targets are incomplete",
                )

        self.assertNotIn(
            "vmc_getup_partial_target_theta",
            source,
            "the partial get-up does not always target theta=0",
        )

    def test_theta_bad_zone_recovery_uses_the_reversed_winding(self) -> None:
        self.assertLess(_macro("VMC_GETUP_LEG_RECOVER_DIRECTION"), 0.0)
        self.assertAlmostEqual(_macro("VMC_GETUP_LEG_RECOVER_TARGET_RAD"), 1.30)
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_getup_state")
        self.assertGreaterEqual(
            body.count("direction = (int8_t)VMC_GETUP_LEG_RECOVER_DIRECTION;"),
            2,
            "the left and right theta recovery paths do not share the reversed winding",
        )

    def test_precondition_priority_matches_shanhai(self) -> None:
        body = _body(
            SOURCE.read_text(encoding="utf-8"),
            "vmc_getup_next_precondition_state",
        )
        body_align = body.index("VMC_GETUP_STATE_BODY_ALIGN")
        turn = body.index("VMC_GETUP_STATE_TURN")
        leg_align = body.index("VMC_GETUP_STATE_LEG_RECOVER")
        self.assertLess(body_align, turn)
        self.assertLess(turn, leg_align)


class ControlIntegrationTest(unittest.TestCase):
    def test_active_sequence_holds_wheels_and_lqr_joint_output(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_getup_is_active")
        for state in (
            "BRACE",
            "TURN",
            "SHRINK",
            "LEG_RECOVER",
            "UPRIGHT",
            "BODY_ALIGN",
            "BODY_FLIP",
            "BODY_FAN",
            "BODY_FAN_SYNC",
        ):
            with self.subTest(state=state):
                _has(body, f"VMC_GETUP_STATE_{state}", "state is not treated as active")
        _has(body, "getup_lqr_release_pending", "pending release is not treated as active")

    def test_getup_uses_its_own_leg_profile(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        for needle in (
            "getup_force_profile",
            "VMC_GETUP_LENGTH_KP",
            "VMC_GETUP_LENGTH_KD",
            "VMC_GETUP_FORCE_MAX",
            "VMC_GETUP_SUPPORT_FORCE",
            "VMC_GETUP_PHI0_KP",
            "VMC_GETUP_PHI0_KD",
            "VMC_GETUP_PHI0_TORQUE_MAX",
        ):
            with self.subTest(needle=needle):
                _has(body, needle, "the get-up leg profile is incomplete")

    def test_shrink_uses_length_pd_without_extension_feedforward(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        shrink_start = body.index(
            "} else if (vmc_control.getup_state == VMC_GETUP_STATE_SHRINK) {"
        )
        recover_start = body.index(
            "} else if (vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER) {",
            shrink_start,
        )
        shrink = body[shrink_start:recover_start]
        _has(
            shrink,
            "left_support_force = 0.0;",
            "SHRINK retains the extension feedforward on the left leg",
        )
        _has(
            shrink,
            "right_support_force = 0.0;",
            "SHRINK retains the extension feedforward on the right leg",
        )

    def test_brace_holds_measured_length_and_posture(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "VMC_GETUP_STATE_BRACE", "the brace target is missing")
        _has(body, "left_leg.length + vmc_probe_state.right_leg.length",
             "brace does not hold the measured length")
        _has(body, "left_target_phi0 = vmc_probe_state.left_leg.phi0",
             "brace does not hold the measured posture")

    def test_upright_target_is_absolute_and_rate_limited(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "vmc_angle_nearest_to", "the upright angle is not wrap-safe")
        _has(body, "VMC_PHI0_VERTICAL_RAD + (double)vmc_probe_state.body.pitch",
             "theta is not being driven toward zero")
        _has(body, "VMC_GETUP_UPRIGHT_PHI0_RATE_RADPS",
             "the final posture ramp has no rate limit")

    def test_body_fan_uses_velocity_control_and_gravity_feedforward(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        torque = _body(
            SOURCE.read_text(encoding="utf-8"),
            "vmc_body_fan_speed_torque",
        )
        for needle in (
            "VMC_GETUP_BODY_FAN_SPEED_RADPS",
            "fan_speed_target",
            "VMC_GETUP_BODY_FAN_SPEED_KP",
            "VMC_GETUP_BODY_FAN_SPEED_TORQUE_MAX",
            "VMC_GETUP_BODY_FAN_PHI0_KD",
            "left_phi0_feedforward",
            "right_phi0_feedforward",
            "vmc_getup_phi0_feedforward_slew",
        ):
            with self.subTest(needle=needle):
                _has(body, needle, "the Shanhai-style windmill control is incomplete")
        for needle in (
            "VMC_GETUP_BODY_FAN_SPEED_DEADBAND_RADPS",
            "speed_torque_limit",
            "speed_error",
            "speed_torque",
            "VMC_GETUP_BODY_FAN_GRAVITY_GAIN",
            "apply_zero_deadband",
        ):
            with self.subTest(needle=needle):
                _has(torque, needle, "the Shanhai-style speed torque helper is incomplete")
        self.assertNotIn(
            "left_phi0_lead",
            body,
            "BODY_FAN still converts speed error into a position target",
        )

    def test_body_fan_sync_uses_shared_theta_and_long_legs(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        for needle in (
            "VMC_GETUP_STATE_BODY_FAN_SYNC",
            "VMC_GETUP_BODY_FAN_SYNC_LENGTH",
            "VMC_GETUP_BODY_FAN_SYNC_LENGTH_RATE",
            "VMC_GETUP_BODY_FAN_SYNC_RATE_RADPS",
            "getup_body_fan_sync_theta_left",
            "getup_body_fan_sync_theta_right",
            "vmc_angle_forward_to",
        ):
            with self.subTest(needle=needle):
                _has(body, needle, "BODY_FAN_SYNC does not align both legs")

    def test_recovery_preconditions_use_per_leg_phi0_commands(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "getup_left_direction", "the left leg has no independent direction")
        _has(body, "getup_right_direction", "the right leg has no independent direction")
        _has(body, "left_support_force", "the left support force is not independent")
        _has(body, "right_support_force", "the right support force is not independent")
        _has(body, "VMC_GETUP_FAN_SUPPORT_FORCE",
             "the fan stages do not use Shanhai's 20 N support")
        _has(body, "VMC_GETUP_BODY_FAN_SYNC_SUPPORT_FORCE",
             "the synchronization stage does not unload ground pressure")

    def test_ready_hard_switches_to_lqr_joint_and_wheel_output(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        leg_control = _body(source, "update_leg_length_control")
        _has(leg_control, "(use_lqr_tp != 0U) ? 0.0 : phi0_torque_raw",
             "the get-up joint PD is not hard-disabled at READY")
        _has(leg_control, "vmc_balance.left_tp_cmd",
             "the normal LQR joint torque is not used directly")
        self.assertNotIn(
            "vmc_getup_ready_release_scale",
            leg_control,
            "READY still blends the LQR joint output",
        )
        wheel_control = _body(source, "vmc_wheel_control_update")
        _has(wheel_control, "release_scale = vmc_stair_release_scale()",
             "the wheel path still contains the get-up release ramp")
        self.assertNotIn(
            "vmc_getup_ready_release_scale",
            wheel_control,
            "READY still ramps wheel current",
        )
        state = _body(source, "vmc_update_getup_state")
        _has(state, "wheel_output_enable = 1U",
             "READY does not enable normal wheel output immediately")

    def test_board_comms_inhibits_gimbal_for_all_new_stages(self) -> None:
        body = BOARD_COMM.read_text(encoding="utf-8")
        for state in (
            "BRACE",
            "UPRIGHT",
            "BODY_ALIGN",
            "BODY_FLIP",
            "BODY_FAN",
            "BODY_FAN_SYNC",
            "FOLDED_LOWER",
        ):
            with self.subTest(state=state):
                _has(body, f"VMC_GETUP_STATE_{state}",
                     "gimbal heartbeat ignores a get-up stage")


if __name__ == "__main__":
    unittest.main()
