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
            "VMC_GETUP_RELEASE_RAMP_MS",
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
        self.assertEqual(_macro("VMC_GETUP_BODY_FAN_PHI0_KD"), 0.0)
        self.assertGreater(
            _macro("VMC_GETUP_BODY_FAN_TIMEOUT_MS"),
            _macro("VMC_GETUP_TIMEOUT_MS"),
            "the slow inverted windmill inherits the normal 10 s timeout",
        )
        self.assertLessEqual(_macro("VMC_GETUP_BODY_FAN_EXIT_RAD"), 0.2)


class StateMachineTest(unittest.TestCase):
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
        for needle in (
            "VMC_GETUP_BODY_FAN_SPEED_RADPS",
            "VMC_GETUP_BODY_FAN_SPEED_GAIN",
            "VMC_GETUP_BODY_FAN_PHI0_KD",
            "left_speed_error",
            "right_speed_error",
            "left_phi0_feedforward",
            "right_phi0_feedforward",
            "VMC_GETUP_BODY_FAN_GRAVITY_GAIN",
        ):
            with self.subTest(needle=needle):
                _has(body, needle, "the Shanhai-style windmill control is incomplete")

    def test_recovery_preconditions_use_per_leg_phi0_commands(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_control_update")
        _has(body, "getup_left_direction", "the left leg has no independent direction")
        _has(body, "getup_right_direction", "the right leg has no independent direction")
        _has(body, "left_support_force", "the left support force is not independent")
        _has(body, "right_support_force", "the right support force is not independent")
        _has(body, "VMC_GETUP_FAN_SUPPORT_FORCE",
             "the fan stages do not use Shanhai's 20 N support")

    def test_ready_blends_lqr_joint_and_wheel_output(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        leg_control = _body(source, "update_leg_length_control")
        _has(leg_control, "vmc_getup_ready_release_scale()",
             "the LQR joint output is not blended in")
        _has(leg_control, "phi0_torque_raw * (1.0 - lqr_release_scale)",
             "the get-up PD output does not fade out")
        _has(leg_control, "* lqr_release_scale",
             "the LQR joint output does not fade in")
        wheel_control = _body(source, "vmc_wheel_control_update")
        _has(wheel_control, "vmc_getup_ready_release_scale()",
             "wheel current is not blended in")
        _has(wheel_control, "release_scale", "the wheel current scale is unused")

    def test_board_comms_inhibits_gimbal_for_all_new_stages(self) -> None:
        body = BOARD_COMM.read_text(encoding="utf-8")
        for state in ("BRACE", "UPRIGHT", "BODY_ALIGN", "BODY_FLIP", "BODY_FAN"):
            with self.subTest(state=state):
                _has(body, f"VMC_GETUP_STATE_{state}",
                     "gimbal heartbeat ignores a get-up stage")


if __name__ == "__main__":
    unittest.main()
