#!/usr/bin/env python3
"""Source-level regression tests for CH7 manual pre-positioning.

The firmware has no host-side controller harness. These checks lock in the
arming order and the ownership boundaries that keep automatic recovery,
stairs, jumps, LQR Tp and balance wheel output out of manual pre-positioning.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "App" / "Inc" / "vmc_probe.h"
SOURCE = ROOT / "App" / "Src" / "vmc_probe.c"


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


class ManualPreposeTest(unittest.TestCase):
    def setUp(self) -> None:
        self.source = SOURCE.read_text(encoding="utf-8")
        self.header = HEADER.read_text(encoding="utf-8")

    def test_ch7_must_be_on_before_ch4_enables(self) -> None:
        body = _body(self.source, "vmc_update_manual_prepose_mode")
        disabled = body.index("if (enabled == 0U)")
        arm = body.index(
            "vmc_control.manual_prepose_armed = stair_on;", disabled
        )
        activate = body.index(
            "vmc_control.manual_prepose_armed != 0U && stair_on != 0U",
            arm,
        )
        self.assertLess(disabled, arm)
        self.assertLess(arm, activate)
        for needle in (
            "vmc_control.manual_prepose_phi0_left =\n          vmc_probe_state.left_leg.phi0",
            "vmc_control.manual_prepose_phi0_right =\n          vmc_probe_state.right_leg.phi0",
            "vmc_control.manual_prepose_length_left",
            "vmc_control.manual_prepose_length_right",
            "if (stair_on == 0U) {\n"
            "    vmc_control.manual_prepose_exit_length_check_active = 1U;\n"
            "    vmc_control.manual_prepose_active = 0U;",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, body)

    def test_right_stick_moves_both_legs_without_changing_length(self) -> None:
        body = _body(self.source, "vmc_update_manual_prepose_mode")
        for needle in (
            "remote_control_channel(RC_GIMBAL_X)",
            "VMC_MANUAL_PREPOSE_PHI0_RATE_RADPS",
            "(double)input * vmc_loop_dt()",
            "(double)vmc_control.manual_prepose_phi0_left + step",
            "(double)vmc_control.manual_prepose_phi0_right + step",
            "VMC_MANUAL_PREPOSE_PHI0_MIN_RAD",
            "VMC_MANUAL_PREPOSE_PHI0_MAX_RAD",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, body)
        self.assertNotIn("manual_prepose_length_left +", body)
        self.assertNotIn("manual_prepose_length_right +", body)

    def test_leg_knob_controls_relative_length_with_hard_limits(self) -> None:
        self.assertIn(
            "#define VMC_MANUAL_PREPOSE_LENGTH_MIN      0.180f",
            self.header,
        )
        self.assertIn(
            "#define VMC_MANUAL_PREPOSE_LENGTH_MAX      0.380f",
            self.header,
        )

        body = _body(self.source, "vmc_update_manual_prepose_mode")
        for needle in (
            "vmc_control.manual_prepose_knob_position =\n"
            "          remote_control_leg_length_position();",
            "vmc_control.manual_prepose_base_length_left",
            "vmc_control.manual_prepose_base_length_right",
            "knob_position - (double)vmc_control.manual_prepose_knob_position",
            "knob_delta * 0.5 *",
            "(double)vmc_control.manual_prepose_base_length_left + length_delta",
            "(double)vmc_control.manual_prepose_base_length_right + length_delta",
            "VMC_MANUAL_PREPOSE_LENGTH_MIN,\n"
            "        VMC_MANUAL_PREPOSE_LENGTH_MAX);",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, body)

    def test_manual_mode_suppresses_every_automatic_action(self) -> None:
        getup = _body(self.source, "vmc_update_getup_state")
        manual = getup.index("vmc_control.manual_prepose_active != 0U")
        direction = getup.index("if (remote_control.s[1] == RC_SW_DOWN)")
        self.assertLess(manual, direction)
        for needle in (
            "vmc_control.getup_state = VMC_GETUP_STATE_IDLE;",
            "vmc_control.getup_lqr_release_pending = 0U;",
            "vmc_fall_monitor_armed = 0U;",
            "return;",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, getup[manual:direction])

        for function in ("vmc_jump_start_is_allowed", "vmc_stair_start_is_allowed"):
            with self.subTest(function=function):
                body = _body(self.source, function)
                self.assertIn("vmc_control.manual_prepose_active == 0U", body)
                self.assertIn(
                    "vmc_control.manual_prepose_exit_length_check_active == 0U",
                    body,
                )
        self.assertIn(
            "vmc_control.manual_prepose_active != 0U",
            _body(self.source, "vmc_jump_update"),
        )
        self.assertIn(
            "vmc_control.manual_prepose_active != 0U",
            _body(self.source, "vmc_stair_update"),
        )
        self.assertIn(
            "vmc_control.manual_prepose_exit_length_check_active == 0U &&\n"
            "          vmc_getup_fully_fallen == 0U",
            _body(self.source, "vmc_update_remote_mode"),
        )

    def test_leaving_ch7_waits_for_measured_safe_leg_length(self) -> None:
        manual = _body(self.source, "vmc_update_manual_prepose_mode")
        exit_length = manual.index(
            "vmc_control.manual_prepose_exit_length_check_active = 1U;"
        )
        clear_active = manual.index(
            "vmc_control.manual_prepose_active = 0U;",
            exit_length,
        )
        self.assertLess(exit_length, clear_active)

        ready = _body(
            self.source,
            "vmc_manual_prepose_exit_length_ready",
        )
        for needle in (
            "vmc_probe_state.left_leg.length <=",
            "vmc_probe_state.right_leg.length <=",
            "VMC_GETUP_SAFE_LENGTH_MAX",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, ready)

        getup = _body(self.source, "vmc_update_getup_state")
        manual_guard = getup.index(
            "vmc_control.manual_prepose_active != 0U"
        )
        length_guard = getup.index(
            "vmc_control.manual_prepose_exit_length_check_active != 0U",
            manual_guard,
        )
        ch4_guard = getup.index(
            "if (remote_control.s[1] == RC_SW_DOWN)",
            length_guard,
        )
        self.assertLess(length_guard, ch4_guard)
        length_block = getup[length_guard:ch4_guard]
        for needle in (
            "vmc_manual_prepose_exit_length_ready() != 0U",
            "vmc_control.manual_prepose_exit_length_check_active = 0U;",
            "vmc_control.getup_state = VMC_GETUP_STATE_IDLE;",
            "return;",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, length_block)

    def test_manual_joint_target_owns_lqr_and_airborne_control(self) -> None:
        leg_control = _body(self.source, "update_leg_length_control")
        for needle in (
            "vmc_control.manual_prepose_active != 0U ||\n"
            "       vmc_control.manual_prepose_exit_length_check_active != 0U",
            "vmc_balance.tp_output_enable != 0U &&\n"
            "       manual_pd_hold == 0U",
            "if (manual_pd_hold != 0U) {\n"
            "    leg_sync_torque = 0.0;",
            "airborne_posture != 0U &&\n"
            "       manual_pd_hold == 0U",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, leg_control)

        control = _body(self.source, "vmc_control_update")
        for needle in (
            "left_target_phi0 = vmc_control.manual_prepose_phi0_left;",
            "right_target_phi0 = vmc_control.manual_prepose_phi0_right;",
            "vmc_control.manual_prepose_active != 0U ||\n"
            "      vmc_control.manual_prepose_exit_length_check_active != 0U",
            "left_target_length =\n        (double)vmc_control.manual_prepose_length_left;",
            "right_target_length =\n        (double)vmc_control.manual_prepose_length_right;",
            "vmc_control.manual_prepose_active == 0U &&\n"
            "         vmc_control.manual_prepose_exit_length_check_active == 0U &&\n"
            "         vmc_getup_is_active() == 0U",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, control)

    def test_left_stick_is_direct_forward_and_rotation(self) -> None:
        remote = _body(self.source, "vmc_update_remote_mode")
        manual = remote.index(
            "if (vmc_control.manual_prepose_active != 0U) {\n"
            "    /* Manual pre-positioning uses the left vertical stick"
        )
        manual_else = remote.index("  } else {", manual)
        manual_block = remote[manual:manual_else]
        wheel_command = remote.index(
            "vmc_wheel_control.manual_forward_current = forward_current;",
            manual,
        )
        for needle in (
            "(int16_t)(-remote_control_channel(RC_CHASSIS_Y) *",
            "(int16_t)(remote_control_channel(RC_CHASSIS_X) *",
            "VMC_RC_WHEEL_TURN_CURRENT_MAX",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, manual_block)
        self.assertNotIn(
            "spin_line_gate",
            manual_block,
        )
        self.assertIn(
            "vmc_control.manual_prepose_active == 0U &&\n"
            "      vmc_balance.yaw_hold_enable != 0U",
            remote,
        )
        self.assertIn(
            "vmc_balance.speed_target = 0.0f;",
            remote[wheel_command:],
        )

        wheel = _body(self.source, "vmc_balance_apply_wheel_output")
        for needle in (
            "vmc_control.manual_prepose_active != 0U",
            "vmc_wheel_control.manual_forward_current",
            "vmc_wheel_control.manual_turn_current",
            "vmc_wheel_control.manual_forward_current +\n"
            "                            vmc_wheel_control.manual_turn_current",
            "vmc_wheel_control.manual_forward_current -\n"
            "                            vmc_wheel_control.manual_turn_current",
            "vmc_manual_prepose_allows_wheel_output() != 0U",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, wheel)

    def test_manual_wheel_path_bypasses_airborne_gate(self) -> None:
        safety = _body(
            self.source,
            "vmc_manual_prepose_allows_wheel_output",
        )
        for needle in (
            "vmc_control.manual_prepose_active != 0U",
            "vmc_fall_rearm_required == 0U",
            "vmc_probe_state.feedback_timeout_active == 0U",
            "remote_control_allows_joint_enable() != 0U",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, safety)
        self.assertNotIn(
            "airborne_active",
            safety,
            "manual pre-positioning is still cut by the airborne detector",
        )

        wheel = _body(self.source, "vmc_wheel_control_update")
        for needle in (
            "manual_output_ready",
            "automatic_output_ready",
            "vmc_manual_prepose_allows_wheel_output() != 0U",
            "manual_output_ready != 0U ||\n"
            "         vmc_probe_state.airborne_side_active"
            "[VMC_AIRBORNE_SIDE_LEFT] == 0U",
            "if (vmc_control.manual_prepose_active != 0U) {\n"
            "    release_scale = 1.0;",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, wheel)

    def test_state_is_initialized_and_has_captured_lengths(self) -> None:
        for needle in (
            "uint8_t manual_prepose_armed;",
            "uint8_t manual_prepose_active;",
            "uint8_t manual_prepose_exit_length_check_active;",
            "float manual_prepose_phi0_left;",
            "float manual_prepose_phi0_right;",
            "float manual_prepose_knob_position;",
            "float manual_prepose_base_length_left;",
            "float manual_prepose_base_length_right;",
            "float manual_prepose_length_left;",
            "float manual_prepose_length_right;",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, self.header)
        init = _body(self.source, "vmc_control_init")
        self.assertIn("vmc_control.manual_prepose_armed = 0U;", init)
        self.assertIn("vmc_control.manual_prepose_active = 0U;", init)
        self.assertIn(
            "vmc_control.manual_prepose_exit_length_check_active = 0U;",
            init,
        )
        self.assertIn(
            "vmc_control.manual_prepose_length_left = VMC_LEG_LENGTH_DEFAULT;",
            init,
        )
        self.assertIn(
            "vmc_control.manual_prepose_knob_position = 0.0f;",
            init,
        )
        self.assertIn(
            "vmc_control.manual_prepose_base_length_left = VMC_LEG_LENGTH_DEFAULT;",
            init,
        )


if __name__ == "__main__":
    unittest.main()
