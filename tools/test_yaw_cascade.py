#!/usr/bin/env python3
"""Contract tests for the yaw angle -> yaw rate cascade.

The yaw loop used to be a single PD: ``kp * angle_error - kd * wz`` straight
into the differential wheel current, with kp = 10000 counts/rad and kd = 1000
counts/(rad/s).  Two things were wrong with that:

* the D term ate the raw gyro, so at the 9 rad/s spin setting the kd * 9 term
  was a 9000-count bias (75% of the 12000 default limit) before any angle
  term, and gyro noise rode on top of it until the current clipped;
* nothing bounded the rate the controller would ask for, so no error could be
  refused, only clipped after the fact by the current limit.

The loop is now a cascade - angle error -> clipped rate command -> rate
error -> differential current - with the clip on the inner loop's reference,
which is where the reference implementation puts ``w_limit``.  Spin mode feeds
its commanded rate forward so the outer loop does not have to chase a ramp.

These tests pin the structure, because every one of the properties above is
something a later "simplification" could quietly undo.
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


class CascadeStructureTest(unittest.TestCase):
    """The loop must be angle -> clipped rate -> rate error -> current."""

    def setUp(self) -> None:
        self.body = _body(SOURCE.read_text(encoding="utf-8"),
                          "vmc_update_yaw_hold_current")

    def test_outer_law_is_p_on_angle_error_inside_the_clip(self) -> None:
        """The clip wraps the outer command, not the output current.

        Putting the limit only on the current would let kp_outer * error grow
        without bound; the inner loop would then spend its authority
        cancelling a demand that was never achievable.
        """
        self.assertIn(
            "yaw_rate_cmd_outer = clamp_double(\n"
            "      (double)vmc_balance.yaw_hold_kp_outer * yaw_error,\n"
            "      -yaw_rate_limit, yaw_rate_limit);",
            self.body,
        )

    def test_rate_limit_comes_from_the_runtime_parameter(self) -> None:
        self.assertIn(
            "clamp_double(vmc_balance.yaw_hold_rate_max, 0.0, 20.0)", self.body
        )
        self.assertIn("-yaw_rate_limit, yaw_rate_limit", self.body)

    def test_spin_rate_is_fed_forward_after_the_clip(self) -> None:
        """Feedforward must not pass through the outer clip.

        The clip exists to bound the CORRECTION.  A 9 rad/s spin command
        pushed through a 2.5 rad/s clip would make spin mode unreachable.
        """
        self.assertIn("vmc_balance.spin_mode_enabled != 0U", self.body)
        self.assertIn("(double)vmc_balance.yaw_spin_rate", self.body)
        self.assertIn(
            "yaw_rate_cmd = yaw_rate_cmd_outer + yaw_rate_ff;", self.body
        )
        clip = self.body.index("-yaw_rate_limit, yaw_rate_limit")
        sum_up = self.body.index("yaw_rate_cmd = yaw_rate_cmd_outer + yaw_rate_ff")
        self.assertLess(clip, sum_up)

    def test_turn_stick_rate_is_fed_forward_outside_spin(self) -> None:
        """Stick deflects -> the inner loop follows that RATE.

        Without this the stick only integrated an angle target, so the machine
        had to lag by a growing error before it turned; that lag is what kept
        rotating after release.
        """
        self.assertIn("vmc_balance.yaw_stick_rate", self.body)

    def test_inner_loop_is_p_on_rate_error(self) -> None:
        self.assertIn(
            "yaw_rate_error = yaw_rate_cmd - (double)vmc_probe_state.body.wz;",
            self.body,
        )
        self.assertIn(
            "(double)VMC_YAW_HOLD_DIRECTION * (double)vmc_balance.yaw_hold_kd *\n"
            "      yaw_rate_error;",
            self.body,
        )

    def test_current_limit_is_applied_last(self) -> None:
        rate_error = self.body.index("yaw_rate_error = yaw_rate_cmd")
        clamp = self.body.index("clamp_double(yaw_current")
        self.assertLess(rate_error, clamp)
        self.assertIn("(double)-yaw_limit, (double)yaw_limit", self.body)

    def test_deadband_and_enable_gates_are_retained(self) -> None:
        self.assertIn(
            "apply_soft_deadband(yaw_error_raw, vmc_balance.yaw_hold_deadband)",
            self.body,
        )
        for gate in (
            "vmc_balance.yaw_hold_enable == 0U",
            "vmc_balance.wheel_output_enable == 0U",
            "vmc_probe_state.body.imu_valid == 0U",
            "vmc_probe_state.body.imu_connected == 0U",
        ):
            with self.subTest(gate=gate):
                self.assertIn(gate, self.body)


class ParameterTest(unittest.TestCase):
    def test_defaults_reproduce_the_retired_single_loop(self) -> None:
        """Small-signal equivalence: kp_outer * kd == the old kp.

        The rewrite must not move the gains the machine was tuned with; it
        adds the clip and the feedforward.  Anything else needs a hardware
        run, not a refactor.
        """
        outer = _macro("VMC_YAW_HOLD_KP_OUTER_DEFAULT")
        inner = _macro("VMC_YAW_HOLD_KD_DEFAULT")
        self.assertAlmostEqual(outer * inner, 10000.0)
        self.assertAlmostEqual(_macro("VMC_YAW_HOLD_RATE_MAX_DEFAULT"), 2.5)
        self.assertAlmostEqual(_macro("VMC_YAW_HOLD_DEADBAND_DEFAULT"), 0.01)
        self.assertAlmostEqual(_macro("VMC_YAW_HOLD_CURRENT_LIMIT_DEFAULT"), 12000)

    def test_heading_debt_is_bounded_while_the_target_advances(self) -> None:
        """The target may lead the measured heading by at most yaw_lead_max.

        Measured on 2026-11-14: a 2 s stick deflection integrated 3.1 rad of
        heading error while the yaw axis could only deliver ~0.43 rad/s, so the
        machine kept rotating for 6 s after release.  The cap bounds that debt.
        """
        self.assertAlmostEqual(_macro("VMC_YAW_LEAD_MAX_DEFAULT"), 0.35)
        source = SOURCE.read_text(encoding="utf-8")
        body = _body(source, "vmc_yaw_target_advance")
        self.assertIn("clamp_double(vmc_balance.yaw_lead_max, 0.0, 3.15)", body)
        self.assertIn("yaw + lead_max", body)
        self.assertIn("yaw - lead_max", body)
        # Both integration sites must go through the clamped helper.
        self.assertEqual(
            source.count("vmc_yaw_target_advance("), 3
        )  # definition, spin ramp, turn stick

    def test_the_retired_single_loop_gain_is_gone(self) -> None:
        header = HEADER.read_text(encoding="utf-8")
        self.assertNotIn("VMC_YAW_HOLD_KP_DEFAULT", header)
        rtt = RTT.read_text(encoding="utf-8")
        self.assertNotIn('"yaw_hold_kp"', rtt)

    def test_step_hook_offsets_the_target_instead_of_setting_it(self) -> None:
        """A step test must not need to know the unwrapped heading.

        yaw_target is absolute, so a script that writes it directly has to
        read the current angle first and can silently command a half turn.
        The delta form produces the same step from any pose.
        """
        rtt = RTT.read_text(encoding="utf-8")
        self.assertIn('"yaw_target_delta", &value', rtt)
        self.assertIn(
            "vmc_balance.yaw_target += "
            "vmc_lqr_rtt_clamp_float(value, -3.15f, 3.15f);",
            rtt,
        )
        # The offset must be relative to the pose the loop is actually
        # holding.  A stale yaw_target (loop disabled for a while) would make
        # a half-radian step command an arbitrary turn instead.
        block = rtt[rtt.index('"yaw_target_delta", &value'):]
        block = block[:block.index("applied = 1U;")]
        self.assertIn("if (vmc_balance.yaw_hold_active == 0U) {", block)
        self.assertIn("vmc_balance.yaw_target = vmc_probe_state.body.yaw;", block)
        self.assertLess(
            block.index("vmc_balance.yaw_target = vmc_probe_state.body.yaw;"),
            block.index("vmc_balance.yaw_target +="),
        )

    def test_runtime_parameters_are_bounded(self) -> None:
        rtt = RTT.read_text(encoding="utf-8")
        self.assertIn(
            '"yaw_hold_kp_outer", &value', rtt
        )
        self.assertIn(
            "vmc_balance.yaw_hold_kp_outer = "
            "vmc_lqr_rtt_clamp_float(value, 0.0f, 100.0f);",
            rtt,
        )
        self.assertIn(
            "vmc_balance.yaw_hold_rate_max = "
            "vmc_lqr_rtt_clamp_float(value, 0.0f, 20.0f);",
            rtt,
        )
        self.assertIn(
            "vmc_balance.yaw_hold_kd = "
            "vmc_lqr_rtt_clamp_float(value, 0.0f, 20000.0f);",
            rtt,
        )

    def test_diagnostics_use_the_reserved_telemetry_slots(self) -> None:
        rtt = RTT.read_text(encoding="utf-8")
        block = rtt[
            rtt.index("fields 177..186 are the reserved compatibility slots"):
        ]
        order = [
            block.index("(double)vmc_balance.odom_x"),
            block.index("(double)vmc_balance.odom_v"),
            block.index("(double)vmc_balance.leso_wheel_disturbance"),
            block.index("(double)vmc_balance.leso_leg_disturbance"),
            block.index("vmc_balance.yaw_rate_cmd)"),
            block.index("vmc_balance.yaw_rate_cmd_outer)"),
            block.index("vmc_balance.yaw_rate_ff)"),
            block.index("vmc_balance.yaw_error)"),
            block.index("reserved < 2U"),
        ]
        self.assertEqual(order, sorted(order))

    def test_tuner_exposes_the_cascade_gains(self) -> None:
        server = SERVER.read_text(encoding="utf-8")
        index = WEB_INDEX.read_text(encoding="utf-8")
        main = WEB_MAIN.read_text(encoding="utf-8")
        for key, macro in (
            ("yaw_hold_kp_outer", "VMC_YAW_HOLD_KP_OUTER_DEFAULT"),
            ("yaw_hold_rate_max", "VMC_YAW_HOLD_RATE_MAX_DEFAULT"),
            ("yaw_lead_max", "VMC_YAW_LEAD_MAX_DEFAULT"),
        ):
            with self.subTest(key=key):
                self.assertIn(f'"{key}": ("{macro}"', server)
                self.assertIn(f'"{key}"', server)
                self.assertIn(f'"{key}"', main)
        self.assertIn('id="yawHoldKpOuter"', index)
        self.assertIn('id="yawHoldRateMax"', index)
        self.assertIn('id="yawLeadMax"', index)


if __name__ == "__main__":
    unittest.main()
