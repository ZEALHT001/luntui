#!/usr/bin/env python3
"""Regression test for the jump stage timing defaults.

The jump state machine advances a stage when EITHER the stage has timed out OR
the leg has reached that stage's target length, so the timeouts are upper
bounds rather than delays.  That made them look harmless to shorten - but they
still have to outlast the legs' physical travel.  Every one of them was once
left at 1 ms, which collapsed the whole 0.18 -> 0.25 -> 0.18 sequence into
about 3 ms: the machine never extended, so jumping was silently impossible
with no error reported anywhere.

These tests keep the values in a range where a jump can actually happen.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "App" / "Inc" / "vmc_probe.h"

# Stage timeouts as the jump feature was written, plus the phase durations the
# state machine actually depends on.
STAGE_TIMEOUTS = {
    "VMC_JUMP_COMPRESS_TIMEOUT_MS_DEFAULT": 180,
    # Raised from the original 120 so the higher KP below has time to act.
    "VMC_JUMP_EXTEND_TIMEOUT_MS_DEFAULT": 160,
    "VMC_JUMP_RETRACT_TIMEOUT_MS_DEFAULT": 140,
    "VMC_JUMP_LAND_TIMEOUT_MS_DEFAULT": 600,
    "VMC_JUMP_COOLDOWN_MS_DEFAULT": 1500,
    "VMC_JUMP_LAND_CONFIRM_MS_DEFAULT": 80,
}


def _macro(name: str) -> float:
    source = HEADER.read_text(encoding="utf-8")
    match = re.search(
        rf"^#define\s+{name}\s+(-?[0-9.]+)[fUu]*", source, re.MULTILINE
    )
    if match is None:
        raise AssertionError(f"macro {name} not found in vmc_probe.h")
    return float(match.group(1))


class JumpDefaultsTest(unittest.TestCase):
    def test_stage_timeouts_allow_real_leg_travel(self) -> None:
        for name, expected in STAGE_TIMEOUTS.items():
            with self.subTest(macro=name):
                self.assertAlmostEqual(_macro(name), expected)

    def test_no_stage_timeout_is_degenerate(self) -> None:
        """A 1 ms stage makes the jump finish before the legs move."""
        for name in STAGE_TIMEOUTS:
            with self.subTest(macro=name):
                self.assertGreaterEqual(
                    _macro(name),
                    50.0,
                    f"{name} is too short for the stage to complete",
                )

    def test_total_timeout_leaves_room_for_every_stage(self) -> None:
        """The overall budget must exceed the sum of the stage timeouts."""
        total = _macro("VMC_JUMP_TOTAL_TIMEOUT_MS_DEFAULT")
        stages = sum(
            _macro(name)
            for name in (
                "VMC_JUMP_COMPRESS_TIMEOUT_MS_DEFAULT",
                "VMC_JUMP_EXTEND_TIMEOUT_MS_DEFAULT",
                "VMC_JUMP_RETRACT_TIMEOUT_MS_DEFAULT",
                "VMC_JUMP_LAND_TIMEOUT_MS_DEFAULT",
            )
        )
        self.assertGreater(total, stages)

    def test_stage_timeouts_survive_the_firmware_clamp(self) -> None:
        """The firmware clamps each stage timeout to [1, 2000] ms.

        Anything at the ceiling would be silently truncated, so the useful
        values have to sit inside it.
        """
        for name in STAGE_TIMEOUTS:
            with self.subTest(macro=name):
                self.assertLessEqual(_macro(name), 2000.0)

    def test_jump_is_enabled_by_default(self) -> None:
        self.assertEqual(_macro("VMC_JUMP_ENABLE_DEFAULT"), 1.0)

    def test_airborne_latch_is_not_single_frame(self) -> None:
        """One telemetry frame must not be able to declare the robot airborne.

        Both times were 1 ms, so a single wheel-speed spike or a single
        dropped torque sample cut the wheels mid-drive.  A landing impact
        produces exactly those spikes, which is why this looked like the robot
        randomly dying after a jump.
        """
        confirm = _macro("VMC_AIRBORNE_CONFIRM_MS_DEFAULT")
        release = _macro("VMC_AIRBORNE_RELEASE_MS_DEFAULT")
        self.assertGreaterEqual(confirm, 50.0)
        self.assertGreaterEqual(release, 100.0)
        self.assertGreater(release, confirm)

    def test_jump_can_lift_the_machine(self) -> None:
        """The jump force must exceed the machine's weight, with margin.

        About 20 kg means ~200 N just to stand: a 100 N per-leg ceiling left
        no net upward force at all, so the legs extended and the robot never
        left the ground.  Also note the jump force is clamped by the general
        leg-force ceiling, so raising only one of the two does nothing.
        """
        per_leg = _macro("VMC_JUMP_FORCE_MAX_DEFAULT")
        ceiling = _macro("VMC_LEG_FORCE_MAX_DEFAULT")
        weight = 20.0 * 9.81
        self.assertGreaterEqual(per_leg, 300.0, "jump force is too small to lift off")
        self.assertGreater(
            ceiling,
            per_leg,
            "the general leg-force ceiling would clip the jump to below its own limit",
        )
        # Net lift, as a multiple of g.
        net_g = (2.0 * per_leg - weight) / weight
        self.assertGreater(net_g, 1.0, f"only {net_g:.2f} g of net lift")

    def test_jump_length_gain_has_torque_headroom(self) -> None:
        """The length gain drives joint torque, which the motors cap.

        A position PD commands force proportional to the remaining error, so
        the gain - not the force limit - is what sets how hard the legs push.
        The measured peak was 36 N.m per joint against a 54 N.m motor limit at
        Kp = 3000, which leaves about 1.4x.  Keep the gain inside that.
        """
        kp = _macro("VMC_JUMP_LENGTH_KP_DEFAULT")
        self.assertGreater(kp, 3000.0, "the stock gain could not launch the robot")
        self.assertLessEqual(
            kp,
            4500.0,
            "beyond this the legs hit the 54 N.m joint limit and clip",
        )
        # The force cap must stay under the joint-limit equivalent (~390 N).
        self.assertLessEqual(_macro("VMC_JUMP_FORCE_MAX_DEFAULT"), 400.0)

    def test_extension_feedforward_clears_its_clamp(self) -> None:
        """The feedforward must be large enough to reach the tail of the stroke.

        The length loop is a position PD, so its force collapses as the leg
        closes on the target (measured: ~380 N down to 19 N over one EXTEND),
        and the last third of the stroke was braking. A small feedforward is
        swamped by the ~390 N the PD already commands early on - 80 N measured
        as no change at all - so it has to be big enough to matter where the PD
        has faded.

        It is clamped to +/-VMC_LEG_SUPPORT_FORCE_MAX at the call site, so a
        value above that ceiling would be silently truncated.
        """
        support = _macro("VMC_JUMP_SUPPORT_FORCE_DEFAULT")
        ceiling = _macro("VMC_LEG_SUPPORT_FORCE_MAX")
        self.assertGreaterEqual(
            support, 150.0, "below this the term does nothing measurable"
        )
        self.assertLessEqual(
            support,
            ceiling,
            "the call site clamps to this, so more than it would do nothing",
        )

    def test_the_force_ceiling_does_not_change_how_the_robot_stands(self) -> None:
        """The ceiling must not bound the standing support force.

        VMC_LEG_SUPPORT_FORCE_MAX only caps the feedforward used by the jump
        and by the leg-length loop's constant term.  Standing asks for
        VMC_LEG_SUPPORT_FORCE_DEFAULT, which is a live tuning knob (the web UI
        writes it), so the property that matters is that the ceiling leaves it
        room - not a particular number.  It has been 120 N and 50 N in
        practice; pinning the exact value broke the suite on every write.

        The jump clamp has its own, separate check below.
        """
        self.assertGreater(_macro("VMC_LEG_SUPPORT_FORCE_MAX"),
                           _macro("VMC_LEG_SUPPORT_FORCE_DEFAULT"))
        self.assertGreater(_macro("VMC_LEG_SUPPORT_FORCE_DEFAULT"), 0.0)

    def test_jump_extension_stays_inside_the_leg_stroke(self) -> None:
        """The EXTEND target must remain reachable and leave room to retract."""
        extend = _macro("VMC_JUMP_EXTEND_LENGTH_DEFAULT")
        lo = _macro("VMC_LEG_LENGTH_MIN")
        hi = _macro("VMC_LEG_LENGTH_MAX")
        self.assertGreater(extend, _macro("VMC_JUMP_COMPRESS_LENGTH_DEFAULT"))
        self.assertLess(extend, hi, "the leg cannot reach past its own limit")
        # The latch length must come back down inside the stroke too.
        self.assertGreater(extend, _macro("VMC_JUMP_RETRACT_LENGTH_DEFAULT"))
        self.assertGreater(hi - extend, 0.0)
        self.assertGreater(extend, lo)

    def test_feedforward_adds_extension_not_retraction(self) -> None:
        """A sign error here would fight the jump instead of helping it.

        Two direction constants both multiply by -1 on the way to the force:
        the length PID and the support term. During EXTEND the commanded force
        comes out negative while the leg extends, so a POSITIVE
        jump_support_force must make the force more negative (more extension).
        If either direction were flipped, the same positive value would
        subtract force and the robot would jump lower.
        """
        self.assertEqual(_macro("VMC_LEG_SUPPORT_FORCE_DIRECTION"), -1.0)
        self.assertEqual(_macro("VMC_LEG_LENGTH_PID_DIRECTION"), -1.0)


SOURCE = ROOT / "App" / "Src" / "vmc_probe.c"


def _function_body(source: str, name: str) -> str:
    """Body of the *definition* of name, skipping the forward declaration."""
    pattern = re.compile(
        rf"^(?:static\s+)?\w+\s+{re.escape(name)}\s*\([^;{{]*\)\s*\{{",
        re.MULTILINE,
    )
    match = pattern.search(source)
    if match is None:
        raise AssertionError(f"definition of {name} not found")
    body_start = match.end() - 1
    depth = 0
    for index in range(body_start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[body_start:index + 1]
    raise AssertionError(f"unterminated body for {name}")


class MovingJumpTest(unittest.TestCase):
    def test_attitude_gates_the_launch_but_not_the_flight(self) -> None:
        """Attitude must be checked BEFORE the jump, never during it.

        Landing pitches the body hard - measured transients of 0.46 rad
        (26 deg) against a 0.35 rad limit - and that is ordinary landing nod.
        While the abort path also tested attitude, roughly half of otherwise
        good jumps were marked failed, and the aborted state disturbed the
        stage timing of the next jump (57 mm of push instead of 145-176 mm).

        The launch check must keep the test, otherwise a jump could be
        commanded from a pose the machine cannot recover from.
        """
        source = SOURCE.read_text(encoding="utf-8")
        launch = _function_body(source, "vmc_jump_start_is_allowed")
        abort = _function_body(source, "vmc_jump_abort_reason")
        self.assertIn("body.pitch", launch, "launch must still gate on attitude")
        self.assertIn("body.roll", launch)
        self.assertNotIn(
            "body.pitch", abort, "attitude must not abort a running jump"
        )
        self.assertNotIn("body.roll", abort)
        # IMU health still aborts: without attitude the controller cannot balance.
        self.assertIn("imu_ready", abort)

    """A jump must be startable while the machine is already driving.

    The jump itself was never speed-gated; what made it stationary-only was
    that the wheel output was blanked for the WHOLE jump.  That also kills the
    balance loop through the 300 ms of COMPRESS and EXTEND, which the robot
    spends on the ground, so a moving jump tips over before it can leave.
    """

    def test_moving_jump_is_enabled_by_default(self) -> None:
        self.assertEqual(_macro("VMC_JUMP_MOVING_ENABLE_DEFAULT"), 1.0)

    def test_wheel_blanking_does_not_cover_the_ground_phases(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        body = _function_body(source, "vmc_jump_blocks_wheel_output")
        # The default (moving-enabled) path must key off the airborne-only
        # predicate.  vmc_jump_is_active() may appear, but only inside the
        # legacy branch guarded by jump_moving_enable == 0.
        self.assertIn("vmc_jump_joint_lqr_active()", body)
        self.assertIn("vmc_control.jump_moving_enable == 0U", body)
        legacy = body.index("vmc_control.jump_moving_enable == 0U")
        # Whatever the legacy guard returns must come before the default
        # return, and the default return must be the airborne-only one.
        tail = body[body.rindex("return vmc_jump_joint_lqr_active();"):]
        self.assertTrue(tail.startswith("return vmc_jump_joint_lqr_active();"))
        self.assertGreater(len(tail), 0)
        self.assertLess(legacy, body.rindex("return vmc_jump_joint_lqr_active();"))

    def test_the_leg_path_still_sees_every_stage(self) -> None:
        """The leg torque path and the airborne detector keep the wide gate."""
        source = SOURCE.read_text(encoding="utf-8")
        wide = _function_body(source, "vmc_jump_is_active_or_aborted")
        self.assertIn("vmc_jump_is_active()", wide)
        # Consumers that need the wide gate: the airborne detector, the
        # leg-torque (Tp) path, anti-splay, and both stair start/abort gates.
        # Each must stay out of the way while a jump owns the legs or wheels.
        self.assertEqual(source.count("vmc_jump_is_active_or_aborted()"), 5)

    def test_an_aborted_jump_still_returns_the_wheels(self) -> None:
        """An aborted jump must not leave the wheels dead.

        The wheel gate used to include the "stage == ABORT and S1 is still up"
        latch.  Since the only way out of ABORT is moving S1 to the middle, an
        abort the pilot never acknowledged held the wheels off indefinitely -
        no balance loop, and the machine fell over.  Jump aborts are ordinary
        (a landing that tips past the attitude limit, one missed CAN frame),
        so that failure was intermittent.
        """
        source = SOURCE.read_text(encoding="utf-8")
        body = _function_body(source, "vmc_jump_blocks_wheel_output")
        self.assertNotIn(
            "VMC_JUMP_STAGE_ABORT",
            body,
            "the abort latch must not gate the wheel output",
        )
        self.assertNotIn(
            "RC_SW_UP",
            body,
            "the wheel output must not depend on the jump switch position",
        )

    def test_the_wide_gate_still_carries_the_abort_latch(self) -> None:
        """The leg target and the detector legitimately keep the latch."""
        source = SOURCE.read_text(encoding="utf-8")
        wide = _function_body(source, "vmc_jump_is_active_or_aborted")
        self.assertIn("VMC_JUMP_STAGE_ABORT", wide)

    def test_every_jump_exit_clears_the_airborne_latch(self) -> None:
        """A finished jump must hand the wheels back on BOTH exit paths.

        The LAND phase ends either when leg torque confirms contact or when it
        times out.  Only the first path used to clear the airborne latch, so a
        jump that ended on the timeout left airborne_active set and the wheels
        cut until the normal detector released it - which needs 500 ms of
        sustained leg load and never arrives if the landing is soft.
        """
        source = SOURCE.read_text(encoding="utf-8")
        body = _function_body(source, "vmc_jump_finish")
        self.assertIn(
            "vmc_clear_airborne_latch()",
            body,
            "vmc_jump_finish must clear the latch so both exits behave alike",
        )


class AutoJumpTest(unittest.TestCase):
    """The jump is fired by the automatic gate, not by flipping S1 up.

    Operator conditions: S1 must be UP, the machine must actually be moving at
    1 m/s, and the laser must read strictly inside (80, 100) cm.
    """

    def test_thresholds_stay_in_a_sane_range(self) -> None:
        """These are tunable, so check the engineering bounds, not exact values.

        The macros are only the power-on defaults; the live values are set over
        RTT from the web UI ("写入固件默认值" writes this header).  Pinning the
        exact numbers here meant every tuning session broke the suite, so this
        checks the properties that actually matter instead:

          - a positive speed floor, or the gate would fire while stationary;
          - a distance window inside the TF02-Pro's usable span;
          - a non-zero confirmation, or a single frame could launch a jump.

        The shipped pair at the time of writing was 1.3 m/s / 90-110 cm.
        """
        self.assertGreater(_macro("VMC_JUMP_AUTO_SPEED_MIN"), 0.0)
        self.assertLess(_macro("VMC_JUMP_AUTO_SPEED_MIN"), 5.0)
        self.assertGreater(_macro("VMC_JUMP_AUTO_DIST_MIN_CM"), 0.0)
        self.assertLess(_macro("VMC_JUMP_AUTO_DIST_MAX_CM"), 500.0)
        self.assertGreater(_macro("VMC_JUMP_AUTO_CONFIRM_MS"), 0.0)

    def test_distance_window_is_ordered(self) -> None:
        self.assertLess(
            _macro("VMC_JUMP_AUTO_DIST_MIN_CM"),
            _macro("VMC_JUMP_AUTO_DIST_MAX_CM"),
        )

    def test_gate_requires_all_three_conditions(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        gate = _function_body(source, "vmc_jump_auto_ready")
        # Speed, read from the runtime value so the web tuner works.
        self.assertIn("motion.kf_dx", gate)
        self.assertIn("vmc_control.jump_auto_speed_min", gate)
        # Distance, both bounds, exclusive
        self.assertIn("vmc_control.jump_auto_dist_min_cm", gate)
        self.assertIn("vmc_control.jump_auto_dist_max_cm", gate)
        self.assertIn("<=", gate)
        self.assertIn(">=", gate)
        # A stale laser must never arm a jump.
        self.assertIn("tf02_connected", gate)

    def test_s1_up_gates_the_automatic_trigger(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        idle = _function_body(source, "vmc_jump_update")
        self.assertIn("s1_is_up", idle)
        self.assertIn("vmc_jump_auto_ready()", idle)

    def test_s1_up_does_not_oscillate_the_leg_length(self) -> None:
        """S1 UP is a jump gate, not the removed chassis-test waveform.

        An early firmware revision deliberately drove a triangular leg-length
        sweep while S1 was UP.  It was removed when the jump state machine
        replaced that test mode, but the visible symptom - the machine
        periodically dropping and rising while standing - is easy to mistake
        for a leg-length control fault.  S2 is the only switch allowed to
        choose a leg-length preset.
        """
        source = SOURCE.read_text(encoding="utf-8")
        body = _function_body(source, "vmc_update_remote_target_length")
        self.assertIn("remote_control.s[0]", body)
        self.assertNotIn("remote_control.s[1]", body)
        self.assertNotIn("s1_up_oscillation_start_ms", source)

    def test_the_manual_arm_step_is_gone(self) -> None:
        """S1 MID used to arm and S1 UP used to fire; the gate replaced both.

        If the old arming branch came back, moving S1 up would fire a jump
        regardless of speed or distance, which is exactly what this change
        removed.
        """
        source = SOURCE.read_text(encoding="utf-8")
        idle = _function_body(source, "vmc_jump_update")
        self.assertNotIn(
            "jump_armed = 1U",
            idle,
            "jump_armed must be computed from the automatic gate, not set by "
            "an S1-MID arming step",
        )

    def test_confirmation_window_is_required(self) -> None:
        """One frame must not launch a jump."""
        source = SOURCE.read_text(encoding="utf-8")
        idle = _function_body(source, "vmc_jump_update")
        self.assertIn("vmc_control.jump_auto_confirm_ms", idle)
        self.assertIn("vmc_jump_auto_hold_ms", idle)

    def test_manual_trigger_helpers_are_untouched(self) -> None:
        """The pre-launch safety gates must still run before an automatic jump."""
        source = SOURCE.read_text(encoding="utf-8")
        idle = _function_body(source, "vmc_jump_update")
        self.assertIn(
            "vmc_jump_start_is_allowed()",
            idle,
            "the attitude/feedback/enable gate must still guard the launch",
        )


class AutoJumpTuningTest(unittest.TestCase):
    """The gate thresholds are runtime tunable, not compile-time only.

    They are re-tuned from the LQR tuner web UI while the robot is running, so
    the live values must come from vmc_control.  Reading the macros directly
    would make the web controls silently do nothing.
    """

    RUNTIME_KEYS = (
        "jump_auto_speed_min",
        "jump_auto_dist_min_cm",
        "jump_auto_dist_max_cm",
        "jump_auto_confirm_ms",
    )

    def test_gate_reads_runtime_values_not_the_macros(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        gate = _function_body(source, "vmc_jump_auto_ready")
        for key in self.RUNTIME_KEYS[:3]:
            with self.subTest(key=key):
                self.assertIn(
                    f"vmc_control.{key}",
                    gate,
                    f"{key} must be read from vmc_control so the web UI works",
                )
        # The confirmation time is used by the state machine instead.
        idle = _function_body(source, "vmc_jump_update")
        self.assertIn("vmc_control.jump_auto_confirm_ms", idle)

    def test_every_runtime_key_is_parsed_from_rtt(self) -> None:
        rtt = (ROOT / "App" / "Src" / "vmc_lqr_rtt.c").read_text(encoding="utf-8")
        for key in self.RUNTIME_KEYS:
            with self.subTest(key=key):
                self.assertIn(
                    f'"{key}"',
                    rtt,
                    f"{key} must be accepted over RTT to be web-tunable",
                )

    def test_every_runtime_key_is_initialised(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        init = _function_body(source, "vmc_control_init")
        for key in self.RUNTIME_KEYS:
            with self.subTest(key=key):
                self.assertIn(f"vmc_control.{key} =", init)

    def test_tuner_exposes_every_runtime_key(self) -> None:
        from tools.lqr_tuner.server import (
            BALANCE_PARAM_DEFAULT_MACROS,
            _firmware_balance_params,
        )

        resolved = _firmware_balance_params()
        for key in self.RUNTIME_KEYS:
            with self.subTest(key=key):
                self.assertIn(key, BALANCE_PARAM_DEFAULT_MACROS)
                self.assertIn(key, resolved)
        # The values must match the firmware defaults, not a stale copy.
        self.assertAlmostEqual(resolved["jump_auto_speed_min"],
                               _macro("VMC_JUMP_AUTO_SPEED_MIN"))
        self.assertAlmostEqual(resolved["jump_auto_dist_min_cm"],
                               _macro("VMC_JUMP_AUTO_DIST_MIN_CM"))
        self.assertAlmostEqual(resolved["jump_auto_dist_max_cm"],
                               _macro("VMC_JUMP_AUTO_DIST_MAX_CM"))
        self.assertAlmostEqual(resolved["jump_auto_confirm_ms"],
                               _macro("VMC_JUMP_AUTO_CONFIRM_MS"))


if __name__ == "__main__":
    unittest.main()
