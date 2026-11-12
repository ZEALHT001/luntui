#!/usr/bin/env python3
"""Host checks for the generated MPC model and fixed-size solver."""

from __future__ import annotations

from dataclasses import replace
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.linalg import solve_discrete_are

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from derive_vmc_lqr_abk import (  # noqa: E402
    PRESETS,
    discrete_mpc_model,
    emit_mpc_model,
)

MPC_Q = np.diag([1.0, 100.0, 400.0, 30.0, 5000.0, 1.0, 0.05])


class GeneratedModelTest(unittest.TestCase):
    def test_generated_model_matches_the_python_plant(self) -> None:
        lengths = np.linspace(0.180, 0.385, num=9)
        expected = emit_mpc_model(
            replace(PRESETS["abk"], q=MPC_Q), lengths, 0.003
        )
        actual = (ROOT / "App" / "Src" / "vmc_mpc_model.c").read_text(
            encoding="utf-8"
        )
        self.assertEqual(actual, expected)

    def test_model_sample_time_matches_the_control_period(self) -> None:
        """The MPC table cannot absorb a measured dt, so it must be generated
        at the period the loop is actually pinned to."""
        header = (ROOT / "App" / "Inc" / "vmc_probe.h").read_text(
            encoding="utf-8"
        )
        model = (ROOT / "App" / "Src" / "vmc_mpc_model.c").read_text(
            encoding="utf-8"
        )
        period_us = float(
            header.split("#define VMC_CONTROL_PERIOD_US", 1)[1]
            .split()[0]
            .rstrip("U")
        )
        sample_s = float(
            model.split("vmc_mpc_model_sample_time_s =", 1)[1]
            .split("f;")[0]
            .strip()
        )
        self.assertAlmostEqual(sample_s, period_us * 1.0e-6, places=9)

    def test_closed_loop_is_stable_with_stiction(self) -> None:
        plant = PRESETS["abk"]
        a, b = discrete_mpc_model(plant, 0.28, 0.003)
        q = MPC_Q
        r = np.diag([20.0, 2.5])
        terminal = solve_discrete_are(a, b, q, r)
        horizon = 10
        control_horizon = 10
        state_count = MPC_Q.shape[0]
        decision_count = control_horizon * 2

        influence = np.zeros((horizon * state_count, decision_count))
        previous = np.zeros((state_count, decision_count))
        for step in range(horizon):
            current = a @ previous
            if step < control_horizon:
                current[:, step * 2 : step * 2 + 2] += b
            influence[step * state_count : (step + 1) * state_count] = current
            previous = current

        weights = np.zeros((horizon * state_count, horizon * state_count))
        for step in range(horizon):
            block = terminal if step == horizon - 1 else q
            section = slice(step * state_count, (step + 1) * state_count)
            weights[section, section] = block
        delta_weights = np.kron(
            np.eye(control_horizon), np.diag([1.0, 0.05])
        )
        hessian = influence.T @ weights @ influence + delta_weights
        rho = 0.20
        inverse_regularized = np.linalg.inv(hessian + rho * np.eye(decision_count))
        lower = np.tile([-0.2, -2.0], control_horizon)
        upper = np.tile([0.2, 2.0], control_horizon)
        absolute_lower = np.array([-5.5, -108.0])
        absolute_upper = np.array([5.5, 108.0])

        def solve(state: np.ndarray, previous_input: np.ndarray) -> np.ndarray:
            base = state.copy()
            base_prediction = np.zeros(horizon * state_count)
            for step in range(horizon):
                base = a @ base + b @ previous_input
                base_prediction[
                    step * state_count : (step + 1) * state_count
                ] = base
            linear = influence.T @ weights @ base_prediction
            z = np.zeros(decision_count)
            dual = np.zeros(decision_count)
            for _ in range(10):
                u = inverse_regularized @ (rho * (z - dual) - linear)
                z = np.clip(u + dual, lower, upper)
                dual += u - z
            command = previous_input + z[:2]
            command = np.clip(command, absolute_lower, absolute_upper)
            return command

        state = np.array([0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0])
        command = np.zeros(2)
        positions: list[float] = []
        for _ in range(3000):
            command = solve(state, command)
            applied = command.copy()
            # Approximate the measured high-friction wheel threshold.
            if abs(state[3]) < 0.01 and abs(applied[0]) < 1.2:
                applied[0] = 0.0
            state = a @ state + b @ applied
            positions.append(float(state[2]))
            self.assertLess(abs(state[2]), 0.1)

        tail_span = max(positions[-500:]) - min(positions[-500:])

        lqr_gain = np.linalg.solve(
            b.T @ terminal @ b + r, b.T @ terminal @ a
        )
        lqr_state = np.array([0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0])
        lqr_positions: list[float] = []
        for _ in range(3000):
            applied = -lqr_gain @ lqr_state
            # Apply the same Coulomb-friction approximation as the MPC run.
            if abs(lqr_state[3]) < 0.01 and abs(applied[0]) < 1.2:
                applied[0] = 0.0
            lqr_state = a @ lqr_state + b @ applied
            lqr_positions.append(float(lqr_state[2]))
        lqr_tail_span = (
            max(lqr_positions[-500:]) - min(lqr_positions[-500:])
        )

        # The aligned MPC inherits the LQR model's stiction limit cycle.  Guard
        # against a regression relative to that baseline rather than retaining
        # the old four-step controller's unrelated 20 mm result.
        self.assertLess(tail_span, lqr_tail_span * 1.10 + 0.002)


class FirmwareSolverTest(unittest.TestCase):
    def test_forward_state_keeps_verified_kalman_fusion(self) -> None:
        source = (ROOT / "App" / "Src" / "vmc_probe.c").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "vmc_body_motion_kf_update(vmc_probe_state.motion.d_x_raw",
            source,
        )
        self.assertIn("body_motion_kf_p[0] = 1.0", source)
        self.assertIn(
            "vmc_probe_state.motion.kf_x + x0 * dt", source
        )
        self.assertIn("if (acceleration_valid != 0U)", source)
        self.assertIn("#define VMC_LQR_GAIN_LENGTH_TOLERANCE 0.02f", source)
        self.assertIn("#define VMC_LQR_GAIN_REFRESH_CYCLES   45U", source)
        self.assertIn("vmc_lqr_gain_cache_valid = 0U;", source)
        self.assertIn(
            "#define VMC_MPC_X_INTEGRAL_SPEED_LIMIT 0.08f", source
        )
        self.assertIn(
            "#define VMC_MPC_X_INTEGRAL_POS_LIMIT   0.25f", source
        )
        self.assertIn(
            "#define VMC_MPC_X_INTEGRAL_LOW_SPEED_MS 1000U", source
        )
        self.assertIn(
            "#define VMC_MPC_X_INTEGRAL_LIMIT       0.02f", source
        )
        self.assertIn("reset_integral", source)
        self.assertIn("mpc_handoff_requested", source)
        self.assertIn("vmc_balance_controller_was_enabled", source)
        self.assertIn("vmc_mpc_handoff_was_active", source)
        self.assertIn("vmc_balance.x_offset = (float)motion->kf_x;", source)
        self.assertIn("VMC_MPC_INTEGRAL_HOLD_HIGH_SPEED", source)
        self.assertIn("VMC_MPC_INTEGRAL_HOLD_POSITION", source)
        self.assertIn("VMC_MPC_INTEGRAL_HOLD_SPEED_SETTLE", source)
        self.assertIn("VMC_MPC_INTEGRAL_HOLD_AUTHORITY_RAMP", source)
        self.assertIn("VMC_MPC_INTEGRAL_HOLD_RETURNING_TO_ANCHOR", source)
        self.assertIn("VMC_MPC_INTEGRAL_HOLD_POSITION_DEADBAND", source)
        self.assertIn("VMC_MPC_INTEGRAL_HOLD_NONE", source)
        self.assertIn("VMC_MPC_AUTHORITY_FULL_EPSILON 0.001f", source)
        self.assertIn(
            "VMC_MPC_X_INTEGRAL_POSITION_DEADBAND 0.01f", source
        )
        self.assertIn("VMC_MPC_X_INTEGRAL_FAST_DECAY 0.99f", source)
        self.assertIn(
            "mpc_x_error * mpc_dx_error < 0.0f", source
        )
        # u_prev may only be taken from the applied command in pure shadow.
        # Feeding the blended command back during an authority ramp makes
        # u(k) = (1 - a) * LQR + a * u(k - 1) + a * delta_u, a positive
        # feedback loop whose gain a / (1 - a) diverges near full authority.
        sync_block = source.split(
            "const uint8_t mpc_sync_previous_input =", 1
        )[1].split(";", 1)[0]
        self.assertIn("mpc_apply_requested == 0U", sync_block)
        self.assertNotIn("mpc_wheel_authority", sync_block)
        self.assertNotIn("mpc_leg_authority", sync_block)
        # The crossfade must be against the LQR demand.  Using the previous
        # applied command as the base degenerates at zero authority to
        # u(k) = u(k-1): in the 2026-11-09 run the legs kept their last torque
        # (leg authority is 0 for the whole wheel test) and lifted the machine,
        # and the wheel command became a 0.96-pole low-pass.
        mix_block = source.split("if (mpc_apply_requested != 0U) {", 1)[
            1
        ].split("vmc_balance.mpc_wheel_authority_applied =", 1)[0]
        self.assertIn("(1.0f - wheel_authority) * left_wheel_lqr", mix_block)
        self.assertIn("(1.0f - wheel_authority) * right_wheel_lqr", mix_block)
        self.assertIn("(1.0f - leg_authority) * left_tp_lqr", mix_block)
        self.assertIn("(1.0f - leg_authority) * right_tp_lqr", mix_block)
        self.assertNotIn("left_wheel_torque_limited", mix_block)
        self.assertNotIn("left_tp_cmd", mix_block)
        # Static-hold damping: the stronger gain only engages after the stick
        # has been released for a while, and the applied gain is ramped.
        self.assertIn("vmc_dx_static_hold_ms", source)
        self.assertIn("dx_feedback_gain_static", source)
        self.assertIn("vmc_dx_gain_applied", source)
        self.assertIn("if (stick_speed_active != 0U ||", source)
        # A get-up runs with the stick released, so the strong gain must be
        # gated out of every scripted sequence.
        self.assertIn("static_hold_allowed", source)
        for mode in (
            "vmc_getup_is_active() == 0U",
            "vmc_jump_is_active_or_aborted() == 0U",
            "vmc_stair_is_active() == 0U",
            "vmc_control.manual_prepose_active == 0U",
            "vmc_probe_state.airborne_active == 0U",
            "vmc_probe_state.feedback_timeout_active == 0U",
        ):
            self.assertIn(mode, source)
        self.assertIn("ramp_rate * dt", source)
        self.assertIn("VMC_BALANCE_DX_GAIN_RELEASE_RATE", source)
        # The strong gain is only safe near zero speed; a kick must drop back
        # to the normal gain instead of ringing, and the band between the two
        # knees is interpolated so the residual ring still sees damping.
        self.assertIn("dx_static_speed_limit", source)
        self.assertIn("dx_static_speed_low", source)
        self.assertIn("normal_gain + (static_gain - normal_gain) * blend", source)
        # The enable edge used to carry the previous strong gain straight into
        # the leg-extension ramp; it now snaps back and waits for the settle.
        self.assertIn("vmc_dx_enabled_ms", source)
        self.assertIn("VMC_BALANCE_DX_STATIC_SETTLE_MS", source)
        self.assertIn("settled == 0U", source)
        self.assertNotIn("static_speed_ok", source)
        self.assertIn("const uint8_t authority_ramping =", source)
        self.assertIn(
            "hold_reason = VMC_MPC_INTEGRAL_HOLD_AUTHORITY_RAMP", source
        )
        self.assertIn(
            "vmc_balance.mpc_wheel_authority_applied <", source
        )
        self.assertIn(
            "1.0f - VMC_MPC_AUTHORITY_FULL_EPSILON", source
        )
        self.assertIn("vmc_mpc_integral_awake =", source)
        self.assertIn("mpc_integral_saturated", source)
        for signal in (
            "left_lqr_theta_error",
            "left_lqr_d_theta_limited",
            "right_lqr_theta_error",
            "right_lqr_d_theta_limited",
        ):
            self.assertIn(f"-({signal} *", source)
        self.assertIn("-phi_error", source)
        self.assertIn("-dpitch_lpf", source)
        for signal in (
            "mpc_left_state_theta",
            "mpc_left_state_phi",
            "mpc_right_state_theta",
            "mpc_right_state_phi",
            "mpc_left_previous_wheel",
            "mpc_right_previous_tp",
            "mpc_left_plan[plan_index]",
            "mpc_right_plan[plan_index]",
        ):
            self.assertIn(signal, source)

        rtt = (ROOT / "App" / "Src" / "vmc_lqr_rtt.c").read_text(
            encoding="utf-8"
        )
        solver = (ROOT / "App" / "Src" / "vmc_mpc.c").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "#define VMC_MPC_FACTOR_LEG_LENGTH_TOLERANCE 0.001f",
            solver,
        )
        self.assertIn("if (rebuild_hessian != 0U)", solver)
        self.assertIn("vmc_mpc_h_factor_valid = 1U;", solver)
        self.assertIn("advance_state_with_input", solver)
        self.assertIn("project_delta_sequence", solver)
        self.assertIn("vmc_mpc_sync_previous_input", solver)
        self.assertIn(
            "axis->plan[0] = next_input[0] - previous_input[0]", solver
        )
        self.assertIn(
            "previous_input[0] + requested_delta[0]", solver
        )
        # The increment weights are runtime-tunable, but only as an offset from
        # what the table was built with: the model file stays the single source
        # of the default, and the factor cache is invalidated on a change.
        self.assertIn("vmc_mpc_delta_weight_offset[column]", solver)
        self.assertIn("vmc_mpc_h_factor_valid = 0U;", solver)
        self.assertIn("vmc_mpc_set_delta_weight", solver)
        self.assertIn('"dx_feedback_gain_static"', rtt)
        self.assertIn('"dx_static_delay_ms"', rtt)
        self.assertIn('"dx_static_speed_limit"', rtt)
        self.assertIn('"dx_static_speed_low"', rtt)
        self.assertIn('"mpc_delta_weight_wheel"', rtt)
        self.assertIn('"mpc_delta_weight_leg"', rtt)
        self.assertIn("#define VMC_LQR_RTT_SCHEMA_VERSION 30U", rtt)
        for signal in (
            "mpc_leg_length",
            "mpc_left_state_dphi",
            "mpc_right_state_dphi",
            "mpc_left_previous_tp",
            "mpc_right_previous_tp",
            "mpc_x_integral",
            "mpc_integral_awake",
            "mpc_integral_hold_reason",
            "mpc_integral_saturated",
            "x_offset",
            "x_error",
            "b->mpc_left_plan[plan_index]",
            "b->mpc_right_plan[plan_index]",
        ):
            self.assertIn(signal, rtt)

    def test_runtime_toggle_is_wired_through_the_tuner(self) -> None:
        parser = (ROOT / "App" / "Src" / "vmc_lqr_rtt.c").read_text(
            encoding="utf-8"
        )
        server = (ROOT / "tools" / "lqr_tuner" / "server.py").read_text(
            encoding="utf-8"
        )
        web = (ROOT / "tools" / "lqr_tuner" / "web" / "index.html").read_text(
            encoding="utf-8"
        )
        self.assertIn('"mpc_enable"', parser)
        self.assertIn('"mpc_shadow_enable"', parser)
        self.assertIn('"compute_only_enable"', parser)
        self.assertIn('"mpc_wheel_authority"', parser)
        self.assertIn('"mpc_leg_authority"', parser)
        self.assertIn('"mpc_enable"', server)
        self.assertIn('"mpc_shadow_enable"', server)
        self.assertIn('"compute_only_enable"', server)
        self.assertIn('"mpc_wheel_authority"', server)
        self.assertIn('"mpc_leg_authority"', server)
        self.assertIn('id="mpcEnable"', web)
        self.assertIn('id="mpcShadowEnable"', web)
        self.assertIn('id="computeOnlyEnable"', web)
        self.assertIn('id="mpcWheelAuthority"', web)
        self.assertIn('id="mpcLegAuthority"', web)

    def test_fixed_size_solver_builds_and_runs(self) -> None:
        compiler = (
            shutil.which("cc") or shutil.which("clang") or shutil.which("gcc")
        )
        if compiler is None:
            self.skipTest("no host C compiler available")
        with tempfile.TemporaryDirectory() as temporary:
            executable = Path(temporary) / "test_vmc_mpc"
            command = [
                compiler,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                f"-I{ROOT / 'App' / 'Inc'}",
                str(ROOT / "tools" / "test_vmc_mpc.c"),
                str(ROOT / "App" / "Src" / "vmc_mpc.c"),
                str(ROOT / "App" / "Src" / "vmc_mpc_model.c"),
                "-lm",
                "-o",
                str(executable),
            ]
            build = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(
                build.returncode, 0, f"compile failed:\n{build.stderr}"
            )
            run = subprocess.run(
                [str(executable)], capture_output=True, text=True
            )
            self.assertEqual(
                run.returncode, 0, f"test failed:\n{run.stdout}\n{run.stderr}"
            )


if __name__ == "__main__":
    unittest.main()
