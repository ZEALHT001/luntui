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
    MPC_INPUT_WEIGHT_SCALE,
    PRESETS,
    discrete_mpc_model,
    emit_mpc_model,
)

MPC_Q = np.diag([1.0, 100.0, 400.0, 30.0, 5000.0, 1.0, 0.05])


class GeneratedModelTest(unittest.TestCase):
    def test_generated_model_matches_the_python_plant(self) -> None:
        lengths = np.linspace(0.180, 0.385, num=9)
        expected = emit_mpc_model(
            replace(PRESETS["abk"], q=MPC_Q), lengths, 0.002222
        )
        actual = (ROOT / "App" / "Src" / "vmc_mpc_model.c").read_text(
            encoding="utf-8"
        )
        self.assertEqual(actual, expected)

    def test_closed_loop_is_stable_with_stiction(self) -> None:
        plant = PRESETS["abk"]
        a, b = discrete_mpc_model(plant, 0.28, 0.002222)
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
        delta_vector = np.diag(delta_weights)
        hessian = influence.T @ weights @ influence + delta_weights
        for step in range(horizon):
            held_step = min(step, control_horizon - 1)
            for input_index in range(2):
                index = held_step * 2 + input_index
                hessian[index, index] += (
                    MPC_INPUT_WEIGHT_SCALE[input_index]
                    * r[input_index, input_index]
                )
        rho = 0.20
        inverse_regularized = np.linalg.inv(hessian + rho * np.eye(decision_count))
        lower = np.tile([-5.5, -108.0], control_horizon)
        upper = np.tile([5.5, 108.0], control_horizon)

        def solve(state: np.ndarray, previous_input: np.ndarray) -> np.ndarray:
            base = state.copy()
            base_prediction = np.zeros(horizon * state_count)
            for step in range(horizon):
                base = a @ base
                base_prediction[
                    step * state_count : (step + 1) * state_count
                ] = base
            linear = (
                influence.T @ weights @ base_prediction
                - delta_vector * np.tile(previous_input, control_horizon)
            )
            z = np.clip(
                np.tile(previous_input, control_horizon), lower, upper
            )
            dual = np.zeros(decision_count)
            for _ in range(10):
                u = inverse_regularized @ (rho * (z - dual) - linear)
                z = np.clip(u + dual, lower, upper)
                dual += u - z
            delta = np.clip(z[:2] - previous_input, [-0.2, -2.0], [0.2, 2.0])
            return previous_input + delta

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
        self.assertIn("VMC_MPC_INTEGRAL_HOLD_NONE", source)
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
