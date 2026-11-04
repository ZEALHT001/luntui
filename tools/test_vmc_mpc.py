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

MPC_Q = np.diag([1.0, 100.0, 400.0, 100.0, 5000.0, 1.0])


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
        state_count = 6
        decision_count = horizon * 2

        influence = np.zeros((horizon * state_count, decision_count))
        previous = np.zeros((state_count, decision_count))
        for step in range(horizon):
            current = a @ previous
            current[:, step * 2 : step * 2 + 2] += b
            influence[step * state_count : (step + 1) * state_count] = current
            previous = current

        weights = np.zeros((horizon * state_count, horizon * state_count))
        for step in range(horizon):
            block = terminal if step == horizon - 1 else q
            section = slice(step * state_count, (step + 1) * state_count)
            weights[section, section] = block
        delta_weights = np.kron(np.eye(horizon), np.diag([1.0, 0.05]))
        hessian = influence.T @ weights @ influence + delta_weights
        rho = 0.20
        inverse_regularized = np.linalg.inv(hessian + rho * np.eye(decision_count))
        lower = np.tile([-5.5, -108.0], horizon)
        upper = np.tile([5.5, 108.0], horizon)

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
                - delta_weights @ np.tile(previous_input, horizon)
            )
            z = np.clip(np.tile(previous_input, horizon), lower, upper)
            dual = np.zeros(decision_count)
            for _ in range(10):
                u = inverse_regularized @ (rho * (z - dual) - linear)
                z = np.clip(u + dual, lower, upper)
                dual += u - z
            delta = np.clip(z[:2] - previous_input, [-0.2, -2.0], [0.2, 2.0])
            return previous_input + delta

        state = np.array([0.0, 0.0, 0.02, 0.0, 0.0, 0.0])
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
        self.assertLess(tail_span, 0.020)


class FirmwareSolverTest(unittest.TestCase):
    def test_forward_state_uses_average_wheel_odometry(self) -> None:
        source = (ROOT / "App" / "Src" / "vmc_probe.c").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "vmc_body_motion_kf_update(vmc_probe_state.motion.wheel_dx",
            source,
        )
        self.assertNotIn("body_motion_kf_p", source)
        self.assertIn(
            "vmc_probe_state.motion.kf_x + filtered_velocity * dt", source
        )

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
        self.assertIn('"mpc_enable"', server)
        self.assertIn('"mpc_shadow_enable"', server)
        self.assertIn('"compute_only_enable"', server)
        self.assertIn('id="mpcEnable"', web)
        self.assertIn('id="mpcShadowEnable"', web)
        self.assertIn('id="computeOnlyEnable"', web)

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
