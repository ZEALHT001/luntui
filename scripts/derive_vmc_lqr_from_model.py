#!/usr/bin/env python3
"""
Derive VMC LQR gains from the wheel-leg model used by the Matlab scripts.

This mirrors the dynamics in:
  SPR Balanced-infantry/matlab/matlab_hid/get_k_length.m
  SPR Balanced-infantry/matlab/matlab_hid/untitled2.m

It avoids Matlab/Symbolic Toolbox by solving the acceleration equations
numerically, linearizing around the upright equilibrium, solving continuous
LQR, and fitting cubic polynomials K_i(L).

State order:
  [theta, d_theta, x, d_x, phi, d_phi]

Input order:
  [T, Tp]

Default output order matches vmc_lqr_update_gain():
  k[0..5]  = first LQR row, wheel torque T
  k[6..11] = second LQR row, hip torque Tp
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.linalg import solve_continuous_are


@dataclass(frozen=True)
class ModelParams:
    wheel_radius: float
    body_com_to_hip: float
    wheel_mass: float
    leg_mass: float
    body_mass: float
    body_height: float
    body_width: float
    leg_width: float = 0.05
    gravity: float = 9.8


PRESETS = {
    # Parameters from matlab_hid/untitled2.m.
    "untitled2": ModelParams(
        wheel_radius=0.086,
        body_com_to_hip=0.03,
        wheel_mass=1.18,
        leg_mass=1.11,
        body_mass=10.3,
        body_height=0.30,
        body_width=0.12,
    ),
    # Parameters from matlab_hid/get_k_length.m. Note: that Matlab file
    # currently overwrites input leg_length to 0.32, so use this preset only
    # when intentionally testing those estimated parameters across lengths.
    "get_k_length": ModelParams(
        wheel_radius=0.077,
        body_com_to_hip=0.134,
        wheel_mass=0.644,
        leg_mass=5.52,
        body_mass=8.4,
        body_height=0.25,
        body_width=0.20,
    ),
}

Q_DEFAULT = np.diag([100.0, 1.0, 500.0, 100.0, 5000.0, 1.0])
R_DEFAULT = np.diag([240.0, 25.0])


def inertias(p: ModelParams, leg_length: float) -> tuple[float, float, float]:
    l1 = leg_length / 2.0
    lm1 = leg_length / 2.0
    iw = p.wheel_mass * p.wheel_radius**2
    ip = p.leg_mass * ((l1 + lm1) ** 2 + p.leg_width**2) / 12.0
    im = p.body_mass * (p.body_height**2 + p.body_width**2) / 12.0
    return iw, ip, im


def acceleration_residual(
    f: np.ndarray,
    state: np.ndarray,
    control: np.ndarray,
    p: ModelParams,
    leg_length: float,
) -> np.ndarray:
    theta, d_theta, _x, _d_x, phi, d_phi = state
    t_dd, x_dd, p_dd = f
    torque, hip_torque = control

    r = p.wheel_radius
    l = leg_length / 2.0
    lm = leg_length / 2.0
    iw, ip, im = inertias(p, leg_length)

    sin_t = np.sin(theta)
    cos_t = np.cos(theta)
    sin_p = np.sin(phi)
    cos_p = np.cos(phi)

    body_x_dd = (
        x_dd
        + (l + lm) * (cos_t * t_dd - sin_t * d_theta**2)
        - p.body_com_to_hip * (cos_p * p_dd - sin_p * d_phi**2)
    )
    body_y_dd = (
        (l + lm) * (-sin_t * t_dd - cos_t * d_theta**2)
        + p.body_com_to_hip * (-sin_p * p_dd - cos_p * d_phi**2)
    )
    leg_x_dd = x_dd + l * (cos_t * t_dd - sin_t * d_theta**2)
    leg_y_dd = l * (-sin_t * t_dd - cos_t * d_theta**2)

    nm = p.body_mass * body_x_dd
    pm = p.body_mass * p.gravity + p.body_mass * body_y_dd
    n = nm + p.leg_mass * leg_x_dd
    force_p = pm + p.leg_mass * p.gravity + p.leg_mass * leg_y_dd

    eq1 = x_dd - (torque - n * r) / (iw / r + p.wheel_mass * r)
    eq2 = (
        ip * t_dd
        - (force_p * l + pm * lm) * sin_t
        + (n * l + nm * lm) * cos_t
        + torque
        - hip_torque
    )
    eq3 = im * p_dd - hip_torque - nm * p.body_com_to_hip * cos_p - pm * p.body_com_to_hip * sin_p
    return np.array([eq1, eq2, eq3], dtype=float)


def solve_acceleration(
    state: np.ndarray,
    control: np.ndarray,
    p: ModelParams,
    leg_length: float,
) -> np.ndarray:
    zero = np.zeros(3)
    c = acceleration_residual(zero, state, control, p, leg_length)
    mat = np.column_stack(
        [
            acceleration_residual(np.eye(3)[i], state, control, p, leg_length) - c
            for i in range(3)
        ]
    )
    return np.linalg.solve(mat, -c)


def dynamics(
    state: np.ndarray,
    control: np.ndarray,
    p: ModelParams,
    leg_length: float,
) -> np.ndarray:
    acc = solve_acceleration(state, control, p, leg_length)
    return np.array([state[1], acc[0], state[3], acc[1], state[5], acc[2]], dtype=float)


def linearize(p: ModelParams, leg_length: float, eps: float = 1e-6) -> tuple[np.ndarray, np.ndarray]:
    x0 = np.zeros(6)
    u0 = np.zeros(2)
    a = np.zeros((6, 6))
    b = np.zeros((6, 2))

    for i in range(6):
        delta = np.zeros(6)
        delta[i] = eps
        a[:, i] = (
            dynamics(x0 + delta, u0, p, leg_length)
            - dynamics(x0 - delta, u0, p, leg_length)
        ) / (2.0 * eps)

    for i in range(2):
        delta = np.zeros(2)
        delta[i] = eps
        b[:, i] = (
            dynamics(x0, u0 + delta, p, leg_length)
            - dynamics(x0, u0 - delta, p, leg_length)
        ) / (2.0 * eps)

    return a, b


def lqr_gain(a: np.ndarray, b: np.ndarray, q: np.ndarray, r: np.ndarray) -> np.ndarray:
    p = solve_continuous_are(a, b, q, r)
    return np.linalg.solve(r, b.T @ p)


def fit_gains(
    preset: str,
    leg_min: float,
    leg_max: float,
    leg_step: float,
    degree: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    params = PRESETS[preset]
    lengths = np.arange(leg_min, leg_max + leg_step * 0.5, leg_step)
    gains = []
    for length in lengths:
        a, b = linearize(params, float(length))
        gains.append(lqr_gain(a, b, Q_DEFAULT, R_DEFAULT).reshape(-1))
    gains = np.vstack(gains)
    coeffs = np.vstack([np.polyfit(lengths, gains[:, i], degree) for i in range(12)])
    return lengths, gains, coeffs


def emit_c(coeffs: np.ndarray) -> str:
    lines = [
        '#include "vmc_lqr.h"',
        "",
        "static double vmc_lqr_pow3(double x)",
        "{",
        "  return x * x * x;",
        "}",
        "",
        "void vmc_lqr_update_gain(double leg_length, double k[12])",
        "{",
        "  double L_0 = leg_length;",
        "  double t2 = L_0 * L_0;",
        "  double t3 = vmc_lqr_pow3(L_0);",
        "",
    ]
    for i, c in enumerate(coeffs):
        lines.append(
            f"  k[{i}] = ((L_0 * {c[2]:.17g} + t2 * {c[1]:.17g}) + "
            f"t3 * {c[0]:.17g}) + {c[3]:.17g};"
        )
    lines += ["}", ""]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--preset", choices=sorted(PRESETS), default="untitled2")
    parser.add_argument("--leg-min", type=float, default=0.10)
    parser.add_argument("--leg-max", type=float, default=0.40)
    parser.add_argument("--leg-step", type=float, default=0.01)
    parser.add_argument("--degree", type=int, default=3)
    parser.add_argument("--out-c", type=Path)
    parser.add_argument("--print-sample", type=float, help="Print K at a single leg length.")
    args = parser.parse_args()

    if args.print_sample is not None:
        a, b = linearize(PRESETS[args.preset], args.print_sample)
        k = lqr_gain(a, b, Q_DEFAULT, R_DEFAULT)
        np.set_printoptions(precision=8, suppress=False)
        print(k)
        return

    lengths, gains, coeffs = fit_gains(
        args.preset,
        args.leg_min,
        args.leg_max,
        args.leg_step,
        args.degree,
    )
    if args.out_c:
        args.out_c.write_text(emit_c(coeffs))
    else:
        np.set_printoptions(precision=10, suppress=False)
        print("lengths:", lengths)
        print("coeffs:")
        print(coeffs)
        print("first K:", gains[0].reshape(2, 6))
        print("last K:", gains[-1].reshape(2, 6))


if __name__ == "__main__":
    main()
