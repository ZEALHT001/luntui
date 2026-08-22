#!/usr/bin/env python3
"""
Recompute the VMC LQR gain schedule from the original ABK Matlab scripts.

This is a Matlab-free equivalent for:
  SPR Balanced-infantry/matlab/matlab/ABK.m
  SPR Balanced-infantry/matlab/matlab/ABK2.m

The original scripts:
  1. build the continuous wheel-leg model for each leg length,
  2. discretize it with c2d(),
  3. solve dlqr(),
  4. fit each K element as a cubic polynomial in leg length.

State order:
  [theta, theta_dot, x, x_dot, phi, phi_dot]

Input order:
  [T, Tp]

Flat C output order matches Matlab Coder's column-major array:
  [K(1,1), K(2,1), K(1,2), K(2,2), ..., K(1,6), K(2,6)]
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete


@dataclass(frozen=True)
class AbkPreset:
    wheel_radius: float
    body_com_to_hip: float
    wheel_mass: float
    leg_mass: float
    body_mass: float
    wheel_inertia: float
    leg_inertia: float
    body_inertia: float
    q: np.ndarray
    r: np.ndarray
    dt: float
    # ABK/ABK2 use "-L*(phi2*cos(phi)-phi1^2*sin(phi))" in Nm, not "-l*...".
    use_leg_half_length_for_body_x_pitch_arm: bool = True
    gravity: float = 9.8


PRESETS: dict[str, AbkPreset] = {
    "abk": AbkPreset(
        # Current A-board wheel-leg chassis parameters.
        # User measurements:
        #   two wheels total: 0.32 kg -> 0.16 kg each
        #   wheel radius: 55 mm
        #   two legs total: 3.5 kg -> 1.75 kg each
        #   body mass: 11 kg + gimbal 2.3 kg = 13.3 kg
        # Leg/body inertias are mass-scaled from the original ABK values until
        # measured CAD inertia values are available.
        wheel_radius=0.055,
        body_com_to_hip=0.0,
        wheel_mass=0.32 / 2.0,
        leg_mass=3.5 / 2.0,
        body_mass=13.3,
        wheel_inertia=0.5 * (0.32 / 2.0) * 0.055**2,
        leg_inertia=0.042737089 * ((3.5 / 2.0) / 0.95),
        body_inertia=(0.393175574 / 2.0 * 3.0) *
        (13.3 / (22.0 - 0.6 * 2.0 - 0.95 * 2.0)),
        q=np.diag([1.0, 100.0, 200.0, 500.0, 20000.0, 1.0]),
        r=np.diag([35.0, 1.0]),
        dt=0.001,
    ),
    "abk2": AbkPreset(
        wheel_radius=0.09,
        body_com_to_hip=0.0,
        wheel_mass=1.54,
        leg_mass=1.0,
        body_mass=13.6 - 2.0 * 1.0 - 2.0 * 1.54,
        wheel_inertia=0.5 * 1.54 * 0.09**2,
        leg_inertia=0.012330563,
        body_inertia=0.172475013,
        q=np.diag([1.0, 1.0, 500.0, 1000.0, 20000.0, 1.0]),
        r=np.diag([1.0, 1.0]),
        dt=0.005,
    ),
}


def parse_diag(value: str, expected: int, name: str) -> np.ndarray:
    parts = [float(part) for part in value.replace(",", " ").split()]
    if len(parts) != expected:
        raise argparse.ArgumentTypeError(
            f"{name} expects {expected} values, got {len(parts)}"
        )
    return np.diag(parts)


def residual(
    acc: np.ndarray,
    state: np.ndarray,
    control: np.ndarray,
    p: AbkPreset,
    leg_length: float,
) -> np.ndarray:
    theta, theta_dot, _x, _x_dot, phi, phi_dot = state
    theta_dd, x_dd, phi_dd = acc
    torque, hip_torque = control

    l_leg = leg_length / 2.0
    lm = leg_length / 2.0
    pitch_x_arm = l_leg if p.use_leg_half_length_for_body_x_pitch_arm else p.body_com_to_hip

    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)

    nm = p.body_mass * (
        x_dd
        + (l_leg + lm) * theta_dd * cos_theta
        - (l_leg + lm) * theta_dot**2 * sin_theta
        - pitch_x_arm * (phi_dd * cos_phi - phi_dot**2 * sin_phi)
    )
    pm = p.body_mass * p.gravity + p.body_mass * (
        -(l_leg + lm) * theta_dd * sin_theta
        - (l_leg + lm) * theta_dot**2 * cos_theta
        - p.body_com_to_hip * phi_dd * sin_phi
        - p.body_com_to_hip * phi_dot**2 * cos_phi
    )
    n = nm + p.leg_mass * (
        x_dd + l_leg * theta_dd * cos_theta - l_leg * theta_dot**2 * sin_theta
    )
    force_p = pm + p.leg_mass * p.gravity + p.leg_mass * (
        -l_leg * theta_dot**2 * cos_theta - l_leg * theta_dd * sin_theta
    )

    eq1 = x_dd - (torque - n * p.wheel_radius) / (
        p.wheel_inertia / p.wheel_radius + p.wheel_mass * p.wheel_radius
    )
    eq2 = (
        (force_p * l_leg + pm * lm) * sin_theta
        - (n * l_leg + nm * lm) * cos_theta
        - torque
        + hip_torque
        - p.leg_inertia * theta_dd
    )
    eq3 = (
        hip_torque
        + nm * p.body_com_to_hip * cos_phi
        + pm * p.body_com_to_hip * sin_phi
        - p.body_inertia * phi_dd
    )
    return np.array([eq1, eq2, eq3], dtype=float)


def solve_acceleration(
    state: np.ndarray,
    control: np.ndarray,
    p: AbkPreset,
    leg_length: float,
) -> np.ndarray:
    zero = np.zeros(3)
    offset = residual(zero, state, control, p, leg_length)
    mat = np.column_stack(
        [residual(np.eye(3)[i], state, control, p, leg_length) - offset for i in range(3)]
    )
    # Return [theta_dd, x_dd, phi_dd].
    return np.linalg.solve(mat, -offset)


def continuous_dynamics(
    state: np.ndarray,
    control: np.ndarray,
    p: AbkPreset,
    leg_length: float,
) -> np.ndarray:
    theta_dd, x_dd, phi_dd = solve_acceleration(state, control, p, leg_length)
    return np.array([state[1], theta_dd, state[3], x_dd, state[5], phi_dd], dtype=float)


def linearize(p: AbkPreset, leg_length: float, eps: float = 1e-6) -> tuple[np.ndarray, np.ndarray]:
    x0 = np.zeros(6)
    u0 = np.zeros(2)
    a = np.zeros((6, 6))
    b = np.zeros((6, 2))

    for i in range(6):
        delta = np.zeros(6)
        delta[i] = eps
        a[:, i] = (
            continuous_dynamics(x0 + delta, u0, p, leg_length)
            - continuous_dynamics(x0 - delta, u0, p, leg_length)
        ) / (2.0 * eps)

    for i in range(2):
        delta = np.zeros(2)
        delta[i] = eps
        b[:, i] = (
            continuous_dynamics(x0, u0 + delta, p, leg_length)
            - continuous_dynamics(x0, u0 - delta, p, leg_length)
        ) / (2.0 * eps)

    return a, b


def discrete_lqr(g: np.ndarray, h: np.ndarray, q: np.ndarray, r: np.ndarray) -> np.ndarray:
    p = solve_discrete_are(g, h, q, r)
    return np.linalg.solve(h.T @ p @ h + r, h.T @ p @ g)


def k_at_length(p: AbkPreset, leg_length: float) -> np.ndarray:
    a, b = linearize(p, leg_length)
    g, h, _c, _d, _dt = cont2discrete((a, b, np.eye(6), np.zeros((6, 2))), p.dt)
    return discrete_lqr(g, h, p.q, p.r)


def fit_schedule(
    p: AbkPreset,
    leg_min: float,
    leg_max: float,
    leg_step: float,
    degree: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    lengths = np.arange(leg_min, leg_max + leg_step * 0.5, leg_step)
    gains = np.vstack([k_at_length(p, float(length)).reshape(-1, order="F") for length in lengths])
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
    lines.extend(["}", ""])
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--preset", choices=sorted(PRESETS), default="abk")
    parser.add_argument("--leg-min", type=float, default=0.03)
    parser.add_argument("--leg-max", type=float, default=0.50)
    parser.add_argument("--leg-step", type=float, default=0.005)
    parser.add_argument("--degree", type=int, default=3)
    parser.add_argument(
        "--q",
        help="six diagonal Q values: theta dtheta x dx phi dphi, e.g. '1 100 200 500 8000 1'",
    )
    parser.add_argument(
        "--r",
        help="two diagonal R values: wheel_torque hip_torque, e.g. '35 1'",
    )
    parser.add_argument("--print-sample", type=float)
    parser.add_argument("--out-c", type=Path)
    args = parser.parse_args()

    preset = PRESETS[args.preset]
    if args.q:
        preset = replace(preset, q=parse_diag(args.q, 6, "Q"))
    if args.r:
        preset = replace(preset, r=parse_diag(args.r, 2, "R"))
    if args.print_sample is not None:
        np.set_printoptions(precision=12, suppress=False)
        print(k_at_length(preset, args.print_sample))
        return

    _lengths, gains, coeffs = fit_schedule(
        preset,
        args.leg_min,
        args.leg_max,
        args.leg_step,
        args.degree,
    )
    if args.out_c:
        args.out_c.write_text(emit_c(coeffs))
    else:
        np.set_printoptions(precision=12, suppress=False)
        print("coeffs:")
        print(coeffs)
        print("K at min:")
        print(gains[0].reshape(2, 6))
        print("K at max:")
        print(gains[-1].reshape(2, 6))


if __name__ == "__main__":
    main()
