#!/usr/bin/env python3
"""Design a LESO for the aboard balance model and validate it offline.

The observer extends the 6-state plant with one disturbance state per input
(wheel torque and virtual leg force), following the structure used by the
Qi-Q26 reference: system poles pulled in by a factor, disturbance poles kept
slow, gains from pole placement.  The disturbance estimate is then subtracted
from the LQR command, which removes the steady-state offset that a pure state
feedback leaves under a constant disturbance such as a slope.

This script answers two questions before any firmware changes:

1. How much steady-state error does the current LQR leave on a slope?
2. How much of it does the LESO remove, and how fast?

    python3 scripts/leso_validation.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete, place_poles

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from derive_vmc_lqr_abk import PRESETS, linearize  # noqa: E402

DT = 0.003
N = 6
M = 2
LEG_LENGTH = 0.28
FAST_FACTOR = 0.7
DISTURBANCE_POLE = 0.985
WHEEL_RADIUS = 0.06
MASS_KG = 10.0
GRAVITY = 9.81


def plant(preset, leg_length: float = LEG_LENGTH) -> tuple[np.ndarray, np.ndarray]:
    a, b = linearize(preset, leg_length)
    g, h, *_ = cont2discrete(
        (a, b, np.eye(N), np.zeros((N, M))), DT
    )
    return g, h


def lqr_gain(preset, g, h) -> np.ndarray:
    q = preset.q if preset.q.shape == (N, N) else preset.q[:N, :N]
    return np.linalg.solve(
        h.T @ solve_discrete_are(g, h, q, preset.r) @ h + preset.r,
        h.T @ solve_discrete_are(g, h, q, preset.r) @ g,
    )


def design_leso(g, h):
    """Return (A_e, B_e, C_e, L) for the disturbance-augmented model."""
    a_e = np.zeros((N + M, N + M))
    a_e[:N, :N] = g
    a_e[:N, N:] = h
    a_e[N:, N:] = np.eye(M)
    b_e = np.zeros((N + M, M))
    b_e[:N] = h
    c_e = np.zeros((N, N + M))
    c_e[:, :N] = np.eye(N)

    eigvals = np.linalg.eigvals(a_e)
    magnitudes = np.maximum(np.abs(eigvals) * FAST_FACTOR, 0.05)
    # place_poles rejects repeated poles, and several plant modes share a
    # magnitude; nudge them apart by a hair so the placement is well posed.
    poles = magnitudes + np.arange(len(magnitudes)) * 1e-5
    poles[-M:] = DISTURBANCE_POLE
    result = place_poles(a_e.T, c_e.T, poles)
    gain = result.gain_matrix.T
    return a_e, b_e, c_e, gain


def simulate(g, h, k, a_e, b_e, c_e, gain, disturbance, steps=4000):
    """Closed loop with the LESO compensating a constant input disturbance."""
    x = np.zeros(N)
    x_hat = np.zeros(N + M)
    history = []
    estimate = []
    for _ in range(steps):
        d_hat = x_hat[N:]
        u = -k @ x - d_hat
        history.append(x[2])
        estimate.append(d_hat[0])
        y = x.copy()  # every plant state is measured
        x_hat = a_e @ x_hat + b_e @ u + gain @ (y - c_e @ x_hat)
        x = g @ x + h @ (u + disturbance)
    return np.array(history), np.array(estimate)


def lqr_only_offset(g, h, k, disturbance):
    a_cl = np.eye(N) - g + h @ k
    return float(np.linalg.solve(a_cl, h @ disturbance)[2])


def main() -> int:
    preset = PRESETS["abk"]
    g, h = plant(preset)
    k = lqr_gain(preset, g, h)
    a_e, b_e, c_e, gain = design_leso(g, h)

    closed = a_e - gain @ c_e
    radius = float(np.max(np.abs(np.linalg.eigvals(closed))))
    print(f"LESO observer spectral radius: {radius:.4f} (must be < 1)")
    if radius >= 1.0:
        print("observer is not stable", file=sys.stderr)
        return 1

    print()
    print(f"{'slope':>7} {'wheel torque':>13} {'LQR offset':>12} {'LESO offset':>12} "
          f"{'LESO peak':>10} {'d_hat':>8} {'settle':>8}")
    for slope_deg in (0.0, 2.0, 5.0, 8.0, 12.0):
        force = MASS_KG * GRAVITY * np.sin(np.radians(slope_deg))
        torque = force * WHEEL_RADIUS
        disturbance = np.array([torque, 0.0])

        offset_lqr = lqr_only_offset(g, h, k, disturbance)
        trace, estimate = simulate(g, h, k, a_e, b_e, c_e, gain, disturbance)
        offset_leso = float(np.mean(trace[-500:]))
        peak = float(np.max(np.abs(trace)))
        d_hat = float(np.mean(estimate[-500:]))

        # Time after which the position stays within 5 mm of its final value.
        final = offset_leso
        settle = float("nan")
        for index in range(len(trace)):
            if np.all(np.abs(trace[index:] - final) < 0.005):
                settle = index * DT
                break

        print(
            f"{slope_deg:6.1f}° {torque:12.3f} {offset_lqr * 1000:11.1f} mm "
            f"{offset_leso * 1000:11.1f} mm {peak * 1000:9.1f} mm "
            f"{d_hat:8.3f} {settle:7.2f} s"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
