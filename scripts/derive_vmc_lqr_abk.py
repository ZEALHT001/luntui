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

The wheel-leg model already includes the drivetrain relationship used by the
firmware gain schedule.  Apply T directly here; multiplying it by the gearbox
ratio and efficiency again produces a different controller.

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
    # Drivetrain.  The M3508 rotor is geared to the wheel (ratio n) and its
    # stator is bolted to the leg, so the rotor's absolute speed is
    #   omega_rotor = n * omega_wheel + (1 - n) * omega_leg.
    # Its kinetic energy 0.5*Jr*omega_rotor^2 therefore contributes
    #   n^2 * Jr          to the wheel's inertial term, and
    #   (n - 1)^2 * Jr    to the LEG's inertial term,
    # plus a velocity cross term that is second order in the state and thus
    # vanishes from the LQR linearization.  Both inertia terms are applied
    # inside residual(); the cross term is deliberately not modelled.
    rotor_inertia: float = 0.0
    gear_ratio: float = 1.0
    # Rolling drag that is proportional to speed, expressed as a torque at the
    # wheel per unit BODY speed: tau_drag = -wheel_viscous * x_dot, in
    # N.m/(m/s).  Only the viscous part can enter a linear model; the Coulomb
    # part has no derivative at the origin and is deliberately left out.
    wheel_viscous: float = 0.0


PRESETS: dict[str, AbkPreset] = {
    "abk": AbkPreset(
        # Current A-board wheel-leg chassis parameters.
        # User measurements:
        #   wheel mass: 0.4 kg each
        #   wheel radius: 60 mm
        #   two legs total: 3.5 kg -> 1.75 kg each
        #   body mass: 11 kg + gimbal 2.3 kg = 13.3 kg
        #   body CoM: measured 30 mm BELOW the hip joint (negative = below).
        #     The original ABK script hard-coded l = 0.0, which makes the body
        #     pitch a decoupled double integrator: A[:,phi] = 0 and
        #     B[phi_dot, T] = 0, i.e. wheel torque cannot pitch the body and
        #     pitch cannot affect anything.  On the real machine pitch and
        #     translation are strongly coupled (measured d(pitch)/d(x) =
        #     0.109 rad/m, corr 0.77), so that assumption was wrong here.
        # Leg/body inertias are mass-scaled from the original ABK values until
        # measured CAD inertia values are available.
        wheel_radius=0.06,
        body_com_to_hip=-0.030,
        wheel_mass=0.4,
        leg_mass=3.5 / 2.0,
        body_mass=13.3,
        wheel_inertia=0.5 * 0.4 * 0.06**2,
        leg_inertia=0.042737089 * ((3.5 / 2.0) / 0.95),
        body_inertia=(0.393175574 / 2.0 * 3.0) *
        (13.3 / (22.0 - 0.6 * 2.0 - 0.95 * 2.0)),
        # M3508 drivetrain, measured 2026-10-04 by spinning the free wheels on
        # the stand with the LQR wheel row zeroed.  The wheel-only inertia
        # 0.5*0.4*0.06^2 = 7.2e-4 is 7x too small: the rotor inertia reflected
        # through the 268/17 gearbox is what dominates.
        #   measured (coast-adjusted) 0.00487 +- 0.0004 kg.m^2
        #   measured (raw impulse)    0.00683 kg.m^2 (drag inflates this)
        #   model with this Jr        0.00072 + 15.7647^2*2.0e-5 = 0.00569
        # so Jr is bracketed by 1.7e-5 .. 2.5e-5; 2.0e-5 is the midpoint.
        rotor_inertia=2.0e-5,
        gear_ratio=268.0 / 17.0,
        # Viscous rolling drag, measured 2026-10-04 from the free-wheel
        # coast-down on the stand (the only clean coast available - on the
        # ground the position loop is always active, so its coasts cannot be
        # fitted).  The wheel's own decay rate d(omega)/dt = -(a + b*omega)
        # was a=14.4/b=0.851 (left) and a=8.3/b=2.205 (right).  Converting
        # the viscous part to a torque at the wheel and averaging the two
        # wheels (the model is a single symmetric x axis) gives
        #   c = (0.101 + 0.262)/2 = 0.182 N.m per (m/s).
        # The Coulomb part (0.06-0.10 N.m) has no derivative at the origin and
        # cannot be represented here; it is the source of the low-speed dead
        # band and has to be handled in firmware if it matters.
        wheel_viscous=0.182,
        # Weights the firmware schedule is actually built from.  Keeping the
        # preset equal to the shipped weights means a bare run of this script
        # reproduces App/Src/vmc_lqr.c exactly, so the LQR tuner cannot
        # silently calculate from a different set than the file it writes.
        q=np.diag([1.0, 100.0, 400.0, 30.0, 5000.0, 1.0]),
        r=np.diag([20.0, 2.5]),
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


def drivetrain_inertias(p: AbkPreset) -> tuple[float, float]:
    """Wheel and leg inertias with the geared rotor folded in.

    The M3508 rotor is geared to the wheel (ratio n) and its stator is bolted
    to the leg, so the rotor's absolute speed is

        omega_rotor = n * omega_wheel + (1 - n) * omega_leg

    and its kinetic energy 0.5*Jr*omega_rotor^2 loads both inertias:

        wheel: Iw + n^2 * Jr
        leg:   Il + (n - 1)^2 * Jr

    The remaining n*(1-n)*Jr*omega_wheel*omega_leg cross term is second order
    in the state, so it vanishes from the LQR linearization and is not
    modelled.  Returns (wheel_inertia, leg_inertia) in kg.m^2.
    """
    ratio = p.gear_ratio
    wheel = p.wheel_inertia + ratio * ratio * p.rotor_inertia
    leg = p.leg_inertia + (ratio - 1.0) * (ratio - 1.0) * p.rotor_inertia
    return wheel, leg


def residual(
    acc: np.ndarray,
    state: np.ndarray,
    control: np.ndarray,
    p: AbkPreset,
    leg_length: float,
) -> np.ndarray:
    theta, theta_dot, _x, x_dot, phi, phi_dot = state
    theta_dd, x_dd, phi_dd = acc
    torque, hip_torque = control

    l_leg = leg_length / 2.0
    lm = leg_length / 2.0
    pitch_x_arm = l_leg if p.use_leg_half_length_for_body_x_pitch_arm else p.body_com_to_hip
    wheel_inertia, leg_inertia = drivetrain_inertias(p)

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

    # Viscous rolling drag acts on the wheel against its rotation and is the
    # only friction term a linear plant can carry: it lands directly on
    # A[3,3] = -wheel_viscous / (Iw/R + Mw*R).
    wheel_drag = p.wheel_viscous * x_dot
    eq1 = x_dd - (torque - n * p.wheel_radius - wheel_drag) / (
        wheel_inertia / p.wheel_radius + p.wheel_mass * p.wheel_radius
    )
    eq2 = (
        (force_p * l_leg + pm * lm) * sin_theta
        - (n * l_leg + nm * lm) * cos_theta
        - torque
        + hip_torque
        - leg_inertia * theta_dd
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


def discrete_mpc_model(
    p: AbkPreset, leg_length: float, sample_time_s: float
) -> tuple[np.ndarray, np.ndarray]:
    """Return the row-major A/B pair used by the firmware incremental MPC."""
    a, b = linearize(p, leg_length)
    g, h, _c, _d, _dt = cont2discrete(
        (a, b, np.eye(6), np.zeros((6, 2))), sample_time_s
    )
    return g, h


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


def report(p: AbkPreset, leg_length: float) -> None:
    """Print the plant at one leg length for bench-anchor validation."""
    a, b = linearize(p, leg_length)
    ratio = p.gear_ratio
    i_drivetrain = p.wheel_inertia + ratio * ratio * p.rotor_inertia
    i_leg_effective = p.leg_inertia + (ratio - 1.0) ** 2 * p.rotor_inertia
    np.set_printoptions(precision=6, suppress=True, linewidth=150)
    print(f"leg length            : {leg_length:.4f} m")
    print(f"gear ratio            : {ratio:.4f}")
    print(f"rotor inertia         : {p.rotor_inertia:.4e} kg.m^2")
    print(f"  -> wheel  + n^2*Jr  : {p.wheel_inertia:.4e} "
          f"-> {i_drivetrain:.4e} kg.m^2  (x{ratio * ratio:.1f})")
    print(f"  -> leg    +(n-1)^2*Jr: {p.leg_inertia:.4e} "
          f"-> {i_leg_effective:.4e} kg.m^2 "
          f"  (+{100.0 * ((ratio - 1.0) ** 2 * p.rotor_inertia) / p.leg_inertia:.1f}%)")
    print(f"leg natural frequency : {np.sqrt(a[1, 0]) / (2.0 * np.pi):.3f} Hz "
          f"(sqrt(A[1,0]), A[1,0]={a[1, 0]:.2f})")
    poles = np.linalg.eigvals(a)
    unstable = poles[poles.real > 1.0e-9]
    print(f"unstable open-loop    : {unstable.real.max():.4f} 1/s"
          if unstable.size else "unstable open-loop    : none")
    print("A =")
    print(a)
    print("B =")
    print(b)


def emit_metadata(preset_name: str, p: AbkPreset) -> str:
    """Comment block the LQR tuner reads back to recover the Q/R weights."""
    q_values = ", ".join(f"{float(value):.17g}" for value in np.diag(p.q))
    r_values = ", ".join(f"{float(value):.17g}" for value in np.diag(p.r))
    wheel, leg = drivetrain_inertias(p)
    return "\n".join(
        [
            "/* VMC_LQR_TUNER_METADATA",
            f" * preset: {preset_name}",
            f" * VMC_LQR_TUNER_Q: [{q_values}]",
            f" * VMC_LQR_TUNER_R: [{r_values}]",
            " *",
            " * Wheel-leg plant used to generate this schedule:",
            f" *   body_com_to_hip : {p.body_com_to_hip:+.4f} m",
            f" *   gear_ratio     : {p.gear_ratio:.6g}",
            f" *   rotor_inertia  : {p.rotor_inertia:.6g} kg.m^2",
            f" *   wheel inertia  : {p.wheel_inertia:.6g} (bare) -> "
            f"{wheel:.6g} kg.m^2 (with n^2*Jr)",
            f" *   leg inertia    : {p.leg_inertia:.6g} (bare) -> "
            f"{leg:.6g} kg.m^2 (with (n-1)^2*Jr)",
            f" *   wheel viscous  : {p.wheel_viscous:.6g} N.m/(m/s) "
            f"(sets A[3,3]; Coulomb drag is not modelled)",
            " */",
            "",
        ]
    )


def emit_c(coeffs: np.ndarray) -> str:
    """Write the cubic K schedule as firmware C."""
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


MPC_STATE_COUNT = 6
MPC_INPUT_COUNT = 2
MPC_HORIZON = 10
MPC_CONTROL_HORIZON = MPC_HORIZON
MPC_DECISION_COUNT = MPC_CONTROL_HORIZON * MPC_INPUT_COUNT
MPC_PREDICTION_STATES = MPC_HORIZON * MPC_STATE_COUNT
MPC_DELTA_WEIGHT = np.array([1.0, 0.05])
MPC_INPUT_WEIGHT_SCALE = np.array([1.0, 1.0])


def mpc_prediction_and_hessian(
    a: np.ndarray,
    b: np.ndarray,
    q: np.ndarray,
    r: np.ndarray,
    terminal: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Build the same condensed G/H matrices consumed by the firmware."""
    influence = np.zeros((MPC_PREDICTION_STATES, MPC_DECISION_COUNT))
    previous = np.zeros((MPC_STATE_COUNT, MPC_DECISION_COUNT))
    hessian = np.zeros((MPC_DECISION_COUNT, MPC_DECISION_COUNT))

    for step in range(MPC_HORIZON):
        current = a @ previous
        if step < MPC_CONTROL_HORIZON:
            current[
                :, step * MPC_INPUT_COUNT : (step + 1) * MPC_INPUT_COUNT
            ] += b
        rows = slice(step * MPC_STATE_COUNT, (step + 1) * MPC_STATE_COUNT)
        influence[rows] = current
        weight = terminal if step == MPC_HORIZON - 1 else q
        hessian += current.T @ weight @ current
        previous = current

    for step in range(MPC_CONTROL_HORIZON):
        for input_index, weight in enumerate(MPC_DELTA_WEIGHT):
            index = step * MPC_INPUT_COUNT + input_index
            hessian[index, index] += weight
    # The held control after the control horizon still contributes its
    # absolute-input cost at every remaining prediction step.
    for step in range(MPC_HORIZON):
        held_step = min(step, MPC_CONTROL_HORIZON - 1)
        for input_index in range(MPC_INPUT_COUNT):
            index = held_step * MPC_INPUT_COUNT + input_index
            hessian[index, index] += (
                MPC_INPUT_WEIGHT_SCALE[input_index]
                * r[input_index, input_index]
            )
    return influence, hessian


def emit_mpc_model(
    p: AbkPreset, leg_lengths: np.ndarray, sample_time_s: float
) -> str:
    """Emit the fixed-size linear-time-varying MPC model schedule."""
    def literal(value: float, precision: int = 12) -> str:
        text = f"{float(value):.{precision}g}"
        if "." not in text and "e" not in text.lower():
            text += ".0"
        return f"{text}f"

    lines = [
        "/* Generated by scripts/derive_vmc_lqr_abk.py --out-mpc-model.",
        " * Do not edit by hand. */",
        '#include "vmc_mpc.h"',
        "",
        f"const float vmc_mpc_model_sample_time_s = {literal(sample_time_s)};",
        "",
        "const vmc_mpc_model_point_t",
        "    vmc_mpc_model_points[VMC_MPC_MODEL_POINTS] = {",
    ]
    for leg_length in leg_lengths:
        a, b = discrete_mpc_model(p, float(leg_length), sample_time_s)
        terminal_cost = solve_discrete_are(a, b, p.q, p.r)
        influence, hessian = mpc_prediction_and_hessian(
            a, b, p.q, p.r, terminal_cost
        )
        lines.append(f"  {{{literal(float(leg_length), 9)},")
        lines.append("   {")
        for row in a:
            values = ", ".join(literal(float(value)) for value in row)
            lines.append(f"    {{{values}}},")
        lines.append("   },")
        lines.append("   {")
        for row in b:
            values = ", ".join(literal(float(value)) for value in row)
            lines.append(f"    {{{values}}},")
        lines.append("   },")
        lines.append("   {")
        for row in terminal_cost:
            values = ", ".join(literal(float(value)) for value in row)
            lines.append(f"    {{{values}}},")
        lines.append("   },")
        lines.append("   {")
        for row in influence:
            values = ", ".join(
                literal(float(value), 9) for value in row
            )
            lines.append(f"    {{{values}}},")
        lines.append("   },")
        lines.append("   {")
        for row in hessian:
            values = ", ".join(
                literal(float(value), 9) for value in row
            )
            lines.append(f"    {{{values}}},")
        lines.append("   }},")
    lines.extend(["};", ""])
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
    parser.add_argument(
        "--mpc-q",
        help="six diagonal Q values used for the MPC terminal cost",
    )
    parser.add_argument("--print-sample", type=float)
    parser.add_argument(
        "--rotor-inertia",
        type=float,
        help="M3508 rotor inertia in kg.m^2 (reflected through the gearbox)",
    )
    parser.add_argument(
        "--gear-ratio",
        type=float,
        help="motor-to-wheel reduction ratio (default: preset value)",
    )
    parser.add_argument(
        "--body-com",
        type=float,
        help=(
            "signed body CoM offset from the hip in m, positive forward "
            "(default: preset value)"
        ),
    )
    parser.add_argument(
        "--wheel-viscous",
        type=float,
        help=(
            "viscous rolling drag at the wheel per unit body speed, "
            "N.m/(m/s); sets A[3,3] (default: preset value)"
        ),
    )
    parser.add_argument(
        "--report",
        type=float,
        metavar="LEG_LENGTH",
        help=(
            "print A, B, the leg natural frequency and the reflected "
            "drivetrain inertia for one leg length, then exit"
        ),
    )
    parser.add_argument(
        "--out-mpc-model",
        type=Path,
        help="write the discrete A/B schedule consumed by vmc_mpc.c",
    )
    parser.add_argument(
        "--mpc-dt",
        type=float,
        default=0.002222,
        help="MPC sample time in seconds (default: the 450 Hz firmware period)",
    )
    parser.add_argument(
        "--mpc-leg-min",
        type=float,
        default=0.180,
        help="first leg length in the MPC model schedule",
    )
    parser.add_argument(
        "--mpc-leg-max",
        type=float,
        default=0.385,
        help="last leg length in the MPC model schedule",
    )
    parser.add_argument("--out-c", type=Path)
    args = parser.parse_args()

    preset = PRESETS[args.preset]
    if args.q:
        preset = replace(preset, q=parse_diag(args.q, 6, "Q"))
    if args.r:
        preset = replace(preset, r=parse_diag(args.r, 2, "R"))
    if args.rotor_inertia is not None:
        preset = replace(preset, rotor_inertia=args.rotor_inertia)
    if args.gear_ratio is not None:
        preset = replace(preset, gear_ratio=args.gear_ratio)
    if args.body_com is not None:
        preset = replace(preset, body_com_to_hip=args.body_com)
    if args.wheel_viscous is not None:
        preset = replace(preset, wheel_viscous=args.wheel_viscous)
    if args.report is not None:
        report(preset, args.report)
        return
    if args.print_sample is not None:
        np.set_printoptions(precision=12, suppress=False)
        print(k_at_length(preset, args.print_sample))
        return
    if args.out_mpc_model:
        mpc_lengths = np.linspace(
            args.mpc_leg_min, args.mpc_leg_max, num=9
        )
        mpc_preset = preset
        if args.mpc_q:
            mpc_preset = replace(
                preset, q=parse_diag(args.mpc_q, 6, "MPC Q")
            )
        args.out_mpc_model.write_text(
            emit_mpc_model(mpc_preset, mpc_lengths, args.mpc_dt),
            encoding="utf-8",
        )
        return

    _lengths, gains, coeffs = fit_schedule(
        preset,
        args.leg_min,
        args.leg_max,
        args.leg_step,
        args.degree,
    )
    if args.out_c:
        args.out_c.write_text(
            emit_metadata(args.preset, preset) + emit_c(coeffs),
            encoding="utf-8",
        )
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
