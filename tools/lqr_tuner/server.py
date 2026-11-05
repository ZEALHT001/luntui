#!/usr/bin/env python3
"""Local LQR tuning server.

Receives Q/R weights from the Vite frontend, computes the scheduled VMC LQR
gain matrix using the existing ABK model script, and can send the selected
gain over SEGGER RTT as one JSON line.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))

from derive_vmc_lqr_abk import (  # noqa: E402
    PRESETS,
    emit_c,
    emit_metadata,
    fit_schedule,
    k_at_length,
)


STATE_NAMES = ["theta", "dtheta", "x", "dx", "phi", "dphi"]
INPUT_NAMES = ["T", "Tp"]
DEFAULT_RTT_HOST = "127.0.0.1"
DEFAULT_RTT_PORT = 19021
DEFAULT_VOFA_HOST = "127.0.0.1"
DEFAULT_VOFA_PORT = 1347
# Fallback table.  This is only used for keys that are not backed by a macro
# in App/Inc/vmc_probe.h, or when that header cannot be read.  Everything else
# resolves through _firmware_balance_params() below.
_BALANCE_PARAM_FALLBACK = {
    "theta_target": 0.0,
    "pitch_target": -0.05,
    "phi_deadband": 0.0,
    "tp_output_enable": 1.0,
    "wheel_output_user_enable": 1.0,
    "theta_deadband": 0.0,
    "lqr_dtheta_gain": 1.0,
    "lqr_dtheta_deadband": 0.0,
    "lqr_dtheta_limit": 10.0,
    # LQR T is 3508 motor-shaft torque (N.m), not wheel/axle torque.
    "wheel_torque_limit": 3.3,
    "wheel_current_limit": 5000.0,
    "airborne_detect_enable": 1.0,
    "airborne_wheel_speed_threshold": 4.0,
    "airborne_joint_torque_threshold": 35.0,
    "airborne_confirm_ms": 20.0,
    "airborne_release_ms": 100.0,
    "airborne_pitch_dev_max": 0.20,
    "airborne_pitch_rate_max": 4.0,
    "airborne_side_enable": 1.0,
    "airborne_side_confirm_ms": 30.0,
    "airborne_side_release_ms": 100.0,
    "tp_limit": 108.0,
    "tp_phi_gain": 1.0,
    "tp_x_gain": 1.0,
    "leg_sync_enable": 1.0,
    "leg_sync_kp": 60.0,
    "leg_sync_kd": 0.0,
    "leg_sync_torque_limit": 12.0,
    "dpitch_lpf_alpha": 1.0,
    "mpc_enable": 0.0,
    "mpc_shadow_enable": 1.0,
    "compute_only_enable": 1.0,
    "mpc_wheel_authority": 0.0,
    "mpc_leg_authority": 1.0,
    "mpc_authority_ramp_rate": 0.5,
    "disturbance_observer_enable": 0.0,
    # These act as "rate * dt" terms, and dt is now the measured control period
    # (~2.22 ms) rather than the old assumed 1 ms.  Values are scaled by 0.45 to
    # keep the behaviour the machine was tuned for; left unscaled the
    # disturbance observer limit-cycles while standing still.  Keep in step with
    # App/Inc/vmc_probe.h - see VMC_CONTROL_DT there.
    "disturbance_observer_gain": 0.2,
    "disturbance_observer_leak": 0.06,
    "disturbance_observer_limit": 0.05,
    "disturbance_observer_speed_limit": 0.25,
    "disturbance_observer_delay": 1.11,
    "disturbance_observer_pitch_gain": 0.0,
    "disturbance_observer_theta_gain": -1.0,
    "x_feedback_gain": -1.0,
    "dx_feedback_gain": -1.0,
    "x_feedback_limit": 20.0,
    "dx_feedback_limit": 30.0,
    "speed_control_enable": 1.0,
    "speed_target_max": 2.0,
    "speed_ramp_rate": 2.25,
    "speed_deadband": 0.02,
    "stop_speed_threshold": 0.08,
    "x_hold_max_stretch": 0.15,
    "speed_pitch_gain": 0.0,
    "speed_pitch_limit": 0.12,
    "speed_pitch_ramp_rate": 0.36,
    "kp_length": 2000.0,
    "kd_length": 100.0,
    "max_force": 500.0,
    "support_force": 0.0,
    "length_ramp_rate": 0.225,
    "jump_enable": 1.0,
    "jump_moving_enable": 1.0,
    "jump_compress_length": 0.18,
    "jump_extend_length": 0.36,
    "jump_retract_length": 0.18,
    "jump_land_length": 0.23,
    "jump_kp_length": 3000.0,
    "jump_kd_length": 180.0,
    "jump_max_force": 350.0,
    "jump_support_force": 0.0,
    "jump_compress_timeout_ms": 180.0,
    "jump_extend_timeout_ms": 120.0,
    "jump_retract_timeout_ms": 140.0,
    "jump_land_timeout_ms": 600.0,
    "jump_cooldown_ms": 1500.0,
    "jump_total_timeout_ms": 1200.0,
    "jump_attitude_limit": 0.35,
    "jump_land_torque_threshold": 3.0,
    "jump_land_confirm_ms": 80.0,
    "jump_auto_speed_min": 1.0,
    "jump_auto_dist_min_cm": 80.0,
    "jump_auto_dist_max_cm": 100.0,
    "jump_auto_confirm_ms": 20.0,
    "stair_enable": 1.0,
    "stair_lift_length": 0.248,
    "stair_retract_length": 0.18,
    "stair_lift_theta": 1.5707963,
    "stair_retract_theta": 1.0472,
    "stair_pose_length_tol": 0.06,
    "stair_pose_theta_tol": 0.30,
    "stair_pose_confirm_ms": 100.0,
    "stair_phi0_target": 2.5,
    "stair_lift_phi0_ramp_rate": 4.0,
    "stair_phi0_ramp_rate": 2.5,
    "stair_phi0_kp": 120.0,
    "stair_phi0_kd": 6.0,
    "stair_speed_threshold": 0.35,
    "stair_command_speed_threshold": 0.18,
    "stair_ready_length": 0.33,
    "stair_ready_confirm_ms": 200.0,
    "stair_speed_confirm_ms": 25.0,
    "stair_preimpact_confirm_ms": 20.0,
    "stair_impact_confirm_ms": 15.0,
    "stair_preimpact_diff_max": 0.0,
    "stair_impact_diff_threshold": 0.40,
    "stair_impact_torque_threshold": 1.45,
    "stair_impact_theta_threshold": 0.19,
    "stair_length_ramp_rate": 0.18,
    "stair_retract_timeout_ms": 2000.0,
    "stair_hold_ms": 500.0,
    "stair_recover_ms": 500.0,
    "stair_timeout_ms": 6000.0,
    "stair_cooldown_ms": 1200.0,
    "stair_kp_length": 2000.0,
    "stair_kd_length": 100.0,
    "stair_max_force": 500.0,
    "stair_lift_support_force": 20.0,
    "stair_retract_support_force": -14.0,
    "stair_support_force": 100.0,
    "roll_comp_enable": 1.0,
    "roll_target": 0.0,
    "roll_comp_gain": 0.10,
    "roll_comp_limit": 0.1025,
    "roll_comp_ramp_rate": 0.054,
    "kp_phi0": 50.0,
    "kd_phi0": 0.5,
    "max_phi0_torque": 108.0,
    "yaw_hold_enable": 1.0,
    "yaw_hold_kp": 2000.0,
    "yaw_hold_kd": 0.0,
    "yaw_hold_deadband": 0.01,
    "yaw_target_rate_max": 0.9,
    "yaw_hold_current_limit": 5000.0,
    "gimbal_follow_enable": 1.0,
    "gimbal_follow_kp": 0.8,
    "gimbal_follow_kd": 2.0,
    "gimbal_follow_deadband": 15.0,
    "gimbal_follow_current_limit": 1500.0,
    "gimbal_follow_direction": 1.0,
}
DEFAULT_KALMAN_PARAMS = {
    "q_v": 0.1,
    "q_a": 0.1,
    "r_wheel": 100.0,
    "r_imu": 1.0e12,
}

BALANCE_PARAM_DEFAULT_MACROS: dict[str, tuple[str, str]] = {
    "theta_target": ("VMC_BALANCE_THETA_TARGET_DEFAULT", "float"),
    "pitch_target": ("VMC_BALANCE_PITCH_TARGET_DEFAULT", "float"),
    "phi_deadband": ("VMC_BALANCE_PHI_DEADBAND_DEFAULT", "float"),
    "tp_output_enable": ("VMC_BALANCE_TP_OUTPUT_ENABLE_DEFAULT", "uint"),
    "wheel_output_user_enable": ("VMC_BALANCE_WHEEL_OUTPUT_USER_ENABLE_DEFAULT", "uint"),
    "theta_deadband": ("VMC_BALANCE_THETA_DEADBAND_DEFAULT", "float"),
    "lqr_dtheta_gain": ("VMC_BALANCE_LQR_DTHETA_GAIN_DEFAULT", "float"),
    "lqr_dtheta_deadband": ("VMC_BALANCE_LQR_DTHETA_DEADBAND_DEFAULT", "float"),
    "lqr_dtheta_limit": ("VMC_BALANCE_LQR_DTHETA_LIMIT_DEFAULT", "float"),
    "wheel_torque_limit": ("VMC_BALANCE_WHEEL_TORQUE_LIMIT_DEFAULT", "float"),
    "wheel_current_limit": ("VMC_BALANCE_WHEEL_CURRENT_LIMIT_DEFAULT", "int"),
    "airborne_detect_enable": ("VMC_AIRBORNE_DETECT_ENABLE_DEFAULT", "uint"),
    "airborne_wheel_speed_threshold": ("VMC_AIRBORNE_WHEEL_SPEED_THRESHOLD_DEFAULT", "float"),
    "airborne_joint_torque_threshold": ("VMC_AIRBORNE_JOINT_TORQUE_THRESHOLD_DEFAULT", "float"),
    "airborne_confirm_ms": ("VMC_AIRBORNE_CONFIRM_MS_DEFAULT", "uint_value"),
    "airborne_release_ms": ("VMC_AIRBORNE_RELEASE_MS_DEFAULT", "uint_value"),
    "airborne_pitch_dev_max": ("VMC_AIRBORNE_PITCH_DEV_MAX_DEFAULT", "float"),
    "airborne_pitch_rate_max": ("VMC_AIRBORNE_PITCH_RATE_MAX_DEFAULT", "float"),
    "airborne_side_enable": ("VMC_AIRBORNE_SIDE_ENABLE_DEFAULT", "uint"),
    "airborne_side_confirm_ms": ("VMC_AIRBORNE_SIDE_CONFIRM_MS_DEFAULT", "uint_value"),
    "airborne_side_release_ms": ("VMC_AIRBORNE_SIDE_RELEASE_MS_DEFAULT", "uint_value"),
    "tp_limit": ("VMC_BALANCE_TP_LIMIT_DEFAULT", "float"),
    "tp_phi_gain": ("VMC_BALANCE_TP_PHI_GAIN_DEFAULT", "float"),
    "tp_x_gain": ("VMC_BALANCE_TP_X_GAIN_DEFAULT", "float"),
    "leg_sync_enable": ("VMC_LEG_SYNC_ENABLE_DEFAULT", "uint"),
    "leg_sync_kp": ("VMC_LEG_SYNC_KP_DEFAULT", "float"),
    "leg_sync_kd": ("VMC_LEG_SYNC_KD_DEFAULT", "float"),
    "leg_sync_torque_limit": ("VMC_LEG_SYNC_TORQUE_LIMIT_DEFAULT", "float"),
    "dpitch_lpf_alpha": ("VMC_BALANCE_DPITCH_LPF_ALPHA_DEFAULT", "float"),
    "mpc_enable": ("VMC_MPC_ENABLE_DEFAULT", "uint"),
    "mpc_shadow_enable": ("VMC_MPC_SHADOW_ENABLE_DEFAULT", "uint"),
    "compute_only_enable": ("VMC_COMPUTE_ONLY_DEFAULT", "uint"),
    "mpc_wheel_authority": ("VMC_MPC_WHEEL_AUTHORITY_DEFAULT", "float"),
    "mpc_leg_authority": ("VMC_MPC_LEG_AUTHORITY_DEFAULT", "float"),
    "mpc_authority_ramp_rate": ("VMC_MPC_AUTHORITY_RAMP_RATE_DEFAULT", "float"),
    "disturbance_observer_enable": ("VMC_BALANCE_DISTURBANCE_OBSERVER_ENABLE_DEFAULT", "uint"),
    "disturbance_observer_gain": ("VMC_BALANCE_DISTURBANCE_OBSERVER_GAIN_DEFAULT", "float"),
    "disturbance_observer_leak": ("VMC_BALANCE_DISTURBANCE_OBSERVER_LEAK_DEFAULT", "float"),
    "disturbance_observer_limit": ("VMC_BALANCE_DISTURBANCE_OBSERVER_LIMIT_DEFAULT", "float"),
    "disturbance_observer_speed_limit": ("VMC_BALANCE_DISTURBANCE_OBSERVER_SPEED_LIMIT_DEFAULT", "float"),
    "disturbance_observer_delay": ("VMC_BALANCE_DISTURBANCE_OBSERVER_DELAY_DEFAULT", "float"),
    "disturbance_observer_pitch_gain": ("VMC_BALANCE_DISTURBANCE_OBSERVER_PITCH_GAIN_DEFAULT", "float"),
    "disturbance_observer_theta_gain": ("VMC_BALANCE_DISTURBANCE_OBSERVER_THETA_GAIN_DEFAULT", "float"),
    "x_feedback_gain": ("VMC_BALANCE_X_FEEDBACK_GAIN_DEFAULT", "float"),
    "dx_feedback_gain": ("VMC_BALANCE_DX_FEEDBACK_GAIN_DEFAULT", "float"),
    "x_feedback_limit": ("VMC_BALANCE_X_FEEDBACK_LIMIT_DEFAULT", "float"),
    "dx_feedback_limit": ("VMC_BALANCE_DX_FEEDBACK_LIMIT_DEFAULT", "float"),
    "speed_control_enable": ("VMC_BALANCE_SPEED_CONTROL_ENABLE_DEFAULT", "uint"),
    "speed_target_max": ("VMC_BALANCE_SPEED_TARGET_MAX_DEFAULT", "float"),
    "speed_ramp_rate": ("VMC_BALANCE_SPEED_RAMP_RATE_DEFAULT", "float"),
    "speed_deadband": ("VMC_BALANCE_SPEED_DEADBAND_DEFAULT", "float"),
    "stop_speed_threshold": ("VMC_BALANCE_STOP_SPEED_THRESHOLD_DEFAULT", "float"),
    "x_hold_max_stretch": ("VMC_BALANCE_X_HOLD_MAX_STRETCH_DEFAULT", "float"),
    "speed_pitch_gain": ("VMC_BALANCE_SPEED_PITCH_GAIN_DEFAULT", "float"),
    "speed_pitch_limit": ("VMC_BALANCE_SPEED_PITCH_LIMIT_DEFAULT", "float"),
    "speed_pitch_ramp_rate": ("VMC_BALANCE_SPEED_PITCH_RAMP_RATE_DEFAULT", "float"),
    "kp_length": ("VMC_LEG_LENGTH_KP_DEFAULT", "float"),
    "kd_length": ("VMC_LEG_LENGTH_KD_DEFAULT", "float"),
    "max_force": ("VMC_LEG_FORCE_MAX_DEFAULT", "float"),
    "support_force": ("VMC_LEG_SUPPORT_FORCE_DEFAULT", "float"),
    "length_ramp_rate": ("VMC_LEG_LENGTH_RAMP_RATE_DEFAULT", "float"),
    "jump_enable": ("VMC_JUMP_ENABLE_DEFAULT", "uint"),
    "jump_moving_enable": ("VMC_JUMP_MOVING_ENABLE_DEFAULT", "uint"),
    "jump_compress_length": ("VMC_JUMP_COMPRESS_LENGTH_DEFAULT", "float"),
    "jump_extend_length": ("VMC_JUMP_EXTEND_LENGTH_DEFAULT", "float"),
    "jump_retract_length": ("VMC_JUMP_RETRACT_LENGTH_DEFAULT", "float"),
    "jump_land_length": ("VMC_JUMP_LAND_LENGTH_DEFAULT", "float"),
    "jump_kp_length": ("VMC_JUMP_LENGTH_KP_DEFAULT", "float"),
    "jump_kd_length": ("VMC_JUMP_LENGTH_KD_DEFAULT", "float"),
    "jump_max_force": ("VMC_JUMP_FORCE_MAX_DEFAULT", "float"),
    "jump_support_force": ("VMC_JUMP_SUPPORT_FORCE_DEFAULT", "float"),
    "jump_compress_timeout_ms": ("VMC_JUMP_COMPRESS_TIMEOUT_MS_DEFAULT", "uint_value"),
    "jump_extend_timeout_ms": ("VMC_JUMP_EXTEND_TIMEOUT_MS_DEFAULT", "uint_value"),
    "jump_retract_timeout_ms": ("VMC_JUMP_RETRACT_TIMEOUT_MS_DEFAULT", "uint_value"),
    "jump_land_timeout_ms": ("VMC_JUMP_LAND_TIMEOUT_MS_DEFAULT", "uint_value"),
    "jump_cooldown_ms": ("VMC_JUMP_COOLDOWN_MS_DEFAULT", "uint_value"),
    "jump_total_timeout_ms": ("VMC_JUMP_TOTAL_TIMEOUT_MS_DEFAULT", "uint_value"),
    "jump_attitude_limit": ("VMC_JUMP_ATTITUDE_LIMIT_DEFAULT", "float"),
    "jump_land_torque_threshold": ("VMC_JUMP_LAND_TORQUE_THRESHOLD_DEFAULT", "float"),
    "jump_land_confirm_ms": ("VMC_JUMP_LAND_CONFIRM_MS_DEFAULT", "uint_value"),
    "jump_auto_speed_min": ("VMC_JUMP_AUTO_SPEED_MIN", "float"),
    "jump_auto_dist_min_cm": ("VMC_JUMP_AUTO_DIST_MIN_CM", "float"),
    "jump_auto_dist_max_cm": ("VMC_JUMP_AUTO_DIST_MAX_CM", "float"),
    "jump_auto_confirm_ms": ("VMC_JUMP_AUTO_CONFIRM_MS", "uint_value"),
    "stair_enable": ("VMC_STAIR_ENABLE_DEFAULT", "uint"),
    "stair_lift_length": ("VMC_STAIR_LIFT_LENGTH_DEFAULT", "float"),
    "stair_retract_length": ("VMC_STAIR_RETRACT_LENGTH_DEFAULT", "float"),
    "stair_lift_theta": ("VMC_STAIR_LIFT_THETA_DEFAULT", "float"),
    "stair_retract_theta": ("VMC_STAIR_RETRACT_THETA_DEFAULT", "float"),
    "stair_pose_length_tol": ("VMC_STAIR_POSE_LENGTH_TOL_DEFAULT", "float"),
    "stair_pose_theta_tol": ("VMC_STAIR_POSE_THETA_TOL_DEFAULT", "float"),
    "stair_pose_confirm_ms": ("VMC_STAIR_POSE_CONFIRM_MS_DEFAULT", "uint_value"),
    "stair_phi0_target": ("VMC_STAIR_PHI0_TARGET_DEFAULT", "float"),
    "stair_lift_phi0_ramp_rate": (
        "VMC_STAIR_LIFT_PHI0_RAMP_RATE_DEFAULT",
        "float",
    ),
    "stair_phi0_ramp_rate": ("VMC_STAIR_PHI0_RAMP_RATE_DEFAULT", "float"),
    "stair_phi0_kp": ("VMC_STAIR_PHI0_KP_DEFAULT", "float"),
    "stair_phi0_kd": ("VMC_STAIR_PHI0_KD_DEFAULT", "float"),
    "stair_speed_threshold": ("VMC_STAIR_SPEED_THRESHOLD_DEFAULT", "float"),
    "stair_command_speed_threshold": (
        "VMC_STAIR_COMMAND_SPEED_THRESHOLD_DEFAULT",
        "float",
    ),
    "stair_ready_length": ("VMC_STAIR_READY_LENGTH_DEFAULT", "float"),
    "stair_ready_confirm_ms": ("VMC_STAIR_READY_CONFIRM_MS_DEFAULT", "uint_value"),
    "stair_speed_confirm_ms": ("VMC_STAIR_SPEED_CONFIRM_MS_DEFAULT", "uint_value"),
    "stair_preimpact_confirm_ms": (
        "VMC_STAIR_PREIMPACT_CONFIRM_MS_DEFAULT",
        "uint_value",
    ),
    "stair_impact_confirm_ms": ("VMC_STAIR_IMPACT_CONFIRM_MS_DEFAULT", "uint_value"),
    "stair_preimpact_diff_max": ("VMC_STAIR_PREIMPACT_DIFF_MAX_DEFAULT", "float"),
    "stair_impact_diff_threshold": (
        "VMC_STAIR_IMPACT_DIFF_THRESHOLD_DEFAULT",
        "float",
    ),
    "stair_impact_torque_threshold": (
        "VMC_STAIR_IMPACT_TORQUE_THRESHOLD_DEFAULT",
        "float",
    ),
    "stair_impact_theta_threshold": (
        "VMC_STAIR_IMPACT_THETA_THRESHOLD_DEFAULT",
        "float",
    ),
    "stair_retract_timeout_ms": ("VMC_STAIR_RETRACT_TIMEOUT_MS_DEFAULT", "uint_value"),
    "stair_hold_ms": ("VMC_STAIR_HOLD_MS_DEFAULT", "uint_value"),
    "stair_recover_ms": ("VMC_STAIR_RECOVER_MS_DEFAULT", "uint_value"),
    "stair_timeout_ms": ("VMC_STAIR_TIMEOUT_MS_DEFAULT", "uint_value"),
    "stair_cooldown_ms": ("VMC_STAIR_COOLDOWN_MS_DEFAULT", "uint_value"),
    "stair_kp_length": ("VMC_STAIR_LENGTH_KP_DEFAULT", "float"),
    "stair_kd_length": ("VMC_STAIR_LENGTH_KD_DEFAULT", "float"),
    "stair_length_ramp_rate": ("VMC_STAIR_LENGTH_RAMP_RATE_DEFAULT", "float"),
    "stair_max_force": ("VMC_STAIR_FORCE_MAX_DEFAULT", "float"),
    "stair_lift_support_force": ("VMC_STAIR_LIFT_SUPPORT_FORCE_DEFAULT", "float"),
    "stair_retract_support_force": (
        "VMC_STAIR_RETRACT_SUPPORT_FORCE_DEFAULT",
        "float",
    ),
    "stair_support_force": ("VMC_STAIR_SUPPORT_FORCE_DEFAULT", "float"),
    "roll_comp_enable": ("VMC_ROLL_COMP_ENABLE_DEFAULT", "uint"),
    "roll_target": ("VMC_ROLL_TARGET_DEFAULT", "float"),
    "roll_comp_gain": ("VMC_ROLL_LENGTH_COMP_GAIN_DEFAULT", "float"),
    "roll_comp_limit": ("VMC_ROLL_LENGTH_COMP_LIMIT_DEFAULT", "float"),
    "roll_comp_ramp_rate": ("VMC_ROLL_LENGTH_COMP_RAMP_RATE_DEFAULT", "float"),
    "kp_phi0": ("VMC_LEG_PHI0_KP_DEFAULT", "float"),
    "kd_phi0": ("VMC_LEG_PHI0_KD_DEFAULT", "float"),
    "max_phi0_torque": ("VMC_LEG_PHI0_TORQUE_MAX_DEFAULT", "float"),
    "yaw_hold_enable": ("VMC_YAW_HOLD_ENABLE_DEFAULT", "uint"),
    "yaw_hold_kp": ("VMC_YAW_HOLD_KP_DEFAULT", "float"),
    "yaw_hold_kd": ("VMC_YAW_HOLD_KD_DEFAULT", "float"),
    "yaw_hold_deadband": ("VMC_YAW_HOLD_DEADBAND_DEFAULT", "float"),
    "yaw_target_rate_max": ("VMC_RC_YAW_TARGET_RATE_MAX_DEFAULT", "float"),
    "yaw_hold_current_limit": ("VMC_YAW_HOLD_CURRENT_LIMIT_DEFAULT", "int"),
    "gimbal_follow_enable": ("VMC_GIMBAL_FOLLOW_ENABLE_DEFAULT", "uint"),
    "gimbal_follow_front_ecd": ("VMC_GIMBAL_FOLLOW_FRONT_ECD_DEFAULT", "uint_value"),
    "gimbal_follow_front_valid": ("VMC_GIMBAL_FOLLOW_FRONT_VALID_DEFAULT", "uint"),
    "gimbal_follow_kp": ("VMC_GIMBAL_FOLLOW_KP_DEFAULT", "float"),
    "gimbal_follow_kd": ("VMC_GIMBAL_FOLLOW_KD_DEFAULT", "float"),
    "gimbal_follow_deadband": ("VMC_GIMBAL_FOLLOW_DEADBAND_DEFAULT", "float"),
    "gimbal_follow_current_limit": ("VMC_GIMBAL_FOLLOW_CURRENT_LIMIT_DEFAULT", "int"),
    "gimbal_follow_direction": ("VMC_GIMBAL_FOLLOW_DIRECTION_DEFAULT", "float"),
}

# Parameters used by the web UI but not sent in the runtime balance-parameter
# payload still have a firmware-side default that can be imported.
FIRMWARE_SCALAR_DEFAULT_MACROS: dict[str, tuple[str, str]] = {
    "leg_length": ("VMC_LEG_LENGTH_DEFAULT", "float"),
}

KALMAN_PARAM_DEFAULT_MACROS: dict[str, tuple[str, str]] = {
    "q_v": ("VMC_BODY_MOTION_KF_Q_V_DEFAULT", "float"),
    "q_a": ("VMC_BODY_MOTION_KF_Q_A_DEFAULT", "float"),
    "r_wheel": ("VMC_BODY_MOTION_KF_R_WHEEL_DEFAULT", "float"),
    "r_imu": ("VMC_BODY_MOTION_KF_R_IMU_DEFAULT", "float"),
}


def _rtt_host(data: dict[str, Any]) -> str:
    host = str(data.get("rtt_host") or "").strip()
    if not host or host.lower() in {"undefined", "null", "none"}:
        return DEFAULT_RTT_HOST
    return host


def _rtt_port(data: dict[str, Any]) -> int:
    try:
        port = int(data.get("rtt_port") or 0)
    except (TypeError, ValueError):
        port = 0
    if port <= 0 or port > 65535:
        return DEFAULT_RTT_PORT
    return port


def _diag(values: list[float], expected: int, name: str) -> np.ndarray:
    if len(values) != expected:
        raise ValueError(f"{name} expects {expected} values, got {len(values)}")
    return np.diag([float(v) for v in values])


def _preset_from_request(data: dict[str, Any]):
    preset_name = str(data.get("preset", "abk"))
    if preset_name not in PRESETS:
        raise ValueError(f"unknown preset: {preset_name}")

    preset = PRESETS[preset_name]
    if "q" in data:
        preset = replace(preset, q=_diag(data["q"], 6, "q"))
    if "r" in data:
        preset = replace(preset, r=_diag(data["r"], 2, "r"))
    return preset_name, preset


def _calc_response(data: dict[str, Any]) -> dict[str, Any]:
    preset_name, preset = _preset_from_request(data)
    leg_length = float(data.get("leg_length", 0.28))
    leg_min = float(data.get("leg_min", 0.03))
    leg_max = float(data.get("leg_max", 0.50))
    leg_step = float(data.get("leg_step", 0.005))
    degree = int(data.get("degree", 3))

    k = k_at_length(preset, leg_length)
    lengths, gains, coeffs = fit_schedule(preset, leg_min, leg_max, leg_step, degree)
    c_source = _emit_lqr_c_source(coeffs, preset_name, preset)

    flat_col_major = k.reshape(-1, order="F")
    payload = {
        "type": "vmc_lqr_k",
        "preset": preset_name,
        "leg_length": leg_length,
        "state_order": STATE_NAMES,
        "input_order": INPUT_NAMES,
        "k_flat_col_major": [float(v) for v in flat_col_major],
    }

    result = {
        "preset": preset_name,
        "state_order": STATE_NAMES,
        "input_order": INPUT_NAMES,
        "q": [float(v) for v in np.diag(preset.q)],
        "r": [float(v) for v in np.diag(preset.r)],
        "leg_length": leg_length,
        "k_matrix": [[float(v) for v in row] for row in k],
        "k_flat_col_major": payload["k_flat_col_major"],
        "schedule": {
            "leg_min": leg_min,
            "leg_max": leg_max,
            "leg_step": leg_step,
            "degree": degree,
            "sample_count": int(len(lengths)),
            "coeffs": [[float(v) for v in row] for row in coeffs],
            "k_at_min": [[float(v) for v in row] for row in gains[0].reshape(2, 6)],
            "k_at_max": [[float(v) for v in row] for row in gains[-1].reshape(2, 6)],
        },
        "rtt_payload": json.dumps(payload, separators=(",", ":")),
        "c_source": c_source,
    }

    if bool(data.get("send_rtt", False)):
        result["rtt"] = send_rtt_line(
            result["rtt_payload"],
            host=_rtt_host(data),
            port=_rtt_port(data),
            timeout_s=float(data.get("rtt_timeout_s", 1.0)),
            dry_run=bool(data.get("dry_run", False)),
        )
    return result


def _firmware_balance_params() -> dict[str, float]:
    """Web-UI defaults, resolved from App/Inc/vmc_probe.h.

    The tuner used to carry its own hard-coded copy of the defaults.  That copy
    drifted a long way from the firmware: a fresh page plus "write firmware
    defaults" pushed 35 stale values back into the header, including
    wheel_torque_limit, support_force and gimbal_follow_enable.  Read the
    header instead, so the page always shows what the firmware actually runs
    and a blind write is a no-op.  The literal table stays only as a fallback
    for keys the header does not define, or if the header cannot be parsed.
    """
    params = dict(_BALANCE_PARAM_FALLBACK)
    try:
        params.update(_read_firmware_defaults()["balance_params"])
    except (OSError, ValueError):
        pass
    return params


def _balance_params_payload(data: dict[str, Any]) -> str:
    params = _firmware_balance_params()
    params.update(data.get("params", {}))
    payload = {"type": "vmc_balance_params"}
    for key in _BALANCE_PARAM_FALLBACK:
        payload[key] = float(params[key])
    return json.dumps(payload, separators=(",", ":"))


def _kalman_params_payload(data: dict[str, Any]) -> str:
    params = dict(DEFAULT_KALMAN_PARAMS)
    params.update(data.get("kalman_params", {}))
    payload = {"type": "chassis_kalman_params"}
    for key in DEFAULT_KALMAN_PARAMS:
        value = float(params[key])
        if not np.isfinite(value):
            raise ValueError(f"invalid Kalman parameter: {key}")
        payload[key] = value
    if payload["q_v"] < 0 or payload["q_a"] < 0 or payload["r_wheel"] <= 0 or payload["r_imu"] <= 0:
        raise ValueError("Kalman Q must be >= 0 and R must be > 0")
    return json.dumps(payload, separators=(",", ":"))


def _balance_params_payload_chunks(data: dict[str, Any], max_len: int = 700) -> list[str]:
    params = _firmware_balance_params()
    params.update(data.get("params", {}))
    chunks: list[str] = []
    payload: dict[str, Any] = {"type": "vmc_balance_params"}

    for key in _BALANCE_PARAM_FALLBACK:
        candidate = dict(payload)
        candidate[key] = float(params[key])
        candidate_text = json.dumps(candidate, separators=(",", ":"))
        if len(candidate_text) > max_len and len(payload) > 1:
            chunks.append(json.dumps(payload, separators=(",", ":")))
            payload = {"type": "vmc_balance_params", key: float(params[key])}
        else:
            payload = candidate

    if len(payload) > 1:
        chunks.append(json.dumps(payload, separators=(",", ":")))
    return chunks


def _send_balance_params_response(data: dict[str, Any]) -> dict[str, Any]:
    payload = _balance_params_payload(data)
    payloads = _balance_params_payload_chunks(data)
    rtt_results = [
        send_rtt_line(
            chunk,
            host=_rtt_host(data),
            port=_rtt_port(data),
            timeout_s=float(data.get("rtt_timeout_s", 1.0)),
            dry_run=bool(data.get("dry_run", False)),
        )
        for chunk in payloads
    ]
    return {
        "payload": payload,
        "payloads": payloads,
        "rtt_results": rtt_results,
    }


def _send_kalman_params_response(data: dict[str, Any]) -> dict[str, Any]:
    payload = _kalman_params_payload(data)
    return {
        "kalman_payload": payload,
        "rtt": send_rtt_line(
            payload,
            host=_rtt_host(data),
            port=_rtt_port(data),
            timeout_s=float(data.get("rtt_timeout_s", 1.0)),
            dry_run=bool(data.get("dry_run", False)),
        ),
    }


def _format_c_literal(value: float, value_type: str) -> str:
    if value_type == "uint":
        return "1U" if float(value) > 0.5 else "0U"
    if value_type == "uint_value":
        return f"{max(0, int(round(float(value))))}U"
    if value_type == "int":
        return str(int(round(float(value))))

    numeric = float(value)
    if abs(numeric) < 1e-12:
        numeric = 0.0
    text = f"{numeric:.9g}"
    if "e" not in text.lower() and "." not in text:
        text += ".0"
    return f"{text}f"


def _replace_define(source: str, macro: str, value: str) -> tuple[str, bool]:
    pattern = re.compile(rf"^(#define\s+{re.escape(macro)}\s+)(\S+)(.*)$", re.MULTILINE)

    def repl(match: re.Match[str]) -> str:
        return f"{match.group(1)}{value}{match.group(3)}"

    updated, count = pattern.subn(repl, source, count=1)
    return updated, count == 1


def _write_balance_defaults(params: dict[str, Any]) -> list[str]:
    header_path = ROOT / "App" / "Inc" / "vmc_probe.h"
    source = header_path.read_text()
    updated = source
    changed_macros: list[str] = []
    missing: list[str] = []

    for key, (macro, value_type) in BALANCE_PARAM_DEFAULT_MACROS.items():
        if key not in params:
            continue
        literal = _format_c_literal(float(params[key]), value_type)
        updated, changed = _replace_define(updated, macro, literal)
        if changed:
            changed_macros.append(macro)
        else:
            missing.append(macro)

    if missing:
        raise ValueError(f"missing default macro(s): {', '.join(missing)}")
    if updated != source:
        header_path.write_text(updated)
    return changed_macros


_C_NUMERIC_LITERAL_RE = re.compile(
    r"^[+-]?(?:(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)(?:[fFuUlL]+)?$"
)


def _parse_c_numeric_literal(token: str) -> float:
    """Parse a simple numeric C literal used by the default-value macros."""
    value = token.strip()
    # The defaults are simple literals, but tolerate one or more wrapping
    # parentheses (for example ``(1.0f)``) and integer/float suffixes.
    while len(value) >= 2 and value[0] == "(" and value[-1] == ")":
        value = value[1:-1].strip()
    if not _C_NUMERIC_LITERAL_RE.fullmatch(value):
        raise ValueError(f"unsupported C numeric literal: {token}")
    value = re.sub(r"[fFuUlL]+$", "", value)
    return float(value)


def _read_lqr_weights(lqr_path: Path) -> tuple[list[float] | None, list[float] | None, list[str]]:
    """Read Q/R metadata written by the tuner from ``vmc_lqr.c`` comments."""
    if not lqr_path.exists():
        try:
            display_path = str(lqr_path.relative_to(ROOT))
        except ValueError:
            display_path = str(lqr_path)
        return None, None, [f"LQR file not found: {display_path}"]
    source = lqr_path.read_text(encoding="utf-8")
    values: dict[str, list[float] | None] = {"q": None, "r": None}
    errors: list[str] = []
    for name, expected in (("q", 6), ("r", 2)):
        match = re.search(
            rf"VMC_LQR_TUNER_{name.upper()}\s*[:=]\s*\[([^\]]*)\]",
            source,
            re.IGNORECASE,
        )
        if not match:
            continue
        tokens = [part.strip() for part in match.group(1).split(",") if part.strip()]
        if len(tokens) != expected:
            errors.append(f"VMC_LQR_TUNER_{name.upper()} expects {expected} values, got {len(tokens)}")
            continue
        try:
            values[name] = [_parse_c_numeric_literal(token) for token in tokens]
        except ValueError as exc:
            errors.append(f"VMC_LQR_TUNER_{name.upper()}: {exc}")
    return values["q"], values["r"], errors


def _emit_lqr_c_source(coeffs: np.ndarray, preset_name: str, preset: Any) -> str:
    """Emit K schedule C code with the full plant metadata preserved.

    This used to write its own reduced comment block carrying only preset/Q/R.
    Saving from the web UI therefore silently deleted the recorded plant
    parameters - body_com_to_hip, gear_ratio, rotor_inertia, the drivetrain
    inertias and the viscous drag - and that broke
    tools/test_derive_vmc_lqr_abk.py, which regenerates the shipped schedule
    from exactly those numbers.  Delegate to the derivation script's own
    emitter so there is a single source of truth for the block.
    """
    return emit_metadata(preset_name, preset) + emit_c(coeffs)


def _read_firmware_defaults() -> dict[str, Any]:
    """Read balance parameter defaults directly from ``vmc_probe.h``.

    Only macros listed in the web-parameter mapping tables are imported.  A
    missing or non-literal macro is reported to the caller so the UI can still
    apply all values that were parsed successfully.
    """
    header_path = ROOT / "App" / "Inc" / "vmc_probe.h"
    source = header_path.read_text(encoding="utf-8")
    # Capture the first token after each #define.  All defaults mapped above
    # are single numeric literals; comments and trailing tokens are ignored.
    defines: dict[str, str] = {}
    for match in re.finditer(r"^\s*#define\s+(?P<macro>\w+)\s+(?P<value>[^\s/]+)", source, re.MULTILINE):
        defines[match.group("macro")] = match.group("value")

    params: dict[str, float] = {}
    macro_values: dict[str, float] = {}
    missing_macros: list[str] = []
    parse_errors: list[str] = []
    for key, (macro, _value_type) in {
        **FIRMWARE_SCALAR_DEFAULT_MACROS,
        **BALANCE_PARAM_DEFAULT_MACROS,
    }.items():
        token = defines.get(macro)
        if token is None:
            missing_macros.append(macro)
            continue
        try:
            numeric = _parse_c_numeric_literal(token)
        except ValueError as exc:
            parse_errors.append(f"{macro}: {exc}")
            continue
        params[key] = numeric
        macro_values[macro] = numeric

    lqr_path = ROOT / "App" / "Src" / "vmc_lqr.c"
    q_values, r_values, q_r_errors = _read_lqr_weights(lqr_path)
    parse_errors.extend(q_r_errors)

    kalman_params = dict(DEFAULT_KALMAN_PARAMS)
    for key, (macro, _value_type) in KALMAN_PARAM_DEFAULT_MACROS.items():
        token = defines.get(macro)
        if token is None:
            missing_macros.append(macro)
            continue
        try:
            numeric = _parse_c_numeric_literal(token)
        except ValueError as exc:
            parse_errors.append(f"{macro}: {exc}")
            continue
        kalman_params[key] = numeric
        macro_values[macro] = numeric

    return {
        "defaults_file": str(header_path.relative_to(ROOT)),
        "leg_length": params.pop("leg_length", None),
        "balance_params": params,
        "lqr_file": str(lqr_path.relative_to(ROOT)),
        "q": q_values,
        "r": r_values,
        "q_r_found": q_values is not None and r_values is not None,
        "macro_values": macro_values,
        "missing_macros": missing_macros,
        "parse_errors": parse_errors,
        "kalman_params": kalman_params,
    }


def _save_firmware_defaults_response(data: dict[str, Any]) -> dict[str, Any]:
    preset_name, preset = _preset_from_request(data)
    leg_min = float(data.get("leg_min", 0.03))
    leg_max = float(data.get("leg_max", 0.50))
    leg_step = float(data.get("leg_step", 0.005))
    degree = int(data.get("degree", 3))
    _lengths, _gains, coeffs = fit_schedule(preset, leg_min, leg_max, leg_step, degree)

    lqr_path = ROOT / "App" / "Src" / "vmc_lqr.c"
    c_source = _emit_lqr_c_source(coeffs, preset_name, preset)
    lqr_path.write_text(c_source)

    params = _firmware_balance_params()
    params.update(data.get("params", {}))
    changed_macros = _write_balance_defaults(params)

    return {
        "preset": preset_name,
        "q": [float(v) for v in np.diag(preset.q)],
        "r": [float(v) for v in np.diag(preset.r)],
        "lqr_file": str(lqr_path.relative_to(ROOT)),
        "defaults_file": "App/Inc/vmc_probe.h",
        "changed_macros": changed_macros,
    }


def _safe_text(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


class PersistentRttConnection:
    def __init__(self) -> None:
        self._sock: socket.socket | None = None
        self._target: tuple[str, int] | None = None
        self._banner = ""
        self._lock = threading.Lock()

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._target = None
        self._banner = ""

    def _drain_banner(self, sock: socket.socket) -> str:
        return self._read_available(sock, quiet_window_s=0.30, max_wait_s=1.20)

    def _read_available(
        self,
        sock: socket.socket,
        *,
        quiet_window_s: float,
        max_wait_s: float,
        stop_tokens: tuple[str, ...] = (),
    ) -> str:
        deadline = time.monotonic() + max_wait_s
        quiet_deadline = time.monotonic() + quiet_window_s
        banner = ""
        sock.settimeout(min(quiet_window_s, 0.10))
        while time.monotonic() < deadline and time.monotonic() < quiet_deadline:
            try:
                chunk = sock.recv(512)
            except socket.timeout:
                continue
            if not chunk:
                break
            banner += chunk.decode("utf-8", errors="replace")
            if stop_tokens and any(token in banner for token in stop_tokens):
                break
            quiet_deadline = time.monotonic() + quiet_window_s
        return banner

    def _connect(self, host: str, port: int, timeout_s: float) -> str:
        self.close()
        sock = socket.create_connection((host, port), timeout=timeout_s)
        banner = self._drain_banner(sock)
        self._sock = sock
        self._target = (host, port)
        self._banner = banner
        return banner

    def _ensure_connected(self, host: str, port: int, timeout_s: float) -> tuple[socket.socket, str, bool]:
        if self._sock is None or self._target != (host, port):
            banner = self._connect(host, port, timeout_s)
            return self._sock, banner, True  # type: ignore[return-value]
        return self._sock, "", False

    def read_available(self, host: str, port: int, timeout_s: float = 0.2) -> str:
        with self._lock:
            try:
                sock, _, _ = self._ensure_connected(host, port, timeout_s)
                return self._read_available(
                    sock,
                    quiet_window_s=0.02,
                    max_wait_s=0.05,
                )
            except OSError:
                self.close()
                return ""

    def send(self, frame: str, host: str, port: int, timeout_s: float) -> dict[str, Any]:
        with self._lock:
            reconnect_count = 0
            last_error = ""
            banner = ""

            for attempt in range(2):
                try:
                    sock, connect_banner, connected_now = self._ensure_connected(host, port, timeout_s)
                    if connected_now:
                        reconnect_count += 1
                        banner += connect_banner

                    sock.settimeout(timeout_s)
                    stale_output = self._read_available(
                        sock,
                        quiet_window_s=0.05,
                        max_wait_s=0.15,
                        stop_tokens=("VMC_LQR_RTT", "VMC_BALANCE_PARAMS", "CHASSIS_KALMAN_PARAMS"),
                    )
                    sock.sendall(frame.encode("utf-8"))

                    ack = self._read_available(
                        sock,
                        quiet_window_s=0.08,
                        max_wait_s=max(timeout_s, 1.0),
                        stop_tokens=("VMC_LQR_RTT", "VMC_BALANCE_PARAMS", "CHASSIS_KALMAN_PARAMS"),
                    )

                    return {
                        "ok": True,
                        "persistent": True,
                        "reconnect_count": reconnect_count,
                        "server_banner": banner.strip(),
                        "stale_output": stale_output.strip(),
                        "target_ack": ack.strip(),
                        "target_applied": (
                            "VMC_LQR_RTT OK" in ack
                            or "VMC_BALANCE_PARAMS OK" in ack
                            or "CHASSIS_KALMAN_PARAMS OK" in ack
                        ),
                        "host": host,
                        "port": port,
                        "frame": frame,
                    }
                except OSError as exc:
                    last_error = str(exc)
                    self.close()
                    if attempt == 0:
                        continue

            return {
                "ok": False,
                "persistent": True,
                "error": f"RTT TCP send failed to {host}:{port}: {last_error}",
                "host": host,
                "port": port,
                "frame": frame,
            }


_RTT_CONNECTION = PersistentRttConnection()


class VofaTcpBridge:
    def __init__(
        self,
        *,
        vofa_host: str,
        vofa_port: int,
        rtt_host: str,
        rtt_port: int,
    ) -> None:
        self.vofa_host = vofa_host
        self.vofa_port = vofa_port
        self.rtt_host = rtt_host
        self.rtt_port = rtt_port
        self._clients: list[socket.socket] = []
        self._clients_lock = threading.Lock()
        self._line_buffer = ""
        self._stop = threading.Event()

    def start(self) -> None:
        threading.Thread(target=self._serve_clients, daemon=True).start()
        threading.Thread(target=self._poll_rtt, daemon=True).start()
        print(f"VOFA TCP bridge listening on tcp://{self.vofa_host}:{self.vofa_port}")

    def _serve_clients(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.vofa_host, self.vofa_port))
            server.listen(4)
            server.settimeout(0.5)
            while not self._stop.is_set():
                try:
                    client, addr = server.accept()
                except socket.timeout:
                    continue
                except OSError as exc:
                    print(f"VOFA TCP accept stopped: {exc}")
                    return
                client.setblocking(False)
                with self._clients_lock:
                    self._clients.append(client)
                print(f"VOFA client connected from {addr[0]}:{addr[1]}")

    def _broadcast(self, line: str) -> None:
        data = line.encode("utf-8", errors="replace")
        with self._clients_lock:
            alive: list[socket.socket] = []
            for client in self._clients:
                try:
                    client.sendall(data)
                    alive.append(client)
                except OSError:
                    try:
                        client.close()
                    except OSError:
                        pass
            self._clients = alive

    def _handle_rtt_text(self, text: str) -> None:
        self._line_buffer += text
        while "\n" in self._line_buffer:
            line, self._line_buffer = self._line_buffer.split("\n", 1)
            line = line.strip()
            if not line.startswith("VMCVOFA,"):
                continue
            self._broadcast(line[len("VMCVOFA,"):] + "\n")

    def _poll_rtt(self) -> None:
        while not self._stop.is_set():
            text = _RTT_CONNECTION.read_available(self.rtt_host, self.rtt_port, timeout_s=0.2)
            if text:
                self._handle_rtt_text(text)
            else:
                time.sleep(0.01)


def send_rtt_line(
    line: str,
    *,
    host: str = DEFAULT_RTT_HOST,
    port: int = DEFAULT_RTT_PORT,
    timeout_s: float = 1.0,
    dry_run: bool = False,
) -> dict[str, Any]:
    frame = line.rstrip("\n") + "\n"
    if dry_run:
        return {"ok": True, "dry_run": True, "frame": frame}
    return _RTT_CONNECTION.send(frame, host, port, timeout_s)


class Handler(BaseHTTPRequestHandler):
    server_version = "LQRTuner/0.1"

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "content-type")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("content-length", "0"))
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_OPTIONS(self) -> None:  # noqa: N802
        self._send_json(200, {"ok": True})

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/api/firmware-defaults":
            self._send_json(200, {"ok": True, **_read_firmware_defaults()})
            return
        if self.path == "/api/defaults":
            preset = PRESETS["abk"]
            self._send_json(
                200,
                {
                    "presets": sorted(PRESETS),
                    "default_preset": "abk",
                    "state_order": STATE_NAMES,
                    "input_order": INPUT_NAMES,
                    "q": [float(v) for v in np.diag(preset.q)],
                    "r": [float(v) for v in np.diag(preset.r)],
                    "leg_length": 0.28,
                    "rtt_host": DEFAULT_RTT_HOST,
                    "rtt_port": DEFAULT_RTT_PORT,
                    "balance_params": _firmware_balance_params(),
                    "kalman_params": DEFAULT_KALMAN_PARAMS,
                },
            )
            return
        self._send_json(404, {"ok": False, "error": "not found"})

    def do_POST(self) -> None:  # noqa: N802
        try:
            data = self._read_json()
            if self.path == "/api/calc":
                self._send_json(200, {"ok": True, **_calc_response(data)})
                return
            if self.path == "/api/send-rtt":
                line = str(data.get("payload", ""))
                if not line:
                    raise ValueError("payload is required")
                self._send_json(
                    200,
                    {
                        "ok": True,
                        "rtt": send_rtt_line(
                            line,
                            host=_rtt_host(data),
                            port=_rtt_port(data),
                            timeout_s=float(data.get("rtt_timeout_s", 1.0)),
                            dry_run=bool(data.get("dry_run", False)),
                        ),
                    },
                )
                return
            if self.path == "/api/send-params":
                self._send_json(200, {"ok": True, **_send_balance_params_response(data)})
                return
            if self.path == "/api/send-kalman-params":
                self._send_json(200, {"ok": True, **_send_kalman_params_response(data)})
                return
            if self.path == "/api/save-firmware-defaults":
                self._send_json(200, {"ok": True, **_save_firmware_defaults_response(data)})
                return
            self._send_json(404, {"ok": False, "error": "not found"})
        except Exception as exc:  # noqa: BLE001
            self._send_json(400, {"ok": False, "error": str(exc)})

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"{self.address_string()} - {fmt % args}")


def _is_same_backend_process(pid: int) -> bool:
    if pid == os.getpid():
        return False
    try:
        result = subprocess.run(
            ["ps", "-p", str(pid), "-o", "command="],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    command = result.stdout.strip()
    return str(Path(__file__).resolve()) in command or "tools/lqr_tuner/server.py" in command


def _list_listening_pids(port: int) -> list[int]:
    try:
        result = subprocess.run(
            ["lsof", "-nP", f"-iTCP:{port}", "-sTCP:LISTEN", "-t"],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    pids = []
    for line in result.stdout.splitlines():
        try:
            pids.append(int(line.strip()))
        except ValueError:
            pass
    return pids


def _stop_stale_backend_on_port(port: int) -> bool:
    stopped = False
    for pid in _list_listening_pids(port):
        if not _is_same_backend_process(pid):
            continue
        print(f"Stopping stale LQR tuner backend pid={pid} on port {port}")
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            continue
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                os.kill(pid, 0)
            except OSError:
                stopped = True
                break
            time.sleep(0.05)
        else:
            try:
                os.kill(pid, signal.SIGKILL)
                stopped = True
            except OSError:
                pass
    return stopped


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--rtt-host", default=DEFAULT_RTT_HOST)
    parser.add_argument("--rtt-port", type=int, default=DEFAULT_RTT_PORT)
    parser.add_argument("--vofa-host", default=DEFAULT_VOFA_HOST)
    parser.add_argument("--vofa-port", type=int, default=DEFAULT_VOFA_PORT)
    parser.add_argument("--no-vofa", action="store_true")
    args = parser.parse_args()

    _stop_stale_backend_on_port(args.port)
    if not args.no_vofa:
        _stop_stale_backend_on_port(args.vofa_port)
        VofaTcpBridge(
            vofa_host=args.vofa_host,
            vofa_port=args.vofa_port,
            rtt_host=args.rtt_host,
            rtt_port=args.rtt_port,
        ).start()
    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"LQR tuner API listening on http://{args.host}:{args.port}")
    httpd.serve_forever()


if __name__ == "__main__":
    main()
