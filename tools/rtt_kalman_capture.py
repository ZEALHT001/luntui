#!/usr/bin/env python3
"""Capture and assess the chassis Kalman-filter telemetry over SEGGER RTT.

The default run captures 120 seconds of VMCVOFA data, evaluates overlapping
windows, and writes the raw samples plus a machine-readable and human-readable
report.  It supports both a running J-Link GDB server (RTT Telnet) and a direct
USB J-Link connection through pylink-square.
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import select
import shutil
import socket
import statistics
import subprocess
import sys
import termios
import threading
import time
import tty
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Iterable, Iterator, TextIO


DEFAULT_DURATION_S = 120.0
DEFAULT_WINDOW_S = 5.0
DEFAULT_HOP_S = 1.0
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 19021
DEFAULT_DEVICE = "STM32F427II"
DEFAULT_JLINK_SPEED = 4000
DEFAULT_PROFILE = "auto"
TELEMETRY_PERIOD_S = 0.020
INVALID_VALUE = float("nan")
DEFAULT_STOP_KEY = "q"
# How long to wait for the J-Link helper to report ready.  Generous on purpose:
# a probe left in a stale state by an unclean exit can take tens of seconds to
# re-enumerate, and timing out early just costs another attempt.
JLINK_READY_TIMEOUT_S = 60.0


@dataclass(frozen=True)
class Profile:
    name: str
    field_count: int
    channels: dict[str, int]


# VMCVOFA is comma-delimited.  The indices below are zero-based positions in
# the comma-separated vector.  Keeping all offsets in one place makes version
# detection obvious when the firmware telemetry grows.
#
# Torque layout.  These live in the preserved 0..98 block, so the indices are
# identical in every profile.  Tp is a virtual-leg torque: vmc_leg_force()
# sends exactly half of it to each joint, so one joint reaches its rating when
# |Tp| = 2 * 54 = 108.  Comparing tp_raw against tp_cmd shows whether the
# firmware clamps, and comparing tp_cmd against 108 shows how much of the
# joint rating the pitch loop actually asked for.
TORQUE_CHANNELS = {
    "left_wheel_torque_raw": 16,
    "right_wheel_torque_raw": 24,
    "left_tp_theta_term": 26,
    "left_tp_dtheta_term": 27,
    "left_tp_x_term": 28,
    "left_tp_dx_term": 29,
    "left_tp_phi_term": 30,
    "left_tp_dphi_term": 31,
    "left_tp_raw": 32,
    "left_tp_cmd": 33,
    "right_tp_theta_term": 34,
    "right_tp_dtheta_term": 35,
    "right_tp_x_term": 36,
    "right_tp_dx_term": 37,
    "right_tp_phi_term": 38,
    "right_tp_dphi_term": 39,
    "right_tp_raw": 40,
    "right_tp_cmd": 41,
}

FORMAT_LEGACY_PROFILE = Profile(
    name="format-v1",
    field_count=99,
    channels={
        **TORQUE_CHANNELS,
        "kf_x": 4,
        "kf_dx": 5,
        "left_wheel_current": 60,
        "right_wheel_current": 61,
        "q_v": 81,
        "q_a": 82,
        "r_wheel": 83,
        "r_imu": 84,
    },
)

# Previous format layout: 106 fields, no gravity diagnostic fields.  Kept so
# recordings made before gravity compensation was added still replay correctly.
FORMAT_V2_PROFILE = Profile(
    name="format-v2",
    field_count=106,
    channels={
        **TORQUE_CHANNELS,
        "kf_x": 4,
        "kf_dx": 5,
        "left_wheel_current": 60,
        "right_wheel_current": 61,
        "q_v": 81,
        "q_a": 82,
        "r_wheel": 83,
        "r_imu": 84,
        "mcu_time_ms": 99,
        "kf_wheel_measurement": 100,
        "kf_accel_measurement": 101,
        "kf_innovation_v": 102,
        "kf_innovation_a": 103,
        "kf_slip": 104,
        "kf_slip_metric": 105,
        "dist_obs_enable": 62,
        "dist_obs_output": 63,
        "dist_obs_gain": 64,
        "dist_obs_residual": 65,
        "dist_obs_active": 66,
        "dist_obs_neutral_time": 67,
        "dist_obs_pitch_comp": 68,
        "dist_obs_theta_comp": 69,
        "rc_source": 46,
        "rc_age_ms": 47,
        "state_age_ms": 48,
        "airborne_active": 85,
        "airborne_candidate": 86,
        "airborne_wheel_speed": 87,
        "airborne_joint_torque": 88,
        "jump_stage": 91,
        "jump_abort_reason": 93,
    },
)

# Layout with gravity_sign but before the raw/compensated acceleration
# diagnostics were added.
FORMAT_V3_PROFILE = Profile(
    name="format-v3",
    field_count=107,
    channels={
        **FORMAT_V2_PROFILE.channels,
        "gravity_sign": 106,
    },
)

# Layout with the acceleration diagnostics but before the accelerometer update
# counter was added.
FORMAT_V4_PROFILE = Profile(
    name="format-v4",
    field_count=109,
    channels={
        **FORMAT_V2_PROFILE.channels,
        "gravity_sign": 106,
        "body_ax_g": 107,
        "body_ax_lin_g": 108,
    },
)

# Layout with the gravity diagnostics but before the operator-command channels
# were added.
FORMAT_V5_PROFILE = Profile(
    name="format-v5",
    field_count=110,
    channels={
        **FORMAT_V2_PROFILE.channels,
        "gravity_sign": 106,
        "body_ax_g": 107,
        "body_ax_lin_g": 108,
        "kf_accel_update_count": 109,
    },
)

# Operator-command channels added, Q/R still a three-decimal fixed point.
FORMAT_V6_PROFILE = Profile(
    name="format-v6",
    field_count=115,
    channels={
        **FORMAT_V5_PROFILE.channels,
        "stick_forward": 110,
        "speed_target": 111,
        "speed_target_ramped": 112,
        "dx_target": 113,
        "dx_error": 114,
    },
)

# Schema version field added (value 3 at the time).
FORMAT_V7_PROFILE = Profile(
    name="format-v7",
    field_count=116,
    channels={
        **FORMAT_V5_PROFILE.channels,
        "stick_forward": 110,
        "speed_target": 111,
        "speed_target_ramped": 112,
        "dx_target": 113,
        "dx_error": 114,
        "schema_version": 115,
    },
)

# Slip-gate diagnostics added (schema 4).
FORMAT_V8_PROFILE = Profile(
    name="format-v8",
    field_count=122,
    channels={
        **FORMAT_V7_PROFILE.channels,
        "kf_slip_left": 116,
        "kf_slip_right": 117,
        "kf_wheel_left_raw": 118,
        "kf_wheel_right_raw": 119,
        "slip_accel_threshold": 120,
        "slip_enable": 121,
    },
)

# Layout with slip-gate diagnostics, attitude and roll rate (schema 5).
FORMAT_V9_PROFILE = Profile(
    name="format-v9",
    field_count=126,
    channels={
        **FORMAT_V8_PROFILE.channels,
        "roll": 122,
        "pitch": 123,
        "lqr_theta": 124,
        "roll_rate": 125,
    },
)

# Airborne-latch inputs added (schema 6): the weakest and strongest joint loads
# replace the mean, plus the attitude reference the pitch guard uses and the
# pitch rate the controller consumes.  The airborne decision is only
# reproducible offline if all four values are recorded.
# Layout with the airborne-latch inputs (schema 6).
FORMAT_V10_PROFILE = Profile(
    name="format-v10",
    field_count=130,
    channels={
        **FORMAT_V9_PROFILE.channels,
        "airborne_joint_torque_min": 126,
        "airborne_joint_torque_max": 127,
        "airborne_pitch_reference": 128,
        "pitch_rate": 129,
    },
)

# Per-side airborne latch added (schema 7).  A single lifted leg leaves the
# four-joint average high, so the whole-machine latch cannot see it; each side
# now has its own load and latch state, and gates only its own wheel.
# Layout with the per-side airborne latch (schema 7).
FORMAT_V11_PROFILE = Profile(
    name="format-v11",
    field_count=134,
    channels={
        **FORMAT_V10_PROFILE.channels,
        "airborne_side_torque_left": 130,
        "airborne_side_torque_right": 131,
        "airborne_side_active_left": 132,
        "airborne_side_active_right": 133,
    },
)

# Measured control-loop timing added (schema 8), without the TF02 fields.
# Kept as its own layout so recordings made before the laser telemetry was
# added still parse.
FORMAT_V12_PROFILE = Profile(
    name="format-v12",
    field_count=136,
    channels={
        **FORMAT_V11_PROFILE.channels,
        # Jump and leg-length diagnostics.  These were emitted but unnamed, so
        # a jump could not be diagnosed from a capture at all: jump_stage and
        # jump_abort_reason were visible but the leg length the state machine
        # actually acts on was not.
        "getup_state": 70,
        # Per-leg virtual angles.  Named because they are the only way to tell
        # a calibration fault from an odd pose on this layout: the average
        # theta alone cannot show which leg disagrees.
        "theta": 71,
        "left_theta": 72,
        "right_theta": 73,
        "leg_length_left": 75,
        "leg_length_right": 76,
        "leg_length_target": 77,
        "airborne_candidate_ms": 89,
        "airborne_release_ms": 90,
        "jump_armed": 92,
        "jump_target_length": 94,
        "jump_stage_ms": 95,
        "jump_land_contact_ms": 96,
        "jump_trigger_count": 97,
        "jump_abort_count": 98,
        "loop_dt_us": 134,
        "loop_count": 135,
    },
)

# TF02-Pro laser telemetry added (schema 9), without the roll-compensation
# fields.  Kept as its own layout so schema-9 recordings still parse.
FORMAT_V13_PROFILE = Profile(
    name="format-v13",
    field_count=138,
    channels={
        **FORMAT_V12_PROFILE.channels,
        "tf02_distance_cm": 136,
        "tf02_connected": 137,
    },
)

# Roll-compensation output added (schema 10), without the ground-following
# fields.  Kept as its own layout so schema-10 recordings still parse.
FORMAT_V14_PROFILE = Profile(
    name="format-v14",
    field_count=141,
    channels={
        **FORMAT_V13_PROFILE.channels,
        "roll_comp_offset": 138,
        # Per-leg length targets, after roll compensation and after the
        # VMC_LEG_LENGTH_MIN/MAX clamp.
        "leg_target_left": 139,
        "leg_target_right": 140,
    },
)

# Ground-following extension added (schema 11), without the scope fields.
FORMAT_V15_PROFILE = Profile(
    name="format-v15",
    field_count=143,
    channels={
        **FORMAT_V14_PROFILE.channels,
        "leg_follow_offset_left": 141,
        "leg_follow_offset_right": 142,
    },
)

# Roll-selected follow scope added (schema 12), without the "settled" flag.
FORMAT_V16_PROFILE = Profile(
    name="format-v16",
    field_count=145,
    channels={
        **FORMAT_V15_PROFILE.channels,
        "leg_follow_scope_machine_wide": 143,
        "leg_follow_episode_active": 144,
    },
)

# Scope-resolution flag added (schema 13), while field 143 still carried the
# per-leg/whole-machine flag.
FORMAT_V17_PROFILE = Profile(
    name="format-v17",
    field_count=146,
    channels={
        **FORMAT_V16_PROFILE.channels,
        "leg_follow_scope_resolved": 145,
    },
)

# Low-side selection (schema 14).  Field 143 CHANGED MEANING: it used to carry
# the per-leg/whole-machine flag and now carries the side the follow feature is
# extending - VMC_AIRBORNE_SIDE_LEFT (0), RIGHT (1) or
# VMC_LEG_FOLLOW_SIDE_NONE (2).  That is why this layout had to gain a field
# rather than reuse the count: the two are not distinguishable by length.
#
# Field 146 publishes the classification still being timed out, so a recording
# can show that a transient WAS seen and deliberately not believed.
FORMAT_V18_PROFILE = Profile(
    name="format-v18",
    field_count=147,
    channels={
        **FORMAT_V17_PROFILE.channels,
        "leg_follow_low_side": 143,
        "leg_follow_candidate_side": 146,
    },
)

# Full accelerometer vector (schema 15).  Only X was published before, which
# left "the accelerometer disagrees with the wheels" undiagnosable: the wrong
# axis, a gravity projection that is not being removed, and a genuinely
# unmapped axis are indistinguishable from one channel.  The raw triplet
# identifies the mounting (gravity shows up on whichever axis is vertical at
# rest) and the paired _lin values show whether the quaternion-based removal
# takes the same projection back out.
FORMAT_V19_PROFILE = Profile(
    name="format-v19",
    field_count=151,
    channels={
        **FORMAT_V18_PROFILE.channels,
        "body_ay_g": 147,
        "body_az_g": 148,
        "body_ay_lin_g": 149,
        "body_az_lin_g": 150,
    },
)

# Slip LQR weight switching (schema 16).  The applied posture scale is a
# latch-dependent per-side value that only exists inside the control cycle, so
# without publishing it a recording cannot tell "the response never fired" from
# "it fired and the effect was too small to see in the outputs".
FORMAT_V20_PROFILE = Profile(
    name="format-v20",
    field_count=153,
    channels={
        **FORMAT_V19_PROFILE.channels,
        "slip_lqr_applied_left": 151,
        "slip_lqr_applied_right": 152,
    },
)

# Slip wheel-current scaling (schema 17), the second half of the same response.
# Same argument as the posture scale above: publish the applied value so the
# off/on comparison is verifiable rather than inferred.
FORMAT_V21_PROFILE = Profile(
    name="format-v21",
    field_count=155,
    channels={
        **FORMAT_V20_PROFILE.channels,
        "slip_curr_applied_left": 153,
        "slip_curr_applied_right": 154,
    },
)

# Anti-splay (leg synchronisation) state (schema 18).  left_theta and
# right_theta are already published, but a difference computed from two
# separately-reported channels cannot say whether the anti-splay loop fired or
# was simply never needed.  These two carry the loop's own error
# (left_theta - right_theta) and the differential torque it applied.
FORMAT_V22_PROFILE = Profile(
    name="format-v22",
    field_count=157,
    channels={
        **FORMAT_V21_PROFILE.channels,
        "leg_sync_error": 155,
        "leg_sync_torque": 156,
    },
)

# Automatic stair climb (schema 19).  Stage and abort reason show the state
# machine; ready, pre-impact and impact show how far the trigger progressed;
# the per-leg impact flags show whether only one side saw the step.
FORMAT_V23_PROFILE = Profile(
    name="format-v23",
    field_count=170,
    channels={
        **FORMAT_V22_PROFILE.channels,
        "stair_stage": 157,
        "stair_abort_reason": 158,
        "stair_ready_detected": 159,
        "stair_preimpact_armed": 160,
        "stair_impact_detected": 161,
        "stair_trigger_count": 162,
        "stair_abort_count": 163,
        "stair_speed_abs": 164,
        "stair_speed_diff": 165,
        "stair_accel": 166,
        "stair_theta_ramped": 167,
        "stair_left_impact": 168,
        "stair_right_impact": 169,
    },
)

# Stair confirmation counters (schema 20).  The armed/impact flags alone cannot
# distinguish "never above threshold" from "threshold met for too short a
# time"; these counters expose the confirmation accumulation directly.
FORMAT_V24_PROFILE = Profile(
    name="format-v24",
    field_count=174,
    channels={
        **FORMAT_V23_PROFILE.channels,
        "stair_fast_detected": 170,
        "stair_speed_ms": 171,
        "stair_preimpact_ms": 172,
        "stair_impact_ms": 173,
    },
)

# Remote identity and switch state (schema 21).  rc_age alone cannot tell an
# operator S1-down stop from a source handover or a stale frame.
FORMAT_V25_PROFILE = Profile(
    name="format-v25",
    field_count=177,
    channels={
        **FORMAT_V24_PROFILE.channels,
        "rc_source": 174,
        "remote_s1": 175,
        "remote_s2": 176,
    },
)

# Legacy schema-22 descent slots.  The firmware now emits fixed zeros here so
# older capture files and every earlier telemetry index remain compatible.
FORMAT_PROFILE = Profile(
    name="format",
    field_count=187,
    channels={
        **FORMAT_V25_PROFILE.channels,
        "descent_stage": 177,
        "descent_abort_reason": 178,
        "descent_contact_detected": 179,
        "descent_entry_length": 180,
        "descent_extend_target": 181,
        "descent_length_command": 182,
        "descent_contact_torque": 183,
        "descent_contact_ms": 184,
        "descent_trigger_count": 185,
        "descent_abort_count": 186,
    },
)

# NOTE: the liuche development line has its own VMCVOFA layout.  A profile for
# it was previously guessed here from that branch's source and was wrong, as
# well as colliding with a format field count.  Do not reintroduce a layout
# without deriving every index from the branch that emits it.

PROFILES = {
    "format": FORMAT_PROFILE,
    "format-v1": FORMAT_LEGACY_PROFILE,
    "format-v2": FORMAT_V2_PROFILE,
    "format-v3": FORMAT_V3_PROFILE,
    "format-v4": FORMAT_V4_PROFILE,
    "format-v5": FORMAT_V5_PROFILE,
    "format-v6": FORMAT_V6_PROFILE,
    "format-v7": FORMAT_V7_PROFILE,
    "format-v8": FORMAT_V8_PROFILE,
    "format-v9": FORMAT_V9_PROFILE,
    "format-v10": FORMAT_V10_PROFILE,
    "format-v11": FORMAT_V11_PROFILE,
    "format-v12": FORMAT_V12_PROFILE,
    "format-v13": FORMAT_V13_PROFILE,
    "format-v14": FORMAT_V14_PROFILE,
    "format-v15": FORMAT_V15_PROFILE,
    "format-v16": FORMAT_V16_PROFILE,
    "format-v17": FORMAT_V17_PROFILE,
    "format-v18": FORMAT_V18_PROFILE,
    "format-v19": FORMAT_V19_PROFILE,
    "format-v20": FORMAT_V20_PROFILE,
    "format-v21": FORMAT_V21_PROFILE,
    "format-v22": FORMAT_V22_PROFILE,
    "format-v23": FORMAT_V23_PROFILE,
    "format-v24": FORMAT_V24_PROFILE,
    "format-v25": FORMAT_V25_PROFILE,
}
FORMAT_LAYOUTS = (
    FORMAT_LEGACY_PROFILE,
    FORMAT_V2_PROFILE,
    FORMAT_V3_PROFILE,
    FORMAT_V4_PROFILE,
    FORMAT_V5_PROFILE,
    FORMAT_V6_PROFILE,
    FORMAT_V7_PROFILE,
    FORMAT_V8_PROFILE,
    FORMAT_V9_PROFILE,
    FORMAT_V10_PROFILE,
    FORMAT_V11_PROFILE,
    FORMAT_V12_PROFILE,
    FORMAT_V13_PROFILE,
    FORMAT_V14_PROFILE,
    FORMAT_V15_PROFILE,
    FORMAT_V16_PROFILE,
    FORMAT_V17_PROFILE,
    FORMAT_V18_PROFILE,
    FORMAT_V19_PROFILE,
    FORMAT_V20_PROFILE,
    FORMAT_V21_PROFILE,
    FORMAT_V22_PROFILE,
    FORMAT_V23_PROFILE,
    FORMAT_V24_PROFILE,
    FORMAT_V25_PROFILE,
    FORMAT_PROFILE,
)
PROFILE_BY_FIELD_COUNT = {
    FORMAT_LEGACY_PROFILE.field_count: FORMAT_LEGACY_PROFILE,
    FORMAT_V2_PROFILE.field_count: FORMAT_V2_PROFILE,
    FORMAT_V3_PROFILE.field_count: FORMAT_V3_PROFILE,
    FORMAT_V4_PROFILE.field_count: FORMAT_V4_PROFILE,
    FORMAT_V5_PROFILE.field_count: FORMAT_V5_PROFILE,
    FORMAT_V6_PROFILE.field_count: FORMAT_V6_PROFILE,
    FORMAT_V7_PROFILE.field_count: FORMAT_V7_PROFILE,
    FORMAT_V8_PROFILE.field_count: FORMAT_V8_PROFILE,
    FORMAT_V9_PROFILE.field_count: FORMAT_V9_PROFILE,
    FORMAT_V10_PROFILE.field_count: FORMAT_V10_PROFILE,
    FORMAT_V11_PROFILE.field_count: FORMAT_V11_PROFILE,
    FORMAT_V12_PROFILE.field_count: FORMAT_V12_PROFILE,
    FORMAT_V13_PROFILE.field_count: FORMAT_V13_PROFILE,
    FORMAT_V14_PROFILE.field_count: FORMAT_V14_PROFILE,
    FORMAT_V15_PROFILE.field_count: FORMAT_V15_PROFILE,
    FORMAT_V16_PROFILE.field_count: FORMAT_V16_PROFILE,
    FORMAT_V17_PROFILE.field_count: FORMAT_V17_PROFILE,
    FORMAT_V18_PROFILE.field_count: FORMAT_V18_PROFILE,
    FORMAT_V19_PROFILE.field_count: FORMAT_V19_PROFILE,
    FORMAT_V20_PROFILE.field_count: FORMAT_V20_PROFILE,
    FORMAT_V21_PROFILE.field_count: FORMAT_V21_PROFILE,
    FORMAT_V22_PROFILE.field_count: FORMAT_V22_PROFILE,
    FORMAT_V23_PROFILE.field_count: FORMAT_V23_PROFILE,
    FORMAT_V24_PROFILE.field_count: FORMAT_V24_PROFILE,
    FORMAT_V25_PROFILE.field_count: FORMAT_V25_PROFILE,
    FORMAT_PROFILE.field_count: FORMAT_PROFILE,
}

CSV_FIELDS = [
    "host_time_s",
    "mcu_time_ms",
    "kf_x",
    "kf_dx",
    "q_v",
    "q_a",
    "r_wheel",
    "r_imu",
    "left_wheel_current",
    "right_wheel_current",
    "kf_acc",
    "d_x_raw",
    "dd_x",
    "kf_wheel_measurement",
    "kf_accel_measurement",
    "kf_innovation_v",
    "kf_innovation_a",
    "kf_slip",
    "kf_slip_metric",
    "rc_age_ms",
    "state_age_ms",
    "airborne_active",
    "airborne_candidate",
    "airborne_wheel_speed",
    "airborne_joint_torque",
    "airborne_joint_torque_min",
    "airborne_joint_torque_max",
    "airborne_pitch_reference",
    "airborne_side_torque_left",
    "airborne_side_torque_right",
    "airborne_side_active_left",
    "airborne_side_active_right",
    "loop_dt_us",
    "loop_count",
    "jump_stage",
    "jump_abort_reason",
    "jump_armed",
    "jump_target_length",
    "jump_stage_ms",
    "jump_land_contact_ms",
    "jump_trigger_count",
    "jump_abort_count",
    "leg_length_left",
    "leg_length_right",
    "leg_length_target",
    "getup_state",
    "theta",
    "left_theta",
    "right_theta",
    "gravity_sign",
    "body_ax_g",
    "body_ax_lin_g",
    # The rest of the accelerometer vector.  The raw triplet identifies the
    # mounting (gravity sits on whichever axis is vertical at rest) and the
    # paired _lin values show whether the gravity removal works.
    "body_ay_g",
    "body_az_g",
    "body_ay_lin_g",
    "body_az_lin_g",
    # Slip LQR posture scale actually applied per side (1.0 = not scaled).
    "slip_lqr_applied_left",
    "slip_lqr_applied_right",
    "slip_curr_applied_left",
    "slip_curr_applied_right",
    "kf_accel_update_count",
    "stick_forward",
    "speed_target",
    "speed_target_ramped",
    "dx_target",
    "dx_error",
    "schema_version",
    "dist_obs_output",
    "dist_obs_active",
    "dist_obs_neutral_time",
    "dist_obs_pitch_comp",
    "dist_obs_theta_comp",
    "kf_slip_left",
    "kf_slip_right",
    "kf_wheel_left_raw",
    "kf_wheel_right_raw",
    "slip_accel_threshold",
    "slip_enable",
    "roll",
    "pitch",
    "lqr_theta",
    "roll_rate",
    "pitch_rate",
    # TF02-Pro laser range and health.  The automatic jump gate needs the
    # distance inside (80, 100) cm, so both are recorded here.
    "tf02_distance_cm",
    "tf02_connected",
    # Roll compensation output.  Recorded so a levelling failure can be told
    # apart from the compensator never running or its differential being
    # clipped at VMC_LEG_LENGTH_MIN/MAX.
    "roll_comp_offset",
    "leg_target_left",
    "leg_target_right",
    # Ground-following extension, per side.  Included in the targets above;
    # recorded separately so the report can tell it from an S2 change.
    "leg_follow_offset_left",
    "leg_follow_offset_right",
    # Which side the follow feature is extending (0 left, 1 right, 2 none),
    # whether an unload episode is in progress, and whether the side has been
    # settled yet.  The side is classified from the roll direction over a
    # confirmation window, so these plus roll are what show which leg was
    # actually pushing.
    "leg_follow_low_side",
    "leg_follow_episode_active",
    # Whether the side was actually settled.  While undecided the follow only
    # pushes a side that is both unloaded and low, and that must not be
    # confused with a settled decision.
    "leg_follow_scope_resolved",
    # The classification still being timed out.  Lets a recording show that a
    # transient was SEEN and deliberately not believed.
    "leg_follow_candidate_side",
    # Torque observations.  Appended so every older CSV keeps its column order.
    "left_wheel_torque_raw",
    "right_wheel_torque_raw",
    "left_tp_theta_term",
    "left_tp_dtheta_term",
    "left_tp_x_term",
    "left_tp_dx_term",
    "left_tp_phi_term",
    "left_tp_dphi_term",
    "left_tp_raw",
    "left_tp_cmd",
    "right_tp_theta_term",
    "right_tp_dtheta_term",
    "right_tp_x_term",
    "right_tp_dx_term",
    "right_tp_phi_term",
    "right_tp_dphi_term",
    "right_tp_raw",
    "right_tp_cmd",
    # Anti-splay (leg synchronisation): the loop's error and the differential
    # torque it applied.  Appended last so every older CSV keeps its columns.
    "leg_sync_error",
    "leg_sync_torque",
    # Automatic stair-climb diagnostics.
    "stair_stage",
    "stair_abort_reason",
    "stair_ready_detected",
    "stair_preimpact_armed",
    "stair_impact_detected",
    "stair_trigger_count",
    "stair_abort_count",
    "stair_speed_abs",
    "stair_speed_diff",
    "stair_accel",
    "stair_theta_ramped",
    "stair_left_impact",
    "stair_right_impact",
    "stair_fast_detected",
    "stair_speed_ms",
    "stair_preimpact_ms",
    "stair_impact_ms",
    "rc_source",
    "remote_s1",
    "remote_s2",
    # Legacy reserved descent slots (always zero in current firmware).
    "descent_stage",
    "descent_abort_reason",
    "descent_contact_detected",
    "descent_entry_length",
    "descent_extend_target",
    "descent_length_command",
    "descent_contact_torque",
    "descent_contact_ms",
    "descent_trigger_count",
    "descent_abort_count",
]

OPTIONAL_CHANNELS = [
    "kf_acc",
    "d_x_raw",
    "dd_x",
    "kf_wheel_measurement",
    "kf_accel_measurement",
    "kf_innovation_v",
    "kf_innovation_a",
    "kf_slip",
    "kf_slip_metric",
]


@dataclass
class KalmanSample:
    host_time_s: float
    kf_x: float
    kf_dx: float
    mcu_time_ms: float = INVALID_VALUE
    q_v: float = INVALID_VALUE
    q_a: float = INVALID_VALUE
    r_wheel: float = INVALID_VALUE
    r_imu: float = INVALID_VALUE
    left_wheel_current: float = INVALID_VALUE
    right_wheel_current: float = INVALID_VALUE
    kf_acc: float = INVALID_VALUE
    d_x_raw: float = INVALID_VALUE
    dd_x: float = INVALID_VALUE
    kf_wheel_measurement: float = INVALID_VALUE
    kf_accel_measurement: float = INVALID_VALUE
    kf_innovation_v: float = INVALID_VALUE
    kf_innovation_a: float = INVALID_VALUE
    kf_slip: float = INVALID_VALUE
    kf_slip_metric: float = INVALID_VALUE
    rc_source: float = INVALID_VALUE
    rc_age_ms: float = INVALID_VALUE
    # Transmitter switch positions (1 = up, 3 = mid, 2 = down).  No layout on
    # this branch carries them yet, so these stay INVALID and consumers must
    # treat "missing" as "unknown" rather than as a position.  The pre-flash
    # safety check accepts a stop command when S1 reads DOWN, and falls back to
    # the gated output signals when the switch is absent.
    remote_s1: float = INVALID_VALUE
    remote_s2: float = INVALID_VALUE
    state_age_ms: float = INVALID_VALUE
    airborne_active: float = INVALID_VALUE
    airborne_candidate: float = INVALID_VALUE
    airborne_wheel_speed: float = INVALID_VALUE
    airborne_joint_torque: float = INVALID_VALUE
    airborne_joint_torque_min: float = INVALID_VALUE
    airborne_joint_torque_max: float = INVALID_VALUE
    airborne_pitch_reference: float = INVALID_VALUE
    airborne_side_torque_left: float = INVALID_VALUE
    airborne_side_torque_right: float = INVALID_VALUE
    airborne_side_active_left: float = INVALID_VALUE
    airborne_side_active_right: float = INVALID_VALUE
    loop_dt_us: float = INVALID_VALUE
    loop_count: float = INVALID_VALUE
    jump_stage: float = INVALID_VALUE
    jump_abort_reason: float = INVALID_VALUE
    jump_armed: float = INVALID_VALUE
    jump_target_length: float = INVALID_VALUE
    jump_stage_ms: float = INVALID_VALUE
    jump_land_contact_ms: float = INVALID_VALUE
    jump_trigger_count: float = INVALID_VALUE
    jump_abort_count: float = INVALID_VALUE
    leg_length_left: float = INVALID_VALUE
    leg_length_right: float = INVALID_VALUE
    leg_length_target: float = INVALID_VALUE
    getup_state: float = INVALID_VALUE
    theta: float = INVALID_VALUE
    left_theta: float = INVALID_VALUE
    right_theta: float = INVALID_VALUE
    gravity_sign: float = INVALID_VALUE
    body_ax_g: float = INVALID_VALUE
    body_ax_lin_g: float = INVALID_VALUE
    body_ay_g: float = INVALID_VALUE
    body_az_g: float = INVALID_VALUE
    body_ay_lin_g: float = INVALID_VALUE
    body_az_lin_g: float = INVALID_VALUE
    slip_lqr_applied_left: float = INVALID_VALUE
    slip_lqr_applied_right: float = INVALID_VALUE
    slip_curr_applied_left: float = INVALID_VALUE
    slip_curr_applied_right: float = INVALID_VALUE
    kf_accel_update_count: float = INVALID_VALUE
    stick_forward: float = INVALID_VALUE
    speed_target: float = INVALID_VALUE
    speed_target_ramped: float = INVALID_VALUE
    dx_target: float = INVALID_VALUE
    dx_error: float = INVALID_VALUE
    schema_version: float = INVALID_VALUE
    dist_obs_output: float = INVALID_VALUE
    dist_obs_active: float = INVALID_VALUE
    dist_obs_neutral_time: float = INVALID_VALUE
    dist_obs_pitch_comp: float = INVALID_VALUE
    dist_obs_theta_comp: float = INVALID_VALUE
    kf_slip_left: float = INVALID_VALUE
    kf_slip_right: float = INVALID_VALUE
    kf_wheel_left_raw: float = INVALID_VALUE
    kf_wheel_right_raw: float = INVALID_VALUE
    slip_accel_threshold: float = INVALID_VALUE
    slip_enable: float = INVALID_VALUE
    roll: float = INVALID_VALUE
    pitch: float = INVALID_VALUE
    lqr_theta: float = INVALID_VALUE
    roll_rate: float = INVALID_VALUE
    pitch_rate: float = INVALID_VALUE
    # TF02-Pro laser: distance in centimetres and the driver's health flag.
    # distance_cm == 0 with connected == 0 means "no reading", not "zero".
    tf02_distance_cm: float = INVALID_VALUE
    tf02_connected: float = INVALID_VALUE
    # Roll compensation: the commanded differential and the per-leg targets it
    # produces, after the length clamp.
    roll_comp_offset: float = INVALID_VALUE
    leg_target_left: float = INVALID_VALUE
    leg_target_right: float = INVALID_VALUE
    # Ground-following extension added to each side's target while that leg is
    # unloaded; already included in leg_target_left/right.
    leg_follow_offset_left: float = INVALID_VALUE
    leg_follow_offset_right: float = INVALID_VALUE
    # Scope the follow feature resolved for the current unload episode, and
    # whether one is running.  Absent before schema 12.
    leg_follow_low_side: float = INVALID_VALUE
    leg_follow_episode_active: float = INVALID_VALUE
    leg_follow_scope_resolved: float = INVALID_VALUE
    leg_follow_candidate_side: float = INVALID_VALUE
    # Legacy channel for the schema 12/13 layouts, where field 143 held the
    # per-leg/whole-machine flag instead of the low side.  Not written to CSV;
    # kept so those recordings stay inspectable.
    leg_follow_scope_machine_wide: float = INVALID_VALUE
    # Torque observations.  Tp is a virtual-leg torque; each joint receives
    # half of it, so a joint is at its rating when |Tp| reaches 108.
    left_wheel_torque_raw: float = INVALID_VALUE
    right_wheel_torque_raw: float = INVALID_VALUE
    left_tp_theta_term: float = INVALID_VALUE
    left_tp_dtheta_term: float = INVALID_VALUE
    left_tp_x_term: float = INVALID_VALUE
    left_tp_dx_term: float = INVALID_VALUE
    left_tp_phi_term: float = INVALID_VALUE
    left_tp_dphi_term: float = INVALID_VALUE
    left_tp_raw: float = INVALID_VALUE
    left_tp_cmd: float = INVALID_VALUE
    right_tp_theta_term: float = INVALID_VALUE
    right_tp_dtheta_term: float = INVALID_VALUE
    right_tp_x_term: float = INVALID_VALUE
    right_tp_dx_term: float = INVALID_VALUE
    right_tp_phi_term: float = INVALID_VALUE
    right_tp_dphi_term: float = INVALID_VALUE
    right_tp_raw: float = INVALID_VALUE
    right_tp_cmd: float = INVALID_VALUE
    # Anti-splay (leg synchronisation, schema 18).
    leg_sync_error: float = INVALID_VALUE
    leg_sync_torque: float = INVALID_VALUE
    stair_stage: float = INVALID_VALUE
    stair_abort_reason: float = INVALID_VALUE
    stair_ready_detected: float = INVALID_VALUE
    stair_preimpact_armed: float = INVALID_VALUE
    stair_impact_detected: float = INVALID_VALUE
    stair_trigger_count: float = INVALID_VALUE
    stair_abort_count: float = INVALID_VALUE
    stair_speed_abs: float = INVALID_VALUE
    stair_speed_diff: float = INVALID_VALUE
    stair_accel: float = INVALID_VALUE
    stair_theta_ramped: float = INVALID_VALUE
    stair_left_impact: float = INVALID_VALUE
    stair_right_impact: float = INVALID_VALUE
    stair_fast_detected: float = INVALID_VALUE
    stair_speed_ms: float = INVALID_VALUE
    stair_preimpact_ms: float = INVALID_VALUE
    stair_impact_ms: float = INVALID_VALUE
    # Legacy reserved descent slots (always zero in current firmware).
    descent_stage: float = INVALID_VALUE
    descent_abort_reason: float = INVALID_VALUE
    descent_contact_detected: float = INVALID_VALUE
    descent_entry_length: float = INVALID_VALUE
    descent_extend_target: float = INVALID_VALUE
    descent_length_command: float = INVALID_VALUE
    descent_contact_torque: float = INVALID_VALUE
    descent_contact_ms: float = INVALID_VALUE
    descent_trigger_count: float = INVALID_VALUE
    descent_abort_count: float = INVALID_VALUE


def sample_time_s(sample: KalmanSample) -> float:
    """Use the firmware timebase when available; host arrival time otherwise."""
    if _finite(sample.mcu_time_ms):
        return sample.mcu_time_ms * 0.001
    return sample.host_time_s


class TerminalStopKey:
    """Read one terminal key without requiring Enter."""

    def __init__(self, key: str) -> None:
        self.key = key
        self.enabled = key != "none" and sys.stdin.isatty()
        self.triggered_key: str | None = None
        self._fd: int | None = None
        self._original_settings: list[Any] | None = None

    def open(self) -> bool:
        if not self.enabled:
            return False
        try:
            self._fd = sys.stdin.fileno()
            self._original_settings = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        except (OSError, termios.error):
            self._fd = None
            self._original_settings = None
            self.enabled = False
        return self.enabled

    def close(self) -> None:
        if self._fd is not None and self._original_settings is not None:
            try:
                termios.tcsetattr(
                    self._fd, termios.TCSADRAIN, self._original_settings
                )
            except (OSError, termios.error):
                pass
        self._fd = None
        self._original_settings = None

    def stop_requested(self) -> bool:
        if not self.enabled or self._fd is None:
            return False
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return False
        try:
            raw = os.read(self._fd, 1)
        except OSError:
            return False
        if not raw:
            return False
        key = raw.decode("utf-8", errors="ignore")
        if self.key == "any" or key.lower() == self.key.lower():
            self.triggered_key = key
            return True
        return False


@dataclass
class WindowMetrics:
    start_s: float
    end_s: float
    samples: int
    sample_rate_hz: float
    max_gap_ms: float
    kf_dx_mean: float
    kf_dx_std: float
    kf_dx_max_abs: float
    kf_x_span: float
    kf_x_net: float
    integration_rmse: float
    integration_max_error: float
    speed_step_max: float
    acceleration_p95: float
    low_speed_drift: float | None
    q_v: float
    q_a: float
    r_wheel: float
    r_imu: float
    wheel_gain: float
    imu_gain: float
    wheel_tau_ms: float
    imu_tau_ms: float
    predicted_innov_v_std: float
    predicted_innov_a_std: float
    measured_innov_v_std: float
    measured_innov_a_std: float
    nis_v: float
    nis_a: float
    status: str
    findings: list[str]


def _finite(value: float) -> bool:
    return math.isfinite(value)


def _safe_float(text: str) -> float:
    try:
        value = float(text)
    except (TypeError, ValueError):
        return INVALID_VALUE
    return value if math.isfinite(value) else INVALID_VALUE


def _mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else INVALID_VALUE


def _std(values: list[float]) -> float:
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return INVALID_VALUE
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    ratio = position - lower
    return ordered[lower] * (1.0 - ratio) + ordered[upper] * ratio


def _linear_slope(times: list[float], values: list[float]) -> float:
    if len(times) < 2:
        return 0.0
    mean_t = statistics.fmean(times)
    mean_y = statistics.fmean(values)
    denominator = sum((t - mean_t) ** 2 for t in times)
    if denominator <= 1.0e-15:
        return 0.0
    return sum((t - mean_t) * (y - mean_y)
               for t, y in zip(times, values)) / denominator


def command_tracking_summary(samples: list[KalmanSample]) -> list[str]:
    """Summarize how the achieved speed follows the operator stick and target."""
    pairs = [
        (sample.stick_forward, sample.speed_target, sample.kf_dx)
        for sample in samples
        if _finite(sample.stick_forward)
        and _finite(sample.speed_target)
        and _finite(sample.kf_dx)
    ]
    if len(pairs) < 50:
        return []

    stick = [p[0] for p in pairs]
    target = [p[1] for p in pairs]
    actual = [p[2] for p in pairs]

    def corr(x: list[float], y: list[float]) -> float:
        mx = statistics.fmean(x)
        my = statistics.fmean(y)
        denominator = math.sqrt(
            sum((a - mx) ** 2 for a in x) * sum((b - my) ** 2 for b in y)
        )
        if denominator <= 1.0e-15:
            return INVALID_VALUE
        return sum((a - mx) * (b - my) for a, b in zip(x, y)) / denominator

    unmoved = sum(1 for value in stick if abs(value) <= 20) / len(stick)
    moving = [
        (target_s, actual_s)
        for _stick_s, target_s, actual_s in pairs
        if abs(target_s) > 0.05
    ]

    lines = [
        "",
        "## Operator Command Tracking",
        "",
        f"- Stick centred fraction (|ch3| <= 20 us): `{unmoved * 100.0:.1f}%`",
        f"- corr(stick, speed_target): `{format_value(corr(stick, target))}`",
        f"- corr(speed_target, kf_dx): `{format_value(corr(target, actual))}`",
    ]
    if moving:
        targets = [t for t, _a in moving]
        actuals = [a for _t, a in moving]
        gain = _linear_slope(targets, actuals)
        lines.append(
            f"- Speed loop gain (kf_dx per speed_target) while commanded: "
            f"`{format_value(gain, 4)}`"
        )
        lines.append(
            f"- Commanded samples above 0.05 m/s: `{len(moving)}`"
        )
    lines.append("")
    return lines


def select_profile(field_count: int, requested: str) -> Profile:
    selected = PROFILE_BY_FIELD_COUNT.get(field_count)
    if requested == "auto":
        if selected is None:
            raise ValueError(
                f"unknown VMCVOFA field count {field_count}; "
                "pass the matching --profile (format, format-v1..v7)"
            )
        return selected
    if requested == "format":
        # Accept every format lineage so older recordings still replay.
        if selected in FORMAT_LAYOUTS:
            return selected
    requested_profile = PROFILES[requested]
    if selected is not None and selected != requested_profile:
        raise ValueError(
            f"VMCVOFA field count {field_count} does not match "
            f"profile {requested_profile.name}"
        )
    return requested_profile


def parse_vmcvoa_line(
    line: str, requested_profile: str = DEFAULT_PROFILE
) -> tuple[Profile, KalmanSample] | None:
    """Parse one VMCVOFA line into the filter channels used by the report."""
    text = line.strip()
    if not text.startswith("VMCVOFA"):
        return None

    payload = text[len("VMCVOFA"):]
    if payload.startswith(","):
        payload = payload[1:]
    fields = [field.strip() for field in payload.split(",")]
    if not fields or all(field == "" for field in fields):
        return None

    profile = select_profile(len(fields), requested_profile)
    parsed = [_safe_float(field) for field in fields]

    def channel(name: str) -> float:
        index = profile.channels.get(name, -1)
        if index < 0 or index >= len(parsed):
            return INVALID_VALUE
        return parsed[index]

    sample = KalmanSample(
        host_time_s=0.0,
        kf_x=channel("kf_x"),
        kf_dx=channel("kf_dx"),
        mcu_time_ms=channel("mcu_time_ms"),
        q_v=channel("q_v"),
        q_a=channel("q_a"),
        r_wheel=channel("r_wheel"),
        r_imu=channel("r_imu"),
        left_wheel_current=channel("left_wheel_current"),
        right_wheel_current=channel("right_wheel_current"),
        kf_acc=channel("kf_acc"),
        d_x_raw=channel("d_x_raw"),
        dd_x=channel("dd_x"),
        kf_wheel_measurement=channel("kf_wheel_measurement"),
        kf_accel_measurement=channel("kf_accel_measurement"),
        kf_innovation_v=channel("kf_innovation_v"),
        kf_innovation_a=channel("kf_innovation_a"),
        kf_slip=channel("kf_slip"),
        kf_slip_metric=channel("kf_slip_metric"),
        rc_source=channel("rc_source"),
        rc_age_ms=channel("rc_age_ms"),
        remote_s1=channel("remote_s1"),
        remote_s2=channel("remote_s2"),
        state_age_ms=channel("state_age_ms"),
        airborne_active=channel("airborne_active"),
        airborne_candidate=channel("airborne_candidate"),
        airborne_wheel_speed=channel("airborne_wheel_speed"),
        airborne_joint_torque=channel("airborne_joint_torque"),
        airborne_joint_torque_min=channel("airborne_joint_torque_min"),
        airborne_joint_torque_max=channel("airborne_joint_torque_max"),
        airborne_pitch_reference=channel("airborne_pitch_reference"),
        airborne_side_torque_left=channel("airborne_side_torque_left"),
        airborne_side_torque_right=channel("airborne_side_torque_right"),
        airborne_side_active_left=channel("airborne_side_active_left"),
        airborne_side_active_right=channel("airborne_side_active_right"),
        loop_dt_us=channel("loop_dt_us"),
        loop_count=channel("loop_count"),
        jump_stage=channel("jump_stage"),
        jump_abort_reason=channel("jump_abort_reason"),
        jump_armed=channel("jump_armed"),
        jump_target_length=channel("jump_target_length"),
        jump_stage_ms=channel("jump_stage_ms"),
        jump_land_contact_ms=channel("jump_land_contact_ms"),
        jump_trigger_count=channel("jump_trigger_count"),
        jump_abort_count=channel("jump_abort_count"),
        leg_length_left=channel("leg_length_left"),
        leg_length_right=channel("leg_length_right"),
        leg_length_target=channel("leg_length_target"),
        getup_state=channel("getup_state"),
        theta=channel("theta"),
        left_theta=channel("left_theta"),
        right_theta=channel("right_theta"),
        gravity_sign=channel("gravity_sign"),
        body_ax_g=channel("body_ax_g"),
        body_ax_lin_g=channel("body_ax_lin_g"),
        body_ay_g=channel("body_ay_g"),
        body_az_g=channel("body_az_g"),
        body_ay_lin_g=channel("body_ay_lin_g"),
        body_az_lin_g=channel("body_az_lin_g"),
        slip_lqr_applied_left=channel("slip_lqr_applied_left"),
        slip_lqr_applied_right=channel("slip_lqr_applied_right"),
        slip_curr_applied_left=channel("slip_curr_applied_left"),
        slip_curr_applied_right=channel("slip_curr_applied_right"),
        kf_accel_update_count=channel("kf_accel_update_count"),
        stick_forward=channel("stick_forward"),
        speed_target=channel("speed_target"),
        speed_target_ramped=channel("speed_target_ramped"),
        dx_target=channel("dx_target"),
        dx_error=channel("dx_error"),
        schema_version=channel("schema_version"),
        dist_obs_output=channel("dist_obs_output"),
        dist_obs_active=channel("dist_obs_active"),
        dist_obs_neutral_time=channel("dist_obs_neutral_time"),
        dist_obs_pitch_comp=channel("dist_obs_pitch_comp"),
        dist_obs_theta_comp=channel("dist_obs_theta_comp"),
        kf_slip_left=channel("kf_slip_left"),
        kf_slip_right=channel("kf_slip_right"),
        kf_wheel_left_raw=channel("kf_wheel_left_raw"),
        kf_wheel_right_raw=channel("kf_wheel_right_raw"),
        slip_accel_threshold=channel("slip_accel_threshold"),
        slip_enable=channel("slip_enable"),
        roll=channel("roll"),
        pitch=channel("pitch"),
        lqr_theta=channel("lqr_theta"),
        roll_rate=channel("roll_rate"),
        pitch_rate=channel("pitch_rate"),
        tf02_distance_cm=channel("tf02_distance_cm"),
        tf02_connected=channel("tf02_connected"),
        roll_comp_offset=channel("roll_comp_offset"),
        leg_target_left=channel("leg_target_left"),
        leg_target_right=channel("leg_target_right"),
        leg_follow_offset_left=channel("leg_follow_offset_left"),
        leg_follow_offset_right=channel("leg_follow_offset_right"),
        leg_follow_low_side=channel("leg_follow_low_side"),
        leg_follow_episode_active=channel("leg_follow_episode_active"),
        leg_follow_scope_resolved=channel("leg_follow_scope_resolved"),
        leg_follow_candidate_side=channel("leg_follow_candidate_side"),
        leg_follow_scope_machine_wide=channel("leg_follow_scope_machine_wide"),
        left_wheel_torque_raw=channel("left_wheel_torque_raw"),
        right_wheel_torque_raw=channel("right_wheel_torque_raw"),
        left_tp_theta_term=channel("left_tp_theta_term"),
        left_tp_dtheta_term=channel("left_tp_dtheta_term"),
        left_tp_x_term=channel("left_tp_x_term"),
        left_tp_dx_term=channel("left_tp_dx_term"),
        left_tp_phi_term=channel("left_tp_phi_term"),
        left_tp_dphi_term=channel("left_tp_dphi_term"),
        left_tp_raw=channel("left_tp_raw"),
        left_tp_cmd=channel("left_tp_cmd"),
        right_tp_theta_term=channel("right_tp_theta_term"),
        right_tp_dtheta_term=channel("right_tp_dtheta_term"),
        right_tp_x_term=channel("right_tp_x_term"),
        right_tp_dx_term=channel("right_tp_dx_term"),
        right_tp_phi_term=channel("right_tp_phi_term"),
        right_tp_dphi_term=channel("right_tp_dphi_term"),
        right_tp_raw=channel("right_tp_raw"),
        right_tp_cmd=channel("right_tp_cmd"),
        leg_sync_error=channel("leg_sync_error"),
        leg_sync_torque=channel("leg_sync_torque"),
        stair_stage=channel("stair_stage"),
        stair_abort_reason=channel("stair_abort_reason"),
        stair_ready_detected=channel("stair_ready_detected"),
        stair_preimpact_armed=channel("stair_preimpact_armed"),
        stair_impact_detected=channel("stair_impact_detected"),
        stair_trigger_count=channel("stair_trigger_count"),
        stair_abort_count=channel("stair_abort_count"),
        stair_speed_abs=channel("stair_speed_abs"),
        stair_speed_diff=channel("stair_speed_diff"),
        stair_accel=channel("stair_accel"),
        stair_theta_ramped=channel("stair_theta_ramped"),
        stair_left_impact=channel("stair_left_impact"),
        stair_right_impact=channel("stair_right_impact"),
        stair_fast_detected=channel("stair_fast_detected"),
        stair_speed_ms=channel("stair_speed_ms"),
        stair_preimpact_ms=channel("stair_preimpact_ms"),
        stair_impact_ms=channel("stair_impact_ms"),
        descent_stage=channel("descent_stage"),
        descent_abort_reason=channel("descent_abort_reason"),
        descent_contact_detected=channel("descent_contact_detected"),
        descent_entry_length=channel("descent_entry_length"),
        descent_extend_target=channel("descent_extend_target"),
        descent_length_command=channel("descent_length_command"),
        descent_contact_torque=channel("descent_contact_torque"),
        descent_contact_ms=channel("descent_contact_ms"),
        descent_trigger_count=channel("descent_trigger_count"),
        descent_abort_count=channel("descent_abort_count"),
    )
    return profile, sample


def steady_state_gains(
    q_v: float,
    q_a: float,
    r_wheel: float,
    r_imu: float,
    dt: float = 0.001,
    imu_rate_hz: float = 200.0,
) -> tuple[float, float, float, float, float, float]:
    """Return gains, time constants, and the predicted innovation std devs.

    The filter runs once per control cycle and gates the accelerometer update to
    one new HI12 frame.  Modelling the gate correctly therefore needs BOTH the
    real control period and the real IMU rate: at 1 kHz and 200 Hz the IMU
    corrects every 5 cycles, at the measured 450 Hz it corrects every ~2.2.
    Assume the nominal 1 ms against a loop that actually runs at 2.22 ms and the
    predicted innovation variance - and so NIS - comes out wrong.
    """
    if not all(
        _finite(value)
        for value in (q_v, q_a, r_wheel, r_imu, dt, imu_rate_hz)
    ):
        return (INVALID_VALUE,) * 6

    q_v = min(max(q_v, 0.0), 1.0e12)
    q_a = min(max(q_a, 0.0), 1.0e12)
    r_wheel = min(max(r_wheel, 1.0e-9), 1.0e12)
    r_imu = min(max(r_imu, 1.0e-9), 1.0e12)
    dt = min(max(dt, 0.0), 1.0)
    imu_rate_hz = min(max(imu_rate_hz, 1.0), 1.0 / max(dt, 1.0e-9))
    if dt <= 0.0:
        return (INVALID_VALUE,) * 6

    p00 = p11 = 1.0e5
    p01 = p10 = 0.0
    wheel_gain = 0.0
    imu_gain = 0.0
    imu_gate = max(int(round(1.0 / (imu_rate_hz * dt))), 1)
    s_wheel = s_imu = INVALID_VALUE
    for step in range(200000):
        predicted_00 = p00 + dt * (p10 + p01) + dt * dt * p11 + q_v
        predicted_01 = p01 + dt * p11
        predicted_10 = p10 + dt * p11
        predicted_11 = p11 + q_a

        s_wheel = predicted_00 + r_wheel
        wheel_gain = predicted_00 / s_wheel
        k1 = predicted_10 / s_wheel
        p00 = predicted_00 - wheel_gain * predicted_00
        p01 = predicted_01 - wheel_gain * predicted_01
        p10 = predicted_10 - k1 * predicted_00
        p11 = predicted_11 - k1 * predicted_01

        if step % imu_gate == 0:
            s_imu = p11 + r_imu
            imu_gain = p11 / s_imu
            k0 = p01 / s_imu
            p01_old = p01
            p11_old = p11
            old_p10 = p10
            p00 = p00 - k0 * p10
            p01 = p01_old - k0 * p11_old
            p10 = old_p10 - imu_gain * p10
            p11 = p11_old - imu_gain * p11_old

        if not all(_finite(v) for v in (p00, p01, p10, p11)):
            return (INVALID_VALUE,) * 6

    def tau_ms(gain: float, step_s: float) -> float:
        if gain <= 0.0:
            return math.inf
        if gain >= 1.0:
            return 0.0
        return -step_s * 1000.0 / math.log1p(-gain)

    return (
        wheel_gain,
        imu_gain,
        tau_ms(wheel_gain, dt),
        tau_ms(imu_gain, dt * imu_gate),
        math.sqrt(s_wheel) if _finite(s_wheel) else INVALID_VALUE,
        math.sqrt(s_imu) if _finite(s_imu) else INVALID_VALUE,
    )


def _same_or_missing(a: float, b: float, tolerance: float = 1.0e-12) -> bool:
    if not _finite(a) and not _finite(b):
        return True
    if not _finite(a) or not _finite(b):
        return False
    return abs(a - b) <= tolerance * max(1.0, abs(a), abs(b))


def analyze_window(samples: list[KalmanSample]) -> WindowMetrics:
    if not samples:
        raise ValueError("cannot analyze an empty window")

    samples = sorted(samples, key=sample_time_s)
    times = [sample_time_s(sample) for sample in samples]
    dx_values = [sample.kf_dx for sample in samples if _finite(sample.kf_dx)]
    x_values = [sample.kf_x for sample in samples if _finite(sample.kf_x)]
    duration = max(times[-1] - times[0], 1.0e-9)
    gaps = [times[i] - times[i - 1] for i in range(1, len(times))]
    max_gap_ms = max(gaps, default=0.0) * 1000.0
    nominal_gaps = [gap for gap in gaps if 0.0 < gap <= 0.20]
    sample_rate = (
        1.0 / statistics.median(nominal_gaps) if nominal_gaps else 0.0
    )

    integration_errors: list[float] = []
    speed_steps: list[float] = []
    acceleration_values: list[float] = []
    for i in range(1, len(samples)):
        current = samples[i]
        previous = samples[i - 1]
        dt = sample_time_s(current) - sample_time_s(previous)
        if dt <= 0.0:
            continue

        if _finite(current.kf_x) and _finite(previous.kf_x):
            integrated = previous.kf_x + 0.5 * (
                current.kf_dx + previous.kf_dx
            ) * dt
            integration_errors.append(current.kf_x - integrated)

        if _finite(current.kf_dx) and _finite(previous.kf_dx):
            speed_step = abs(current.kf_dx - previous.kf_dx)
            speed_steps.append(speed_step)
            acceleration_values.append(speed_step / dt)

    kf_dx_mean = _mean(dx_values)
    kf_dx_std = _std(dx_values)
    kf_dx_max_abs = max((abs(value) for value in dx_values), default=0.0)
    kf_x_span = max(x_values) - min(x_values) if len(x_values) > 1 else 0.0
    kf_x_net = x_values[-1] - x_values[0] if len(x_values) > 1 else 0.0
    integration_rmse = (
        math.sqrt(statistics.fmean(value * value for value in integration_errors))
        if integration_errors
        else 0.0
    )
    integration_max_error = max(
        (abs(value) for value in integration_errors), default=0.0
    )
    speed_step_max = max(speed_steps, default=0.0)
    acceleration_p95 = _percentile(
        [abs(value) for value in acceleration_values], 0.95
    )

    low_speed_drift: float | None = None
    if abs(kf_dx_mean) <= 0.05 and kf_dx_max_abs <= 0.10:
        low_speed_drift = abs(kf_x_net) / duration

    q_v = _mean([sample.q_v for sample in samples if _finite(sample.q_v)])
    q_a = _mean([sample.q_a for sample in samples if _finite(sample.q_a)])
    r_wheel = _mean(
        [sample.r_wheel for sample in samples if _finite(sample.r_wheel)]
    )
    r_imu = _mean([sample.r_imu for sample in samples if _finite(sample.r_imu)])
    model_dt = model_control_period_s(samples)
    model_imu_rate = measured_imu_rate_hz(samples)
    if not _finite(model_imu_rate) or model_imu_rate <= 0.0:
        model_imu_rate = 200.0
    (
        wheel_gain,
        imu_gain,
        wheel_tau_ms,
        imu_tau_ms,
        predicted_innov_v_std,
        predicted_innov_a_std,
    ) = steady_state_gains(
        q_v, q_a, r_wheel, r_imu, dt=model_dt, imu_rate_hz=model_imu_rate
    )

    # Normalised innovation squared: the filter is consistent when its own
    # predicted innovation variance matches the observed one, i.e. NIS ~ 1.
    innov_v_values = [
        sample.kf_innovation_v
        for sample in samples
        if _finite(sample.kf_innovation_v)
    ]
    innov_a_values = [
        sample.kf_innovation_a
        for sample in samples
        if _finite(sample.kf_innovation_a)
    ]
    measured_innov_v_std = _std(innov_v_values)
    measured_innov_a_std = _std(innov_a_values)
    nis_v = (
        (measured_innov_v_std / predicted_innov_v_std) ** 2
        if _finite(measured_innov_v_std)
        and _finite(predicted_innov_v_std)
        and predicted_innov_v_std > 0.0
        else INVALID_VALUE
    )
    nis_a = (
        (measured_innov_a_std / predicted_innov_a_std) ** 2
        if _finite(measured_innov_a_std)
        and _finite(predicted_innov_a_std)
        and predicted_innov_a_std > 0.0
        else INVALID_VALUE
    )

    findings: list[str] = []
    status = "OK"

    def add_finding(level: str, message: str) -> None:
        nonlocal status
        findings.append(f"{level}: {message}")
        if level == "FAIL":
            status = "FAIL"
        elif level == "WARN" and status != "FAIL":
            status = "WARN"

    if len(samples) < 10 or sample_rate < 20.0:
        add_finding(
            "FAIL",
            f"采样不足：{len(samples)} 点，估算 {sample_rate:.1f} Hz",
        )
    elif sample_rate < 35.0:
        add_finding("WARN", f"采样率偏低：{sample_rate:.1f} Hz（目标约 50 Hz）")

    if max_gap_ms > 500.0:
        add_finding("FAIL", f"RTT 最长断流 {max_gap_ms:.0f} ms")
    elif max_gap_ms > 100.0:
        add_finding("WARN", f"RTT 最长断流 {max_gap_ms:.0f} ms")

    if not dx_values or not x_values:
        add_finding("FAIL", "kf_dx 或 kf_x 无效")
    if any(not _finite(sample.kf_x) or not _finite(sample.kf_dx) for sample in samples):
        add_finding("FAIL", "滤波遥测包含 NaN/Inf")
    if kf_dx_max_abs > 10.0:
        add_finding("FAIL", f"kf_dx 峰值过大：{kf_dx_max_abs:.3f} m/s")
    elif kf_dx_max_abs > 5.0:
        add_finding("WARN", f"kf_dx 峰值偏高：{kf_dx_max_abs:.3f} m/s")

    if speed_step_max > 3.0:
        add_finding("FAIL", f"kf_dx 单周期突变：{speed_step_max:.3f} m/s")
    elif speed_step_max > 1.0:
        add_finding("WARN", f"kf_dx 单周期突变：{speed_step_max:.3f} m/s")

    if _finite(acceleration_p95) and acceleration_p95 > 100.0:
        add_finding("FAIL", f"估计加速度 P95 过大：{acceleration_p95:.1f} m/s²")
    elif _finite(acceleration_p95) and acceleration_p95 > 40.0:
        add_finding("WARN", f"估计加速度 P95 偏高：{acceleration_p95:.1f} m/s²")

    if integration_rmse > 0.08 or integration_max_error > 0.25:
        add_finding(
            "FAIL",
            "kf_x 与 kf_dx 积分不一致："
            f"RMSE={integration_rmse:.4f} m, max={integration_max_error:.4f} m",
        )
    elif integration_rmse > 0.025 or integration_max_error > 0.08:
        add_finding(
            "WARN",
            "kf_x/kf_dx 存在积分误差："
            f"RMSE={integration_rmse:.4f} m, max={integration_max_error:.4f} m",
        )

    if low_speed_drift is not None and low_speed_drift > 0.008:
        add_finding(
            "WARN",
            f"低速窗口位置漂移：{low_speed_drift * 1000.0:.2f} mm/s",
        )

    qr_available = any(
        _finite(value) for value in (q_v, q_a, r_wheel, r_imu)
    )
    if not qr_available:
        add_finding(
            "WARN",
            "Q/R 遥测字段为空或格式无效，无法校验融合权重",
        )
    elif _finite(imu_gain) and imu_gain < 1.0e-4:
        add_finding(
            "WARN",
            "IMU 加速度通道几乎未参与融合："
            f"K_imu={imu_gain:.3g}，r_imu/r_wheel={r_imu / r_wheel:.3g}",
        )

    # Consistency: the filter's own predicted innovation variance should match
    # what the measurements actually show.  This is the primary Kalman-tuning
    # criterion; the wheel channel is authoritative because r_wheel << r_imu.
    if _finite(nis_v) and nis_v > 0.0:
        if nis_v > 100.0 or nis_v < 0.01:
            add_finding(
                "WARN",
                "轮速通道不自洽：NIS="
                f"{nis_v:.3g}（预测创新"
                f" {predicted_innov_v_std:.4g} m/s，实测"
                f" {measured_innov_v_std:.4g} m/s）；"
                "r_wheel 未按实测量测噪声标定",
            )
        elif nis_v > 4.0:
            add_finding(
                "WARN",
                f"轮速通道偏保守：NIS={nis_v:.3g}（理想值 1）",
            )
        elif nis_v < 0.25:
            add_finding(
                "WARN",
                f"轮速通道偏自信：NIS={nis_v:.3g}（理想值 1）",
            )
    if _finite(nis_a) and nis_a > 0.0 and (nis_a > 100.0 or nis_a < 0.01):
        add_finding(
            "WARN",
            "IMU 通道不自洽：NIS="
            f"{nis_a:.3g}（预测创新"
            f" {predicted_innov_a_std:.4g} m/s²，实测"
            f" {measured_innov_a_std:.4g} m/s²）；"
            "r_imu 未按实测量测噪声标定",
        )

    if _finite(q_v) and q_v <= 0.0:
        add_finding("WARN", "q_v=0，速度过程噪声被关闭")
    if _finite(q_a) and q_a <= 0.0:
        add_finding("WARN", "q_a=0，加速度过程噪声被关闭")

    qr_samples = [
        (sample.q_v, sample.q_a, sample.r_wheel, sample.r_imu)
        for sample in samples
    ]
    if any(
        not _same_or_missing(a[0], b[0])
        or not _same_or_missing(a[1], b[1])
        or not _same_or_missing(a[2], b[2])
        or not _same_or_missing(a[3], b[3])
        for a, b in zip(qr_samples, qr_samples[1:])
    ):
        add_finding("WARN", "窗口内 Q/R 参数发生变化")

    return WindowMetrics(
        start_s=times[0],
        end_s=times[-1],
        samples=len(samples),
        sample_rate_hz=sample_rate,
        max_gap_ms=max_gap_ms,
        kf_dx_mean=kf_dx_mean,
        kf_dx_std=kf_dx_std,
        kf_dx_max_abs=kf_dx_max_abs,
        kf_x_span=kf_x_span,
        kf_x_net=kf_x_net,
        integration_rmse=integration_rmse,
        integration_max_error=integration_max_error,
        speed_step_max=speed_step_max,
        acceleration_p95=acceleration_p95,
        low_speed_drift=low_speed_drift,
        q_v=q_v,
        q_a=q_a,
        r_wheel=r_wheel,
        r_imu=r_imu,
        wheel_gain=wheel_gain,
        imu_gain=imu_gain,
        wheel_tau_ms=wheel_tau_ms,
        imu_tau_ms=imu_tau_ms,
        predicted_innov_v_std=predicted_innov_v_std,
        predicted_innov_a_std=predicted_innov_a_std,
        measured_innov_v_std=measured_innov_v_std,
        measured_innov_a_std=measured_innov_a_std,
        nis_v=nis_v,
        nis_a=nis_a,
        status=status,
        findings=findings,
    )


class TcpRttReader:
    def __init__(self, host: str, port: int, connect_timeout_s: float) -> None:
        self.socket = socket.create_connection(
            (host, port), timeout=connect_timeout_s
        )
        self.socket.settimeout(0.10)
        self.buffer = bytearray()

    def read_lines(self) -> list[str]:
        lines: list[str] = []
        self.buffer.extend(self.read_bytes())
        while b"\n" in self.buffer:
            line, _, remainder = self.buffer.partition(b"\n")
            self.buffer = bytearray(remainder)
            lines.append(line.decode("utf-8", errors="replace").strip())
        return lines

    def read_bytes(self) -> bytes:
        """Raw chunk from the RTT telnet port, or b"" when idle."""
        try:
            return self.socket.recv(8192)
        except socket.timeout:
            return b""
        except OSError:
            return b""

    def close(self) -> None:
        try:
            self.socket.close()
        except OSError:
            pass

    def send(self, line: str) -> None:
        try:
            self.socket.sendall((line.rstrip("\n") + "\n").encode("utf-8"))
        except OSError as exc:
            raise RuntimeError(f"RTT send failed: {exc}") from exc


class JLinkSubprocessReader:
    def __init__(self, device: str, speed: int, writable: bool = False) -> None:
        self.device = device
        self.speed = speed
        self.writable = writable
        self.restarts = 0
        self._spawn()

    def _spawn(self) -> None:
        command = [
            find_pylink_python(),
            str(Path(__file__).resolve()),
            "--jlink-helper",
            "--device",
            self.device,
            "--speed",
            str(self.speed),
        ]
        if self.writable:
            command.append("--jlink-write")
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE if self.writable else subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.buffer = bytearray()
        self.stderr_buffer = bytearray()
        self._wait_until_ready()

    def _wait_until_ready(self) -> None:
        # A J-Link that was not cleanly closed can take tens of seconds to
        # re-enumerate; a probe that then connected fine still failed here
        # twice at 15 s and cost a whole capture attempt each time.
        deadline = time.monotonic() + JLINK_READY_TIMEOUT_S
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                error = self._read_error_tail()
                raise RuntimeError(error or "J-Link helper exited during startup")
            readable, _, _ = select.select(
                [self.process.stderr], [], [], 0.05
            )
            if not readable:
                continue
            chunk = self.process.stderr.read1(4096)
            if not chunk:
                continue
            self.stderr_buffer.extend(chunk)
            if b"JLinkRttReader ready" in self.stderr_buffer:
                return
        error = self._read_error_tail()
        self.close()
        raise TimeoutError(
            f"J-Link helper did not become ready within "
            f"{JLINK_READY_TIMEOUT_S:.0f} seconds"
            + (f": {error}" if error else "")
        )

    def _read_error_tail(self) -> str:
        try:
            remainder = self.process.stderr.read()
        except (AttributeError, OSError):
            remainder = b""
        if remainder:
            self.stderr_buffer.extend(remainder)
        return self.stderr_buffer.decode("utf-8", errors="replace").strip()

    def read_bytes(self) -> bytes:
        """Raw chunk from the J-Link helper, or b"" when idle.

        VMCFAST is binary, so callers that need it must work on bytes.
        """
        if self.process.poll() is not None:
            # The helper exits on any transient SWD/RTT error.  Without this
            # check the parent simply stops receiving data and eventually times
            # out, with no hint that the J-Link side died.  Report it and
            # reconnect: a momentary glitch - easily provoked while the robot
            # is driving - must not end a long capture.
            reason = self._read_error_tail()
            self.restarts += 1
            print(
                f"J-Link helper exited ({reason or 'no message'}); "
                f"reconnecting (restart #{self.restarts})",
                file=sys.stderr,
                flush=True,
            )
            try:
                self.close(keep_streams=True)
                self._spawn()
            except Exception as exc:  # noqa: BLE001 - surfaced to the caller
                raise RuntimeError(
                    f"J-Link helper died and could not reconnect: {exc}"
                ) from exc
            return b""
        readable, _, _ = select.select([self.process.stdout], [], [], 0.0)
        if not readable:
            return b""
        raw = self.process.stdout.read1(8192)
        return raw or b""

    def read_lines(self) -> list[str]:
        self.buffer.extend(self.read_bytes())
        lines: list[str] = []
        while b"\n" in self.buffer:
            line, _, remainder = self.buffer.partition(b"\n")
            self.buffer = bytearray(remainder)
            lines.append(line.decode("utf-8", errors="replace").strip())
        return lines

    def close(self, keep_streams: bool = False) -> None:
        process = getattr(self, "process", None)
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        if not keep_streams:
            for stream in (process.stdout, process.stderr):
                try:
                    stream.close()
                except OSError:
                    pass

    def send(self, line: str) -> None:
        if not self.writable or self.process.stdin is None:
            raise RuntimeError("this reader was opened read-only")
        payload = (line.rstrip("\n") + "\n").encode("utf-8")
        try:
            self.process.stdin.write(payload)
            self.process.stdin.flush()
        except OSError as exc:
            raise RuntimeError(f"RTT send failed: {exc}") from exc


class InteractiveConsole:
    """Line-oriented control channel for live parameter updates."""

    def __init__(self) -> None:
        self.enabled = sys.stdin.isatty()
        self._fd: int | None = None
        self._buffer = b""

    def open(self) -> bool:
        if not self.enabled:
            return False
        try:
            self._fd = sys.stdin.fileno()
        except (OSError, ValueError):
            self._fd = None
            self.enabled = False
        return self.enabled

    def read_line(self) -> str | None:
        if self._fd is None:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None
        try:
            chunk = os.read(self._fd, 4096)
        except OSError:
            return None
        if not chunk:
            return None
        self._buffer += chunk
        if b"\n" not in self._buffer:
            return None
        line, _, remainder = self._buffer.partition(b"\n")
        self._buffer = remainder
        return line.decode("utf-8", errors="replace").strip()

    def close(self) -> None:
        self._fd = None


class FlagWatcher:
    """Print safety-relevant state transitions while the capture runs."""

    def __init__(self) -> None:
        self.airborne = False
        self.airborne_side = False
        self.slip = False
        self.jump_stage = 0.0
        self.gravity_sign: float | None = None
        self._last_saturation_ms = -1.0e9
        self._last_rc_warn_ms = -1.0e9

    def update(self, sample: KalmanSample, elapsed_s: float) -> None:
        if _finite(sample.gravity_sign):
            if self.gravity_sign is None or (
                (self.gravity_sign < 0.5) != (sample.gravity_sign < 0.5)
            ):
                self.gravity_sign = sample.gravity_sign
                print(
                    f"  [FLAG {elapsed_s:7.2f}s] GRAVITY SIGN "
                    f"{'resolved = %+d' % int(sample.gravity_sign) if abs(sample.gravity_sign) >= 0.5 else 'unresolved (IMU accel withheld)'}"
                )
        if _finite(sample.airborne_active):
            airborne = sample.airborne_active > 0.5
            if airborne != self.airborne:
                self.airborne = airborne
                if airborne:
                    # The joint-load extremes are what the latch keys on; the
                    # mean alone cannot show that one leg still carried the
                    # body, which is the supported-but-slipping case.
                    print(
                        f"  [FLAG {elapsed_s:7.2f}s] AIRBORNE ACTIVE - wheel "
                        f"output gated (wheel="
                        f"{format_value(sample.airborne_wheel_speed, 2)} rad/s, "
                        f"joint load min="
                        f"{format_value(sample.airborne_joint_torque_min, 2)} "
                        f"max={format_value(sample.airborne_joint_torque_max, 2)} "
                        f"Nm, pitch={format_value(sample.pitch, 3)} rad)"
                    )
                else:
                    print(f"  [FLAG {elapsed_s:7.2f}s] AIRBORNE released")
        if _finite(sample.airborne_side_active_left) or _finite(
            sample.airborne_side_active_right
        ):
            left = _finite(sample.airborne_side_active_left) and (
                sample.airborne_side_active_left > 0.5
            )
            right = _finite(sample.airborne_side_active_right) and (
                sample.airborne_side_active_right > 0.5
            )
            if (left or right) != self.airborne_side:
                self.airborne_side = left or right
                if left or right:
                    # A single lifted leg: only that side's wheel is cut, the
                    # grounded side keeps balancing.
                    print(
                        f"  [FLAG {elapsed_s:7.2f}s] AIRBORNE leg lifted "
                        f"L={int(left)} R={int(right)} (torque L="
                        f"{format_value(sample.airborne_side_torque_left, 2)} "
                        f"R={format_value(sample.airborne_side_torque_right, 2)} "
                        f"Nm)"
                    )
                else:
                    print(f"  [FLAG {elapsed_s:7.2f}s] AIRBORNE leg back down")
        if _finite(sample.kf_slip):
            slip = sample.kf_slip > 0.5
            if slip and not self.slip:
                print(
                    f"  [FLAG {elapsed_s:7.2f}s] SLIP gate latched "
                    f"(condition C={format_value(sample.kf_slip_metric, 2)} m/s2, "
                    f"L={format_value(sample.kf_slip_left, 0)} "
                    f"R={format_value(sample.kf_slip_right, 0)})"
                )
            self.slip = slip
        if _finite(sample.jump_stage) and sample.jump_stage != self.jump_stage:
            self.jump_stage = sample.jump_stage
            if sample.jump_stage > 0.5:
                print(
                    f"  [FLAG {elapsed_s:7.2f}s] JUMP stage="
                    f"{sample.jump_stage:.0f}"
                )
        if (
            _finite(sample.left_wheel_current)
            and _finite(sample.right_wheel_current)
            and max(
                abs(sample.left_wheel_current), abs(sample.right_wheel_current)
            ) >= 9000.0
            and elapsed_s - self._last_saturation_ms >= 2.0
        ):
            self._last_saturation_ms = elapsed_s
            print(
                f"  [FLAG {elapsed_s:7.2f}s] WHEEL CURRENT saturated "
                f"L={sample.left_wheel_current:.0f} "
                f"R={sample.right_wheel_current:.0f}"
            )
        if (
            _finite(sample.rc_age_ms)
            and 0.0 <= sample.rc_age_ms <= 10000.0
            and sample.rc_age_ms > 100.0
            and elapsed_s - self._last_rc_warn_ms >= 2.0
        ):
            self._last_rc_warn_ms = elapsed_s
            print(
                f"  [FLAG {elapsed_s:7.2f}s] RC frame stale "
                f"{sample.rc_age_ms:.0f} ms"
            )


def find_pylink_python() -> str:
    candidates = [
        sys.executable,
        shutil.which("python3") or "",
        "/Library/Frameworks/Python.framework/Versions/3.13/bin/python3",
        "/opt/homebrew/bin/python3",
        "/usr/local/bin/python3",
        "/usr/bin/python3",
    ]
    candidates.extend(
        sorted(
            glob.glob(
                "/Library/Frameworks/Python.framework/Versions/*/bin/python3"
            ),
            reverse=True,
        )
    )
    seen: set[str] = set()
    for candidate in candidates:
        if not candidate:
            continue
        resolved = str(Path(candidate).resolve())
        if resolved in seen:
            continue
        seen.add(resolved)
        try:
            result = subprocess.run(
                [candidate, "-c", "import pylink"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            continue
        if result.returncode == 0:
            return candidate
    raise RuntimeError(
        "direct J-Link mode requires a Python with pylink-square; "
        "install it with: python3 -m pip install pylink-square"
    )


def open_reader(args: argparse.Namespace) -> Any:
    errors: list[str] = []
    writable = bool(args.interactive) or bool(args.at)
    if args.source in ("auto", "tcp"):
        try:
            reader = TcpRttReader(args.host, args.port, args.connect_timeout)
            print(f"RTT source: TCP {args.host}:{args.port}")
            return reader
        except OSError as exc:
            errors.append(f"TCP {args.host}:{args.port}: {exc}")
            if args.source == "tcp":
                raise

    if args.source in ("auto", "jlink"):
        try:
            reader = JLinkSubprocessReader(args.device, args.speed, writable)
            print(f"RTT source: direct J-Link {args.device} @ {args.speed} kHz")
            return reader
        except Exception as exc:
            errors.append(f"J-Link {args.device}: {exc}")

    raise RuntimeError("no RTT source available; " + "; ".join(errors))


def _sample_to_row(sample: KalmanSample) -> dict[str, Any]:
    row: dict[str, Any] = {}
    for field in CSV_FIELDS:
        value = getattr(sample, field)
        row[field] = "" if not _finite(value) else value
    return row


def _parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def iter_csv_samples(path: Path) -> Iterator[KalmanSample]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            yield KalmanSample(
                host_time_s=_safe_float(row.get("host_time_s", "")),
                kf_x=_safe_float(row.get("kf_x", "")),
                kf_dx=_safe_float(row.get("kf_dx", "")),
                mcu_time_ms=_safe_float(row.get("mcu_time_ms", "")),
                q_v=_safe_float(row.get("q_v", "")),
                q_a=_safe_float(row.get("q_a", "")),
                r_wheel=_safe_float(row.get("r_wheel", "")),
                r_imu=_safe_float(row.get("r_imu", "")),
                left_wheel_current=_safe_float(row.get("left_wheel_current", "")),
                right_wheel_current=_safe_float(row.get("right_wheel_current", "")),
                kf_acc=_safe_float(row.get("kf_acc", "")),
                d_x_raw=_safe_float(row.get("d_x_raw", "")),
                dd_x=_safe_float(row.get("dd_x", "")),
                kf_wheel_measurement=_safe_float(
                    row.get("kf_wheel_measurement", "")
                ),
                kf_accel_measurement=_safe_float(
                    row.get("kf_accel_measurement", "")
                ),
                kf_innovation_v=_safe_float(row.get("kf_innovation_v", "")),
                kf_innovation_a=_safe_float(row.get("kf_innovation_a", "")),
                kf_slip=_safe_float(row.get("kf_slip", "")),
                kf_slip_metric=_safe_float(row.get("kf_slip_metric", "")),
                rc_age_ms=_safe_float(row.get("rc_age_ms", "")),
                state_age_ms=_safe_float(row.get("state_age_ms", "")),
                airborne_active=_safe_float(row.get("airborne_active", "")),
                airborne_candidate=_safe_float(row.get("airborne_candidate", "")),
                airborne_wheel_speed=_safe_float(
                    row.get("airborne_wheel_speed", "")
                ),
                airborne_joint_torque=_safe_float(
                    row.get("airborne_joint_torque", "")
                ),
                airborne_joint_torque_min=_safe_float(
                    row.get("airborne_joint_torque_min", "")
                ),
                airborne_joint_torque_max=_safe_float(
                    row.get("airborne_joint_torque_max", "")
                ),
                airborne_pitch_reference=_safe_float(
                    row.get("airborne_pitch_reference", "")
                ),
                airborne_side_torque_left=_safe_float(
                    row.get("airborne_side_torque_left", "")
                ),
                airborne_side_torque_right=_safe_float(
                    row.get("airborne_side_torque_right", "")
                ),
                airborne_side_active_left=_safe_float(
                    row.get("airborne_side_active_left", "")
                ),
                airborne_side_active_right=_safe_float(
                    row.get("airborne_side_active_right", "")
                ),
                loop_dt_us=_safe_float(row.get("loop_dt_us", "")),
                loop_count=_safe_float(row.get("loop_count", "")),
                jump_stage=_safe_float(row.get("jump_stage", "")),
                jump_abort_reason=_safe_float(
                    row.get("jump_abort_reason", "")
                ),
                jump_armed=_safe_float(row.get("jump_armed", "")),
                jump_target_length=_safe_float(row.get("jump_target_length", "")),
                jump_stage_ms=_safe_float(row.get("jump_stage_ms", "")),
                jump_land_contact_ms=_safe_float(
                    row.get("jump_land_contact_ms", "")
                ),
                jump_trigger_count=_safe_float(row.get("jump_trigger_count", "")),
                jump_abort_count=_safe_float(row.get("jump_abort_count", "")),
                leg_length_left=_safe_float(row.get("leg_length_left", "")),
                leg_length_right=_safe_float(row.get("leg_length_right", "")),
                leg_length_target=_safe_float(row.get("leg_length_target", "")),
                getup_state=_safe_float(row.get("getup_state", "")),
                theta=_safe_float(row.get("theta", "")),
                left_theta=_safe_float(row.get("left_theta", "")),
                right_theta=_safe_float(row.get("right_theta", "")),
                gravity_sign=_safe_float(row.get("gravity_sign", "")),
                body_ax_g=_safe_float(row.get("body_ax_g", "")),
                body_ax_lin_g=_safe_float(row.get("body_ax_lin_g", "")),
                body_ay_g=_safe_float(row.get("body_ay_g", "")),
                body_az_g=_safe_float(row.get("body_az_g", "")),
                body_ay_lin_g=_safe_float(row.get("body_ay_lin_g", "")),
                body_az_lin_g=_safe_float(row.get("body_az_lin_g", "")),
                slip_lqr_applied_left=_safe_float(
                    row.get("slip_lqr_applied_left", "")
                ),
                slip_lqr_applied_right=_safe_float(
                    row.get("slip_lqr_applied_right", "")
                ),
                slip_curr_applied_left=_safe_float(
                    row.get("slip_curr_applied_left", "")
                ),
                slip_curr_applied_right=_safe_float(
                    row.get("slip_curr_applied_right", "")
                ),
                kf_accel_update_count=_safe_float(
                    row.get("kf_accel_update_count", "")
                ),
                stick_forward=_safe_float(row.get("stick_forward", "")),
                speed_target=_safe_float(row.get("speed_target", "")),
                speed_target_ramped=_safe_float(
                    row.get("speed_target_ramped", "")
                ),
                dx_target=_safe_float(row.get("dx_target", "")),
                dx_error=_safe_float(row.get("dx_error", "")),
                schema_version=_safe_float(row.get("schema_version", "")),
                dist_obs_output=_safe_float(row.get("dist_obs_output", "")),
                dist_obs_active=_safe_float(row.get("dist_obs_active", "")),
                dist_obs_neutral_time=_safe_float(
                    row.get("dist_obs_neutral_time", "")
                ),
                dist_obs_pitch_comp=_safe_float(
                    row.get("dist_obs_pitch_comp", "")
                ),
                dist_obs_theta_comp=_safe_float(
                    row.get("dist_obs_theta_comp", "")
                ),
                kf_slip_left=_safe_float(row.get("kf_slip_left", "")),
                kf_slip_right=_safe_float(row.get("kf_slip_right", "")),
                kf_wheel_left_raw=_safe_float(
                    row.get("kf_wheel_left_raw", "")
                ),
                kf_wheel_right_raw=_safe_float(
                    row.get("kf_wheel_right_raw", "")
                ),
                slip_accel_threshold=_safe_float(
                    row.get("slip_accel_threshold", "")
                ),
                slip_enable=_safe_float(row.get("slip_enable", "")),
                roll=_safe_float(row.get("roll", "")),
                pitch=_safe_float(row.get("pitch", "")),
                lqr_theta=_safe_float(row.get("lqr_theta", "")),
                roll_rate=_safe_float(row.get("roll_rate", "")),
                pitch_rate=_safe_float(row.get("pitch_rate", "")),
                tf02_distance_cm=_safe_float(row.get("tf02_distance_cm", "")),
                tf02_connected=_safe_float(row.get("tf02_connected", "")),
                roll_comp_offset=_safe_float(row.get("roll_comp_offset", "")),
                leg_target_left=_safe_float(row.get("leg_target_left", "")),
                leg_target_right=_safe_float(row.get("leg_target_right", "")),
                leg_follow_offset_left=_safe_float(
                    row.get("leg_follow_offset_left", "")
                ),
                leg_follow_offset_right=_safe_float(
                    row.get("leg_follow_offset_right", "")
                ),
                leg_follow_low_side=_safe_float(
                    row.get("leg_follow_low_side", "")
                ),
                leg_follow_episode_active=_safe_float(
                    row.get("leg_follow_episode_active", "")
                ),
                leg_follow_scope_resolved=_safe_float(
                    row.get("leg_follow_scope_resolved", "")
                ),
                leg_follow_candidate_side=_safe_float(
                    row.get("leg_follow_candidate_side", "")
                ),
                left_wheel_torque_raw=_safe_float(
                    row.get("left_wheel_torque_raw", "")
                ),
                right_wheel_torque_raw=_safe_float(
                    row.get("right_wheel_torque_raw", "")
                ),
                left_tp_theta_term=_safe_float(row.get("left_tp_theta_term", "")),
                left_tp_dtheta_term=_safe_float(row.get("left_tp_dtheta_term", "")),
                left_tp_x_term=_safe_float(row.get("left_tp_x_term", "")),
                left_tp_dx_term=_safe_float(row.get("left_tp_dx_term", "")),
                left_tp_phi_term=_safe_float(row.get("left_tp_phi_term", "")),
                left_tp_dphi_term=_safe_float(row.get("left_tp_dphi_term", "")),
                left_tp_raw=_safe_float(row.get("left_tp_raw", "")),
                left_tp_cmd=_safe_float(row.get("left_tp_cmd", "")),
                right_tp_theta_term=_safe_float(row.get("right_tp_theta_term", "")),
                right_tp_dtheta_term=_safe_float(row.get("right_tp_dtheta_term", "")),
                right_tp_x_term=_safe_float(row.get("right_tp_x_term", "")),
                right_tp_dx_term=_safe_float(row.get("right_tp_dx_term", "")),
                right_tp_phi_term=_safe_float(row.get("right_tp_phi_term", "")),
                right_tp_dphi_term=_safe_float(row.get("right_tp_dphi_term", "")),
                right_tp_raw=_safe_float(row.get("right_tp_raw", "")),
                right_tp_cmd=_safe_float(row.get("right_tp_cmd", "")),
                leg_sync_error=_safe_float(row.get("leg_sync_error", "")),
                leg_sync_torque=_safe_float(row.get("leg_sync_torque", "")),
                stair_stage=_safe_float(row.get("stair_stage", "")),
                stair_abort_reason=_safe_float(row.get("stair_abort_reason", "")),
                stair_ready_detected=_safe_float(row.get("stair_ready_detected", "")),
                stair_preimpact_armed=_safe_float(row.get("stair_preimpact_armed", "")),
                stair_impact_detected=_safe_float(row.get("stair_impact_detected", "")),
                stair_trigger_count=_safe_float(row.get("stair_trigger_count", "")),
                stair_abort_count=_safe_float(row.get("stair_abort_count", "")),
                stair_speed_abs=_safe_float(row.get("stair_speed_abs", "")),
                stair_speed_diff=_safe_float(row.get("stair_speed_diff", "")),
                stair_accel=_safe_float(row.get("stair_accel", "")),
                stair_theta_ramped=_safe_float(row.get("stair_theta_ramped", "")),
                stair_left_impact=_safe_float(row.get("stair_left_impact", "")),
                stair_right_impact=_safe_float(row.get("stair_right_impact", "")),
                stair_fast_detected=_safe_float(row.get("stair_fast_detected", "")),
                stair_speed_ms=_safe_float(row.get("stair_speed_ms", "")),
                stair_preimpact_ms=_safe_float(row.get("stair_preimpact_ms", "")),
                stair_impact_ms=_safe_float(row.get("stair_impact_ms", "")),
                rc_source=_safe_float(row.get("rc_source", "")),
                remote_s1=_safe_float(row.get("remote_s1", "")),
                remote_s2=_safe_float(row.get("remote_s2", "")),
                descent_stage=_safe_float(row.get("descent_stage", "")),
                descent_abort_reason=_safe_float(
                    row.get("descent_abort_reason", "")
                ),
                descent_contact_detected=_safe_float(
                    row.get("descent_contact_detected", "")
                ),
                descent_entry_length=_safe_float(
                    row.get("descent_entry_length", "")
                ),
                descent_extend_target=_safe_float(
                    row.get("descent_extend_target", "")
                ),
                descent_length_command=_safe_float(
                    row.get("descent_length_command", "")
                ),
                descent_contact_torque=_safe_float(
                    row.get("descent_contact_torque", "")
                ),
                descent_contact_ms=_safe_float(row.get("descent_contact_ms", "")),
                descent_trigger_count=_safe_float(
                    row.get("descent_trigger_count", "")
                ),
                descent_abort_count=_safe_float(
                    row.get("descent_abort_count", "")
                ),
            )


def iter_raw_text_samples(
    handle: TextIO, requested_profile: str
) -> Iterator[KalmanSample]:
    index = 0
    for line in handle:
        parsed = parse_vmcvoa_line(line, requested_profile)
        if parsed is None:
            continue
        _, sample = parsed
        sample.host_time_s = index * TELEMETRY_PERIOD_S
        index += 1
        yield sample


def load_samples(path: Path, requested_profile: str) -> list[KalmanSample]:
    if path.suffix.lower() == ".csv":
        return list(iter_csv_samples(path))
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        return list(iter_raw_text_samples(handle, requested_profile))


def sliding_windows(
    samples: list[KalmanSample],
    window_s: float,
    hop_s: float,
) -> Iterator[list[KalmanSample]]:
    ordered = sorted(samples, key=sample_time_s)
    if not ordered:
        return
    if window_s <= 0.0 or hop_s <= 0.0:
        raise ValueError("window and hop must be positive")

    start = sample_time_s(ordered[0])
    end = sample_time_s(ordered[-1])
    window_start = start
    while window_start + window_s <= end + 1.0e-9:
        window_end = window_start + window_s
        current = [
            sample
            for sample in ordered
            if sample_time_s(sample) >= window_start
            and sample_time_s(sample) < window_end
        ]
        if current:
            yield current
        window_start += hop_s


def analyze_continuity(samples: list[KalmanSample]) -> list[str]:
    """Return findings that depend on the whole capture rather than a window."""
    if len(samples) < 2:
        return []
    ordered = sorted(samples, key=sample_time_s)
    times = [sample_time_s(sample) for sample in ordered]
    max_gap_ms = max(
        (current - previous) * 1000.0
        for previous, current in zip(times, times[1:])
    )
    findings: list[str] = []
    if max_gap_ms > 500.0:
        findings.append(f"FAIL: RTT 最长断流 {max_gap_ms:.0f} ms")
    elif max_gap_ms > 100.0:
        findings.append(f"WARN: RTT 最长断流 {max_gap_ms:.0f} ms")

    counters = [
        (sample_time_s(sample), sample.kf_accel_update_count)
        for sample in ordered
        if _finite(sample.kf_accel_update_count)
    ]
    if len(counters) >= 2:
        # The counter only advances while the estimator is running, so measure
        # the rate across the advancing span.  When the chassis is disabled the
        # filter is reset every cycle and the counter is frozen; that is
        # expected and must not be reported as a stalled IMU.
        advancing = [
            (time_s, count)
            for (previous_t, previous_c), (time_s, count) in zip(
                counters, counters[1:]
            )
            if count != previous_c
        ]
        if advancing:
            first_t, first_c = counters[0]
            last_t, last_c = counters[-1]
            elapsed = last_t - first_t
            applied = last_c - first_c
            if elapsed > 0.5 and applied > 0:
                rate = applied / elapsed
                if rate < 50.0:
                    findings.append(
                        f"WARN: 加速度修正速率仅 {rate:.1f} Hz"
                        "（IMU 标称 200 Hz，检查 IMU 或控制环负载）"
                    )

    signs = [
        sample.gravity_sign
        for sample in ordered
        if _finite(sample.gravity_sign)
    ]
    if signs:
        unresolved = sum(1 for value in signs if abs(value) < 0.5) / len(signs)
        if unresolved > 0.5:
            findings.append(
                "WARN: 重力符号未标定（gravity_sign=0），"
                f"{unresolved * 100.0:.0f}% 的样本未使用 IMU 加速度"
            )
    return findings


def summarize_findings(
    windows: list[WindowMetrics], extra_findings: Iterable[str] = ()
) -> tuple[str, list[str]]:
    priorities = {"OK": 0, "WARN": 1, "FAIL": 2}
    status = "OK"
    findings: list[str] = []
    seen: set[str] = set()
    for finding in extra_findings:
        if finding in seen:
            continue
        seen.add(finding)
        findings.append(finding)
        level = finding.split(":", 1)[0]
        if level in priorities and priorities[level] > priorities[status]:
            status = level
    for window in windows:
        if priorities[window.status] > priorities[status]:
            status = window.status
        for finding in window.findings:
            if finding not in seen:
                seen.add(finding)
                findings.append(finding)
    return status, findings


def unique_config_values(
    samples: list[KalmanSample], field: str
) -> list[float]:
    values: list[float] = []
    for sample in samples:
        value = getattr(sample, field)
        if not _finite(value):
            continue
        if not any(_same_or_missing(value, existing) for existing in values):
            values.append(value)
    return values


def format_value(value: float, digits: int = 3) -> str:
    if not _finite(value):
        return "N/A"
    if math.isinf(value):
        return "inf"
    return f"{value:.{digits}f}"


def write_csv(
    path: Path,
    samples: list[KalmanSample],
) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for sample in samples:
            writer.writerow(_sample_to_row(sample))


def control_loop_rate_hz(samples: list[KalmanSample]) -> float:
    """Average control-loop rate from the firmware's own iteration counter.

    The control loop is not fixed-rate - ``osDelay(1)`` only sets a floor, so
    the real period is however long the loop body takes plus the sleep.  The
    firmware counts its own iterations, so dividing the change in
    ``loop_count`` by the firmware timebase yields the true average rate and
    is immune to host-side RTT jitter, which makes the host timestamps
    useless for this.
    """
    usable = [
        sample
        for sample in samples
        if _finite(sample.loop_count) and _finite(sample.mcu_time_ms)
    ]
    if len(usable) < 2:
        return INVALID_VALUE
    elapsed_s = (usable[-1].mcu_time_ms - usable[0].mcu_time_ms) * 0.001
    if elapsed_s <= 0.0:
        return INVALID_VALUE
    return (usable[-1].loop_count - usable[0].loop_count) / elapsed_s


def mean_loop_period_us(samples: list[KalmanSample]) -> float:
    """True mean control period, from the iteration counter.

    Note this is NOT the mean of the sampled ``loop_dt_us`` values.  Telemetry
    is emitted once every ~9 iterations and the transmitting iteration is
    systematically the shortest, so the sampled mean (~2000 us) understates the
    real mean (~2222 us).  Only the counter gives the true average.
    """
    rate = control_loop_rate_hz(samples)
    return (1.0e6 / rate) if _finite(rate) and rate > 0.0 else INVALID_VALUE


def measured_imu_rate_hz(samples: list[KalmanSample]) -> float:
    """IMU fusion rate from the firmware's own accelerometer-update counter.

    The estimator gates its acceleration update on a new HI12 frame, so this is
    the rate the IMU channel actually corrects at, not the sensor's nominal
    rate.

    The counter only advances while the controller is ENABLED - it lives in the
    same branch that runs the estimator - so dividing the total delta by the
    total recording span is wrong whenever the machine sat disabled.  On a
    capture that was enabled for 28.6 s of 180.8 s that reported 32 Hz for a
    200 Hz sensor, and the "only ~32 Hz" reading went on to be cited as a
    reason the accelerometer looked untrustworthy; it was an arithmetic
    mistake, not a firmware problem.

    The enabled time is therefore estimated from the counter itself: a stretch
    where it never advanced is a disabled stretch, so only the spans whose
    counter advanced count towards the denominator.  Verified against the
    enabled duration measured from the fast stream's own timestamps - 199.8 Hz
    on every capture that has a long enabled stretch.
    """
    usable = [
        sample
        for sample in samples
        if _finite(sample.kf_accel_update_count) and _finite(sample.mcu_time_ms)
    ]
    if len(usable) < 2:
        return INVALID_VALUE
    enabled_s = 0.0
    updates = 0.0
    for previous, current in zip(usable, usable[1:]):
        delta = current.kf_accel_update_count - previous.kf_accel_update_count
        if delta <= 0.0:
            continue
        enabled_s += (current.mcu_time_ms - previous.mcu_time_ms) * 0.001
        updates += delta
    if enabled_s <= 0.0 or updates <= 0.0:
        return INVALID_VALUE
    return updates / enabled_s


def model_control_period_s(samples: list[KalmanSample]) -> float:
    """Control period to use when modelling the filter.

    Prefers the measured period; falls back to the nominal constant for older
    recordings that do not carry the loop-timing channels.  Getting this wrong
    is not cosmetic: the accelerometer update is gated to one frame, so the
    number of control cycles between IMU corrections changes with the period,
    which moves the predicted innovation variance and therefore NIS.
    """
    period_us = mean_loop_period_us(samples)
    if _finite(period_us) and period_us > 0.0:
        return period_us * 1.0e-6
    return 0.001


def sampled_loop_period_us(samples: list[KalmanSample]) -> float:
    """Median of the periods the telemetry actually carried, for reference only.

    Biased low: see mean_loop_period_us.
    """
    values = [
        sample.loop_dt_us
        for sample in samples
        if _finite(sample.loop_dt_us) and sample.loop_dt_us > 0.0
    ]
    return statistics.median(values) if values else INVALID_VALUE


def render_report(
    *,
    source: str,
    profile: Profile,
    samples: list[KalmanSample],
    windows: list[WindowMetrics],
    status: str,
    findings: list[str],
    duration_s: float,
    ended_early: bool,
) -> str:
    total_duration = (
        sample_time_s(samples[-1]) - sample_time_s(samples[0])
        if len(samples) > 1
        else 0.0
    )
    ordered_times = [sample_time_s(sample) for sample in samples]
    nominal_gaps = [
        current - previous
        for previous, current in zip(ordered_times, ordered_times[1:])
        if 0.0 < current - previous <= 0.20
    ]
    overall_rate = (
        1.0 / statistics.median(nominal_gaps) if nominal_gaps else 0.0
    )
    q_v = unique_config_values(samples, "q_v")
    q_a = unique_config_values(samples, "q_a")
    r_wheel = unique_config_values(samples, "r_wheel")
    r_imu = unique_config_values(samples, "r_imu")

    lines = [
        "# Kalman RTT capture report",
        "",
        f"- Capture source: `{source}`",
        f"- Telemetry profile: `{profile.name}`",
        f"- Requested duration: `{duration_s:.1f} s`",
        f"- Captured span: `{total_duration:.1f} s`",
        f"- End reason: `{'stopped by user' if ended_early else 'completed'}`",
        f"- Samples: `{len(samples)}`",
        f"- Median sample rate excluding gaps: `{overall_rate:.2f} Hz`",
        f"- Measured control loop: `{format_value(control_loop_rate_hz(samples), 0)} Hz`",
        f"- Mean control period: `{format_value(mean_loop_period_us(samples), 0)} us`",
        f"- Sampled control period (median, biased low): `{format_value(sampled_loop_period_us(samples), 0)} us`",
        f"- Overall verdict: `{status}`",
        "",
        "## Runtime Q/R",
        "",
        f"- `q_v`: {', '.join(format_value(v, 6) for v in q_v) or 'N/A'}",
        f"- `q_a`: {', '.join(format_value(v, 6) for v in q_a) or 'N/A'}",
        f"- `r_wheel`: {', '.join(format_value(v, 6) for v in r_wheel) or 'N/A'}",
        f"- `r_imu`: {', '.join(format_value(v, 6) for v in r_imu) or 'N/A'}",
        "",
        "## Findings",
        "",
    ]
    if findings:
        lines.extend(f"- {finding}" for finding in findings)
    else:
        lines.append("- 未发现规则范围内的异常。")

    lines.extend(command_tracking_summary(samples))

    lines.extend(
        [
            "",
            "## Sliding windows",
            "",
            "| Window (s) | N | Rate Hz | kf_dx mean | kf_dx std | "
            "kf_dx max | kf_x net | Integ RMSE | Accel P95 | K wheel | "
            "K IMU | NIS v | Status |",
            "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|",
        ]
    )
    for window in windows:
        lines.append(
            f"| {window.start_s - sample_time_s(samples[0]):.1f}.."
            f"{window.end_s - sample_time_s(samples[0]):.1f} "
            f"| {window.samples} "
            f"| {window.sample_rate_hz:.1f} "
            f"| {format_value(window.kf_dx_mean)} "
            f"| {format_value(window.kf_dx_std)} "
            f"| {format_value(window.kf_dx_max_abs)} "
            f"| {format_value(window.kf_x_net, 4)} "
            f"| {format_value(window.integration_rmse, 4)} "
            f"| {format_value(window.acceleration_p95, 1)} "
            f"| {format_value(window.wheel_gain, 4)} "
            f"| {format_value(window.imu_gain, 3)} "
            f"| {format_value(window.nis_v, 3)} "
            f"| {window.status} |"
        )

    lines.extend(
        [
            "",
            "## Notes",
            "",
            "- `K wheel` / `K IMU` are the steady-state Kalman gains under the "
            "runtime Q/R values.",
            "- `Integ RMSE` compares each observed `kf_x` step with the "
            "trapezoidal integral of `kf_dx`; a large value indicates a reset, "
            "telemetry corruption, or an inconsistent estimator output.",
        ]
    )
    if profile.name == "format":
        lines.extend(
            [
                "- The `format` profile uses the MCU tick millisecond timestamp "
                "for sample-rate and integration calculations when available.",
                "- Q/R are transmitted as integer values scaled by 1000 and "
                "decoded back to their original units by this report.",
                "- `kf_wheel_measurement`, `kf_accel_measurement`, "
                "`kf_innovation_v`, and `kf_innovation_a` are recorded in the "
                "CSV for direct innovation analysis.",
                "- `kf_accel_measurement` is the gravity-compensated body-X "
                "linear acceleration in m/s2.  `gravity_sign` reports the "
                "resolved accelerometer convention; while it is 0 the "
                "accelerometer is withheld from the estimator.",
                "",
            ]
        )
    else:
        lines.extend(
            [
                "- This older telemetry layout does not expose all measurement "
                "and innovation channels.",
                "",
            ]
        )
    return "\n".join(lines)


def capture_live(
    reader: Any,
    *,
    duration_s: float,
    window_s: float,
    hop_s: float,
    startup_stabilize_s: float,
    rx_timeout_s: float,
    stop_requested: Callable[[], bool] | None,
    console: InteractiveConsole | None,
    scheduled_sends: list[tuple[float, str]] | None,
    requested_profile: str,
    csv_handle: TextIO,
    csv_writer: csv.DictWriter,
) -> tuple[Profile | None, list[KalmanSample], list[WindowMetrics], bool]:
    samples: list[KalmanSample] = []
    windows: list[WindowMetrics] = []
    profile: Profile | None = None
    start = time.monotonic()
    next_progress = window_s
    last_rx = start
    last_sample_clock: float | None = None
    ended_early = False
    watcher = FlagWatcher()
    schedule = list(scheduled_sends or [])
    schedule_index = 0

    try:
        while True:
            now = time.monotonic()
            elapsed = now - start
            while schedule_index < len(schedule) and elapsed >= schedule[schedule_index][0]:
                send_at, payload = schedule[schedule_index]
                schedule_index += 1
                try:
                    reader.send(payload)
                    print(f"[SCHEDULE {send_at:.0f}s] TX> {payload}")
                except RuntimeError as exc:
                    print(f"[SCHEDULE {send_at:.0f}s] send failed: {exc}",
                          file=sys.stderr)
            if now - start >= duration_s:
                break
            if console is not None:
                command = console.read_line()
                if command is not None:
                    if command in ("stop", "q", "quit", "exit"):
                        ended_early = True
                        print("Stop command received; finalizing captured data...")
                        break
                    if command.startswith("#"):
                        pass
                    elif command:
                        try:
                            reader.send(command)
                            print(f"TX> {command}")
                        except RuntimeError as exc:
                            print(f"send failed: {exc}", file=sys.stderr)
            if stop_requested is not None and stop_requested():
                ended_early = True
                print("Stop key pressed; finalizing captured data...")
                break
            lines = reader.read_lines()
            if lines:
                last_rx = now
            for line in lines:
                if not line:
                    continue
                try:
                    parsed = parse_vmcvoa_line(line, requested_profile)
                except ValueError as exc:
                    print(f"RTT parse skipped: {exc}", file=sys.stderr)
                    continue
                if parsed is None:
                    if line.strip():
                        print(f"RX< {line.strip()}")
                    continue
                parsed_profile, sample = parsed
                if profile is None:
                    profile = parsed_profile
                    print(
                        f"Telemetry profile detected: {profile.name} "
                        f"({profile.field_count} fields)"
                    )
                sample_clock = sample_time_s(sample)
                if (
                    last_sample_clock is not None
                    and sample_clock - last_sample_clock > 1.0
                    and now - start <= startup_stabilize_s
                ):
                    # A J-Link connection can first expose stale RTT data from
                    # before the reader attached.  Restart on the first
                    # continuous segment instead of mixing two timebases.
                    print(
                        "Discarded pre-connection RTT segment "
                        f"({sample_clock - last_sample_clock:.1f} s gap)"
                    )
                    samples.clear()
                    windows.clear()
                    csv_handle.seek(0)
                    csv_handle.truncate()
                    csv_writer.writeheader()
                    start = now
                    next_progress = window_s
                    last_rx = now
                sample.host_time_s = now - start
                samples.append(sample)
                last_sample_clock = sample_clock
                watcher.update(sample, sample.host_time_s)
                csv_writer.writerow(_sample_to_row(sample))
                csv_handle.flush()

                while samples and sample.host_time_s >= next_progress:
                    window = [
                        candidate
                        for candidate in samples
                        if candidate.host_time_s >= next_progress - window_s
                        and candidate.host_time_s < next_progress
                    ]
                    if window:
                        metrics = analyze_window(window)
                        windows.append(metrics)
                    print(
                        f"[{next_progress - window_s:6.1f}.."
                        f"{next_progress:6.1f}s] "
                        f"{metrics.status:4s} "
                        f"N={metrics.samples:3d} "
                        f"dx={metrics.kf_dx_mean:+.4f}"
                        f"±{metrics.kf_dx_std:.4f} m/s "
                        f"x_net={metrics.kf_x_net:+.4f} m "
                        f"K_w={format_value(metrics.wheel_gain, 4)} "
                        f"K_i={format_value(metrics.imu_gain, 4)} "
                        f"NIS_v={format_value(metrics.nis_v, 3)}"
                    )
                    for finding in metrics.findings:
                        print(f"    {finding}")
                    next_progress += hop_s

            if now - last_rx > rx_timeout_s and now - start > 1.0:
                raise TimeoutError(
                    "no VMCVOFA telemetry received for "
                    f"{rx_timeout_s:.0f} seconds"
                )
            time.sleep(0.002)
    except KeyboardInterrupt:
        ended_early = True
        print("\nCtrl-C received; finalizing captured data...")

    if len(samples) >= 2:
        tail = [
            sample
            for sample in samples
            if sample.host_time_s >= samples[-1].host_time_s - window_s
        ]
        if tail:
            metrics = analyze_window(tail)
            if not windows or windows[-1].end_s != metrics.end_s:
                windows.append(metrics)
    return profile, samples, windows, ended_early


def analyze_samples(
    samples: list[KalmanSample],
    *,
    window_s: float,
    hop_s: float,
    print_windows: bool = True,
) -> list[WindowMetrics]:
    windows: list[WindowMetrics] = []
    for window_samples in sliding_windows(samples, window_s, hop_s):
        metrics = analyze_window(window_samples)
        windows.append(metrics)
        if print_windows:
            print(
                f"[{metrics.start_s:6.1f}..{metrics.end_s:6.1f}s] "
                f"{metrics.status:4s} N={metrics.samples:3d} "
                f"dx={metrics.kf_dx_mean:+.4f}±{metrics.kf_dx_std:.4f} m/s "
                f"x_net={metrics.kf_x_net:+.4f} m"
            )
    return windows


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Capture 120 seconds of VMCVOFA telemetry through SEGGER RTT and "
            "assess the Kalman-filter channels in sliding windows."
        )
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help=f"live capture duration in seconds (default: {DEFAULT_DURATION_S:.0f})",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=DEFAULT_WINDOW_S,
        help=f"sliding-window length in seconds (default: {DEFAULT_WINDOW_S:.0f})",
    )
    parser.add_argument(
        "--hop",
        type=float,
        default=DEFAULT_HOP_S,
        help=f"sliding-window hop in seconds (default: {DEFAULT_HOP_S:.0f})",
    )
    parser.add_argument(
        "--startup-stabilize",
        type=float,
        default=2.0,
        help=(
            "discard an initial stale RTT segment when a time gap appears "
            "within this many seconds (default: 2.0; 0 disables)"
        ),
    )
    parser.add_argument(
        "--rx-timeout",
        type=float,
        default=10.0,
        help=(
            "abort if no telemetry arrives for this many seconds "
            "(default: 10; a longer value tolerates brief J-Link stalls)"
        ),
    )
    parser.add_argument(
        "--stop-key",
        default=DEFAULT_STOP_KEY,
        metavar="KEY",
        help=(
            "terminal key that ends capture early and writes the partial "
            f"report (default: {DEFAULT_STOP_KEY}; use 'any' or 'none')"
        ),
    )
    parser.add_argument(
        "--source",
        choices=("auto", "tcp", "jlink"),
        default="auto",
        help="RTT transport; auto tries TCP first and then direct J-Link",
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help="RTT Telnet host")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="RTT Telnet port")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="J-Link device name")
    parser.add_argument(
        "--speed",
        type=int,
        default=DEFAULT_JLINK_SPEED,
        help="J-Link SWD speed in kHz",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=0.5,
        help="TCP connection timeout in seconds",
    )
    parser.add_argument(
        "--profile",
        choices=(
            "auto",
            "format",
            "format-v1",
            "format-v2",
            "format-v3",
            "format-v4",
            "format-v5",
            "format-v6",
            "format-v7",
            "format-v8",
            "format-v9",
            "format-v10",
            "format-v11",
            "format-v12",
            "format-v13",
            "format-v14",
        ),
        default=DEFAULT_PROFILE,
        help="VMCVOFA layout; auto detects by field count",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="directory for CSV and reports (default: aboard/logs)",
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=None,
        help="replay a CSV or raw VMCVOFA log instead of reading hardware",
    )
    parser.add_argument(
        "--jlink-helper",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--jlink-write",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help=(
            "open a bidirectional RTT session: typed lines are sent to the "
            "firmware, and typing 'stop' finalizes the capture"
        )
    )
    parser.add_argument(
        "--at",
        type=float,
        action="append",
        default=[],
        metavar="SECONDS",
        help=(
            "send the matching --send payload this many seconds into the "
            "capture; repeat paired with --send for a timed parameter sweep"
        ),
    )
    parser.add_argument(
        "--send",
        action="append",
        default=[],
        metavar="PAYLOAD",
        help="RTT payload paired with the corresponding --at",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.jlink_helper:
        return run_jlink_helper(args.device, args.speed, args.jlink_write)
    if args.window <= 0.0 or args.hop <= 0.0:
        print("--window and --hop must be positive", file=sys.stderr)
        return 2
    if args.startup_stabilize < 0.0:
        print("--startup-stabilize must be non-negative", file=sys.stderr)
        return 2
    if args.stop_key not in ("any", "none") and len(args.stop_key) != 1:
        print("--stop-key must be one character, 'any', or 'none'", file=sys.stderr)
        return 2
    if len(args.at) != len(args.send):
        print("--at and --send must be given the same number of times",
              file=sys.stderr)
        return 2
    if any(at < 0.0 for at in args.at):
        print("--at values must be non-negative", file=sys.stderr)
        return 2
    scheduled_sends = sorted(zip(args.at, args.send), key=lambda pair: pair[0])

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.output_dir is None:
        output_dir = Path(__file__).resolve().parents[1] / "logs" / timestamp
    else:
        output_dir = args.output_dir / timestamp
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "kalman_samples.csv"
    json_path = output_dir / "kalman_summary.json"
    report_path = output_dir / "kalman_report.md"

    reader = None
    stop_key = TerminalStopKey(args.stop_key)
    console: InteractiveConsole | None = None
    ended_early = False
    try:
        if args.input is not None:
            profile = (
                PROFILES[args.profile] if args.profile != "auto" else FORMAT_PROFILE
            )
            samples = load_samples(args.input, args.profile)
            source_name = f"replay:{args.input}"
            print(f"Replaying {len(samples)} samples from {args.input}")
            windows = analyze_samples(
                samples, window_s=args.window, hop_s=args.hop
            )
        else:
            reader = open_reader(args)
            if args.interactive:
                console = InteractiveConsole()
                if console.open():
                    print(
                        "Interactive mode. Type a firmware line and press Enter "
                        "to send it; type 'stop' to finish."
                    )
                else:
                    print(
                        "Interactive mode needs a terminal on stdin.",
                        file=sys.stderr,
                    )
                    return 2
            elif args.stop_key != "none":
                if stop_key.open():
                    print(f"Press '{args.stop_key}' to stop the capture early.")
                else:
                    print(
                        "Stop key disabled: standard input is not a terminal.",
                        file=sys.stderr,
                    )
            with csv_path.open("w", encoding="utf-8", newline="") as csv_handle:
                writer = csv.DictWriter(csv_handle, fieldnames=CSV_FIELDS)
                writer.writeheader()
                profile, samples, windows, ended_early = capture_live(
                    reader,
                    duration_s=args.duration,
                    window_s=args.window,
                    hop_s=args.hop,
                    startup_stabilize_s=args.startup_stabilize,
                    rx_timeout_s=args.rx_timeout,
                    stop_requested=stop_key.stop_requested,
                    console=console,
                    scheduled_sends=scheduled_sends,
                    requested_profile=args.profile,
                    csv_handle=csv_handle,
                    csv_writer=writer,
                )
            source_name = {
                "tcp": f"tcp://{args.host}:{args.port}",
                "jlink": f"jlink://{args.device}",
                "auto": "auto",
            }[args.source]
    except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
        print(f"capture failed: {exc}", file=sys.stderr)
        return 3
    finally:
        if reader is not None:
            reader.close()
        stop_key.close()
        if console is not None:
            console.close()

    if not samples:
        print("no VMCVOFA samples captured", file=sys.stderr)
        return 3
    if profile is None:
        profile = FORMAT_PROFILE
    if not windows:
        print("capture was shorter than one analysis window", file=sys.stderr)
        return 3

    if args.input is not None:
        write_csv(csv_path, samples)
    status, findings = summarize_findings(
        windows, analyze_continuity(samples)
    )
    report = render_report(
        source=source_name,
        profile=profile,
        samples=samples,
        windows=windows,
        status=status,
        findings=findings,
        duration_s=args.duration,
        ended_early=ended_early,
    )
    report_path.write_text(report, encoding="utf-8")

    summary = {
        "source": source_name,
        "profile": profile.name,
        "status": status,
        "findings": findings,
        "samples": len(samples),
        "ended_early": ended_early,
        "windows": [asdict(window) for window in windows],
    }
    json_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print()
    print(f"Overall: {status}")
    if findings:
        for finding in findings:
            print(f"- {finding}")
    print(f"CSV:    {csv_path}")
    print(f"JSON:   {json_path}")
    print(f"Report: {report_path}")
    return 0 if status == "OK" else 1


def run_jlink_helper(device: str, speed: int, writable: bool = False) -> int:
    """Stream raw RTT channel-0 bytes; pylink stays isolated in this process."""
    try:
        import pylink
    except ImportError:
        print(
            "direct J-Link mode requires pylink-square; "
            "install it with: python3 -m pip install pylink-square",
            file=sys.stderr,
        )
        return 4

    jlink = pylink.JLink()
    io_lock = threading.Lock()

    def stdin_pump() -> None:
        """Forward typed lines to the RTT down channel.

        Do not give up on the first failure.  The host can send before pylink's
        RTT handle is fully usable, and a silent return here loses every later
        command while the parent still reports the send as successful.  Retry
        briefly, then report the error so the failure is visible instead of
        looking like a firmware-side parse problem.
        """
        failures = 0
        for raw in sys.stdin.buffer:
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            if not line:
                continue
            payload = (line + "\n").encode("utf-8")
            for attempt in range(25):
                try:
                    with io_lock:
                        remaining = payload
                        deadline = time.monotonic() + 2.0
                        while remaining and time.monotonic() < deadline:
                            written = jlink.rtt_write(0, remaining)
                            if written > 0:
                                remaining = remaining[written:]
                            else:
                                # The J-Link RTT down buffer can legitimately
                                # report zero space for a short startup or
                                # drain interval.  Treating that as success
                                # silently drops the command.
                                time.sleep(0.02)
                        if remaining:
                            raise TimeoutError(
                                f"RTT down buffer stalled with {len(remaining)} "
                                "bytes pending"
                            )
                    break
                except Exception as exc:  # noqa: BLE001 - report and retry
                    failures += 1
                    if attempt == 0:
                        print(
                            f"RTT write failed ({exc}); retrying",
                            file=sys.stderr,
                            flush=True,
                        )
                    time.sleep(0.02)
            else:
                print(
                    f"RTT write abandoned after {failures} failures",
                    file=sys.stderr,
                    flush=True,
                )

    try:
        jlink.open()
        jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
        jlink.connect(device, speed=speed)
        jlink.rtt_start()
        print("JLinkRttReader ready", file=sys.stderr, flush=True)
        if writable:
            threading.Thread(target=stdin_pump, daemon=True).start()
        while True:
            with io_lock:
                raw = bytes(jlink.rtt_read(0, 8192))
            if raw:
                sys.stdout.buffer.write(raw)
                sys.stdout.buffer.flush()
            else:
                time.sleep(0.002)
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"J-Link helper failed: {exc}", file=sys.stderr, flush=True)
        return 4
    finally:
        try:
            jlink.rtt_stop()
        except Exception:
            pass
        try:
            jlink.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
