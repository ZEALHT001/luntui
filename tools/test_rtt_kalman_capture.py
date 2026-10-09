#!/usr/bin/env python3
"""Focused tests for the RTT Kalman capture parser and window checks."""

from __future__ import annotations

import csv
import io
import math
import tempfile
import unittest
from pathlib import Path

from tools.rtt_kalman_capture import (
    CSV_FIELDS,
    FORMAT_LEGACY_PROFILE,
    FORMAT_PROFILE,
    FORMAT_V12_PROFILE,
    FORMAT_V13_PROFILE,
    FORMAT_V14_PROFILE,
    FORMAT_V15_PROFILE,
    FORMAT_V16_PROFILE,
    KalmanSample,
    analyze_samples,
    analyze_continuity,
    analyze_window,
    capture_live,
    control_loop_rate_hz,
    iter_raw_text_samples,
    parse_vmcvoa_line,
    sliding_windows,
    steady_state_gains,
)


def format_line(**updates: float) -> str:
    fields = [0.001] * FORMAT_PROFILE.field_count
    fields[FORMAT_PROFILE.channels["kf_x"]] = updates.get("kf_x", 0.0)
    fields[FORMAT_PROFILE.channels["kf_dx"]] = updates.get("kf_dx", 0.0)
    # Q/R travel as plain decimal with nine fractional digits.
    fields[FORMAT_PROFILE.channels["q_v"]] = updates.get("q_v", 0.1)
    fields[FORMAT_PROFILE.channels["q_a"]] = updates.get("q_a", 0.1)
    fields[FORMAT_PROFILE.channels["r_wheel"]] = updates.get("r_wheel", 100.0)
    fields[FORMAT_PROFILE.channels["r_imu"]] = updates.get("r_imu", 1.0e12)
    fields[FORMAT_PROFILE.channels["schema_version"]] = updates.get(
        "schema_version", 3.0
    )
    fields[FORMAT_PROFILE.channels["mcu_time_ms"]] = updates.get(
        "mcu_time_ms", 1000.0
    )
    fields[FORMAT_PROFILE.channels["kf_wheel_measurement"]] = updates.get(
        "kf_wheel_measurement", 0.25
    )
    fields[FORMAT_PROFILE.channels["kf_accel_measurement"]] = updates.get(
        "kf_accel_measurement", -0.15
    )
    fields[FORMAT_PROFILE.channels["kf_innovation_v"]] = updates.get(
        "kf_innovation_v", 0.05
    )
    fields[FORMAT_PROFILE.channels["kf_innovation_a"]] = updates.get(
        "kf_innovation_a", -0.02
    )
    fields[FORMAT_PROFILE.channels["kf_slip"]] = updates.get("kf_slip", 0.0)
    fields[FORMAT_PROFILE.channels["kf_slip_metric"]] = updates.get(
        "kf_slip_metric", -0.1
    )
    fields[FORMAT_PROFILE.channels["gravity_sign"]] = updates.get(
        "gravity_sign", 1.0
    )
    fields[FORMAT_PROFILE.channels["slip_accel_threshold"]] = updates.get(
        "slip_accel_threshold", 20.0
    )
    fields[FORMAT_PROFILE.channels["slip_enable"]] = updates.get(
        "slip_enable", 1.0
    )
    fields[FORMAT_PROFILE.channels["schema_version"]] = updates.get(
        "schema_version", 5.0
    )
    fields[FORMAT_PROFILE.channels["tf02_distance_cm"]] = updates.get(
        "tf02_distance_cm", 91.0
    )
    fields[FORMAT_PROFILE.channels["tf02_connected"]] = updates.get(
        "tf02_connected", 1.0
    )
    fields[FORMAT_PROFILE.channels["roll_comp_offset"]] = updates.get(
        "roll_comp_offset", 0.0
    )
    fields[FORMAT_PROFILE.channels["leg_target_left"]] = updates.get(
        "leg_target_left", 0.28
    )
    fields[FORMAT_PROFILE.channels["leg_target_right"]] = updates.get(
        "leg_target_right", 0.28
    )
    fields[FORMAT_PROFILE.channels["leg_follow_offset_left"]] = updates.get(
        "leg_follow_offset_left", 0.0
    )
    fields[FORMAT_PROFILE.channels["leg_follow_offset_right"]] = updates.get(
        "leg_follow_offset_right", 0.0
    )
    fields[FORMAT_PROFILE.channels["leg_follow_scope_machine_wide"]] = updates.get(
        "leg_follow_scope_machine_wide", 0.0
    )
    fields[FORMAT_PROFILE.channels["leg_follow_episode_active"]] = updates.get(
        "leg_follow_episode_active", 0.0
    )
    fields[FORMAT_PROFILE.channels["leg_follow_scope_resolved"]] = updates.get(
        "leg_follow_scope_resolved", 0.0
    )
    fields[FORMAT_PROFILE.channels["roll"]] = updates.get("roll", 0.0)
    return "VMCVOFA," + ",".join(str(value) for value in fields)


def legacy_format_line(**updates: float) -> str:
    fields = [0.001] * FORMAT_LEGACY_PROFILE.field_count
    fields[FORMAT_LEGACY_PROFILE.channels["kf_x"]] = updates.get("kf_x", 0.0)
    fields[FORMAT_LEGACY_PROFILE.channels["kf_dx"]] = updates.get("kf_dx", 0.0)
    return "VMCVOFA," + ",".join(str(value) for value in fields)


class ParseTests(unittest.TestCase):
    def test_parse_format_line(self) -> None:
        parsed = parse_vmcvoa_line(
            format_line(kf_x=1.25, kf_dx=-0.75, q_v=2.0, r_imu=100.0)
        )
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format")
        self.assertAlmostEqual(sample.kf_x, 1.25)
        self.assertAlmostEqual(sample.kf_dx, -0.75)
        self.assertAlmostEqual(sample.q_v, 2.0)
        self.assertAlmostEqual(sample.r_imu, 100.0)
        self.assertAlmostEqual(sample.mcu_time_ms, 1000.0)
        self.assertAlmostEqual(sample.kf_wheel_measurement, 0.25)
        self.assertAlmostEqual(sample.kf_innovation_v, 0.05)
        self.assertAlmostEqual(sample.gravity_sign, 1.0)

    def test_previous_format_layout_still_parses(self) -> None:
        """The 106-field layout predates gravity_sign and must keep working."""
        fields = [0.001] * 106
        fields[4] = 1.5
        fields[5] = -0.25
        parsed = parse_vmcvoa_line("VMCVOFA," + ",".join(map(str, fields)))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format-v2")
        self.assertAlmostEqual(sample.kf_x, 1.5)
        self.assertFalse(math.isfinite(sample.gravity_sign))

    def test_slip_gate_channels_parse(self) -> None:
        line = format_line(schema_version=4.0)
        fields = line.split(",")
        fields[1 + FORMAT_PROFILE.channels["kf_slip_left"]] = "1.0"
        parsed = parse_vmcvoa_line(",".join(fields))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format")
        self.assertAlmostEqual(sample.schema_version, 4.0)
        self.assertAlmostEqual(sample.kf_slip_left, 1.0)

    def test_v7_layout_still_parses(self) -> None:
        """The 116-field layout (schema 3) predates the slip diagnostics."""
        fields = [0.001] * 116
        fields[4] = 1.0
        fields[115] = 3.0
        parsed = parse_vmcvoa_line("VMCVOFA," + ",".join(map(str, fields)))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format-v7")
        self.assertAlmostEqual(sample.schema_version, 3.0)
        self.assertFalse(math.isfinite(sample.kf_slip_left))

    def test_small_qr_values_survive_the_wire_format(self) -> None:
        """Calibrated Q/R are ~1e-7..1e-4 and must not round to zero."""
        parsed = parse_vmcvoa_line(
            format_line(
                q_v=4.038e-7,
                q_a=9.6353e-5,
                r_wheel=4.0209e-4,
                r_imu=3.8763e-2,
            )
        )
        self.assertIsNotNone(parsed)
        _profile, sample = parsed  # type: ignore[misc]
        self.assertAlmostEqual(sample.q_v, 4.038e-7, places=12)
        self.assertAlmostEqual(sample.q_a, 9.6353e-5, places=9)
        self.assertAlmostEqual(sample.r_wheel, 4.0209e-4, places=9)
        self.assertAlmostEqual(sample.r_imu, 3.8763e-2, places=8)
        self.assertNotEqual(sample.q_v, 0.0)

    def test_legacy_profile_still_parses(self) -> None:
        parsed = parse_vmcvoa_line(legacy_format_line(kf_x=0.5, kf_dx=0.2))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format-v1")
        self.assertFalse(math.isfinite(sample.mcu_time_ms))
        self.assertAlmostEqual(sample.kf_x, 0.5)

    def test_unknown_layout_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            parse_vmcvoa_line("VMCVOFA,1,2,3")

    def test_airborne_latch_channels_parse(self) -> None:
        """Schema 6 records the joint-load extremes behind the airborne latch."""
        line = format_line()
        fields = line.split(",")
        fields[1 + FORMAT_PROFILE.channels["airborne_joint_torque_min"]] = "0.4"
        fields[1 + FORMAT_PROFILE.channels["airborne_joint_torque_max"]] = "1.9"
        fields[1 + FORMAT_PROFILE.channels["airborne_pitch_reference"]] = "-0.03"
        fields[1 + FORMAT_PROFILE.channels["pitch_rate"]] = "0.75"
        parsed = parse_vmcvoa_line(",".join(fields))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format")
        self.assertAlmostEqual(sample.airborne_joint_torque_min, 0.4)
        self.assertAlmostEqual(sample.airborne_joint_torque_max, 1.9)
        self.assertAlmostEqual(sample.airborne_pitch_reference, -0.03)
        self.assertAlmostEqual(sample.pitch_rate, 0.75)

    def test_v9_layout_still_parses(self) -> None:
        """The 126-field layout (schema 5) predates the airborne-latch inputs."""
        fields = [0.001] * 126
        fields[4] = 1.0
        fields[115] = 5.0
        parsed = parse_vmcvoa_line("VMCVOFA," + ",".join(map(str, fields)))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format-v9")
        self.assertAlmostEqual(sample.schema_version, 5.0)
        self.assertFalse(math.isfinite(sample.airborne_joint_torque_min))

    def test_per_side_airborne_channels_parse(self) -> None:
        """Schema 7 adds the per-side airborne latch behind a single-leg lift."""
        line = format_line()
        fields = line.split(",")
        fields[1 + FORMAT_PROFILE.channels["airborne_side_torque_left"]] = "0.42"
        fields[1 + FORMAT_PROFILE.channels["airborne_side_torque_right"]] = "9.51"
        fields[1 + FORMAT_PROFILE.channels["airborne_side_active_left"]] = "1.0"
        fields[1 + FORMAT_PROFILE.channels["airborne_side_active_right"]] = "0.0"
        parsed = parse_vmcvoa_line(",".join(fields))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format")
        self.assertAlmostEqual(sample.airborne_side_torque_left, 0.42)
        self.assertAlmostEqual(sample.airborne_side_torque_right, 9.51)
        self.assertAlmostEqual(sample.airborne_side_active_left, 1.0)
        self.assertAlmostEqual(sample.airborne_side_active_right, 0.0)

    def test_v10_layout_still_parses(self) -> None:
        """The 130-field layout (schema 6) predates the per-side latch."""
        fields = [0.001] * 130
        fields[4] = 1.0
        fields[115] = 6.0
        parsed = parse_vmcvoa_line("VMCVOFA," + ",".join(map(str, fields)))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format-v10")
        self.assertAlmostEqual(sample.schema_version, 6.0)
        self.assertFalse(math.isfinite(sample.airborne_side_torque_left))

    def test_loop_timing_channels_parse(self) -> None:
        """Schema 8 records the measured control period and iteration count."""
        line = format_line()
        fields = line.split(",")
        fields[1 + FORMAT_PROFILE.channels["loop_dt_us"]] = "2207"
        fields[1 + FORMAT_PROFILE.channels["loop_count"]] = "123456"
        parsed = parse_vmcvoa_line(",".join(fields))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format")
        self.assertAlmostEqual(sample.loop_dt_us, 2207.0)
        self.assertAlmostEqual(sample.loop_count, 123456.0)

    def test_tf02_laser_channels_parse(self) -> None:
        """Schema 9 carries the laser range and its health flag.

        The automatic jump gate fires on the laser reading strictly inside
        (80, 100) cm, so a capture without these cannot show why a jump did
        not fire.
        """
        parsed = parse_vmcvoa_line(format_line(tf02_distance_cm=91.0,
                                               tf02_connected=1.0))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format")
        self.assertEqual(profile.field_count, FORMAT_PROFILE.field_count)
        self.assertAlmostEqual(sample.tf02_distance_cm, 91.0)
        self.assertAlmostEqual(sample.tf02_connected, 1.0)

    def test_v12_layout_still_parses(self) -> None:
        """The 136-field layout (schema 8) predates the laser fields."""
        fields = [0.001] * FORMAT_V12_PROFILE.field_count
        fields[FORMAT_V12_PROFILE.channels["kf_x"]] = 1.0
        fields[FORMAT_V12_PROFILE.channels["schema_version"]] = 8.0
        parsed = parse_vmcvoa_line("VMCVOFA," + ",".join(map(str, fields)))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format-v12")
        self.assertAlmostEqual(sample.schema_version, 8.0)
        # The laser channels do not exist on this layout.
        self.assertFalse(math.isfinite(sample.tf02_distance_cm))
        self.assertFalse(math.isfinite(sample.tf02_connected))

    def test_roll_compensation_channels_parse(self) -> None:
        """Schema 10 publishes the roll compensator's output and per-leg targets.

        Without these, "the compensator never ran", "the differential was
        cancelled" and "a target was clipped at VMC_LEG_LENGTH_MIN/MAX" all
        look the same, because field 77 is the COMMON target before the
        differential is added.
        """
        parsed = parse_vmcvoa_line(
            format_line(roll_comp_offset=-0.05,
                        leg_target_left=0.33,
                        leg_target_right=0.12)
        )
        self.assertIsNotNone(parsed)
        _profile, sample = parsed  # type: ignore[misc]
        self.assertAlmostEqual(sample.roll_comp_offset, -0.05)
        self.assertAlmostEqual(sample.leg_target_left, 0.33)
        # The short-leg clip is visible: the retract side sits on the minimum.
        self.assertAlmostEqual(sample.leg_target_right, 0.12)

    def test_v13_layout_still_parses(self) -> None:
        """The 138-field layout (schema 9) predates the roll-comp fields."""
        fields = [0.001] * FORMAT_V13_PROFILE.field_count
        fields[FORMAT_V13_PROFILE.channels["kf_x"]] = 1.0
        fields[FORMAT_V13_PROFILE.channels["tf02_distance_cm"]] = 91.0
        parsed = parse_vmcvoa_line("VMCVOFA," + ",".join(map(str, fields)))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format-v13")
        # The laser is present on this layout, the roll fields are not.
        self.assertAlmostEqual(sample.tf02_distance_cm, 91.0)
        self.assertFalse(math.isfinite(sample.roll_comp_offset))
        self.assertFalse(math.isfinite(sample.leg_target_left))

    def test_leg_follow_channels_parse(self) -> None:
        """Schema 11 carries the ground-following extension per side.

        The per-leg targets already include it; these fields are what separate
        "the operator moved S2" from "that leg lost the ground".
        """
        parsed = parse_vmcvoa_line(
            format_line(leg_follow_offset_left=0.0,
                        leg_follow_offset_right=0.055,
                        leg_target_left=0.28,
                        leg_target_right=0.335)
        )
        self.assertIsNotNone(parsed)
        _profile, sample = parsed  # type: ignore[misc]
        self.assertAlmostEqual(sample.leg_follow_offset_left, 0.0)
        self.assertAlmostEqual(sample.leg_follow_offset_right, 0.055)
        # The target already contains the extension.
        self.assertAlmostEqual(
            sample.leg_target_right - 0.28, 0.055, places=6
        )

    def test_leg_follow_scope_channels_parse(self) -> None:
        """Schema 12 adds the scope the follow feature resolved.

        The scope is chosen from the roll direction when a side unloads, so it
        is not a setting the operator dialled in - the only way to know which
        mode was active is to record it.
        """
        parsed = parse_vmcvoa_line(
            format_line(leg_follow_scope_machine_wide=1.0,
                        leg_follow_episode_active=1.0,
                        leg_follow_scope_resolved=1.0,
                        roll=0.12)
        )
        self.assertIsNotNone(parsed)
        _profile, sample = parsed  # type: ignore[misc]
        self.assertAlmostEqual(sample.leg_follow_scope_machine_wide, 1.0)
        self.assertAlmostEqual(sample.leg_follow_episode_active, 1.0)
        self.assertAlmostEqual(sample.leg_follow_scope_resolved, 1.0)
        self.assertAlmostEqual(sample.roll, 0.12)

    def test_scope_is_absent_on_a_schema_11_capture(self) -> None:
        """A pre-schema-12 line must still parse, with the scope unknown."""
        fields = [0.001] * 143
        for name, index in FORMAT_PROFILE.channels.items():
            if index >= 143:
                continue
            fields[index] = 0.0
        fields[FORMAT_PROFILE.channels["mcu_time_ms"]] = 1234.0
        parsed = parse_vmcvoa_line("VMCVOFA," + ",".join(map(str, fields)))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format-v15")
        self.assertFalse(math.isfinite(sample.leg_follow_scope_machine_wide))

    def test_resolution_flag_is_absent_on_a_schema_12_capture(self) -> None:
        """145 fields = schema 12: scope present, settled flag not yet.

        A capture from that layout must still parse; the flag reads invalid
        rather than silently defaulting to zero, because a pending scope also
        reads zero and the two must not be confused.
        """
        fields = [0.001] * FORMAT_V16_PROFILE.field_count
        for name, index in FORMAT_V16_PROFILE.channels.items():
            fields[index] = 0.0
        fields[FORMAT_V16_PROFILE.channels["mcu_time_ms"]] = 1234.0
        parsed = parse_vmcvoa_line("VMCVOFA," + ",".join(map(str, fields)))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format-v16")
        self.assertAlmostEqual(sample.leg_follow_scope_machine_wide, 0.0)
        self.assertFalse(math.isfinite(sample.leg_follow_scope_resolved))

    def test_v14_layout_still_parses(self) -> None:
        """The 141-field layout (schema 10) predates the leg-follow fields."""
        fields = [0.001] * FORMAT_V14_PROFILE.field_count
        fields[FORMAT_V14_PROFILE.channels["kf_x"]] = 1.0
        fields[FORMAT_V14_PROFILE.channels["roll_comp_offset"]] = -0.02
        parsed = parse_vmcvoa_line("VMCVOFA," + ",".join(map(str, fields)))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format-v14")
        self.assertAlmostEqual(sample.roll_comp_offset, -0.02)
        self.assertFalse(math.isfinite(sample.leg_follow_offset_left))

    def test_torque_channels_parse(self) -> None:
        """The Tp decomposition and wheel torque are recorded verbatim.

        These live in the preserved 0..98 block, so the same indices must
        resolve for both the current and the legacy layouts.  left_tp_raw vs
        left_tp_cmd is what shows whether the firmware clamps the command.
        """
        line = format_line()
        fields = line.split(",")
        fields[1 + FORMAT_PROFILE.channels["left_tp_raw"]] = "107.5"
        fields[1 + FORMAT_PROFILE.channels["left_tp_cmd"]] = "54.0"
        fields[1 + FORMAT_PROFILE.channels["left_tp_dx_term"]] = "-38.25"
        fields[1 + FORMAT_PROFILE.channels["right_tp_cmd"]] = "-54.0"
        fields[1 + FORMAT_PROFILE.channels["left_wheel_torque_raw"]] = "3.30"
        parsed = parse_vmcvoa_line(",".join(fields))
        self.assertIsNotNone(parsed)
        _profile, sample = parsed  # type: ignore[misc]
        self.assertAlmostEqual(sample.left_tp_raw, 107.5)
        self.assertAlmostEqual(sample.left_tp_cmd, 54.0)
        self.assertAlmostEqual(sample.left_tp_dx_term, -38.25)
        self.assertAlmostEqual(sample.right_tp_cmd, -54.0)
        self.assertAlmostEqual(sample.left_wheel_torque_raw, 3.30)

    def test_torque_channels_exist_in_legacy_layout(self) -> None:
        """Fields 0..98 are preserved across layouts, so v1 must resolve them."""
        legacy = FORMAT_LEGACY_PROFILE
        self.assertEqual(legacy.channels["left_tp_raw"], 32)
        self.assertEqual(legacy.channels["right_tp_cmd"], 41)

    def test_v11_layout_still_parses(self) -> None:
        """The 134-field layout (schema 7) predates the loop-timing fields."""
        fields = [0.001] * 134
        fields[4] = 1.0
        fields[115] = 7.0
        parsed = parse_vmcvoa_line("VMCVOFA," + ",".join(map(str, fields)))
        self.assertIsNotNone(parsed)
        profile, sample = parsed  # type: ignore[misc]
        self.assertEqual(profile.name, "format-v11")
        self.assertAlmostEqual(sample.schema_version, 7.0)
        self.assertFalse(math.isfinite(sample.loop_dt_us))

    def test_control_loop_rate_uses_firmware_timebase(self) -> None:
        """The loop rate comes from loop_count / MCU time, not host time.

        Host timestamps are quantised by RTT batching, so a rate derived from
        them would be wrong; the firmware counter is immune to that.
        """
        samples = [
            KalmanSample(
                host_time_s=0.0, mcu_time_ms=1000.0, kf_x=0.0, kf_dx=0.0,
                loop_count=0.0,
            ),
            KalmanSample(
                host_time_s=9.0, mcu_time_ms=2000.0, kf_x=1.0, kf_dx=1.0,
                loop_count=450.0,
            ),
        ]
        self.assertAlmostEqual(control_loop_rate_hz(samples), 450.0)

    def test_control_loop_rate_is_absent_on_old_layouts(self) -> None:
        samples = [
            KalmanSample(
                host_time_s=0.0, mcu_time_ms=1000.0, kf_x=0.0, kf_dx=0.0
            ),
            KalmanSample(
                host_time_s=1.0, mcu_time_ms=2000.0, kf_x=1.0, kf_dx=1.0
            ),
        ]
        self.assertFalse(math.isfinite(control_loop_rate_hz(samples)))

    def test_raw_text_replay_assigns_20ms_clock(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.log"
            path.write_text(
                "banner\n" + format_line(kf_x=0.0) + "\n"
                + format_line(kf_x=0.02, kf_dx=1.0) + "\n",
                encoding="utf-8",
            )
            with path.open(encoding="utf-8") as handle:
                samples = list(iter_raw_text_samples(handle, "format"))
        self.assertEqual(len(samples), 2)
        self.assertAlmostEqual(samples[1].host_time_s, 0.020)


class AnalysisTests(unittest.TestCase):
    def test_consistent_position_and_velocity_pass(self) -> None:
        samples = []
        for index in range(101):
            time_s = index * 0.020
            velocity = 0.2
            samples.append(
                KalmanSample(
                    host_time_s=time_s,
                    kf_x=velocity * time_s,
                    kf_dx=velocity,
                    q_v=0.1,
                    q_a=0.1,
                    r_wheel=100.0,
                    r_imu=100.0,
                )
            )
        metrics = analyze_window(samples)
        self.assertNotEqual(metrics.status, "FAIL")
        self.assertLess(metrics.integration_rmse, 1.0e-9)

    def test_position_reset_is_flagged(self) -> None:
        samples = []
        for index in range(101):
            time_s = index * 0.020
            position = 0.2 * time_s
            if index > 50:
                position += 1.0
            samples.append(
                KalmanSample(
                    host_time_s=time_s,
                    kf_x=position,
                    kf_dx=0.2,
                    q_v=0.1,
                    q_a=0.1,
                    r_wheel=100.0,
                    r_imu=100.0,
                )
            )
        metrics = analyze_window(samples)
        self.assertEqual(metrics.status, "FAIL")
        self.assertGreater(metrics.integration_max_error, 0.9)

    def test_imu_disabled_by_huge_r_is_flagged(self) -> None:
        samples = [
            KalmanSample(
                host_time_s=index * 0.020,
                kf_x=0.0,
                kf_dx=0.0,
                q_v=0.1,
                q_a=0.1,
                r_wheel=100.0,
                r_imu=1.0e12,
            )
            for index in range(101)
        ]
        metrics = analyze_window(samples)
        self.assertTrue(any("IMU" in finding for finding in metrics.findings))

    def test_sliding_window_hop(self) -> None:
        samples = [
            KalmanSample(
                host_time_s=index * 0.020,
                kf_x=0.0,
                kf_dx=0.0,
            )
            for index in range(151)
        ]
        windows = list(sliding_windows(samples, 2.0, 0.5))
        self.assertEqual(len(windows), 3)
        self.assertAlmostEqual(windows[1][0].host_time_s, 0.5, places=6)

    def test_gain_extremes(self) -> None:
        wheel_gain, imu_gain, *_rest = steady_state_gains(
            0.1, 0.1, 100.0, 1.0e12
        )
        self.assertGreater(wheel_gain, 0.0)
        self.assertLess(imu_gain, 1.0e-8)

    def test_accelerometer_gate_matches_firmware(self) -> None:
        """The IMU update runs at 200 Hz on a 1 kHz loop, not every cycle."""
        gated = steady_state_gains(1.1224e-7, 2.0721e-5, 1.1176e-4, 8.3363e-3)
        ungated = steady_state_gains(
            1.1224e-7, 2.0721e-5, 1.1176e-4, 8.3363e-3, imu_rate_hz=1000.0
        )
        # Reusing one frame for five 1 kHz updates would double the IMU weight.
        self.assertAlmostEqual(gated[1], 0.099995, places=5)
        self.assertAlmostEqual(ungated[1], 0.048370, places=5)
        self.assertGreater(gated[1], ungated[1] * 1.9)
        # The IMU time constant is measured against its own 5 ms period, not
        # against the 1 ms control period.
        expected_tau = -5.0 / math.log1p(-gated[1])
        self.assertAlmostEqual(gated[3], expected_tau, places=6)
        self.assertAlmostEqual(gated[3], 47.46, places=1)

    def test_mcu_timestamp_drives_window_timing(self) -> None:
        samples = [
            KalmanSample(
                host_time_s=10.0,
                mcu_time_ms=1000.0 + index * 20.0,
                kf_x=index * 0.004,
                kf_dx=0.2,
            )
            for index in range(101)
        ]
        metrics = analyze_window(samples)
        self.assertAlmostEqual(metrics.sample_rate_hz, 50.0, places=6)
        self.assertLess(metrics.integration_rmse, 1.0e-9)

    def test_outage_does_not_lower_median_sample_rate(self) -> None:
        samples = []
        for index in range(301):
            mcu_ms = index * 20.0
            if index >= 150:
                mcu_ms += 10000.0
            samples.append(
                KalmanSample(
                    host_time_s=index * 0.020,
                    mcu_time_ms=mcu_ms,
                    kf_x=0.0,
                    kf_dx=0.0,
                )
            )
        metrics = analyze_window(samples)
        self.assertAlmostEqual(metrics.sample_rate_hz, 50.0, places=6)
        self.assertGreater(metrics.max_gap_ms, 9000.0)

        findings = analyze_continuity(samples)
        self.assertTrue(any(finding.startswith("FAIL:") for finding in findings))

    def test_early_stop_keeps_partial_capture(self) -> None:
        class FakeReader:
            def __init__(self) -> None:
                self.lines = [
                    format_line(kf_x=0.0, mcu_time_ms=1000),
                    format_line(kf_x=0.0, mcu_time_ms=1020),
                    format_line(kf_x=0.0, mcu_time_ms=1040),
                ]

            def read_lines(self) -> list[str]:
                return [self.lines.pop(0)] if self.lines else []

        stop_polls = [0]

        def stop_requested() -> bool:
            stop_polls[0] += 1
            return stop_polls[0] >= 3

        csv_handle = io.StringIO()
        writer = csv.DictWriter(csv_handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        profile, samples, _windows, ended_early = capture_live(
            FakeReader(),
            duration_s=10.0,
            window_s=5.0,
            hop_s=1.0,
            startup_stabilize_s=0.0,
            rx_timeout_s=10.0,
            stop_requested=stop_requested,
            console=None,
            scheduled_sends=None,
            requested_profile="format",
            csv_handle=csv_handle,
            csv_writer=writer,
        )
        self.assertIsNotNone(profile)
        self.assertEqual(len(samples), 2)
        self.assertTrue(ended_early)

    def test_scheduled_sends_are_issued(self) -> None:
        class FakeReader:
            def __init__(self) -> None:
                self.sent: list[str] = []
                self.n = 0

            def read_lines(self) -> list[str]:
                self.n += 1
                return [format_line(kf_x=0.0)] if self.n < 4 else []

            def send(self, line: str) -> None:
                self.sent.append(line)

        reader = FakeReader()
        csv_handle = io.StringIO()
        writer = csv.DictWriter(csv_handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        capture_live(
            reader,
            duration_s=0.05,
            window_s=5.0,
            hop_s=1.0,
            startup_stabilize_s=0.0,
            rx_timeout_s=10.0,
            stop_requested=None,
            console=None,
            scheduled_sends=[(0.0, "FIRST"), (0.0, "SECOND")],
            requested_profile="format",
            csv_handle=csv_handle,
            csv_writer=writer,
        )
        self.assertEqual(reader.sent, ["FIRST", "SECOND"])


if __name__ == "__main__":
    unittest.main()
