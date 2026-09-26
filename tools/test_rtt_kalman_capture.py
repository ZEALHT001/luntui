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
    KalmanSample,
    analyze_samples,
    analyze_continuity,
    analyze_window,
    capture_live,
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
