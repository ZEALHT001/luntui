#!/usr/bin/env python3
"""Regression tests for world-direction gating during small-gyro motion."""

from __future__ import annotations

import math
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "App" / "Inc" / "vmc_probe.h"
SOURCE = ROOT / "App" / "Src" / "vmc_probe.c"


def _macro(name: str) -> float:
    source = HEADER.read_text(encoding="utf-8")
    match = re.search(rf"^#define\s+{name}\s+(-?[0-9.]+)[fU]*", source, re.MULTILINE)
    if match is None:
        raise AssertionError(f"macro {name} not found")
    return float(match.group(1))


def _body(source: str, name: str) -> str:
    pattern = re.compile(
        rf"^(?:static\s+)?\w+\s+{re.escape(name)}\s*\([^;{{]*\)\s*\{{",
        re.MULTILINE,
    )
    match = pattern.search(source)
    if match is None:
        raise AssertionError(f"definition of {name} not found")
    start = match.end() - 1
    depth = 0
    for index in range(start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start:index + 1]
    raise AssertionError(f"unterminated body for {name}")


class SpinLineTest(unittest.TestCase):
    def test_default_window_is_twenty_five_degrees(self) -> None:
        self.assertAlmostEqual(
            _macro("VMC_SPIN_LINE_WINDOW_RAD_DEFAULT"),
            math.radians(25.0),
            places=6,
        )

    def test_spin_entry_captures_world_yaw(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_mode")
        self.assertIn("spin_enabled != vmc_spin_mode_last", body)
        self.assertIn("vmc_balance.spin_line_yaw = vmc_probe_state.body.yaw", body)
        self.assertIn("vmc_balance.spin_line_heading_valid = imu_ready", body)

    def test_forward_is_gated_by_wrapped_heading_error(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        body = _body(source, "vmc_update_remote_mode")
        gate = _body(source, "vmc_spin_line_gate")
        self.assertIn("vmc_rad_format(", body)
        self.assertIn("vmc_spin_line_gate(phase_error, window)", body)
        self.assertIn("cos(VMC_PI * phase_error / window_rad)", gate)
        self.assertIn("(double)vmc_balance.spin_line_gate);", body)
        self.assertIn("forward_speed_scale", body)

    def test_opposite_heading_uses_reverse_drive_for_the_same_world_direction(
        self,
    ) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_mode")
        for needle in (
            "phase_error - (double)VMC_PI",
            "vmc_spin_line_gate(reverse_phase_error, window)",
            "forward_gate - reverse_gate",
            "spin_line_reverse_gate",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, body)

    def test_left_stick_vector_selects_any_world_travel_direction(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_mode")
        for needle in (
            "remote_control_channel(RC_CHASSIS_Y)",
            "-remote_control_channel(RC_CHASSIS_X)",
            "sqrtf(spin_forward_input * spin_forward_input",
            "atan2f(spin_left_input, spin_forward_input)",
            "vmc_balance.spin_line_origin_yaw + direction_offset",
            "spin_input_magnitude * RC_CHANNAL_ERROR_VALUE",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, body)

    def test_window_expands_to_preserve_minimum_open_time(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_mode")
        self.assertEqual(_macro("VMC_SPIN_LINE_MIN_OPEN_MS_DEFAULT"), 200.0)
        for needle in (
            "vmc_balance.spin_line_min_open_ms",
            "adaptive_window",
            "0.5 * (double)vmc_balance.yaw_spin_rate",
            "vmc_balance.spin_line_effective_window_rad",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, body)

    def test_gate_is_open_when_spin_is_disabled(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_mode")
        self.assertIn("vmc_balance.spin_line_gate = 1.0f;", body)


if __name__ == "__main__":
    unittest.main()
