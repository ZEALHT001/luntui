#!/usr/bin/env python3
"""Regression tests for continuous world-direction projection."""

from __future__ import annotations

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
    def test_default_projection_power_is_five(self) -> None:
        self.assertEqual(_macro("VMC_SPIN_PROJECTION_POWER_DEFAULT"), 5.0)

    def test_spin_entry_captures_world_yaw(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_mode")
        self.assertIn("spin_enabled != vmc_spin_mode_last", body)
        self.assertIn("vmc_balance.spin_line_origin_yaw = vmc_probe_state.body.yaw", body)
        self.assertIn("vmc_probe_state.body.yaw + (float)path_offset_rad", body)
        self.assertIn("vmc_balance.spin_line_heading_valid = imu_ready", body)

    def test_projection_is_continuous_signed_power_of_cosine(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        projection = _body(source, "vmc_spin_projection_gain")
        for needle in (
            "phase_error + phase_lead",
            "cos(phase)",
            "pow(cosine, exponent)",
            "-pow(-cosine, exponent)",
            "clamp_double(power, 1.0, 8.0)",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, projection)

    def test_phase_lead_increases_with_spin_rate(self) -> None:
        lead = _body(
            SOURCE.read_text(encoding="utf-8"), "vmc_spin_phase_lead_rad"
        )
        for needle in (
            "omega <= 1.0",
            "lead_deg = 0.0",
            "11.0 * (omega - 1.0)",
            "11.0 + 9.0 * (omega - 2.0)",
            "20.0 + 5.0 * (omega - 3.0)",
            "25.0 + 5.0 * (omega - 4.0)",
            "30.0 + 3.0 * (omega - 5.0)",
            "33.0 + 2.0 * (omega - 6.0)",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, lead)

    def test_remote_mode_uses_projection_not_window_gates(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        body = _body(source, "vmc_update_remote_mode")
        for needle in (
            "vmc_spin_phase_lead_rad((double)vmc_balance.yaw_spin_rate)",
            "vmc_spin_projection_gain(",
            "(double)vmc_balance.spin_line_projection_power",
            "(double)vmc_balance.spin_line_gate);",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, body)
        self.assertNotIn("spin_line_window_rad", source)
        self.assertNotIn("spin_line_reverse_gate", source)

    def test_left_stick_vector_selects_any_world_travel_direction(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_mode")
        for needle in (
            "remote_control_channel(RC_CHASSIS_Y)",
            "-remote_control_channel(RC_CHASSIS_X)",
            "sqrtf(spin_forward_input * spin_forward_input",
            "atan2f(spin_left_input, spin_forward_input)",
            "vmc_balance.spin_line_origin_yaw + direction_offset",
            "(float)path_offset_rad",
            "spin_input_magnitude * RC_CHANNAL_ERROR_VALUE",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, body)

    def test_default_path_offset_compensates_left_bias(self) -> None:
        self.assertEqual(_macro("VMC_SPIN_LINE_PATH_OFFSET_DEG_DEFAULT"), -75.0)
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_mode")
        self.assertIn("vmc_balance.spin_line_path_offset_deg", body)
        self.assertIn("clamp_float(vmc_balance.spin_line_path_offset_deg", body)

    def test_projection_is_open_when_spin_is_disabled(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_mode")
        self.assertIn("vmc_balance.spin_line_gate = 1.0f;", body)


if __name__ == "__main__":
    unittest.main()
