#!/usr/bin/env python3
"""Regression tests for knob-controlled small-gyro spin speed."""

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


class SpinSpeedTest(unittest.TestCase):
    def test_range_is_zero_to_nine_rad_per_second(self) -> None:
        self.assertEqual(_macro("VMC_RC_YAW_SPIN_RATE_MIN_DEFAULT"), 0.0)
        self.assertEqual(_macro("VMC_RC_YAW_SPIN_RATE_MAX_DEFAULT"), 9.0)

    def test_channel_eight_knob_scales_spin_speed(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_mode")
        for needle in (
            "remote_control_spin_rate_scale()",
            "vmc_balance.yaw_spin_rate =",
            "VMC_RC_YAW_SPIN_RATE_MIN_DEFAULT",
            "VMC_RC_YAW_SPIN_RATE_MAX_DEFAULT",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, body)

    def test_spin_rate_is_clamped_to_zero_and_nine(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_mode")
        self.assertIn("clamp_double(vmc_balance.yaw_spin_rate_max, 0.0, 9.0)", body)
        self.assertIn("clamp_double(", body)
        self.assertIn("vmc_balance.yaw_spin_rate,", body)
        self.assertNotIn("VMC_RC_YAW_SPIN_RATE_DEFAULT", body)

    def test_spin_mode_uses_stick_speed_but_normal_mode_uses_knob(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_mode")
        self.assertIn("forward_speed_scale =", body)
        self.assertIn("(spin_enabled != 0U) ? 1.0 : knob_scale", body)
        self.assertIn("forward_current * forward_speed_scale", body)

    def test_spin_mode_uses_its_own_four_point_five_ramp(self) -> None:
        self.assertEqual(
            _macro("VMC_BALANCE_SPIN_SPEED_RAMP_RATE_DEFAULT"), 4.5
        )
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_balance_update")
        self.assertIn("vmc_balance.spin_mode_enabled != 0U", body)
        self.assertIn("vmc_balance.spin_speed_ramp_rate", body)
        self.assertIn("active_speed_ramp_rate * dt", body)


if __name__ == "__main__":
    unittest.main()
