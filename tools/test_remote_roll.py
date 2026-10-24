#!/usr/bin/env python3
"""Regression tests for local-SBUS right-stick roll control."""

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


class RemoteRollTest(unittest.TestCase):
    def test_defaults_match_the_existing_roll_target_range(self) -> None:
        self.assertAlmostEqual(_macro("VMC_ROLL_REMOTE_TARGET_MAX_DEFAULT"), 0.50)
        self.assertGreater(_macro("VMC_ROLL_REMOTE_DEADBAND_DEFAULT"), 0.0)

    def test_only_local_sbus_can_own_the_roll_axis(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_roll_target")
        for needle in (
            "remote_control_source == RC_SOURCE_SBUS",
            "remote_control_is_enabled()",
            "remote_control_channel(RC_ROLL)",
            "vmc_balance.roll_remote_active = active",
        ):
            with self.subTest(needle=needle):
                self.assertIn(needle, body)

    def test_right_stick_is_reversed_and_centre_is_zero(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_roll_target")
        self.assertIn("fabsf(input) <= deadband", body)
        self.assertIn("target = clamp_float(input * max_target", body)
        self.assertNotIn("target = clamp_float(-input * max_target", body)
        self.assertIn("vmc_control.roll_target = target", body)

    def test_remote_roll_is_updated_before_the_enable_early_returns(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_mode")
        call = body.index("vmc_update_remote_roll_target()")
        invalid_return = body.index("remote_control.valid == 0U")
        self.assertLess(call, invalid_return)

    def test_left_stick_still_drives_chassis_turn(self) -> None:
        body = _body(SOURCE.read_text(encoding="utf-8"), "vmc_update_remote_mode")
        self.assertIn("-remote_control_channel(RC_CHASSIS_X)", body)
        self.assertIn("turn_input * yaw_rate_max * dt", body)


if __name__ == "__main__":
    unittest.main()
