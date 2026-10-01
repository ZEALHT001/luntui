#!/usr/bin/env python3
"""Regression tests for firmware-default serialization."""

from __future__ import annotations

import re
import unittest
from pathlib import Path

from tools.lqr_tuner.server import (
    BALANCE_PARAM_DEFAULT_MACROS,
    _format_c_literal,
    _read_firmware_defaults,
)

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "App" / "Inc" / "vmc_probe.h"


def _macro(name: str) -> float:
    source = HEADER.read_text(encoding="utf-8")
    match = re.search(rf"^#define\s+{name}\s+([0-9.]+)f?", source, re.MULTILINE)
    if match is None:
        raise AssertionError(f"macro {name} not found as a numeric literal")
    return float(match.group(1))


class FirmwareDefaultFormattingTest(unittest.TestCase):
    def test_millisecond_defaults_keep_numeric_values(self) -> None:
        for key, (_macro, value_type) in BALANCE_PARAM_DEFAULT_MACROS.items():
            if not key.endswith("_ms"):
                continue
            with self.subTest(key=key):
                self.assertEqual(value_type, "uint_value")
                self.assertEqual(_format_c_literal(60.0, value_type), "60U")
                self.assertEqual(_format_c_literal(200.0, value_type), "200U")

    def test_disturbance_observer_defaults_match_vehicle_tuning(self) -> None:
        """The wheel-error integrator was re-tuned on the 60 mm wheels.

        gain/leak sets the steady-state authority and 1/leak the response
        time, so both numbers have to change together.  Raise either one only
        after a long standstill capture - 0.28/0.06 already produced a growing
        slow limit cycle.
        """
        defaults = _read_firmware_defaults()["balance_params"]
        self.assertEqual(defaults["disturbance_observer_gain"], 0.2)
        self.assertEqual(defaults["disturbance_observer_leak"], 0.06)

    def test_leg_torque_domains_stay_consistent(self) -> None:
        """Tp is a virtual-leg torque; each joint only sees half of it.

        The firmware used to clamp the virtual-leg limit against the per-joint
        constant, which silently capped every motor at 27 N.m.  Keep the two
        domains tied to the 2x relationship so that cannot come back.
        """
        joint_max = _macro("VMC_DM_J8009_TMAX")
        leg_max = _macro("VMC_LEG_TP_MAX")
        self.assertAlmostEqual(leg_max, 2.0 * joint_max)

        self.assertAlmostEqual(_macro("VMC_BALANCE_TP_LIMIT_DEFAULT"), leg_max)
        self.assertAlmostEqual(_macro("VMC_LEG_PHI0_TORQUE_MAX_DEFAULT"), leg_max)

        defaults = _read_firmware_defaults()["balance_params"]
        self.assertAlmostEqual(defaults["tp_limit"], leg_max)
        self.assertAlmostEqual(defaults["max_phi0_torque"], leg_max)


if __name__ == "__main__":
    unittest.main()
