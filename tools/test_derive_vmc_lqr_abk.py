#!/usr/bin/env python3
"""Tests for the geared-rotor inertia terms in the wheel-leg model.

The M3508 rotor is geared to the wheel and its stator is bolted to the leg,
so its inertia loads both the wheel term (n^2*Jr) and the leg term
((n-1)^2*Jr).  These tests pin the reflection down and lock the historical
behaviour when the term is disabled.
"""

from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path
import re

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from derive_vmc_lqr_abk import (  # noqa: E402
    PRESETS,
    drivetrain_inertias,
    fit_schedule,
    k_at_length,
    linearize,
)

ROOT = Path(__file__).resolve().parents[1]
LQR_C = ROOT / "App" / "Src" / "vmc_lqr.c"

_K_LINE = re.compile(
    r"\s*k\[(\d+)\] = \(\(L_0 \* ([-\d.eE+]+) \+ t2 \* ([-\d.eE+]+)\)"
    r" \+ t3 \* ([-\d.eE+]+)\) \+ ([-\d.eE+]+);"
)


def _shipped_coefficients(path: Path) -> np.ndarray:
    """Parse the per-gain cubic coefficients out of a generated vmc_lqr.c."""
    found: dict[int, list[float]] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        match = _K_LINE.match(line)
        if match:
            # np.polyval wants descending powers: [t3, t2, L_0, constant].
            found[int(match.group(1))] = [
                float(match.group(i)) for i in (4, 3, 2, 5)
            ]
    if len(found) != 12:
        raise AssertionError(f"expected 12 gains in {path}, found {len(found)}")
    return np.array([found[index] for index in range(12)])


def _metadata_float(source: str, key: str) -> float:
    match = re.search(rf"{key}\s*:\s*([-\d.eE+]+)", source)
    if match is None:
        raise AssertionError(f"metadata key {key!r} missing from vmc_lqr.c")
    return float(match.group(1))

GEAR_RATIO = 268.0 / 17.0
"""Rotor-to-wheel reduction ratio used by the firmware telemetry scaling."""

ROTOR_INERTIA = 2.5e-5
"""M3508 rotor inertia, kg.m^2, from the free-wheel bench spin-up."""


class RotorInertiaTests(unittest.TestCase):
    def test_abk_preset_carries_the_measured_drivetrain(self) -> None:
        """The shipped abk preset must include the measured geared rotor."""
        p = PRESETS["abk"]
        self.assertEqual(p.gear_ratio, GEAR_RATIO)
        self.assertGreaterEqual(p.rotor_inertia, 1.7e-5)
        self.assertLessEqual(p.rotor_inertia, 2.5e-5)

    def test_disabling_the_term_reproduces_the_old_model(self) -> None:
        """rotor_inertia=0 must still give the historical 2.47 Hz leg mode."""
        p = replace(PRESETS["abk"], rotor_inertia=0.0, gear_ratio=1.0)
        a, _b = linearize(p, 0.199)
        self.assertAlmostEqual(a[1, 0], 240.883775, places=4)

    def test_rotor_loads_wheel_and_leg(self) -> None:
        p = replace(
            PRESETS["abk"], rotor_inertia=ROTOR_INERTIA, gear_ratio=GEAR_RATIO
        )
        wheel, leg = drivetrain_inertias(p)
        self.assertAlmostEqual(
            wheel, 7.2e-4 + GEAR_RATIO**2 * ROTOR_INERTIA, places=12
        )
        self.assertAlmostEqual(
            leg,
            PRESETS["abk"].leg_inertia + (GEAR_RATIO - 1.0) ** 2 * ROTOR_INERTIA,
            places=12,
        )
        # The wheel term is the big one; the leg term is a few percent.
        self.assertGreater(wheel / PRESETS["abk"].wheel_inertia, 9.0)
        self.assertLess((leg - PRESETS["abk"].leg_inertia) / leg, 0.08)

    def test_leg_rate_falls_when_the_wheel_gets_heavier(self) -> None:
        """A wheel that resists rolling slows the leg-fall mode."""
        base = replace(PRESETS["abk"], rotor_inertia=0.0, gear_ratio=1.0)
        heavy = PRESETS["abk"]
        a0, b0 = linearize(base, 0.199)
        a1, b1 = linearize(heavy, 0.199)
        self.assertLess(a1[1, 0], a0[1, 0])
        # Wheel torque becomes a weaker pitch/x input, not a stronger one.
        self.assertLess(abs(b1[1, 0]), abs(b0[1, 0]))
        self.assertLess(abs(b1[3, 0]), abs(b0[3, 0]))

    def test_bench_free_wheel_inertia_is_reproduced(self) -> None:
        """The bench spin-up measured 0.0049-0.0068 kg.m^2 at the wheel."""
        wheel, _leg = drivetrain_inertias(PRESETS["abk"])
        self.assertGreater(wheel, 0.0045)
        self.assertLess(wheel, 0.0070)

    def test_gear_ratio_one_is_a_no_op_for_the_wheel_term(self) -> None:
        """With no gearing the rotor is just extra wheel inertia."""
        p = replace(PRESETS["abk"], rotor_inertia=ROTOR_INERTIA, gear_ratio=1.0)
        wheel, leg = drivetrain_inertias(p)
        self.assertAlmostEqual(wheel, PRESETS["abk"].wheel_inertia + ROTOR_INERTIA)
        self.assertAlmostEqual(leg, PRESETS["abk"].leg_inertia)

    def test_gains_stay_close_to_the_shipped_schedule(self) -> None:
        """Enabling the term must not wildly reshape the LQR gains."""
        base = replace(PRESETS["abk"], rotor_inertia=0.0, gear_ratio=1.0)
        heavy = PRESETS["abk"]
        k0 = k_at_length(base, 0.388)
        k1 = k_at_length(heavy, 0.388)
        relative = np.abs(k1 - k0) / np.abs(k0)
        self.assertLess(relative.max(), 0.20)


class ViscousDragTests(unittest.TestCase):
    """Viscous rolling drag lands on A[3,3] and nothing else."""

    def test_viscous_drag_only_adds_x_damping(self) -> None:
        base = replace(PRESETS["abk"], wheel_viscous=0.0)
        damped = PRESETS["abk"]
        self.assertGreater(damped.wheel_viscous, 0.0)
        for leg in (0.205, 0.280, 0.388):
            a0, b0 = linearize(base, leg)
            a1, b1 = linearize(damped, leg)
            # Drag enters through x_dot, so only column 3 changes.  It shows up
            # in the theta, x and phi acceleration rows because the wheel
            # reaction is what couples those: drag is a real torque on the
            # machine, not just a damper on one state.
            for row in (1, 3, 5):
                self.assertNotAlmostEqual(a1[row, 3], a0[row, 3], places=6)
            self.assertLess(a1[3, 3], 0.0)
            self.assertAlmostEqual(a1[3, 3], -0.7, delta=0.35)
            # Nothing else in A moves, and the input matrix is untouched.
            for row in range(6):
                for col in range(6):
                    if col == 3:
                        continue
                    self.assertAlmostEqual(a1[row, col], a0[row, col], places=9)
            np.testing.assert_allclose(b1, b0, atol=1e-12)

    def test_viscous_removes_energy_from_the_plant(self) -> None:
        """Drag must not raise the real part of any open-loop pole."""
        base = replace(PRESETS["abk"], wheel_viscous=0.0)
        damped = PRESETS["abk"]
        for leg in (0.205, 0.280, 0.388):
            a0, _b = linearize(base, leg)
            a1, _b = linearize(damped, leg)
            p0 = np.linalg.eigvals(a0)
            p1 = np.linalg.eigvals(a1)
            self.assertLessEqual(p1.real.max(), p0.real.max() + 1e-9)


class GeneratedScheduleTests(unittest.TestCase):
    """The shipped vmc_lqr.c must be reproducible from its own metadata."""

    def test_shipped_schedule_regenerates_exactly(self) -> None:
        source = LQR_C.read_text(encoding="utf-8")
        match = re.search(
            r"VMC_LQR_TUNER_Q\s*[:=]\s*\[([^\]]*)\]", source, re.IGNORECASE
        )
        self.assertIsNotNone(match, "Q metadata missing from vmc_lqr.c")
        q = [float(part) for part in match.group(1).split(",")]
        match = re.search(
            r"VMC_LQR_TUNER_R\s*[:=]\s*\[([^\]]*)\]", source, re.IGNORECASE
        )
        self.assertIsNotNone(match, "R metadata missing from vmc_lqr.c")
        r = [float(part) for part in match.group(1).split(",")]

        preset = replace(
            PRESETS["abk"],
            q=np.diag(q),
            r=np.diag(r),
            body_com_to_hip=_metadata_float(source, "body_com_to_hip"),
            gear_ratio=_metadata_float(source, "gear_ratio"),
            rotor_inertia=_metadata_float(source, "rotor_inertia"),
            wheel_viscous=_metadata_float(source, "wheel viscous"),
        )
        _lengths, _gains, coeffs = fit_schedule(preset, 0.03, 0.50, 0.005, 3)
        shipped = _shipped_coefficients(LQR_C)
        # Compare relatively, with an absolute floor for near-zero terms.
        tol = 1.0e-9 + 1.0e-6 * np.abs(shipped)
        worst = np.max(np.abs(coeffs - shipped) / tol)
        self.assertLess(
            worst,
            1.0,
            f"regenerated schedule does not match App/Src/vmc_lqr.c "
            f"(worst normalised error {worst:.3g})",
        )


if __name__ == "__main__":
    unittest.main()
