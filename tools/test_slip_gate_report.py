#!/usr/bin/env python3
"""Tests for the slip-gate validation tool."""

from __future__ import annotations

import csv
import json
import tempfile
import unittest
from pathlib import Path

from tools.slip_gate_report import (
    Capture,
    analyse,
    find_slip_windows,
    verdict,
)

DT_MS = 4.0
FAST_FIELDS = [
    "t_ms",
    "enabled",
    "slip_left",
    "slip_right",
    "kf_dx",
    "body_ay_lin_g",
    "wheel_left_speed",
    "wheel_right_speed",
    "leg_left_phi0",
    "leg_right_phi0",
    "leg_left_d_phi0",
    "leg_right_d_phi0",
    "leg_left_length",
    "leg_right_length",
    "leg_left_d_length",
    "leg_right_d_length",
    "body_pitch",
    "body_pitch_rate",
]


SLIP_START = 400
SLIP_FRAMES = 25
GRIP_START = 25
GRIP_FRAMES = 25


def _ramp(
    velocity: list[float], start: int, frames: int, from_v: float, to_v: float
) -> None:
    """Ramp the wheel velocity from ``from_v`` to ``to_v`` over the window."""
    for step in range(frames):
        velocity[start + step] = from_v + (to_v - from_v) * (step + 1) / frames


def _write_capture(
    root: Path,
    name: str,
    gate: bool,
    *,
    kf_mode: str,
    slip_excursion: float = 1.2,
    grip_ramp: float = 0.2,
    grip_accel_agrees: bool = True,
) -> Path:
    """Write a synthetic capture with a known answer.

    Two stretches are generated.  A *grip* ramp moves the wheel and the
    accelerometer reports the matching acceleration, which is what a wheel
    that is gripping looks like.  A *slip* ramp then moves the wheel by
    ``slip_excursion`` while the accelerometer stays flat, which is the lie the
    gate exists to reject.

    ``kf_mode`` picks what the filter does: ``"wheel"`` copies its measurement
    (an open gate), ``"accel"`` follows only the accelerometer (a gate that
    rejects the wheel).  Every quantity the tool reconstructs is generated from
    this, so the expected verdict is known exactly.
    """
    path = root / name
    path.mkdir(parents=True)

    count = 900
    wheel = [0.0] * count
    _ramp(wheel, GRIP_START, GRIP_FRAMES, 0.0, grip_ramp)
    for i in range(GRIP_START + GRIP_FRAMES, count):
        wheel[i] = grip_ramp
    _ramp(wheel, SLIP_START, SLIP_FRAMES, grip_ramp,
          grip_ramp + slip_excursion)
    for i in range(SLIP_START + SLIP_FRAMES, count):
        wheel[i] = grip_ramp + slip_excursion

    accel = [0.0] * count
    if grip_accel_agrees:
        for i in range(GRIP_START + 1, GRIP_START + GRIP_FRAMES):
            accel[i] = (wheel[i] - wheel[i - 1]) / (DT_MS / 1000.0)

    kf = [0.0] * count
    running = 0.0
    for i in range(1, count):
        running += accel[i] * (DT_MS / 1000.0)
        if kf_mode == "wheel":
            kf[i] = wheel[i]
        elif kf_mode == "accel":
            kf[i] = running
        elif kf_mode == "frozen":
            kf[i] = 0.0
        else:
            raise ValueError(f"unknown kf_mode {kf_mode}")

    rows = []
    for i in range(count):
        in_slip = SLIP_START <= i < SLIP_START + SLIP_FRAMES
        rows.append(
            {
                "t_ms": f"{i * DT_MS:.1f}",
                "enabled": "1",
                "slip_left": "1" if in_slip else "0",
                "slip_right": "0",
                "kf_dx": f"{kf[i]:.6f}",
                # The fore-aft channel carries g and the firmware negates it:
                # dd_x = -body_ay_lin_g * 9.80665, so store -accel to make the
                # reconstructed dd_x equal the intended acceleration.
                "body_ay_lin_g": f"{-accel[i] / 9.80665:.6f}",
                # d_x_raw = (-wL - wR) * R / 2, so wL = -2E/R, wR = 0
                # reproduces a body velocity of exactly E.
                "wheel_left_speed": f"{-2.0 * wheel[i] / 0.060:.6f}",
                "wheel_right_speed": "0.000000",
                "leg_left_phi0": "1.5708",
                "leg_right_phi0": "1.5708",
                "leg_left_d_phi0": "0",
                "leg_right_d_phi0": "0",
                "leg_left_length": "0.2",
                "leg_right_length": "0.2",
                "leg_left_d_length": "0",
                "leg_right_d_length": "0",
                "body_pitch": "0",
                "body_pitch_rate": "0",
            }
        )
    with (path / "fast_samples.csv").open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=FAST_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    with (path / "kalman_samples.csv").open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=["slip_enable"])
        writer.writeheader()
        writer.writerow({"slip_enable": "1" if gate else "0"})
    (path / "summary.json").write_text(
        json.dumps({"slip_applied": {"gate_active": 1.0 if gate else 0.0}}),
        encoding="utf-8",
    )
    return path


class WindowTest(unittest.TestCase):
    def test_windows_are_the_latch_runs(self) -> None:
        rows = [{"slip_left": "0", "slip_right": "0", "enabled": "1"}] * 900
        for i in range(300, 325):
            rows[i] = {"slip_left": "1", "slip_right": "0", "enabled": "1"}
            rows[i]["t_ms"] = str(i * 4)
        for i, row in enumerate(rows):
            row.setdefault("t_ms", str(i * 4))
        windows = find_slip_windows(rows)
        self.assertEqual(windows, [(300, 324)])

    def test_windows_ignore_disabled_samples(self) -> None:
        rows = [
            {"slip_left": "1", "slip_right": "0", "enabled": "0", "t_ms": str(i)}
            for i in range(50)
        ]
        rows += [
            {"slip_left": "0", "slip_right": "0", "enabled": "1", "t_ms": str(i)}
            for i in range(50, 100)
        ]
        self.assertEqual(find_slip_windows(rows), [])


class GateStateTest(unittest.TestCase):
    def test_gate_state_comes_from_the_slow_stream(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            off = _write_capture(root, "off", False, kf_mode="wheel")
            on = _write_capture(root, "on", True, kf_mode="accel")
            self.assertIs(Capture(off).gate, False)
            self.assertIs(Capture(on).gate, True)

    def test_a_gate_that_changes_mid_run_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            path = _write_capture(root, "mixed", False, kf_mode="wheel")
            with (path / "kalman_samples.csv").open(
                "w", newline="", encoding="utf-8"
            ) as fh:
                writer = csv.DictWriter(fh, fieldnames=["slip_enable"])
                writer.writeheader()
                writer.writerow({"slip_enable": "0"})
                writer.writerow({"slip_enable": "1"})
            with self.assertRaises(ValueError):
                Capture(path)


class MetricsTest(unittest.TestCase):
    def test_a_filter_that_follows_the_wheel_shows_the_wheel_error(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = _write_capture(
                Path(tmp), "off", False, kf_mode="wheel",
                slip_excursion=1.2,
            )
            result = analyse(Capture(path))
            events = result["events"]
            self.assertEqual(len(events), 1)
            # The wheel lies by the whole excursion and the filter copies it.
            # The latch turns on one ramp step in, so the window sees 24/25 of
            # the excursion rather than all of it.
            self.assertAlmostEqual(events[0]["dv_wheel"], 1.2 * 24 / 25, delta=1e-3)
            self.assertAlmostEqual(events[0]["dv_kf"], 1.2 * 24 / 25, delta=1e-3)
            self.assertAlmostEqual(events[0]["kf_error"], 1.2 * 24 / 25, delta=1e-3)

    def test_a_filter_that_ignores_the_wheel_shows_no_error(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = _write_capture(
                Path(tmp), "on", True, kf_mode="accel",
                slip_excursion=1.2,
            )
            result = analyse(Capture(path))
            events = result["events"]
            self.assertEqual(len(events), 1)
            self.assertAlmostEqual(events[0]["kf_error"], 0.0, places=3)

    def test_the_accelerometer_is_integrated_not_sampled(self) -> None:
        """Over a grip ramp the integral must recover the wheel's excursion."""
        with tempfile.TemporaryDirectory() as tmp:
            path = _write_capture(
                Path(tmp), "grip", False, kf_mode="wheel", grip_ramp=0.6,
            )
            result = analyse(Capture(path))
            self.assertTrue(result["grip"])
            moving = max(result["grip"], key=lambda g: abs(g["dv_wheel"]))
            self.assertAlmostEqual(moving["dv_wheel"], 0.6, places=2)
            # The accelerometer reports the matching acceleration, so its
            # integral must recover the same excursion.
            self.assertAlmostEqual(moving["dv_accel"], 0.6, places=1)


class VerdictTest(unittest.TestCase):
    def test_a_working_gate_is_reported_as_helping(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            off = _write_capture(root, "off", False, kf_mode="wheel")
            on = _write_capture(root, "on", True, kf_mode="accel")
            lines = verdict([analyse(Capture(off)), analyse(Capture(on))])
            self.assertIn("THE GATE HELPS", " ".join(lines))

    def test_an_open_gate_is_not_reported_as_helping(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            off = _write_capture(root, "off", False, kf_mode="wheel")
            on = _write_capture(root, "on", True, kf_mode="wheel")
            lines = verdict([analyse(Capture(off)), analyse(Capture(on))])
            self.assertIn("NO MEASURABLE DIFFERENCE", " ".join(lines))

    def test_two_runs_in_the_same_state_are_refused(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            a = _write_capture(root, "a", False, kf_mode="wheel")
            b = _write_capture(root, "b", False, kf_mode="wheel")
            lines = verdict([analyse(Capture(a)), analyse(Capture(b))])
            self.assertIn("SAME state", " ".join(lines))

    def test_a_broken_accelerometer_reference_is_refused(self) -> None:
        """The check that matters: no verdict when the reference is broken.

        The wheel is gripping and moving, the filter follows it, and the
        accelerometer reports none of it - which is what the real recordings
        look like.
        """
        with tempfile.TemporaryDirectory() as tmp:
            path = _write_capture(
                Path(tmp), "off", False, kf_mode="wheel",
                grip_ramp=1.0, grip_accel_agrees=False,
            )
            lines = verdict([analyse(Capture(path))])
            self.assertIn("REFERENCE UNRELIABLE", " ".join(lines))

    def test_a_filter_diverging_on_grip_windows_does_not_condemn_the_reference(
        self,
    ) -> None:
        """The reference check must compare the wheel against the accelerometer.

        Not the filter against the accelerometer.  With the gate on the filter
        deliberately stops following the wheel, so a filter-vs-accelerometer
        comparison would call a WORKING gate a broken reference - the exact
        inversion of what this tool is for.
        """
        with tempfile.TemporaryDirectory() as tmp:
            path = _write_capture(
                Path(tmp), "frozen", True, kf_mode="frozen",
                grip_ramp=0.6, slip_excursion=1.2,
            )
            result = analyse(Capture(path))
            # The filter is pinned at zero while both the wheel and the
            # accelerometer move, so kf_error is large on grip windows...
            moving = max(result["grip"], key=lambda g: abs(g["dv_wheel"]))
            self.assertGreater(abs(moving["kf_error"]), 0.3)
            # ...while the wheel and the accelerometer agree, which is what the
            # reference check is supposed to look at.
            self.assertLess(abs(moving["wheel_error"]), 0.05)
            self.assertNotIn("REFERENCE UNRELIABLE", " ".join(verdict([result])))

    def test_a_gate_off_run_that_tracks_the_wheel_is_diagnosed_as_inactive(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = _write_capture(
                Path(tmp), "off", False, kf_mode="wheel",
                slip_excursion=1.2,
            )
            lines = verdict([analyse(Capture(path))])
            joined = " ".join(lines)
            self.assertNotIn("REFERENCE UNRELIABLE", joined)
            self.assertIn("GATE NOT ACTING", joined)


if __name__ == "__main__":
    unittest.main()
