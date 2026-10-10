#!/usr/bin/env python3
"""Does rejecting a slipping wheel actually improve the velocity estimate?

The firmware has one slip *action*: while the per-side latch is set and
``kalman_slip_enable`` is on, the body-motion Kalman filter rejects the wheel
measurement - it zeroes the innovation and widens its own weights, so the
estimate coasts on the accelerometer instead of following a wheel that may be
spinning up.  That flag ships OFF (``VMC_BODY_MOTION_KF_SLIP_ENABLE_DEFAULT``),
so the action has never been validated on the vehicle.  This tool is that
validation.

The problem with a naive comparison
    "Run it with the gate on and off and see if kf_dx is better" needs a
    definition of *better*, and the slip events are chaotic - no two are
    alike, so comparing one run against another mostly measures the events.
    What the tool does instead is score both runs against a reference that is
    INDEPENDENT of the wheels.

The reference
    ``dd_x`` is the gravity-compensated FORE-AFT acceleration the filter
    consumes.  Integrated over a slip window it gives how much the BODY's
    velocity actually changed:

        dv_accel = integral of dd_x dt

    Which telemetry channel that is has to be taken from the firmware, not
    guessed, because the HI12's channels are not the body frame's.  Measured:
    gravity sits on channel 2 at rest, moves onto channel 1 when the body
    pitches and onto channel 0 when it rolls, so the fore-aft axis is channel
    1 and ``dd_x = -body_ay_lin_g * 9.81``.  Reading ``body_ax_lin_g`` here -
    the LATERAL axis - is what made this tool report "reference unreliable" on
    perfectly good recordings before the axis was fixed in the firmware.

    The wheel-derived velocity ``d_x_raw`` is what the filter is being fed, and
    ``kf_dx`` is what it ended up believing.  During a real slip the wheel
    claims a large change that the body never made, so:

        kf_error    = (kf_dx_end    - kf_dx_start)    - dv_accel
        wheel_error = (dv_wheel)                       - dv_accel

    A working gate pulls ``kf_error`` well below ``wheel_error``; a gate that
    does nothing leaves them equal.  Both runs are scored the same way, so the
    two are comparable even though their events are not identical.

The sanity check that has to come first
    None of that means anything if the accelerometer reference is wrong, so
    the tool also scores windows where the machine is NOT slipping.  There the
    wheel is trustworthy and the filter should track it, so ``kf_error`` and
    ``wheel_error`` should both be small.  If they are not, the accelerometer
    channel is not a usable reference for this recording and the slip numbers
    must not be read - the tool says so instead of printing a verdict.

Usage:
    # One run: is the wheel actually being corrupted, and does the gate act?
    python3 tools/slip_gate_report.py logs/slip_20261011_010000

    # Two runs, gate off then gate on: does the gate improve the estimate?
    python3 tools/slip_gate_report.py <gate_off_dir> <gate_on_dir>
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

GRAVITY = 9.81
WHEEL_RADIUS = 0.060
PHI0_VERTICAL = 1.5707963267948966
# Minimum samples in a window for its integral to mean anything.
MIN_WINDOW_SAMPLES = 4
# Windows this long (ms) are used for the no-slip reference check.
GRIP_WINDOW_MS = 200.0
GRIP_STRIDE_MS = 2000.0


def _num(row: dict[str, str], key: str) -> float:
    try:
        return float(row.get(key, "") or "nan")
    except (TypeError, ValueError):
        return float("nan")


class Capture:
    """One recording, plus what the summary says about the gate."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.fast_path = path / "fast_samples.csv"
        if not self.fast_path.exists():
            raise ValueError(f"{path} has no fast_samples.csv")
        with self.fast_path.open(newline="", encoding="utf-8") as handle:
            self.rows = [
                r for r in csv.DictReader(handle) if r.get("t_ms") not in (None, "")
            ]
        if not self.rows:
            raise ValueError(f"{self.fast_path} has no samples")
        self.gate = self._read_gate()

    def _read_gate(self) -> bool | None:
        """Read the gate state from the slow stream, which carries it.

        The fast stream has the raw latches but not ``slip_enable``, so the
        slow stream is the authoritative record - and it is per-sample, which
        catches a gate that was changed part-way through a run.
        """
        slow = self.path / "kalman_samples.csv"
        if slow.exists():
            with slow.open(newline="", encoding="utf-8") as handle:
                values = {
                    _num(r, "slip_enable")
                    for r in csv.DictReader(handle)
                    if r.get("slip_enable") not in (None, "")
                }
            values = {v for v in values if v == v}
            if len(values) == 1:
                return bool(values.pop())
            if values:
                raise ValueError(
                    f"{slow} changes slip_enable during the run "
                    f"({sorted(values)}); split it before comparing"
                )
        summary = self.path / "summary.json"
        if summary.exists():
            data = json.loads(summary.read_text(encoding="utf-8"))
            applied = data.get("slip_applied", {}).get("gate_active")
            if applied is not None:
                return bool(applied)
        return None

    def label(self) -> str:
        if self.gate is None:
            return f"{self.path.name} (gate state unknown)"
        return f"{self.path.name} (gate {'ON' if self.gate else 'OFF'})"


def _time_s(row: dict[str, str]) -> float:
    return _num(row, "t_ms") / 1000.0


def _accel(row: dict[str, str]) -> float:
    """Fore-aft acceleration in m/s2 - exactly what the firmware feeds the filter.

    Mirrors ``dd_x = -body.ay_lin * VMC_GRAVITY_ACCEL`` in vmc_probe.c.  The
    negation and the choice of Y over X are both measured, not assumed; see the
    module docstring and tools/test_fore_aft_accel.py.
    """
    return -_num(row, "body_ay_lin_g") * GRAVITY


def _wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _wheel_velocity(row: dict[str, str]) -> float:
    """The body velocity the wheel kinematics claim, i.e. ``d_x_raw``.

    This is a full reconstruction of vmc_update_motion_state(), not the
    simplified wheel-only form.  The simplified form
    ``-(wL + wR) * R / 2`` is WRONG here and it took a wrong conclusion to
    notice: the leg terms are not small.  On a real recording the wheel-only
    approximation disagreed with the firmware's own ``kf_wheel_measurement``
    by the whole signal.

    Every input is carried by the fast stream, so the reconstruction is exact.
    Verified against ``kf_wheel_measurement`` (the firmware's own d_x_raw) on
    20k samples of logs/slip_20261010_004308: median error 0.0006 m/s.
    """
    pitch = _num(row, "body_pitch")
    pitch_rate = _num(row, "body_pitch_rate")
    left_phi0 = _num(row, "leg_left_phi0")
    right_phi0 = _num(row, "leg_right_phi0")
    left_d_phi0 = _num(row, "leg_left_d_phi0")
    right_d_phi0 = _num(row, "leg_right_d_phi0")
    for value in (pitch, pitch_rate, left_phi0, right_phi0, left_d_phi0,
                  right_d_phi0):
        if value != value:
            return float("nan")

    left_theta = _wrap(left_phi0 - PHI0_VERTICAL - pitch)
    right_theta = _wrap(right_phi0 - PHI0_VERTICAL - pitch)
    d_theta = (left_d_phi0 + right_d_phi0) * 0.5 - pitch_rate
    left_wheel_w = _num(row, "wheel_left_speed") - pitch_rate - left_d_phi0
    right_wheel_w = -_num(row, "wheel_right_speed") - pitch_rate - right_d_phi0
    if left_wheel_w != left_wheel_w or right_wheel_w != right_wheel_w:
        return float("nan")

    left_body_dx = (
        -left_wheel_w * WHEEL_RADIUS
        + _num(row, "leg_left_length") * d_theta * math.cos(left_theta)
        + _num(row, "leg_left_d_length") * math.sin(left_theta)
    )
    right_body_dx = (
        -right_wheel_w * WHEEL_RADIUS
        + _num(row, "leg_right_length") * d_theta * math.cos(right_theta)
        + _num(row, "leg_right_d_length") * math.sin(right_theta)
    )
    return (left_body_dx + right_body_dx) * 0.5


def _integrate_accel(rows: list[dict[str, str]], start: int, end: int) -> float:
    """Trapezoidal integral of the body acceleration over [start, end]."""
    total = 0.0
    for i in range(start + 1, end + 1):
        a0, a1 = _accel(rows[i - 1]), _accel(rows[i])
        if a0 != a0 or a1 != a1:
            continue
        dt = _time_s(rows[i]) - _time_s(rows[i - 1])
        if dt <= 0.0:
            continue
        total += 0.5 * (a0 + a1) * dt
    return total


def find_slip_windows(rows: list[dict[str, str]]) -> list[tuple[int, int]]:
    """Contiguous runs where either raw latch is set.

    These are the RAW latches, which the fast stream publishes regardless of
    the gate, so both runs are segmented identically whether the gate is on or
    off.
    """
    windows: list[tuple[int, int]] = []
    start: int | None = None
    for index, row in enumerate(rows):
        latched = _num(row, "slip_left") >= 0.5 or _num(row, "slip_right") >= 0.5
        if latched and start is None:
            start = index
        elif not latched and start is not None:
            windows.append((start, index - 1))
            start = None
    if start is not None:
        windows.append((start, len(rows) - 1))
    return [
        (a, b)
        for a, b in windows
        if b - a + 1 >= MIN_WINDOW_SAMPLES and _num(rows[a], "enabled") >= 0.5
    ]


def _window_metrics(
    rows: list[dict[str, str]], start: int, end: int
) -> dict[str, float]:
    kf = _num(rows[end], "kf_dx") - _num(rows[start], "kf_dx")
    wheel = _wheel_velocity(rows[end]) - _wheel_velocity(rows[start])
    accel = _integrate_accel(rows, start, end)
    return {
        "t_s": _time_s(rows[start]),
        "duration_ms": (_time_s(rows[end]) - _time_s(rows[start])) * 1000.0,
        "dv_kf": kf,
        "dv_wheel": wheel,
        "dv_accel": accel,
        "kf_error": kf - accel,
        "wheel_error": wheel - accel,
    }


def analyse(capture: Capture) -> dict[str, object]:
    rows = capture.rows
    windows = find_slip_windows(rows)
    events = [_window_metrics(rows, a, b) for a, b in windows]

    # Slip-free reference windows: stride through the enabled stretches that
    # contain no latch at all and score fixed-length windows there.
    latched = [
        (_num(r, "slip_left") >= 0.5) or (_num(r, "slip_right") >= 0.5)
        for r in rows
    ]
    enabled = [_num(r, "enabled") >= 0.5 for r in rows]
    # Two-pointer scan: for each stride cue, find the samples in
    # [cue, cue + GRIP_WINDOW_MS] without rescanning the whole recording.
    times_ms = [_time_s(r) * 1000.0 for r in rows]
    grip: list[dict[str, float]] = []
    total = len(rows)
    for cue_ms in _frange(times_ms[0], times_ms[-1] - GRIP_WINDOW_MS, GRIP_STRIDE_MS):
        lo = 0
        while lo < total and times_ms[lo] < cue_ms:
            lo += 1
        hi = lo
        while hi + 1 < total and times_ms[hi + 1] <= cue_ms + GRIP_WINDOW_MS:
            hi += 1
        if lo >= total or hi - lo + 1 < MIN_WINDOW_SAMPLES:
            continue
        if not all(enabled[lo : hi + 1]):
            continue
        if any(latched[lo : hi + 1]):
            continue
        grip.append(_window_metrics(rows, lo, hi))

    return {
        "label": capture.label(),
        "gate": capture.gate,
        "samples": len(rows),
        "enabled_fraction": (
            sum(1 for e in enabled if e) / len(enabled) if enabled else 0.0
        ),
        "events": events,
        "grip": grip,
    }


def _frange(start: float, stop: float, step: float) -> list[float]:
    out = []
    value = start
    while value <= stop:
        out.append(value)
        value += step
    return out


def _abs_percentiles(values: list[float]) -> tuple[float, float, float]:
    magnitudes = sorted(abs(v) for v in values if v == v)
    if not magnitudes:
        return (float("nan"), float("nan"), float("nan"))
    return (
        statistics.median(magnitudes),
        magnitudes[int(0.9 * (len(magnitudes) - 1))],
        magnitudes[-1],
    )


def _print_capture(result: dict[str, object]) -> None:
    print(f"  {result['label']}")
    print(
        f"    samples {result['samples']}, "
        f"enabled {float(result['enabled_fraction']) * 100:.0f}%"
    )
    grip = result["grip"]  # type: ignore[assignment]
    events = result["events"]  # type: ignore[assignment]

    if grip:
        med, p90, worst = _abs_percentiles([g["kf_error"] for g in grip])
        wmed, _, _ = _abs_percentiles([g["wheel_error"] for g in grip])
        print(
            f"    no-slip reference windows: {len(grip)}  "
            f"|kf_error| median {med:.4f} m/s (p90 {p90:.4f}, max {worst:.4f})"
        )
        print(f"      |wheel_error| median {wmed:.4f} m/s")

    if not events:
        print("    no slip events in this recording")
        return

    med, p90, worst = _abs_percentiles([e["kf_error"] for e in events])
    wmed, wp90, _ = _abs_percentiles([e["wheel_error"] for e in events])
    swing, _, _ = _abs_percentiles([e["dv_wheel"] for e in events])
    print(
        f"    slip events: {len(events)}   "
        f"|dv_wheel| median {swing:.4f} m/s   "
        f"duration median {statistics.median(e['duration_ms'] for e in events):.0f} ms"
    )
    print(
        f"      |kf_error|    median {med:.4f}  p90 {p90:.4f}  max {worst:.4f} m/s"
    )
    print(f"      |wheel_error| median {wmed:.4f}  p90 {wp90:.4f} m/s")


def verdict(results: list[dict[str, object]]) -> list[str]:
    lines: list[str] = []

    def reference_is_unreliable(result: dict[str, object]) -> str | None:
        """Is the accelerometer usable as an arbiter for this recording?

        It is only usable if it agrees with the filter on windows where the
        machine is NOT slipping: there the wheel is trustworthy, the filter
        tracks it, and the accelerometer should therefore track them both.  If
        the accelerometer's own disagreement is comparable to the event it is
        supposed to adjudicate, no verdict from it means anything.

        Scored over ALL the slip-free windows, by median.  That is not the
        stricter choice it looks like: picking the highest-motion windows
        instead selects exactly the moments a balancing robot's wheels slip
        hardest, so the "reference check" ends up measuring slip and blaming
        the accelerometer for it.  Measured on a drive capture the two
        disagree by 0.27 m/s median on the top-quartile-motion windows but
        only 0.047 m/s over all of them, and a separate hand-push recording
        with the machine disabled confirms which is right: there the wheel and
        the accelerometer agree to a ratio of 0.94 at every window size from
        100 ms to 1 s.

        The median still discriminates a genuinely broken reference: before
        the fore-aft axis was fixed in the firmware this same median read
        0.37 m/s.
        """
        grip = result["grip"]  # type: ignore[assignment]
        events = result["events"]  # type: ignore[assignment]
        if not grip:
            return (
                "Not enough slip-free, enabled telemetry to check whether the "
                "accelerometer is usable as a reference."
            )
        # Compare the accelerometer against the WHEEL, not against the filter:
        # with the gate on the filter deliberately stops following the wheel,
        # so kf_error would flag a working gate as a broken reference.
        grip_med, grip_p90, grip_max = _abs_percentiles(
            [g["wheel_error"] for g in grip]
        )
        if not events:
            return None
        event_swing, _, _ = _abs_percentiles([e["dv_wheel"] for e in events])
        # A reference that disagrees with the wheel while the wheel is gripping
        # is not measuring body motion, whatever the slip numbers say.
        if grip_med == grip_med and grip_med > max(0.25 * event_swing, 1e-3):
            return (
                "REFERENCE UNRELIABLE: across the slip-free windows the wheel "
                f"and the accelerometer already disagree by {grip_med:.4f} m/s "
                f"median (p90 {grip_p90:.4f}, max {grip_max:.4f}), against slip "
                f"events of {event_swing:.4f} m/s.  The accelerometer does not "
                "track a gripping wheel, so it cannot arbitrate this "
                "recording."
            )
        return None

    if len(results) == 2:
        off, on = results
        if off["gate"] and not on["gate"]:
            off, on = on, off
        if off["gate"] is None or on["gate"] is None:
            lines.append(
                "Cannot compare: at least one capture does not record the gate "
                "state, so it is not known which run was which."
            )
            return lines
        if off["gate"] == on["gate"]:
            lines.append(
                "Both captures have the gate in the SAME state "
                f"({'ON' if on['gate'] else 'OFF'}), so this is not a "
                "comparison - re-record one of them with --slip-gate."
            )
            return lines
        off_events = off["events"]  # type: ignore[assignment]
        on_events = on["events"]  # type: ignore[assignment]
        if not off_events or not on_events:
            lines.append(
                "Cannot compare: at least one capture has no slip events."
            )
            return lines
        for result in results:
            problem = reference_is_unreliable(result)
            if problem:
                lines.append(problem)
                return lines
        off_med, _, _ = _abs_percentiles([e["kf_error"] for e in off_events])
        on_med, _, _ = _abs_percentiles([e["kf_error"] for e in on_events])
        off_swing, _, _ = _abs_percentiles([e["dv_wheel"] for e in off_events])
        on_swing, _, _ = _abs_percentiles([e["dv_wheel"] for e in on_events])
        lines.append(
            f"gate OFF: {len(off_events)} events, |kf_error| median "
            f"{off_med:.4f} m/s (event size |dv_wheel| median {off_swing:.4f})"
        )
        lines.append(
            f"gate ON : {len(on_events)} events, |kf_error| median "
            f"{on_med:.4f} m/s (event size |dv_wheel| median {on_swing:.4f})"
        )
        if off_med <= 0.0 and on_med <= 0.0:
            lines.append(
                "Both are zero to within measurement resolution; these events "
                "are too small to distinguish."
            )
        elif on_med < off_med * 0.8:
            lines.append(
                f"THE GATE HELPS: it cuts the velocity error during slip to "
                f"{100.0 * on_med / off_med:.0f}% of the ungated value."
            )
        elif on_med > off_med * 1.25:
            lines.append(
                f"THE GATE HURTS: error is {100.0 * on_med / off_med:.0f}% of "
                "the ungated value.  Check for false triggers - the gate also "
                "rejects good measurements while it is latched."
            )
        else:
            lines.append(
                "NO MEASURABLE DIFFERENCE at this event size.  Either the "
                "wheel was not being corrupted in these events, or the events "
                "are too small for the gate to matter."
            )
        return lines

    result = results[0]
    grip = result["grip"]  # type: ignore[assignment]
    events = result["events"]  # type: ignore[assignment]
    problem = reference_is_unreliable(result)
    if problem:
        lines.append(problem)
        return lines
    if not events:
        lines.append("No slip events recorded, so there is nothing to score.")
        return lines
    kf_med, _, _ = _abs_percentiles([e["kf_error"] for e in events])
    wheel_med, _, _ = _abs_percentiles([e["wheel_error"] for e in events])
    if wheel_med <= 0.005:
        lines.append(
            "NO WHEEL CORRUPTION: even on the raw wheel signal the velocity "
            "excursion matches the accelerometer, so these events do not look "
            "like wheel slip and the gate has nothing to fix."
        )
        return lines
    if kf_med < wheel_med * 0.6:
        lines.append(
            f"GATE ACTING: the filter's error ({kf_med:.4f} m/s) is well below "
            f"the raw wheel's ({wheel_med:.4f} m/s), so the rejection is doing "
            "its job in this recording."
        )
    elif kf_med > wheel_med * 1.5:
        lines.append(
            f"FILTER EXAGGERATES: with the gate OFF the filter's own error "
            f"({kf_med:.4f} m/s) is larger than its input's ({wheel_med:.4f} "
            "m/s), which a filter tracking its measurement cannot do.  The "
            "estimate is not simply following the wheel, so the accelerometer "
            "prediction it mixes in is the thing to check first."
        )
    else:
        lines.append(
            f"GATE NOT ACTING: the filter's error ({kf_med:.4f} m/s) is close "
            f"to the raw wheel's ({wheel_med:.4f} m/s), which is what an open "
            "gate looks like.  Check slip_enable on this capture."
        )
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("captures", nargs="+", type=Path)
    args = parser.parse_args()

    results = []
    for path in args.captures:
        try:
            results.append(analyse(Capture(path)))
        except (OSError, ValueError) as exc:
            print(f"{path}: {exc}", file=sys.stderr)
            return 2

    print("slip-gate validation")
    print("  reference: dv_accel = integral of dd_x dt = -body_ay_lin_g * 9.81")
    print("  kf_error = dv_kf_dx - dv_accel   (should shrink when the gate works)")
    print("  wheel_error = dv_wheel - dv_accel (what the filter is being fed)")
    print()
    for result in results:
        _print_capture(result)
        print()
    for line in verdict(results):
        print(f"  {line}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
