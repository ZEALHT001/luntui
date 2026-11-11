#!/usr/bin/env python3
"""Wheel-odometry drift report from a VMCVOFA capture.

At low speed the wheels are the ground truth: integrate the wheel speed and you
have the distance the machine actually travelled, with no IMU and no Kalman
filter in the path.  That is the number a static-drift experiment has to be
judged on, because it cannot drift on its own the way a fused position estimate
can.

The firmware stores the two wheel speeds in motor convention - the right motor
is mounted mirrored, so forward motion reads +w on one side and -w on the
other.  Their sum cancels and their difference is twice the body speed, which
is why the firmware averages the two per-side body terms instead of adding the
raw wheel speeds.  The sign is checked against kf_dx rather than assumed.

    v_body = scale * (right_raw - left_raw) / 2

The factor two is not a guess: a hand push of 20 cm read 385 mm before it was
applied (2026-11-11 calibration, four pushes of 173-193 mm after halving).

``scale`` is 1.0 when the telemetry is already in m/s.  Hand-push the machine
along a ruler once and pass the measured/derived ratio to calibrate it.

Usage:
    python3 tools/wheel_odometry.py logs/20261111_234500
    python3 tools/wheel_odometry.py --csv path/to/kalman_samples.csv --scale 0.5
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# A gap this large is an RTT stall or a replayed stale buffer, not a real
# sampling interval; integrating across it would fabricate displacement.
MAX_STEP_S = 0.10


def _float(row: dict[str, str], key: str) -> float | None:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return None


def load_rows(path: Path) -> list[dict[str, str]]:
    if path.is_dir():
        path = path / "kalman_samples.csv"
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def drop_stale_prefix(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    """Cut everything before the last big mcu_time jump.

    The J-Link hands over whatever was still buffered, so a capture usually
    starts with a segment from an earlier boot.  Integrating that as if it
    were continuous invents metres of motion.
    """
    last_jump = 0
    previous = None
    for index, row in enumerate(rows):
        stamp = _float(row, "mcu_time_ms")
        if stamp is None:
            continue
        if previous is not None and stamp - previous > MAX_STEP_S * 1000.0:
            last_jump = index
        previous = stamp
    return rows[last_jump:]


def integrate(
    rows: list[dict[str, str]], scale: float
) -> tuple[list[float], list[float], list[float], list[float]]:
    times: list[float] = []
    velocity: list[float] = []
    position: list[float] = []
    gap: list[float] = []
    travelled = 0.0
    previous_t: float | None = None
    previous_v = 0.0
    for row in rows:
        stamp = _float(row, "mcu_time_ms")
        left = _float(row, "kf_wheel_left_raw")
        right = _float(row, "kf_wheel_right_raw")
        if stamp is None or left is None or right is None:
            continue
        t = stamp / 1000.0
        v = scale * 0.5 * (right - left)
        if previous_t is not None:
            step = t - previous_t
            if 0.0 < step <= MAX_STEP_S:
                travelled += 0.5 * (v + previous_v) * step
        else:
            travelled = 0.0
        times.append(t)
        velocity.append(v)
        gap.append(right + left)
        position.append(travelled)
        previous_t = t
        previous_v = v
    return times, velocity, position, gap


def rms(values: list[float]) -> float:
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0


def correlation(a: list[float], b: list[float]) -> float:
    if len(a) < 2 or len(a) != len(b):
        return 0.0
    mean_a = sum(a) / len(a)
    mean_b = sum(b) / len(b)
    var_a = math.sqrt(sum((x - mean_a) ** 2 for x in a))
    var_b = math.sqrt(sum((x - mean_b) ** 2 for x in b))
    if var_a == 0.0 or var_b == 0.0:
        return 0.0
    return sum((a[i] - mean_a) * (b[i] - mean_b) for i in range(len(a))) / (
        var_a * var_b
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "path",
        type=Path,
        nargs="?",
        help="capture directory or kalman_samples.csv (default: newest)",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="odometry scale applied to (right - left); calibrate with a "
        "hand push (default: 1.0)",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=10.0,
        help="sliding window in seconds for the drift-rate table (default: 10)",
    )
    args = parser.parse_args(argv)

    path = args.path
    if path is None:
        candidates = sorted((ROOT / "logs").glob("*/kalman_samples.csv"))
        if not candidates:
            print("no captures under logs/", file=sys.stderr)
            return 2
        path = candidates[-1]

    rows = drop_stale_prefix(load_rows(path))
    times, velocity, position, gap = integrate(rows, args.scale)
    if len(times) < 2:
        print("not enough samples with wheel data", file=sys.stderr)
        return 2

    span = times[-1] - times[0]
    net = position[-1]
    span_mm = (max(position) - min(position)) * 1000.0
    print(f"source      : {path}")
    print(f"samples     : {len(times)}  span {span:.1f} s")
    print(f"scale       : {args.scale:g}")
    print(f"net drift   : {net * 1000.0:+7.1f} mm  ({net / span * 60.0 * 1000.0:+.1f} mm/min)")
    print(f"peak-peak   : {span_mm:7.1f} mm")
    print(f"speed RMS   : {rms(velocity):7.4f} m/s")
    print(f"L/R gap RMS : {rms(gap):7.4f} m/s  (misalignment or slip)")

    kf_dx = [_float(row, "kf_dx") for row in rows]
    kf_dx = [v for v in kf_dx if v is not None]
    paired = min(len(kf_dx), len(velocity))
    if paired > 1:
        print(
            f"corr(v_odom, kf_dx) = "
            f"{correlation(velocity[-paired:], kf_dx[-paired:]):+.3f}"
        )

    print(f"\n{'window (s)':>14} {'net (mm)':>10} {'pp (mm)':>9} {'mm/min':>9}")
    step = max(1, int(args.window / max(span / max(len(times) - 1, 1), 1e-3)))
    for start in range(0, len(times) - 1, step):
        end = min(start + step, len(times) - 1)
        window_span = times[end] - times[start]
        if window_span <= 0.0:
            continue
        segment = position[start : end + 1]
        net_window = (segment[-1] - segment[0]) * 1000.0
        pp = (max(segment) - min(segment)) * 1000.0
        print(
            f"{times[start]:6.1f}..{times[end]:6.1f} {net_window:10.1f} "
            f"{pp:9.1f} {net_window / window_span * 60.0:9.1f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
