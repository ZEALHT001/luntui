#!/usr/bin/env python3
"""Calibrate the slip gate from a recorded capture.

Condition C is a low-passed difference of the raw wheel speed compared against
the IMU body acceleration, evaluated at the 1 kHz control rate.  It cannot be
reconstructed from 50 Hz telemetry: differentiating over 20 ms instead of 1 ms
changes the result by more than an order of magnitude, and the low-pass state
evolves differently.  The firmware therefore publishes its own condition-C
value (`kf_slip_metric`) in every telemetry frame.

This tool treats that published value as authoritative and answers the two
questions that matter for tuning:

  * what threshold separates a genuine slip from normal driving, and
  * whether the framing conditions (A/B) would have blocked a detection.

Usage:
    python3 tools/slip_gate_replay.py logs/<stamp>/kalman_samples.csv
    python3 tools/slip_gate_replay.py logs/<stamp>/kalman_samples.csv --sweep
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path


MIN_HOLD_MS = 50.0
RELEASE_RATIO = 0.5


def load(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for raw in csv.DictReader(handle):
            row: dict[str, float] = {}
            for key, text in raw.items():
                try:
                    row[key] = float(text) if text != "" else math.nan
                except ValueError:
                    row[key] = math.nan
            rows.append(row)
    return rows


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[int(fraction * (len(ordered) - 1))]


def replay_latch(
    rows: list[dict[str, float]], threshold: float
) -> list[tuple[float, float]]:
    """Run the firmware's latch policy over the recorded condition-C values."""
    release = threshold * RELEASE_RATIO
    latch = False
    since = 0.0
    start = 0.0
    runs: list[tuple[float, float]] = []

    for row in rows:
        t = row["mcu_time_ms"] / 1000.0
        metric = row.get("kf_slip_metric", math.nan)
        if not math.isfinite(metric):
            continue
        if metric >= threshold:
            if not latch:
                latch = True
                since = t
                start = t
        elif latch:
            if (t - since) * 1000.0 < MIN_HOLD_MS:
                continue
            if metric < release:
                runs.append((start, t))
                latch = False

    if latch:
        runs.append((start, rows[-1]["mcu_time_ms"] / 1000.0))
    return runs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument(
        "--sweep",
        action="store_true",
        help="sweep candidate thresholds instead of only the recorded one",
    )
    args = parser.parse_args()

    rows = load(args.csv)
    if not rows:
        print("empty capture")
        return 1
    if "kf_slip_metric" not in rows[0]:
        raise SystemExit("capture has no kf_slip_metric column")

    t0 = rows[0]["mcu_time_ms"] / 1000.0
    span = rows[-1]["mcu_time_ms"] / 1000.0 - t0
    metrics = [r["kf_slip_metric"] for r in rows if math.isfinite(r["kf_slip_metric"])]
    recorded = rows[0].get("slip_accel_threshold", math.nan)

    print(f"{args.csv}")
    print(f"  samples={len(rows)}  span={span:.1f}s  "
          f"recorded threshold={recorded:.1f} m/s2")
    print()
    print("Condition C as published by the firmware (authoritative):")
    print(f"  median={statistics.median(metrics):8.3f}  "
          f"p90={percentile(metrics, 0.90):8.3f}  "
          f"p99={percentile(metrics, 0.99):8.3f}  "
          f"p99.9={percentile(metrics, 0.999):8.3f}  "
          f"max={max(metrics):8.3f}  m/s2")
    print()

    if "kf_dx" in rows[0] and "kf_wheel_measurement" in rows[0]:
        gaps = [
            abs(r["kf_wheel_measurement"]) - abs(r["kf_dx"])
            for r in rows
            if math.isfinite(r["kf_dx"]) and math.isfinite(r["kf_wheel_measurement"])
        ]
        frame_threshold = rows[0].get("slip_vel_diff_threshold")
        if frame_threshold is None or not math.isfinite(frame_threshold):
            frame_threshold = 1.2
        print("Condition B framing (|wheel body speed| - |fused speed|):")
        print(f"  median={statistics.median(gaps):8.3f}  "
              f"p99={percentile(gaps, 0.99):8.3f}  max={max(gaps):8.3f}  m/s")
        print(f"  samples framing at the {frame_threshold:.2f} m/s threshold: "
              f"{sum(1 for g in gaps if g >= frame_threshold)}")
        print("  NOTE: this compares a wheel against a velocity fused from that")
        print("        same wheel, so it is self-limiting and largely inert.")
        print()

    if args.sweep:
        print("Threshold sweep (firmware latch policy applied to the metric):")
        print(f"{'threshold':>10} {'activations':>12} {'held_s':>8} {'samples>=':>10}")
        for candidate in (2.0, 5.0, 10.0, 15.0, 20.0, 30.0, 50.0):
            runs = replay_latch(rows, candidate)
            held = sum(b - a for a, b in runs)
            above = sum(1 for m in metrics if m >= candidate)
            print(f"{candidate:10.1f} {len(runs):12d} {held:8.3f} {above:10d}")
        print()

    if math.isfinite(recorded):
        runs = replay_latch(rows, recorded)
        print(f"With the recorded threshold ({recorded:.1f} m/s2):")
        print(f"  latch activations: {len(runs)}")
        for a, b in runs[:25]:
            print(f"    {a - t0:8.3f} .. {b - t0:8.3f}  ({b - a:.3f}s)")
        if not runs:
            print("    none - lower the threshold")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
