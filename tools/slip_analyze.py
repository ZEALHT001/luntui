#!/usr/bin/env python3
"""Evaluate slip-detection features against the operator's contact markers.

Reads a slip_capture run (fast_samples.csv + summary.json), builds candidate
features from the 225 Hz stream, and sweeps thresholds to report how many
marked contacts are detected and how many false positives occur elsewhere.

Why the event window is asymmetric
    The operator marks "contact" by hand, and reaction time lands the keypress
    0.4-0.7 s AFTER the real disturbance.  Measured on the first labelled run
    the slip peak preceded the marker by 391-737 ms, so a symmetric window
    would mostly measure noise.  The event window is therefore
    [marker - 1.2 s, marker + 0.2 s] by default; override with
    --lag-back / --lag-fwd.

Usage:
    python3 tools/slip_analyze.py logs/slip_20261008_004933
    python3 tools/slip_analyze.py logs/slip_... --sweep
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Callable

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.slip_capture import FAST_SCALARS  # noqa: E402

# One detection is emitted per event; a minimum spacing stops a single
# disturbance from registering as several triggers.
DEFAULT_REFRACTORY_MS = 700.0


class Trace:
    """Column vectors from fast_samples.csv, indexed by frame."""

    def __init__(self, path: Path) -> None:
        self.rows = list(csv.DictReader(path.open(newline="", encoding="utf-8")))
        if not self.rows:
            raise SystemExit(f"no rows in {path}")

    def col(self, name: str) -> list[float]:
        out: list[float] = []
        for row in self.rows:
            raw = row.get(name)
            try:
                out.append(float(raw) if raw not in (None, "") else math.nan)
            except ValueError:
                out.append(math.nan)
        return out

    def derivative(self, name: str) -> list[float]:
        """d(name)/dt with the frame's own measured period."""
        values = self.col(name)
        times = self.col("t_ms")
        out = [0.0] * len(values)
        for i in range(1, len(values)):
            dt_s = max((times[i] - times[i - 1]) / 1000.0, 1.0e-3)
            out[i] = (values[i] - values[i - 1]) / dt_s
        return out


def build_features(trace: Trace) -> dict[str, list[float]]:
    wl = trace.col("wheel_left_speed")
    wr = trace.col("wheel_right_speed")
    il = trace.col("wheel_left_current_output")
    ir = trace.col("wheel_right_current_output")
    tl = trace.col("wheel_left_torque")
    tr = trace.col("wheel_right_torque")
    return {
        "slip_metric": trace.col("slip_metric"),
        "abs_d_wheelspeed": [
            max(abs(a), abs(b))
            for a, b in zip(trace.derivative("wheel_left_speed"),
                            trace.derivative("wheel_right_speed"))
        ],
        "wheel_speed_gap": [abs(a - b) for a, b in zip(wl, wr)],
        "current_gap": [abs(a - b) for a, b in zip(il, ir)],
        "torque_gap": [abs(a - b) for a, b in zip(tl, tr)],
        "abs_body_ax_lin": [abs(x) for x in trace.col("body_ax_lin_g")],
        "abs_body_pitch": [abs(x) for x in trace.col("body_pitch")],
        "wheel_minus_body": [
            abs((abs(a) + abs(b)) * 0.5 * 0.060 - c)
            for a, b, c in zip(wl, wr, trace.col("kf_dx"))
        ],
    }


def percentiles(values: list[float], qs: tuple[float, ...]) -> list[float]:
    clean = sorted(v for v in values if not math.isnan(v))
    if not clean:
        return [0.0] * len(qs)
    return [clean[min(len(clean) - 1, int(q * (len(clean) - 1)))] for q in qs]


def find_runs(
    times: list[float],
    values: list[float],
    threshold: float,
    refractory_ms: float,
) -> list[tuple[float, float]]:
    """Contiguous above-threshold runs, merged closer than the refractory."""
    raw: list[list[float]] = []
    current: list[float] | None = None
    for time_ms, value in zip(times, values):
        if not math.isnan(value) and value >= threshold:
            if current is None:
                current = [time_ms, time_ms]
            else:
                current[1] = time_ms
        elif current is not None:
            raw.append(current)
            current = None
    if current is not None:
        raw.append(current)

    merged: list[tuple[float, float]] = []
    for start, end in raw:
        if merged and start - merged[-1][1] < refractory_ms:
            merged[-1] = (merged[-1][0], end)
        else:
            merged.append((start, end))
    return merged


def evaluate(
    runs: list[tuple[float, float]],
    events: list[float],
    lag_back_ms: float,
    lag_fwd_ms: float,
) -> tuple[int, int]:
    """Return (marked events hit, false positives)."""
    matched: set[int] = set()
    false_positives = 0
    for start, end in runs:
        hit: int | None = None
        for index, marker in enumerate(events):
            if start <= marker + lag_fwd_ms and end >= marker - lag_back_ms:
                hit = index
                break
        if hit is None:
            false_positives += 1
        else:
            matched.add(hit)
    return len(matched), false_positives


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("run", type=Path, help="logs/slip_<stamp> directory")
    parser.add_argument("--lag-back", type=float, default=1200.0,
                        help="search this many ms before the marker (default 1200)")
    parser.add_argument("--lag-fwd", type=float, default=200.0,
                        help="search this many ms after the marker (default 200)")
    parser.add_argument("--refractory-ms", type=float, default=DEFAULT_REFRACTORY_MS)
    parser.add_argument("--sweep", action="store_true",
                        help="sweep thresholds instead of only reporting stats")
    args = parser.parse_args()

    fast_path = args.run / "fast_samples.csv"
    summary_path = args.run / "summary.json"
    if not fast_path.exists():
        raise SystemExit(f"missing {fast_path}")
    trace = Trace(fast_path)
    times = trace.col("t_ms")
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    events = [e["fast_t_ms"] for e in summary.get("events", []) if e["event"] == "contact"]

    print(f"run:      {args.run}")
    print(f"frames:   {len(trace.rows)}")
    span_s = (times[-1] - times[0]) / 1000.0
    print(f"span:     {span_s:.1f} s")
    print(f"contacts: {len(events)}")
    if not events:
        print("\nNo 'contact' markers in this run - cannot evaluate. "
              "Re-record with the operator pressing 'c' on each event.")
        return 1

    # Reaction-lag report: where does the strongest activity actually sit?
    peaks = build_features(trace)["slip_metric"]
    print("\nmarker lag (slip_metric peak vs keypress):")
    for index, marker in enumerate(events, 1):
        window = [
            (t, v) for t, v in zip(times, peaks)
            if marker - args.lag_back <= t <= marker + args.lag_fwd
        ]
        if not window:
            continue
        peak_t, peak_v = max(window, key=lambda pair: pair[1])
        print(f"  contact {index}: peak {peak_v:6.2f} at {peak_t - marker:+7.0f} ms")

    features = build_features(trace)
    inside = [
        i for i, t in enumerate(times)
        if any(m - args.lag_back <= t <= m + args.lag_fwd for m in events)
    ]
    outside = [i for i in range(len(times)) if i not in set(inside)]

    print(f"\n{'feature':20s} {'false max':>10s} {'true min':>10s} {'true max':>10s} {'gap':>7s}")
    separating: list[tuple[str, float, float]] = []
    for name, values in features.items():
        outer = [values[i] for i in outside if not math.isnan(values[i])]
        inner: list[float] = []
        for marker in events:
            window = [
                v for t, v in zip(times, values)
                if marker - args.lag_back <= t <= marker + args.lag_fwd
                and not math.isnan(v)
            ]
            if window:
                inner.append(max(window))
        if not outer or not inner:
            continue
        false_max = max(outer)
        true_min = min(inner)
        gap = true_min / false_max if false_max > 0 else float("inf")
        flag = ""
        if true_min > false_max:
            # A threshold anywhere in (false_max, true_min] separates the four
            # marked slips from everything else on this recording.
            separating.append((name, false_max, true_min))
            flag = "  <-- clean gap"
        print(f"{name:20s} {false_max:10.2f} {true_min:10.2f} {max(inner):10.2f} "
              f"{gap:6.2f}x{flag}")

    if separating:
        print("\nclean threshold windows on this recording:")
        for name, low, high in separating:
            print(f"  {name:20s} ({low:.2f}, {high:.2f}]  "
                  f"-> suggested {low + (high - low) / 2:.1f}")
    else:
        print("\nno feature separates the marked slips from the rest on this "
              "recording")

    if not args.sweep:
        print("\n(pass --sweep to evaluate thresholds)")
        return 0

    print(f"\nthreshold sweep (event window -{args.lag_back:.0f}..+{args.lag_fwd:.0f} ms, "
          f"refractory {args.refractory_ms:.0f} ms)")
    print(f"{'feature':20s} {'thr':>8s} {'dets':>5s} {'hit':>6s} {'false':>6s}")
    sweep_table = {
        "slip_metric": [20, 25, 30, 35, 40, 45, 50, 60],
        "wheel_speed_gap": [40, 45, 50, 55, 60, 65, 70, 80],
        "current_gap": [8000, 10000, 12000, 14000, 16000, 20000],
        "torque_gap": [3, 4, 5, 6, 8, 10],
        "abs_d_wheelspeed": [300, 500, 700, 900, 1200],
        "abs_body_ax_lin": [0.15, 0.25, 0.35, 0.5],
        "wheel_minus_body": [1.0, 1.5, 2.0, 3.0],
    }
    for name, thresholds in sweep_table.items():
        values = features.get(name)
        if values is None:
            continue
        for threshold in thresholds:
            runs = find_runs(times, values, float(threshold), args.refractory_ms)
            hit, false = evaluate(runs, events, args.lag_back, args.lag_fwd)
            flag = "  <-- clean" if hit == len(events) and false == 0 else ""
            print(f"{name:20s} {threshold:8.1f} {len(runs):5d} "
                  f"{hit:3d}/{len(events):<2d} {false:6d}{flag}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
