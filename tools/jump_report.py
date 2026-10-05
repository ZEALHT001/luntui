#!/usr/bin/env python3
"""Explain what a jump capture actually did, stage by stage.

The firmware's jump telemetry is easy to record and easy to misread: the stage
number runs 0..6 and an abort silently resets it, so "it did not jump" has
several very different causes.  This walks a capture and reports the sequence
of stages, how long each lasted, how far the legs actually travelled, and the
abort reason if one fired.

Usage:
    python3 tools/jump_report.py logs/<stamp>/kalman_samples.csv
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

STAGE_NAMES = {
    0: "IDLE",
    1: "COMPRESS",
    2: "EXTEND",
    3: "RETRACT",
    4: "LAND",
    5: "FINISHED",
    6: "ABORT",
}

ABORT_NAMES = {
    0: "NONE",
    1: "REMOTE (S1 down / rc lost / joints not allowed)",
    2: "FEEDBACK (a motor stopped reporting)",
    3: "GETUP (recovery active or fall re-arm pending)",
    4: "ATTITUDE (pitch/roll beyond the jump limit)",
    5: "TIMEOUT (total jump budget exceeded)",
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()

    rows = [
        r for r in csv.DictReader(args.csv.open(newline="", encoding="utf-8"))
        if r.get("mcu_time_ms") not in (None, "")
    ]
    if not rows:
        print("no samples", file=sys.stderr)
        return 2

    def num(row: dict, key: str) -> float:
        try:
            return float(row.get(key, "") or "nan")
        except ValueError:
            return float("nan")

    t = [num(r, "mcu_time_ms") / 1000.0 for r in rows]
    t = [v - t[0] for v in t]
    stage = [num(r, "jump_stage") for r in rows]
    abort = [num(r, "jump_abort_reason") for r in rows]
    length = [num(r, "leg_length_left") for r in rows]
    target = [num(r, "leg_length_target") for r in rows]
    armed = [num(r, "jump_armed") for r in rows]
    triggers = [num(r, "jump_trigger_count") for r in rows]

    print(f"samples {len(rows)}, span {t[-1]:.1f} s")
    print(f"leg length  min {min(length):.4f}  max {max(length):.4f} m")
    print(f"jump triggers seen: {max(triggers):.0f}")
    print()

    # Stage sequence, one line per contiguous run.
    print("stage sequence:")
    runs: list[tuple[float, float, float]] = []
    start = 0
    for k in range(1, len(rows) + 1):
        if k == len(rows) or stage[k] != stage[start]:
            runs.append((stage[start], t[start], t[k - 1]))
            start = k
    for name, t0, t1 in runs:
        label = STAGE_NAMES.get(int(name), f"?{name:.0f}")
        same = [k for k in range(len(rows)) if t0 <= t[k] <= t1]
        lo = min(length[k] for k in same)
        hi = max(length[k] for k in same)
        print(f"  {t0:7.3f}..{t1:7.3f} s  {label:<9} "
              f"leg {lo:.4f}..{hi:.4f} m  target {target[same[0]]:.4f}")
    print()

    aborts = [a for a in abort if a == a and a != 0]
    if aborts:
        reason = int(aborts[0])
        print(f"ABORTED: {ABORT_NAMES.get(reason, reason)}")
    elif max(stage) >= 5:
        print("completed without an abort")
    elif max(triggers) <= 0:
        print("NEVER TRIGGERED: jump_trigger_count stayed 0.")
        print("  The jump only fires on a transition INTO S1-up while already armed.")
        print("  Put S1 in the MIDDLE first (arms it), then move it UP (fires).")
    else:
        print("triggered but never reached FINISHED; see the stage sequence above")

    armed_ever = [a for a in armed if a == 1.0]
    print()
    print(f"jump_armed was 1 at some point: {'yes' if armed_ever else 'NO'}")
    print(f"peak leg extension reached: {max(length):.4f} m "
          f"(starts around {length[0]:.4f} m)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
