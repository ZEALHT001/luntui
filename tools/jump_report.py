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
    distance = [num(r, "tf02_distance_cm") for r in rows]
    connected = [num(r, "tf02_connected") for r in rows]

    print(f"samples {len(rows)}, span {t[-1]:.1f} s")
    print(f"leg length  min {min(length):.4f}  max {max(length):.4f} m")
    print(f"jump triggers seen: {max(triggers):.0f}")
    print()

    # Automatic-gate conditions.  Without these the report cannot say whether a
    # missing jump was a speed, distance, or sensor problem.
    import math as _math

    if any(_math.isfinite(v) for v in distance):
        live = [v for v, c in zip(distance, connected) if c == 1.0 and _math.isfinite(v)]
        print("automatic jump gate inputs:")
        print(f"  laser connected: {sum(1 for c in connected if c == 1.0)}/{len(connected)} frames")
        if live:
            print(f"  distance while connected: min {min(live):.0f}  max {max(live):.0f} cm")
            inside = sum(1 for v in live if 80.0 < v < 100.0)
            print(f"  inside (80, 100) cm: {inside}/{len(live)} frames")
        else:
            print("  distance: NEVER CONNECTED - the automatic jump gate can "
                  "never fire without a live laser")
        print()
    else:
        print("automatic jump gate inputs: not in this capture (pre-schema-9)\n")

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
        print("  The jump is automatic: it fires from IDLE once ALL of these hold")
        print("  for the confirmation window at the same time -")
        print("    - S1 is UP (not MID) and the remote is fresh,")
        print("    - |kf_dx| >= VMC_JUMP_AUTO_SPEED_MIN (1.0 m/s),")
        print("    - the TF02 laser reads strictly inside (80, 100) cm.")
        print("  Check jump_armed in the capture: it reads 1 only while the gate")
        print("  is satisfied.  Also check jump_abort_reason - REMOTE means S1")
        print("  was not up, or the transmitter was stale.")
    else:
        print("triggered but never reached FINISHED; see the stage sequence above")

    armed_ever = [a for a in armed if a == 1.0]
    print()
    print(f"jump_armed was 1 at some point: {'yes' if armed_ever else 'NO'}")
    print(f"peak leg extension reached: {max(length):.4f} m "
          f"(starts around {length[0]:.4f} m)")

    # Take-off speed -> jump height.
    #
    # The robot's upward velocity at the instant the foot leaves the ground is
    # the leg's extension rate at that moment (the foot is planted, so the body
    # moves up exactly as fast as the leg lengthens).  A projectile then rises
    # v^2 / 2g.  That makes the extension rate the one number worth chasing,
    # and it is measurable from the leg-length channel alone.
    print()
    dt = [t[k + 1] - t[k] for k in range(len(t) - 1)]
    step = sorted(v for v in dt if v > 0)[len(dt) // 2] if dt else 0.02
    rates = []
    for k in range(2, len(t) - 2):
        if stage[k] != 2.0:          # EXTEND only
            continue
        # Central difference over +-2 samples to beat the 1 mm quantisation.
        span = t[k + 2] - t[k - 2]
        if span <= 0:
            continue
        rates.append((length[k + 2] - length[k - 2]) / span)
    if rates:
        peak = max(rates)
        height = peak * peak / (2.0 * 9.81)
        print(f"leg extension rate: peak {peak:.2f} m/s "
              f"(median step {step * 1000:.0f} ms, {len(rates)} samples)")
        print(f"  -> take-off speed {peak:.2f} m/s gives {height * 1000:.0f} mm "
              f"of jump height")
        print()
        print("  how the height scales with that speed:")
        for target in (0.05, 0.10, 0.15, 0.20):
            need = (2.0 * 9.81 * target) ** 0.5
            print(f"    {target * 1000:5.0f} mm needs {need:.2f} m/s "
                  f"({need / peak:.2f}x current)")
    else:
        print("no EXTEND samples with a usable leg-rate window")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
