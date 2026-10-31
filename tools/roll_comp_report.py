#!/usr/bin/env python3
"""Explain what the roll compensator actually commanded during a capture.

The compensator levels the body by driving the two legs to different lengths:

    roll_comp_offset = clamp((body_roll - target) * gain, +/-limit)
    left_target      = clamp(ramped_target + offset, MIN, MAX)
    right_target     = clamp(ramped_target - offset, MIN, MAX)

Three different faults produce the same symptom - "it does not level" - and
they are indistinguishable from the measured leg lengths alone:

  1. NOT RESPONDING   the offset stays near zero, so nothing is commanded.
  2. CANCELLED        the offset is non-zero but both targets moved together,
                      so the differential never reached the legs.
  3. CLIPPED          one target is pinned at VMC_LEG_LENGTH_MIN (0.180 m) or
                      MAX (0.385 m).  At short leg lengths the retract side
                      runs out of stroke first, so the pair stops levelling and
                      starts raising the machine instead.
  4. NOT TRACKED      the targets are sensible but the legs do not reach them,
                      so the commanded differential never becomes a real
                      leg-length difference.  Measured on the first low-leg
                      capture: 97 mm commanded, 39 mm achieved.

For (4) the tool also prints the numbers behind it.  The leg-length loop is a
position PD plus a constant support feedforward, and 120 N of feedforward is
worth 60 mm of position error at kp = 2000 N/m.  A retraction demand smaller
than that produces a net EXTENDING force, so small corrections are driven the
wrong way by construction.

Ground contact
    The tool also reports whether each leg was actually CARRYING the machine,
    because a levelling correction can only work on a leg that is on the
    ground.  The observable is joint torque, not leg length: a leg pressing
    into the ground reads high torque, a lifted one collapses to a few N.m.
    The firmware already thresholds this at
    VMC_AIRBORNE_JOINT_TORQUE_THRESHOLD (2 N.m) and latches it per side, so
    `airborne_side_left/right` in the capture is a ready-made contact flag.

    A leg that is off the ground cannot resist the leg-length loop at all: the
    constant support feedforward simply drives it outward, which is how a
    commanded RETRACTION can produce a measured EXTENSION.

This tool reports which of the three happened, and when.

Requires a schema-10 capture (VMCVOFA fields 138..140).  Older captures do not
carry the compensator output at all and are rejected with a clear message.
Schema-11 captures add the ground-following leg extension
(``leg_follow_offset_left/right``); when present that section is reported too.

Usage:
    python3 tools/roll_comp_report.py logs/<stamp>/kalman_samples.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path

# Mirror of VMC_LEG_LENGTH_MIN/MAX in App/Inc/vmc_probe.h.  The measured
# mechanical stop is 0.160 m; the software floor keeps a 20 mm safety margin at
# 0.180 m.
LEG_LENGTH_MIN = 0.180
LEG_LENGTH_MAX = 0.385
# A target this close to the limit is treated as pinned.  The logger quantises
# to 1 mm, so half a millimetre of slack is generous.
CLIP_TOL = 0.0005


def _num(row: dict[str, str], key: str) -> float:
    try:
        return float(row.get(key, "") or "nan")
    except ValueError:
        return float("nan")


def analyse(rows: list[dict[str, str]]) -> dict[str, object]:
    """Summarise the compensator's behaviour over one capture."""
    t = [_num(r, "mcu_time_ms") / 1000.0 for r in rows]
    roll = [_num(r, "roll") for r in rows]
    offset = [_num(r, "roll_comp_offset") for r in rows]
    left = [_num(r, "leg_target_left") for r in rows]
    right = [_num(r, "leg_target_right") for r in rows]

    finite_off = [v for v in offset if math.isfinite(v)]
    finite_roll = [v for v in roll if math.isfinite(v)]
    if not finite_off:
        raise ValueError(
            "capture has no roll_comp_offset: it predates schema 10. "
            "Re-record with the current firmware."
        )

    # The leg lengths themselves, to separate "commanded" from "achieved".
    measured_left = [_num(r, "leg_length_left") for r in rows]
    measured_right = [_num(r, "leg_length_right") for r in rows]

    # Ground contact, per side.  airborne_side_* is the firmware's own latch;
    # the torques are what drives it.
    side_torque_left = [_num(r, "airborne_side_torque_left") for r in rows]
    side_torque_right = [_num(r, "airborne_side_torque_right") for r in rows]
    airborne_left = [_num(r, "airborne_side_active_left") for r in rows]
    airborne_right = [_num(r, "airborne_side_active_right") for r in rows]

    # Ground-following leg extension.  Only schema-11 captures carry these
    # columns; on an older capture every value stays NaN and the section is
    # simply omitted.
    follow_left = [_num(r, "leg_follow_offset_left") for r in rows]
    follow_right = [_num(r, "leg_follow_offset_right") for r in rows]
    follow_present = any(math.isfinite(v) for v in follow_left + follow_right)
    follow_frames = sum(
        1 for a, b in zip(follow_left, follow_right)
        if (math.isfinite(a) and a > 1e-6) or (math.isfinite(b) and b > 1e-6)
    )

    moving = [i for i, v in enumerate(offset) if math.isfinite(v) and abs(v) > 1e-6]
    clipped_left = [
        i for i, v in enumerate(left)
        if math.isfinite(v) and (
            abs(v - LEG_LENGTH_MIN) <= CLIP_TOL or abs(v - LEG_LENGTH_MAX) <= CLIP_TOL
        )
    ]
    clipped_right = [
        i for i, v in enumerate(right)
        if math.isfinite(v) and (
            abs(v - LEG_LENGTH_MIN) <= CLIP_TOL or abs(v - LEG_LENGTH_MAX) <= CLIP_TOL
        )
    ]

    # Differential the targets ask for once the clamp has had its say.
    diff = [
        l - r for l, r in zip(left, right)
        if math.isfinite(l) and math.isfinite(r)
    ]
    # Differential the legs actually produced.
    achieved = [
        l - r for l, r in zip(measured_left, measured_right)
        if math.isfinite(l) and math.isfinite(r)
    ]
    # Tracking error of each leg against its own target.
    err_left = [
        m - t for m, t in zip(measured_left, left) if math.isfinite(m) and math.isfinite(t)
    ]
    err_right = [
        m - t for m, t in zip(measured_right, right) if math.isfinite(m) and math.isfinite(t)
    ]
    # Differential requested: 2 * offset, before clamping.
    want = [
        2.0 * o for o in offset if math.isfinite(o)
    ]

    # Largest sustained stretch where a target sat on a clamp.
    def longest_run(indices: list[int]) -> float:
        if not indices:
            return 0.0
        best = run = 0.0
        prev = None
        for i in indices:
            if prev is not None and i == prev + 1:
                run += (t[i] - t[prev]) if math.isfinite(t[i]) else 0.0
            else:
                run = 0.0
            best = max(best, run)
            prev = i
        return best

    return {
        "samples": len(rows),
        "span_s": (t[-1] - t[0]) if len(t) > 1 else 0.0,
        "roll_abs_max": max((abs(v) for v in finite_roll), default=0.0),
        "offset_abs_max": max((abs(v) for v in finite_off), default=0.0),
        "offset_p99": (
            sorted(abs(v) for v in finite_off)[int(0.99 * (len(finite_off) - 1))]
            if finite_off else 0.0
        ),
        "offset_moving_fraction": len(moving) / len(offset) if offset else 0.0,
        "want_diff_max": max((abs(v) for v in want), default=0.0),
        "commanded_diff_max": max((abs(v) for v in diff), default=0.0),
        "achieved_diff_max": max((abs(v) for v in achieved), default=0.0),
        "err_left_max": max((abs(v) for v in err_left), default=0.0),
        "err_right_max": max((abs(v) for v in err_right), default=0.0),
        "clipped_left_frames": len(clipped_left),
        "clipped_right_frames": len(clipped_right),
        "clipped_total_frames": len(set(clipped_left) | set(clipped_right)),
        "clipped_left_longest_s": longest_run(clipped_left),
        "clipped_right_longest_s": longest_run(clipped_right),
        "load_left_p50": (
            statistics.median([v for v in side_torque_left if math.isfinite(v)])
            if any(math.isfinite(v) for v in side_torque_left) else float("nan")
        ),
        "load_right_p50": (
            statistics.median([v for v in side_torque_right if math.isfinite(v)])
            if any(math.isfinite(v) for v in side_torque_right) else float("nan")
        ),
        "airborne_left_frames": sum(1 for v in airborne_left if v == 1.0),
        "airborne_right_frames": sum(1 for v in airborne_right if v == 1.0),
        "airborne_any_frames": sum(
            1 for a, b in zip(airborne_left, airborne_right)
            if a == 1.0 or b == 1.0
        ),
        "times": t,
        "airborne_right_flag": airborne_right,
        "airborne_left_flag": airborne_left,
        "leg_follow_present": follow_present,
        "leg_follow_left_max": max(
            (v for v in follow_left if math.isfinite(v)), default=float("nan")
        ),
        "leg_follow_right_max": max(
            (v for v in follow_right if math.isfinite(v)), default=float("nan")
        ),
        "leg_follow_frames": follow_frames,
    }


def verdict(result: dict[str, object]) -> list[str]:
    """Turn the numbers into the three-way diagnosis."""
    lines: list[str] = []
    moving = float(result["offset_moving_fraction"])  # type: ignore[arg-type]
    offset_max = float(result["offset_abs_max"])  # type: ignore[arg-type]
    want = float(result["want_diff_max"])  # type: ignore[arg-type]
    commanded = float(result["commanded_diff_max"])  # type: ignore[arg-type]
    achieved = float(result["achieved_diff_max"])  # type: ignore[arg-type]
    clip = int(result["clipped_total_frames"])  # type: ignore[arg-type]
    total = int(result["samples"])  # type: ignore[arg-type]
    roll = float(result["roll_abs_max"])  # type: ignore[arg-type]

    if offset_max < 1e-3:
        lines.append(
            "NOT RESPONDING: the commanded offset never left zero.  Either the "
            "compensator is disabled, the IMU is not valid, or the body never "
            f"tilted enough to move it (peak |roll| {roll:.4f} rad)."
        )
        return lines

    lines.append(
        f"RESPONDING: peak |offset| {offset_max * 1000:.1f} mm, non-zero in "
        f"{moving * 100:.1f}% of frames, with body roll peaking at "
        f"{roll * 1000:.1f} mrad."
    )

    if want > 1e-3 and commanded < want - 1e-3:
        lines.append(
            f"CLIPPED: the requested differential peaked at {want * 1000:.1f} mm "
            f"but only {commanded * 1000:.1f} mm survived the length clamp "
            f"({100.0 * commanded / want:.0f}% of the request)."
        )
    else:
        lines.append(
            f"NOT CLIPPED: the requested differential ({want * 1000:.1f} mm) "
            f"survived the clamp ({commanded * 1000:.1f} mm)."
        )

    # Commanded vs achieved: the legs may simply not follow.
    if commanded > 1e-3:
        ratio = achieved / commanded
        if ratio < 0.7:
            lines.append(
                f"NOT TRACKED: the legs produced only {achieved * 1000:.1f} mm of "
                f"the {commanded * 1000:.1f} mm commanded "
                f"({ratio * 100:.0f}%).  Worst per-leg error: "
                f"left {float(result['err_left_max']) * 1000:.0f} mm, "
                f"right {float(result['err_right_max']) * 1000:.0f} mm.  The "
                "compensator is asking for the right thing; the leg-length loop "
                "is not delivering it."
            )
        else:
            lines.append(
                f"TRACKED: the legs produced {achieved * 1000:.1f} mm of the "
                f"{commanded * 1000:.1f} mm commanded ({ratio * 100:.0f}%)."
            )

    if clip:
        lines.append(
            f"LIMIT CONTACT: {clip}/{total} frames ({100.0 * clip / total:.1f}%) "
            f"had a target pinned at the {LEG_LENGTH_MIN:.3f}/{LEG_LENGTH_MAX:.3f} m "
            f"bound; longest continuous run "
            f"{max(float(result['clipped_left_longest_s']), float(result['clipped_right_longest_s'])):.2f} s.  "
            "While pinned, the pair stops levelling and drives the machine up "
            "or down instead."
        )
    else:
        lines.append(
            "NO LIMIT CONTACT: neither target reached the length bound, so the "
            "stroke was never exhausted on this capture."
        )

    # Ground contact.  A lifted leg cannot be levelled by the length loop.
    total = int(result["samples"])
    air_l = int(result["airborne_left_frames"])
    air_r = int(result["airborne_right_frames"])
    load_l = float(result["load_left_p50"])
    load_r = float(result["load_right_p50"])

    if air_l or air_r:
        which = []
        if air_l:
            which.append(f"LEFT {air_l}/{total} frames ({100.0 * air_l / total:.0f}%)")
        if air_r:
            which.append(f"RIGHT {air_r}/{total} frames ({100.0 * air_r / total:.0f}%)")
        lines.append(
            "LEG OFF THE GROUND: the per-side load latch fired for "
            + "; ".join(which)
            + f".  Median side load was L {load_l:.2f} N.m / R {load_r:.2f} N.m.  "
            "A leg that is not carrying the machine cannot resist the "
            "leg-length loop, so the constant support feedforward drives it "
            "outward - a commanded retraction can then show up as a measured "
            "EXTENSION."
        )
    else:
        lines.append(
            f"BOTH LEGS LOADED: no per-side airborne latch fired.  Median side "
            f"load L {load_l:.2f} N.m / R {load_r:.2f} N.m."
        )

    if bool(result["leg_follow_present"]):
        follow_frames = int(result["leg_follow_frames"])
        follow_l = float(result["leg_follow_left_max"]) * 1000.0
        follow_r = float(result["leg_follow_right_max"]) * 1000.0
        if follow_frames == 0:
            lines.append(
                "LEG FOLLOW IDLE: leg_follow_offset stayed at zero for the "
                "whole capture, so no leg was extended to chase the ground.  "
                "Either the feature is disabled or the per-side load latch "
                "never fired."
            )
        else:
            lines.append(
                f"LEG FOLLOW ACTIVE: offset was non-zero in {follow_frames}/{total} "
                f"frames, peaking at L {follow_l:.1f} mm / R {follow_r:.1f} mm.  "
                "This is the extra outward extension on top of the preset, held "
                "while the leg is unloaded and released once it reloads."
            )

    return lines


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()

    rows = [
        r for r in csv.DictReader(args.csv.open(newline="", encoding="utf-8"))
        if r.get("mcu_time_ms") not in (None, "")
    ]
    if not rows:
        print("no samples", file=sys.stderr)
        return 2

    try:
        result = analyse(rows)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    print(f"{args.csv}")
    print(f"samples {result['samples']}, span {result['span_s']:.1f} s")
    print()
    print(f"  body roll peak        : {float(result['roll_abs_max']):.4f} rad")
    print(f"  offset peak           : {float(result['offset_abs_max']) * 1000:8.1f} mm")
    print(f"  offset p99            : {float(result['offset_p99']) * 1000:8.1f} mm")
    print(f"  offset non-zero       : {float(result['offset_moving_fraction']) * 100:8.1f} % of frames")
    print(f"  differential requested: {float(result['want_diff_max']) * 1000:8.1f} mm")
    print(f"  differential commanded: {float(result['commanded_diff_max']) * 1000:8.1f} mm  (after the length clamp)")
    print(f"  differential achieved : {float(result['achieved_diff_max']) * 1000:8.1f} mm  (measured leg lengths)")
    print(f"  worst leg error       : L {float(result['err_left_max']) * 1000:5.0f} mm   R {float(result['err_right_max']) * 1000:5.0f} mm")
    print(f"  frames on a length limit: {int(result['clipped_total_frames'])}")
    print()
    print("  ground contact")
    print(f"    median side load    : L {float(result['load_left_p50']):6.2f} N.m   R {float(result['load_right_p50']):6.2f} N.m")
    print(f"    latch on (left)     : {int(result['airborne_left_frames'])} frames")
    print(f"    latch on (right)    : {int(result['airborne_right_frames'])} frames")
    print()
    if bool(result["leg_follow_present"]):
        print("  ground-following extension")
        print(
            f"    offset peak         : L {float(result['leg_follow_left_max']) * 1000:6.1f} mm   "
            f"R {float(result['leg_follow_right_max']) * 1000:6.1f} mm"
        )
        print(
            f"    active              : {int(result['leg_follow_frames'])} frames"
        )
        print()
    for line in verdict(result):
        print(f"  {line}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
