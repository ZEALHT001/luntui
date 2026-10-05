#!/usr/bin/env python3
"""Refuse to let you flash a live controller.

Flashing resets the MCU.  If the legs are enabled at that moment the reset
lands in an undefined state and the controller stops driving them mid-motion,
so the robot drops.  Run this BEFORE the J-Link script.

This branch's telemetry does not carry the transmitter switches, so the check
is indirect.  It has to look at signals that the enable gate actually drives,
and most of the stream is not that: `left_tp_cmd` and `left_wheel_torque_raw`
are the LQR's DEMAND, computed every cycle even while the controller is
disabled, so they read large on a dead machine and are useless here.

The two signals that are genuinely gated are:

  * `left_wheel_current` / `right_wheel_current` - the final CAN output, which
    the firmware forces to zero when the controller is disabled.
  * `airborne_joint_torque_max` - the torque the DM motors REPORT BACK.  With
    the controller disabled the motors are disabled too, so this is zero; a
    machine holding its own weight cannot read near zero.

If neither shows anything for the whole window, the controller is disabled.

On a branch whose telemetry carries `remote_s1` (see FORMAT_PROFILE), the
switch is checked directly, which is stronger - it proves the operator
commanded a stop rather than inferring it from the absence of output.

Exit status:
    0   safe to flash (S1 down, controller disabled)
    1   NOT safe - the controller is enabled
    2   could not tell - no telemetry

Usage:
    python3 tools/check_safe_to_flash.py
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.rtt_kalman_capture import (  # noqa: E402
    DEFAULT_DEVICE,
    DEFAULT_JLINK_SPEED,
    JLinkSubprocessReader,
    parse_vmcvoa_line,
)

# Anything above these counts as "the controller is working".  Both signals are
# gated by the controller enable, so the thresholds only have to clear noise.
JOINT_TORQUE_ACTIVE_NM = 1.0
WHEEL_CURRENT_ACTIVE_RAW = 50.0

# Physical ceilings, used to discard corrupt frames rather than believe them.
# A partial RTT line decodes as a nonsense number - one capture read a peak
# wheel current of 259150 against a 16000 hardware limit, which would otherwise
# have blocked every flash. Anything past these is noise, not evidence.
JOINT_TORQUE_MAX_NM = 54.0
WHEEL_CURRENT_MAX_RAW = 16000.0

# A live controller holds the body CONTINUOUSLY, so activity has to be
# sustained to count.  Judging on a single frame is not safe either way: an
# isolated glitch read 2.2 N.m on a switched-off machine and would have blocked
# a legitimate flash, while one good frame is not proof that the legs are
# driving.  Require this fraction of the window to be active.
ACTIVE_FRAME_FRACTION = 0.05


def _activity(sample) -> dict[str, float]:
    """Largest plausible magnitude of each gated signal in one frame."""
    return {
        "measured joint torque": max(
            _plausible(sample.airborne_joint_torque_min, JOINT_TORQUE_MAX_NM),
            _plausible(sample.airborne_joint_torque_max, JOINT_TORQUE_MAX_NM),
        ),
        "wheel output current": max(
            _plausible(sample.left_wheel_current, WHEEL_CURRENT_MAX_RAW),
            _plausible(sample.right_wheel_current, WHEEL_CURRENT_MAX_RAW),
        ),
    }


def _plausible(value: float, ceiling: float) -> float:
    """Magnitude of value, or 0 if it is NaN or beyond the hardware ceiling."""
    if value != value:
        return 0.0
    value = abs(value)
    return value if value <= ceiling else 0.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--speed", type=int, default=DEFAULT_JLINK_SPEED)
    args = parser.parse_args()

    reader = JLinkSubprocessReader(args.device, args.speed, False)
    peaks: dict[str, float] = {}
    active_frames: dict[str, int] = {}
    frames = 0
    rejected = 0
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            for line in reader.read_lines():
                parsed = parse_vmcvoa_line(line, "format")
                if parsed is None:
                    continue
                _profile, sample = parsed
                frames += 1
                for name, raw, ceiling in (
                    ("joint torque", sample.airborne_joint_torque_max,
                     JOINT_TORQUE_MAX_NM),
                    ("wheel current", sample.left_wheel_current,
                     WHEEL_CURRENT_MAX_RAW),
                    ("wheel current", sample.right_wheel_current,
                     WHEEL_CURRENT_MAX_RAW),
                ):
                    if raw == raw and abs(raw) > ceiling:
                        rejected += 1
                # Direct check on branches whose telemetry carries the switch.
                switch = getattr(sample, "remote_s1", float("nan"))
                if switch == switch and switch != 2.0:
                    print(
                        f"NOT SAFE TO FLASH: S1 reads {switch:g}, not DOWN (2), "
                        f"so the controller is enabled.",
                        file=sys.stderr,
                    )
                    return 1
                for name, value in _activity(sample).items():
                    peaks[name] = max(peaks.get(name, 0.0), value)
                    limit = {
                        "measured joint torque": JOINT_TORQUE_ACTIVE_NM,
                        "wheel output current": WHEEL_CURRENT_ACTIVE_RAW,
                    }[name]
                    if value > limit:
                        active_frames[name] = active_frames.get(name, 0) + 1
            time.sleep(0.002)
    finally:
        reader.close()

    if not frames:
        print(
            "CANNOT TELL: no telemetry.  The board may be off.\n"
            "  -> Ask the operator to confirm the controller is disabled "
            "before flashing.",
            file=sys.stderr,
        )
        return 2

    print(f"{frames} frames over {args.seconds:.0f} s; peak activity:")
    if rejected:
        print(f"  ({rejected} out-of-range samples ignored as corrupt)")
    limits = {
        "measured joint torque": JOINT_TORQUE_ACTIVE_NM,
        "wheel output current": WHEEL_CURRENT_ACTIVE_RAW,
    }
    busy = []
    for name, limit in limits.items():
        peak = peaks.get(name, 0.0)
        hot = active_frames.get(name, 0)
        fraction = hot / frames if frames else 0.0
        sustained = fraction >= ACTIVE_FRAME_FRACTION
        flag = "ACTIVE" if sustained else "idle"
        print(
            f"  {name:<14} peak {peak:8.3f}  (>{limit:g} in "
            f"{hot}/{frames} frames = {fraction:.1%})  {flag}"
        )
        if sustained:
            busy.append(f"{name} active in {fraction:.0%} of frames")

    if busy:
        print(
            "\nNOT SAFE TO FLASH: the controller is live - " + "; ".join(busy) + ".\n"
            "  -> Have the operator move S1 to DOWN (emergency stop) and make "
            "sure the robot is supported, then run this again.",
            file=sys.stderr,
        )
        return 1

    print(
        "\nSAFE TO FLASH: no joint torque, no Tp command and no wheel drive "
        "over the whole window, so the controller is disabled."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
