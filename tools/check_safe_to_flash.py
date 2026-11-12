#!/usr/bin/env python3
"""Refuse to let you flash a live controller.

Flashing resets the MCU.  If the legs are enabled at that moment the reset
lands in an undefined state and the controller stops driving them mid-motion,
so the robot drops.  Run this BEFORE the J-Link script.

Two independent kinds of evidence are accepted, and either one is enough:

1. The operator has actually commanded a stop.

   The firmware gates EVERY joint and wheel output on
   ``remote_control_allows_joint_enable()``, which requires the remote to be
   valid, its frames fresh, and SBUS channel 4 to be in the enabled DOWN
   position.  So either of the following states forces the whole drive chain -
   leg torque, wheel current and the DM motor state machine - to zero:

     * channel 4 moved UP to -1 (emergency stop; telemetry remote_s1 == 2)
     * the remote is offline (``rc_source == 0``, RC_SOURCE_NONE)

   Both are read straight out of telemetry, so they are positive evidence of
   the operator's command rather than an inference.

2. The gated outputs themselves read idle.

   This is the fallback for when the remote state cannot be read.  Most of the
   stream is NOT usable for it: ``left_tp_cmd`` and ``left_wheel_torque_raw``
   are the LQR's DEMAND, computed every cycle even while the controller is
   disabled, so they read large on a dead machine.  The signals that are
   genuinely gated are:

     * ``left_wheel_current`` / ``right_wheel_current`` - the final CAN output,
       which the firmware forces to zero when the controller is disabled.
     * ``airborne_joint_torque_max`` - the torque the DM motors REPORT BACK.
       With the controller disabled the motors are disabled too, so this is
       zero; a machine holding its own weight cannot read near zero.

Switch note: ``remote_s1`` is synthesized from new-remote SBUS channel 4 on
this layout.  It reads 2 when channel 4 is UP (-1, emergency stop).  The
historic no-switch fallback remains for older telemetry layouts.

Exit status:
    0   safe to flash (operator commanded stop, or outputs idle)
    1   NOT safe - the controller is enabled
    2   could not tell - no telemetry

Usage:
    python3 tools/check_safe_to_flash.py
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
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

# Remote-control states, mirroring App/Inc/remote_control.h.
RC_SOURCE_NONE = 0.0
RC_SW_DOWN = 2.0

# The operator's command has to hold for most of the window; a single stale or
# mis-parsed frame must not decide the verdict in either direction.
RC_STATE_FRAME_FRACTION = 0.9
# ...and there have to be enough frames for a fraction to mean anything.  At
# 50 Hz this is 0.4 s.  Below it the operator-command paths stay closed and the
# verdict falls back to the gated outputs.
RC_STATE_MIN_FRAMES = 20
# A forward jump in the MCU timestamp means the J-Link handed over whatever was
# still buffered from an earlier session.  That prefix describes a different
# moment in time - often one where the robot was still driving - and must not
# vote on whether it is safe to reset now.
STALE_JUMP_MS = 1000.0


def _finite(value: float) -> bool:
    return value == value and value not in (float("inf"), float("-inf"))


@dataclass
class Probe:
    """Everything the verdict needs, accumulated over one sampling window."""

    frames: int = 0
    rejected: int = 0
    peaks: dict[str, float] = field(default_factory=dict)
    active: dict[str, int] = field(default_factory=dict)
    rc_offline_frames: int = 0
    s1_frames: int = 0
    s1_down_frames: int = 0
    dropped_stale_frames: int = 0
    last_mcu_ms: float | None = None

    def _restart_window(self, dropped: int) -> None:
        """Throw away everything accumulated before a stale prefix."""
        self.dropped_stale_frames += dropped
        self.frames = 0
        self.rejected = 0
        self.peaks.clear()
        self.active.clear()
        self.rc_offline_frames = 0
        self.s1_frames = 0
        self.s1_down_frames = 0

    def observe(self, sample) -> None:
        stamp = getattr(sample, "mcu_time_ms", float("nan"))
        if _finite(stamp):
            if (
                self.last_mcu_ms is not None
                and stamp - self.last_mcu_ms > STALE_JUMP_MS
            ):
                self._restart_window(self.frames)
            self.last_mcu_ms = stamp

        self.frames += 1

        # Corrupt/partial lines decode to nonsense magnitudes; count them for
        # reporting but never let them drive the verdict.
        for raw, ceiling in (
            (sample.airborne_joint_torque_max, JOINT_TORQUE_MAX_NM),
            (sample.left_wheel_current, WHEEL_CURRENT_MAX_RAW),
            (sample.right_wheel_current, WHEEL_CURRENT_MAX_RAW),
        ):
            if _finite(raw) and abs(raw) > ceiling:
                self.rejected += 1

        if _finite(sample.rc_source) and sample.rc_source == RC_SOURCE_NONE:
            self.rc_offline_frames += 1

        # Only meaningful on a layout that actually carries the switch.
        switch = getattr(sample, "remote_s1", float("nan"))
        if _finite(switch):
            self.s1_frames += 1
            if switch == RC_SW_DOWN:
                self.s1_down_frames += 1

        for name, value in _activity(sample).items():
            self.peaks[name] = max(self.peaks.get(name, 0.0), value)
            limit = {
                "measured joint torque": JOINT_TORQUE_ACTIVE_NM,
                "wheel output current": WHEEL_CURRENT_ACTIVE_RAW,
            }[name]
            if value > limit:
                self.active[name] = self.active.get(name, 0) + 1

    def rc_offline(self) -> bool:
        return (
            self.frames >= RC_STATE_MIN_FRAMES
            and self.rc_offline_frames >= RC_STATE_FRAME_FRACTION * self.frames
        )

    def s1_down(self) -> bool:
        return (
            self.s1_frames >= RC_STATE_MIN_FRAMES
            and self.s1_down_frames >= RC_STATE_FRAME_FRACTION * self.s1_frames
        )

    def switched_on(self) -> bool:
        """The remote is present and not commanding a stop."""
        return self.s1_frames > 0 and not self.s1_down()

    def busy(self) -> list[str]:
        out = []
        for name in ("measured joint torque", "wheel output current"):
            hot = self.active.get(name, 0)
            if self.frames and hot / self.frames >= ACTIVE_FRAME_FRACTION:
                out.append(f"{name} active in {hot / self.frames:.0%} of frames")
        return out


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

    # A probe that cannot be opened is "could not tell", not a crash: the
    # caller has to treat it as unsafe and ask the operator, exactly like the
    # no-telemetry case below.
    try:
        reader = JLinkSubprocessReader(args.device, args.speed, False)
    except Exception as exc:  # noqa: BLE001 - report, never traceback
        print(
            f"CANNOT TELL: could not open the J-Link probe ({exc}).\n"
            "  -> Ask the operator to confirm the controller is disabled "
            "before flashing.",
            file=sys.stderr,
        )
        return 2

    probe = Probe()
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            for line in reader.read_lines():
                parsed = parse_vmcvoa_line(line, "format")
                if parsed is None:
                    continue
                _profile, sample = parsed
                probe.observe(sample)
            time.sleep(0.002)
    finally:
        reader.close()

    if not probe.frames:
        print(
            "CANNOT TELL: no telemetry.  The board may be off.\n"
            "  -> Ask the operator to confirm the controller is disabled "
            "before flashing.",
            file=sys.stderr,
        )
        return 2

    print(f"{probe.frames} frames over {args.seconds:.0f} s; peak activity:")
    if probe.dropped_stale_frames:
        print(
            f"  (dropped {probe.dropped_stale_frames} buffered frames from an "
            "earlier session)"
        )
    if probe.rejected:
        print(f"  ({probe.rejected} out-of-range samples ignored as corrupt)")
    limits = {
        "measured joint torque": JOINT_TORQUE_ACTIVE_NM,
        "wheel output current": WHEEL_CURRENT_ACTIVE_RAW,
    }
    for name, limit in limits.items():
        peak = probe.peaks.get(name, 0.0)
        hot = probe.active.get(name, 0)
        fraction = hot / probe.frames
        flag = "ACTIVE" if fraction >= ACTIVE_FRAME_FRACTION else "idle"
        print(
            f"  {name:<14} peak {peak:8.3f}  (>{limit:g} in "
            f"{hot}/{probe.frames} frames = {fraction:.1%})  {flag}"
        )
    if probe.s1_frames:
        print(
            f"  channel 4     STOP in {probe.s1_down_frames}/{probe.s1_frames} "
            f"frames = {probe.s1_down_frames / probe.s1_frames:.1%}"
        )
    else:
        print("  channel 4     not carried by this telemetry layout")
    print(
        f"  remote source offline in {probe.rc_offline_frames}/"
        f"{probe.frames} frames = {probe.rc_offline_frames / probe.frames:.1%}"
    )

    busy = probe.busy()

    # 1. The operator commanded a stop: channel 4 up, or the remote is absent.  Both
    #    force the firmware to disable the whole drive chain.
    if probe.s1_down():
        print(
            "\nSAFE TO FLASH: SBUS channel 4 reads UP for the whole window, so the "
            "firmware is forcing the joints and wheels to zero."
        )
        if busy:
            print(
                "  WARNING: gated outputs still read active ("
                + "; ".join(busy)
                + ") while channel 4 is UP.  That should be impossible - check the "
                "firmware before trusting this.",
                file=sys.stderr,
            )
        return 0

    if probe.rc_offline():
        print(
            "\nSAFE TO FLASH: the remote is offline, so remote_control_"
            "allows_joint_enable() is false and the firmware disables the "
            "joints and wheels."
        )
        if busy:
            print(
                "  WARNING: gated outputs still read active ("
                + "; ".join(busy)
                + ") with the remote offline.  That should be impossible - "
                "check the firmware before trusting this.",
                file=sys.stderr,
            )
        return 0

    # 2. The remote is present and NOT commanding a stop.
    if probe.switched_on():
        print(
            "\nNOT SAFE TO FLASH: the remote is online and channel 4 is enabled, so "
            "the controller is enabled.\n"
            "  -> Have the operator move channel 4 UP (emergency stop) or power "
            "the transmitter off, and make sure the robot is supported, then "
            "run this again.",
            file=sys.stderr,
        )
        return 1

    # 3. Fallback: judge the gated outputs directly.
    if busy:
        print(
            "\nNOT SAFE TO FLASH: the controller is live - "
            + "; ".join(busy)
            + ".\n"
            "  -> Have the operator move channel 4 UP (emergency stop) or switch "
            "the transmitter off, make sure the robot is supported, then run "
            "this again.",
            file=sys.stderr,
        )
        return 1

    print(
        "\nSAFE TO FLASH: no joint torque and no wheel drive over the whole "
        "window, so the controller is disabled."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
