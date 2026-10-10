#!/usr/bin/env python3
"""Capture raw slip-study evidence from the RTT telemetry streams.

The firmware emits two independent streams on SEGGER RTT channel 0:

* ``VMCVOFA`` at 50 Hz: the full controller/state-machine context.
* ``VMCFAST`` at 250 Hz: raw wheel, IMU, leg and actuator observations.

Three recording modes are available, so a link problem can be isolated to one
stream instead of always testing both at once:

* ``both`` (default): record VMCVOFA and VMCFAST together.
* ``fast``: record VMCFAST only, with VMCVOFA pushed to its slowest rate.
* ``slow``: record VMCVOFA only, with the fast stream switched off.

The script explicitly disables the slip gate's influence on the Kalman filter
and writes an event file for the operator's real-time labels.

The default keys are:

    c  contact
    s  slip start
    e  slip end
    r  recover
    b  baseline
    q  stop capture

The firmware still computes the existing slip metric while the gate is
disabled.  That lets the recording compare the old metric against the raw
features without the filter changing the data it is supposed to measure.

Usage:
    cd /Users/zhouzhanen/chuantui/aboard
    python3 tools/slip_capture.py --duration 120 --mode both
    python3 tools/slip_capture.py --duration 120 --mode fast
    python3 tools/slip_capture.py --duration 120 --mode slow
    python3 tools/slip_capture.py --source tcp --host 127.0.0.1 --port 19021

Besides the stream rates, the capture can set tunables at runtime over RTT, so
one run both configures the controller and records the result:

* ``--slip-threshold`` / ``--slip-confirm-ms`` / ``--slip-gate``
* ``--leg-follow on|off``            ground-following leg extension
* ``--leg-follow-machine-wide on|off`` either-side-unloaded => both legs
* ``--leg-follow-rate`` / ``--leg-follow-return-rate``

Any option left unset keeps the firmware's current value, and both the
requested and the telemetry-observed values land in ``summary.json``.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import json
import os
import select
import statistics
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.rtt_kalman_capture import (  # noqa: E402
    CSV_FIELDS as SLOW_CSV_FIELDS,
    DEFAULT_DEVICE,
    DEFAULT_HOST,
    DEFAULT_JLINK_SPEED,
    DEFAULT_PORT,
    JLinkSubprocessReader,
    TcpRttReader,
    _sample_to_row,
    parse_vmcvoa_line,
)
from tools.rtt_logger_capture import GdbServerRttReader  # noqa: E402

# Binary VMCFAST.  The wire layout mirrors App/Src/vmc_lqr_rtt.c:
#   0..1   sync 0xFA 0xCE
#   2      version
#   3      total frame length
#   4..7   seq (u32 LE)           frame counter, proves whether the firmware
#                                 emitted a frame or the host merely missed it
#   8..11  t_ms (u32 LE)
#   12..15 loop_count (u32 LE)
#   16..17 loop_dt_us (u16 LE)
#   18..27 10 flag bytes
#   28..87 30 int16 LE scaled scalars
#   88..89 CRC-16/CCITT over bytes 0..87
FAST_SYNC = b"\xfa\xce"
FAST_VERSION = 1
FAST_FRAME_SIZE = 90

# In wire order.  Keep in lockstep with VMC_LQR_RTT_FAST_FLAGS.
FAST_FLAGS = [
    "enabled",
    "dm_state",
    "feedback_timeout_source",
    "airborne_active",
    "airborne_side_left",
    "airborne_side_right",
    "jump_stage",
    "getup_state",
    "slip_left",
    "slip_right",
]

# In wire order: (name, scale).  Keep in lockstep with
# VMC_LQR_RTT_FAST_SCALARS in App/Src/vmc_lqr_rtt.c; a scale of 0 means the
# value is already an integer.  tools/test_slip_capture.py parses the C table
# and fails if these drift apart.
FAST_SCALARS: list[tuple[str, int]] = [
    ("body_pitch", 10000),
    ("body_pitch_rate", 1000),
    ("body_roll", 10000),
    ("body_roll_rate", 1000),
    ("body_yaw_rate", 1000),
    ("body_ax_g", 1000),
    ("body_ax_lin_g", 1000),
    ("kf_dx", 1000),
    # Output-shaft rad/s.  Scale 100 keeps the int16 range at +/-327 rad/s;
    # at scale 1000 the 32.767 rad/s ceiling clipped real slip peaks.
    ("wheel_left_speed", 100),
    ("wheel_left_torque", 1000),
    ("wheel_left_current_output", 0),
    ("wheel_right_speed", 100),
    ("wheel_right_torque", 1000),
    ("wheel_right_current_output", 0),
    ("leg_left_length", 10000),
    ("leg_left_d_length", 1000),
    ("leg_left_phi0", 10000),
    ("leg_left_d_phi0", 1000),
    ("leg_right_length", 10000),
    ("leg_right_d_length", 1000),
    ("leg_right_phi0", 10000),
    ("leg_right_d_phi0", 1000),
    ("joint_left_ahead_torque", 100),
    ("joint_left_back_torque", 100),
    ("joint_right_ahead_torque", 100),
    ("joint_right_back_torque", 100),
    ("slip_metric", 100),
    ("airborne_wheel_speed", 1000),
    ("airborne_joint_torque_min", 100),
    ("airborne_joint_torque_max", 100),
]

FAST_FIELDS = (
    ["t_ms", "seq", "loop_count", "loop_dt_us"] + FAST_FLAGS
    + [name for name, _scale in FAST_SCALARS]
)
FAST_CSV_FIELDS = ["host_time_s"] + FAST_FIELDS

EVENT_KEYS = {
    "b": "baseline",
    "c": "contact",
    "s": "slip_start",
    "e": "slip_end",
    "r": "recover",
}


def _safe_float(text: str) -> float:
    try:
        return float(text)
    except (TypeError, ValueError):
        return float("nan")


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, matching vmc_rtt_crc16() in the firmware."""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def parse_fast_frame(frame: bytes) -> dict[str, float] | None:
    """Validate and decode one binary VMCFAST frame."""
    if len(frame) != FAST_FRAME_SIZE:
        return None
    if frame[0:2] != FAST_SYNC or frame[2] != FAST_VERSION:
        return None
    if frame[3] != FAST_FRAME_SIZE:
        return None
    if crc16_ccitt(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
        return None

    out: dict[str, float] = {
        "seq": float(int.from_bytes(frame[4:8], "little")),
        "t_ms": float(int.from_bytes(frame[8:12], "little")),
        "loop_count": float(int.from_bytes(frame[12:16], "little")),
        "loop_dt_us": float(int.from_bytes(frame[16:18], "little")),
    }
    for index, name in enumerate(FAST_FLAGS):
        out[name] = float(frame[18 + index])
    base = 18 + len(FAST_FLAGS)
    for index, (name, scale) in enumerate(FAST_SCALARS):
        raw = int.from_bytes(frame[base + 2 * index:base + 2 * index + 2], "little",
                             signed=True)
        out[name] = float(raw) if scale == 0 else raw / float(scale)
    return out


class RttDemux:
    """Split the shared RTT byte stream into VMCVOFA lines and VMCFAST frames.

    Both streams share up-buffer 0.  VMCVOFA is still ASCII terminated by
    newline; VMCFAST is a fixed-length binary frame introduced by a two-byte
    sync word.  A capture can attach mid-stream, so the parser resynchronises
    on either prefix and counts discarded bytes.
    """

    SLOW_PREFIX = b"VMCVOFA"

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.resync_bytes = 0
        self.bad_frames = 0
        self.fast_frames = 0
        self.slow_lines = 0

    def feed(self, data: bytes) -> list[tuple[str, object]]:
        self.buffer.extend(data)
        out: list[tuple[str, object]] = []
        while self.buffer:
            if self.buffer.startswith(self.SLOW_PREFIX):
                newline = self.buffer.find(b"\n")
                if newline < 0:
                    break
                raw = bytes(self.buffer[:newline])
                del self.buffer[: newline + 1]
                out.append(("slow", raw.decode("utf-8", errors="replace").strip()))
                self.slow_lines += 1
                continue

            if self.buffer.startswith(FAST_SYNC):
                if len(self.buffer) < 4:
                    break
                length = self.buffer[3]
                if length != FAST_FRAME_SIZE or self.buffer[2] != FAST_VERSION:
                    del self.buffer[0]
                    self.resync_bytes += 1
                    self.bad_frames += 1
                    continue
                if len(self.buffer) < length:
                    break
                frame = bytes(self.buffer[:length])
                del self.buffer[:length]
                sample = parse_fast_frame(frame)
                if sample is None:
                    self.bad_frames += 1
                    # Skip only the sync word so a following valid frame is
                    # still found.
                    continue
                self.fast_frames += 1
                out.append(("fast", sample))
                continue

            fast_at = self.buffer.find(FAST_SYNC)
            slow_at = self.buffer.find(self.SLOW_PREFIX)
            candidates = [pos for pos in (fast_at, slow_at) if pos >= 0]
            if not candidates:
                self.resync_bytes += len(self.buffer)
                self.buffer.clear()
                break
            advance = min(candidates)
            self.resync_bytes += advance
            del self.buffer[:advance]
        return out


def enable_payload(period_ms: int, slow_period_ms: int) -> str:
    """Enable the fast stream while making the slip-gate action explicit."""
    return _enable_payload("both", period_ms, slow_period_ms)


# The full VMCVOFA frame is ~915 B (151 fields at schema 15).  The firmware
# clamps its period to at most 200 ms, so 5 Hz is the closest no-reflash
# approximation of "fast stream only"; it leaves under 2% of the combined link
# load in place.
FAST_MODE_SLOW_PERIOD_MS = 200


def _enable_payload(
    mode: str,
    period_ms: int,
    slow_period_ms: int,
    slip_threshold: float | None = None,
    slip_confirm_ms: float | None = None,
    slip_gate_active: bool = False,
    leg_follow_enable: bool | None = None,
    leg_follow_rate: float | None = None,
    leg_follow_return_rate: float | None = None,
    leg_follow_machine_wide: bool | None = None,
    leg_follow_auto_scope: bool | None = None,
    leg_follow_roll_deadband: float | None = None,
    leg_follow_episode_timeout_ms: float | None = None,
    leg_follow_scope_confirm_ms: float | None = None,
) -> str:
    """Build the RTT parameter payload for one capture.

    ``slip_threshold`` / ``slip_confirm_ms`` set the detector's condition-C
    threshold and confirmation time at runtime, so a threshold can be tried
    without a reflash.  ``slip_gate_active`` chooses whether the resulting
    latch is allowed to reject the wheel measurement feeding the Kalman
    filter; a pure data-collection run leaves it off so the recording cannot
    influence what it is measuring.

    ``leg_follow_*`` set the ground-following leg extension at runtime.  The
    firmware accepts these keys in any ``vmc_balance_params`` payload, so a
    single capture can pin the feature on or off, change the outward / return
    rates, and switch between per-leg and whole-machine scope.  Whatever is
    left as ``None`` keeps the firmware's current value, so a capture never
    silently resets a parameter the operator did not name.
    """
    payload: dict[str, object] = {
        "type": "vmc_balance_params",
        "slip_enable": 1 if slip_gate_active else 0,
    }
    if slip_threshold is not None:
        payload["slip_accel"] = float(slip_threshold)
    if slip_confirm_ms is not None:
        payload["slip_confirm_ms"] = float(slip_confirm_ms)
    if leg_follow_enable is not None:
        payload["leg_follow_enable"] = 1 if leg_follow_enable else 0
    if leg_follow_rate is not None:
        payload["leg_follow_rate"] = float(leg_follow_rate)
    if leg_follow_return_rate is not None:
        payload["leg_follow_return_rate"] = float(leg_follow_return_rate)
    if leg_follow_machine_wide is not None:
        payload["leg_follow_machine_wide"] = 1 if leg_follow_machine_wide else 0
    if leg_follow_auto_scope is not None:
        payload["leg_follow_auto_scope"] = 1 if leg_follow_auto_scope else 0
    if leg_follow_roll_deadband is not None:
        payload["leg_follow_roll_deadband"] = float(leg_follow_roll_deadband)
    if leg_follow_episode_timeout_ms is not None:
        payload["leg_follow_episode_timeout_ms"] = float(
            leg_follow_episode_timeout_ms
        )
    if leg_follow_scope_confirm_ms is not None:
        payload["leg_follow_scope_confirm_ms"] = float(
            leg_follow_scope_confirm_ms
        )

    if mode == "slow":
        payload.update(
            {"fast_enable": 0, "telem_period_ms": slow_period_ms}
        )
    elif mode == "fast":
        payload.update(
            {
                "fast_enable": 1,
                "fast_period_ms": period_ms,
                "telem_period_ms": FAST_MODE_SLOW_PERIOD_MS,
            }
        )
    else:
        payload.update(
            {
                "fast_enable": 1,
                "fast_period_ms": period_ms,
                "telem_period_ms": slow_period_ms,
            }
        )
    return json.dumps(payload, separators=(",", ":"))


def disable_payload() -> str:
    return '{"type":"vmc_balance_params","fast_enable":0}'


def send_with_ack(
    reader: Any,
    payload: str,
    expected_ack: str,
    timeout_s: float = 10.0,
) -> bool:
    """Send one RTT parameter payload and wait until the firmware confirms it."""
    reader.send(payload)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for line in reader.read_lines():
            if expected_ack in line:
                return True
        time.sleep(0.002)
    return False


class MarkerReader:
    """Read single-key event markers without waiting for Enter."""

    def __init__(self, enabled: bool) -> None:
        self.enabled = enabled and sys.stdin.isatty()
        self._fd: int | None = None
        self._original_settings: list[Any] | None = None

    def open(self) -> bool:
        if not self.enabled:
            return False
        try:
            self._fd = sys.stdin.fileno()
            self._original_settings = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        except (OSError, termios.error):
            self._fd = None
            self._original_settings = None
            self.enabled = False
        return self.enabled

    def read(self) -> tuple[str | None, bool]:
        """Return (event_name, stop_requested)."""
        if not self.enabled or self._fd is None:
            return None, False
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None, False
        try:
            raw = os.read(self._fd, 1)
        except OSError:
            return None, False
        if not raw:
            return None, False
        key = raw.decode("utf-8", errors="ignore").lower()
        if key == "q":
            return None, True
        return EVENT_KEYS.get(key), False

    def close(self) -> None:
        if self._fd is not None and self._original_settings is not None:
            try:
                termios.tcsetattr(
                    self._fd, termios.TCSADRAIN, self._original_settings
                )
            except (OSError, termios.error):
                pass
        self._fd = None
        self._original_settings = None


def open_reader(args: argparse.Namespace) -> Any:
    """Open a writable RTT transport so the fast stream can be enabled."""
    errors: list[str] = []
    if args.source == "gdbserver":
        # Default path: measured at 100% coverage where pylink recovered only
        # 76-81% of frames even at 10 Hz.  The telnet relay is bidirectional,
        # so the same socket carries the enable/disable payloads.
        reader = GdbServerRttReader(
            server=args.gdb_server,
            device=args.device,
            speed=args.speed,
            gdb=args.gdb,
            gdb_port=args.gdb_port,
            rtt_port=args.rtt_port,
            connect_timeout=args.connect_timeout,
            server_log=getattr(args, "jlink_log", None),
        )
        print(
            f"RTT source: JLinkGDBServerCLExe RTT telnet "
            f"{args.rtt_port} ({args.device} @ {args.speed} kHz)"
        )
        return reader

    if args.source in ("auto", "tcp"):
        try:
            reader = TcpRttReader(args.host, args.port, args.connect_timeout)
            print(f"RTT source: TCP {args.host}:{args.port}")
            return reader
        except OSError as exc:
            errors.append(f"TCP {args.host}:{args.port}: {exc}")
            if args.source == "tcp":
                raise

    if args.source in ("auto", "jlink"):
        try:
            reader = JLinkSubprocessReader(args.device, args.speed, writable=True)
            print(f"RTT source: direct J-Link {args.device} @ {args.speed} kHz")
            return reader
        except Exception as exc:  # noqa: BLE001 - include transport details
            errors.append(f"J-Link {args.device}: {exc}")

    raise RuntimeError("no writable RTT source available; " + "; ".join(errors))


def _stream_summary(times_ms: list[float], target_period_ms: float) -> dict[str, Any]:
    # Keep only a strictly increasing firmware timebase.  A truncated or
    # interleaved RTT line can decode as a wildly wrong timestamp; without
    # this filter one bad sample turns a 20 ms interval into a fake
    # multi-minute gap and hides the real loss.
    finite: list[float] = []
    invalid = 0
    for value in times_ms:
        if value != value or (finite and value <= finite[-1]):
            invalid += 1
            continue
        finite.append(value)
    if len(finite) < 2:
        return {"samples": len(times_ms), "invalid_samples": invalid}
    intervals = [
        later - earlier
        for earlier, later in zip(finite, finite[1:])
        if later > earlier
    ]
    span_s = (finite[-1] - finite[0]) * 0.001
    result: dict[str, Any] = {
        "samples": len(times_ms),
        "invalid_samples": invalid,
        "span_s": span_s,
        "rate_hz": (len(finite) - 1) / span_s if span_s > 0.0 else 0.0,
    }
    if intervals:
        result["median_period_ms"] = statistics.median(intervals)
        result["max_period_ms"] = max(intervals)
        if target_period_ms > 0.0:
            # Count lost frames against the device's own cadence, not the
            # requested period.  The firmware gates telemetry on
            # "elapsed >= period", so a 20 ms request lands on 21 ms ticks;
            # treating every 21 ms interval as a shortfall would report ~5%
            # loss on a stream that lost nothing.  A genuinely lost frame
            # appears as a gap that is a multiple of the observed unit.
            # Tolerate one tick of HAL quantisation (21 ms for a 20 ms
            # request) while still catching uniform downsampling: take the
            # smaller of the observed cadence and the requested period.
            observed_ms = float(round(statistics.median(intervals)))
            unit_ms = (
                min(observed_ms, target_period_ms)
                if observed_ms > 0.0
                else target_period_ms
            )
            if unit_ms <= 0.0:
                unit_ms = target_period_ms
            lost = sum(
                max(0, int(round(value / unit_ms)) - 1) for value in intervals
            )
            total = len(finite) + lost
            result["observed_unit_ms"] = unit_ms
            result["estimated_missing_frames"] = lost
            result["coverage_pct"] = (
                100.0 * len(finite) / total if total > 0 else 0.0
            )
            result["gap_count"] = sum(
                1 for value in intervals if int(round(value / unit_ms)) > 1
            )
            result["largest_gap_ms"] = max(intervals)
    return result


def summarize_capture(
    fast_times_ms: list[float],
    slow_times_ms: list[float],
    events: list[dict[str, Any]],
    requested_fast_period_ms: float,
    requested_slow_period_ms: float,
    fast_loop_counts: list[float] | None = None,
    mode: str = "both",
    unparsed_lines: int = 0,
    reader_restarts: int = 0,
    stalls: list[dict[str, float]] | None = None,
    demux_stats: dict[str, int] | None = None,
    slip_requested: dict[str, Any] | None = None,
    slip_applied: dict[str, Any] | None = None,
    leg_follow_requested: dict[str, Any] | None = None,
    leg_follow_applied: dict[str, Any] | None = None,
) -> dict[str, Any]:
    fast_summary = _stream_summary(fast_times_ms, requested_fast_period_ms)
    if fast_loop_counts and len(fast_loop_counts) == len(fast_times_ms):
        usable = [
            (time_ms, count)
            for time_ms, count in zip(fast_times_ms, fast_loop_counts)
            if time_ms == time_ms and count == count
        ]
        if len(usable) >= 2:
            elapsed_s = (usable[-1][0] - usable[0][0]) * 0.001
            if elapsed_s > 0.0:
                fast_summary["control_loop_rate_hz"] = (
                    usable[-1][1] - usable[0][1]
                ) / elapsed_s
    return {
        "mode": mode,
        "fast": fast_summary,
        "slow": _stream_summary(slow_times_ms, requested_slow_period_ms),
        "events": events,
        "unparsed_lines": unparsed_lines,
        "reader_restarts": reader_restarts,
        "link_stalls": stalls or [],
        "demux": demux_stats or {},
        "slip_requested": slip_requested or {},
        "slip_applied": slip_applied or {},
        "leg_follow_requested": leg_follow_requested or {},
        "leg_follow_applied": leg_follow_applied or {},
    }


def _print_stream(name: str, summary: dict[str, Any]) -> None:
    """Print coverage honestly: a single 4 s stall is not a small percentage."""
    print(f"{name} samples: {summary.get('samples', 0)}")
    print(f"{name} rate:    {summary.get('rate_hz', 0.0):.2f} Hz")
    if "coverage_pct" in summary:
        print(
            f"{name} coverage:{summary['coverage_pct']:6.2f}% "
            f"(unit {summary['observed_unit_ms']:.0f} ms, "
            f"{summary['estimated_missing_frames']} frames missing, "
            f"{summary['gap_count']} gaps, "
            f"largest {summary['largest_gap_ms']:.0f} ms)"
        )
    if name == "fast" and "control_loop_rate_hz" in summary:
        print(f"control loop: {summary['control_loop_rate_hz']:.2f} Hz")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument(
        "--mode",
        choices=("both", "fast", "slow"),
        default="both",
        help=(
            "both = VMCVOFA + VMCFAST; "
            "fast = VMCFAST only (VMCVOFA at 5 Hz); "
            "slow = VMCVOFA only (VMCFAST disabled)"
        ),
    )
    parser.add_argument("--fast-rate", type=float, default=250.0)
    parser.add_argument("--slow-period-ms", type=int, default=20)
    parser.add_argument(
        "--slip-threshold",
        type=float,
        default=None,
        help=(
            "runtime condition-C threshold (firmware default 33.0); "
            "sets slip_accel over RTT, no reflash needed"
        ),
    )
    parser.add_argument(
        "--slip-confirm-ms",
        type=float,
        default=None,
        help="runtime confirmation time in ms (firmware default 5)",
    )
    parser.add_argument(
        "--slip-gate",
        action="store_true",
        help=(
            "let the detector latch reject the wheel measurement feeding the "
            "Kalman filter; off by default so the recording cannot affect "
            "what it measures"
        ),
    )
    parser.add_argument(
        "--leg-follow",
        choices=("on", "off"),
        default=None,
        help=(
            "runtime ground-following leg extension (firmware default on); "
            "on = an unloaded leg extends so it keeps chasing the ground, "
            "off = hold the S2 preset"
        ),
    )
    parser.add_argument(
        "--leg-follow-machine-wide",
        choices=("on", "off"),
        default=None,
        help=(
            "fixed follow scope, used only when --leg-follow-auto-scope off; "
            "on = if EITHER side unloads, BOTH legs extend; off = only the "
            "unloaded leg extends"
        ),
    )
    parser.add_argument(
        "--leg-follow-auto-scope",
        choices=("on", "off"),
        default=None,
        help=(
            "on (firmware default) = choose the follow scope per unload "
            "episode from the roll direction: unloaded side HIGH means the "
            "machine was lifted (extend both), LOW means it went over an edge "
            "(extend only that leg). off = use --leg-follow-machine-wide"
        ),
    )
    parser.add_argument(
        "--leg-follow-roll-deadband",
        type=float,
        default=None,
        help=(
            "|roll| below this is treated as level, so the previous episode's "
            "scope is kept (firmware default 0.05 rad)"
        ),
    )
    parser.add_argument(
        "--leg-follow-episode-timeout-ms",
        type=float,
        default=None,
        help=(
            "how long an unload episode survives with nothing latched, so the "
            "reach is not unwound before it arrives (firmware default 3000)"
        ),
    )
    parser.add_argument(
        "--leg-follow-scope-confirm-ms",
        type=float,
        default=None,
        help=(
            "how long a low-side classification must hold before it is "
            "believed; one frame is not evidence (firmware default 250)"
        ),
    )
    parser.add_argument(
        "--leg-follow-rate",
        type=float,
        default=None,
        help=(
            "outward chase rate in m/s while a leg is unloaded "
            "(firmware default 0.2)"
        ),
    )
    parser.add_argument(
        "--leg-follow-return-rate",
        type=float,
        default=None,
        help=(
            "rate in m/s at which the extra extension returns to the preset "
            "once the leg loads again (firmware default 0.05)"
        ),
    )
    parser.add_argument(
        "--source",
        choices=("gdbserver", "auto", "tcp", "jlink"),
        default="gdbserver",
        help=(
            "gdbserver (default) relays RTT through JLinkGDBServerCLExe and "
            "measured 100%% coverage; auto/tcp/jlink keep the older pylink "
            "paths, which dropped 19-24%% of frames on this machine"
        ),
    )
    parser.add_argument(
        "--gdb-server",
        default="/Applications/SEGGER/JLink/JLinkGDBServerCLExe",
    )
    parser.add_argument("--gdb", default="arm-none-eabi-gdb")
    parser.add_argument("--gdb-port", type=int, default=2331)
    parser.add_argument("--rtt-port", type=int, default=19021)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--speed", type=int, default=DEFAULT_JLINK_SPEED)
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "logs")
    parser.add_argument("--label", default="", help="optional run label in the summary")
    parser.add_argument(
        "--no-restore",
        action="store_true",
        help="leave the fast stream enabled when capture ends",
    )
    parser.add_argument(
        "--no-markers",
        action="store_true",
        help="disable the single-key event marker controls",
    )
    args = parser.parse_args()

    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    fast_period_ms = max(2, int(round(1000.0 / args.fast_rate)))
    actual_fast_rate = 1000.0 / fast_period_ms
    if abs(actual_fast_rate - args.fast_rate) > 1.0e-9:
        print(
            f"note: firmware period is whole milliseconds; "
            f"{args.fast_rate:g} Hz -> {fast_period_ms} ms -> "
            f"{actual_fast_rate:.1f} Hz"
        )
    if not (2 <= args.slow_period_ms <= 200):
        parser.error("--slow-period-ms must be in [2, 200]")
    if args.slip_threshold is not None and not (0.0 <= args.slip_threshold <= 1.0e4):
        parser.error("--slip-threshold must be in [0, 10000]")
    if args.slip_confirm_ms is not None and not (0.0 <= args.slip_confirm_ms <= 1000.0):
        parser.error("--slip-confirm-ms must be in [0, 1000]")
    if args.leg_follow_rate is not None and not (0.0 <= args.leg_follow_rate <= 1.0):
        parser.error("--leg-follow-rate must be in [0, 1] m/s")
    if args.leg_follow_return_rate is not None and not (
        0.0 <= args.leg_follow_return_rate <= 1.0
    ):
        parser.error("--leg-follow-return-rate must be in [0, 1] m/s")
    if args.leg_follow_roll_deadband is not None and not (
        0.0 <= args.leg_follow_roll_deadband <= 0.5
    ):
        parser.error("--leg-follow-roll-deadband must be in [0, 0.5] rad")
    if args.leg_follow_episode_timeout_ms is not None and not (
        0.0 <= args.leg_follow_episode_timeout_ms <= 60000.0
    ):
        parser.error("--leg-follow-episode-timeout-ms must be in [0, 60000]")
    if args.leg_follow_scope_confirm_ms is not None and not (
        0.0 <= args.leg_follow_scope_confirm_ms <= 5000.0
    ):
        parser.error("--leg-follow-scope-confirm-ms must be in [0, 5000]")
    leg_follow_enable = (
        None if args.leg_follow is None else (args.leg_follow == "on")
    )
    leg_follow_machine_wide = (
        None
        if args.leg_follow_machine_wide is None
        else (args.leg_follow_machine_wide == "on")
    )
    leg_follow_auto_scope = (
        None
        if args.leg_follow_auto_scope is None
        else (args.leg_follow_auto_scope == "on")
    )
    record_fast = args.mode in ("both", "fast")
    record_slow = args.mode in ("both", "slow")
    active_slow_period_ms = (
        FAST_MODE_SLOW_PERIOD_MS if args.mode == "fast" else args.slow_period_ms
    )

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.output_dir / f"slip_{stamp}"
    out_dir.mkdir(parents=True, exist_ok=True)
    fast_path = out_dir / "fast_samples.csv"
    slow_path = out_dir / "kalman_samples.csv"
    event_path = out_dir / "events.csv"
    summary_path = out_dir / "summary.json"
    # Keep the J-Link GDB server's own output next to the capture so a link
    # stall can be told apart from a host-side problem after the fact.
    args.jlink_log = out_dir / "jlink_server.log"

    reader: Any | None = None
    marker = MarkerReader(enabled=not args.no_markers)
    marker.open()
    events: list[dict[str, Any]] = []
    fast_times_ms: list[float] = []
    fast_loop_counts: list[float] = []
    slow_times_ms: list[float] = []
    last_fast_t_ms = float("nan")
    last_slow_t_ms = float("nan")
    last_flush = time.monotonic()
    unparsed_lines = 0
    demux = RttDemux()
    applied_threshold: float | None = None
    applied_gate: float | None = None
    applied_leg_follow_left: float | None = None
    applied_leg_follow_right: float | None = None

    try:
        reader = open_reader(args)
        if not send_with_ack(
            reader,
            _enable_payload(
                args.mode,
                fast_period_ms,
                args.slow_period_ms,
                args.slip_threshold,
                args.slip_confirm_ms,
                args.slip_gate,
                leg_follow_enable,
                args.leg_follow_rate,
                args.leg_follow_return_rate,
                leg_follow_machine_wide,
                leg_follow_auto_scope,
                args.leg_follow_roll_deadband,
                args.leg_follow_episode_timeout_ms,
                args.leg_follow_scope_confirm_ms,
            ),
            "VMC_BALANCE_PARAMS OK",
        ):
            raise RuntimeError(
                "firmware did not acknowledge the stream enable command"
            )
        if args.mode == "both":
            print(
                "enabled dual stream: "
                f"VMCFAST {actual_fast_rate:.1f} Hz, "
                f"VMCVOFA {1000.0 / args.slow_period_ms:.1f} Hz, "
                "slip gate action off"
            )
        elif args.mode == "fast":
            print(
                "enabled fast stream only: "
                f"VMCFAST {actual_fast_rate:.1f} Hz, "
                f"VMCVOFA reduced to {1000.0 / FAST_MODE_SLOW_PERIOD_MS:.1f} Hz, "
                "slip gate action off"
            )
        else:
            print(
                "enabled slow stream only: "
                f"VMCVOFA {1000.0 / args.slow_period_ms:.1f} Hz, "
                "VMCFAST off, slip gate action off"
            )
        if args.slip_threshold is not None or args.slip_confirm_ms is not None:
            print(
                "slip detector set: "
                + (
                    f"threshold {args.slip_threshold:g}"
                    if args.slip_threshold is not None
                    else "threshold unchanged"
                )
                + ", "
                + (
                    f"confirm {args.slip_confirm_ms:g} ms"
                    if args.slip_confirm_ms is not None
                    else "confirm unchanged"
                )
                + (", gate ACTIVE" if args.slip_gate else ", gate action off")
            )
        if (
            leg_follow_enable is not None
            or leg_follow_machine_wide is not None
            or leg_follow_auto_scope is not None
            or args.leg_follow_rate is not None
            or args.leg_follow_return_rate is not None
            or args.leg_follow_roll_deadband is not None
            or args.leg_follow_episode_timeout_ms is not None
            or args.leg_follow_scope_confirm_ms is not None
        ):
            print(
                "leg follow set: "
                + (
                    f"enable {1 if leg_follow_enable else 0}"
                    if leg_follow_enable is not None
                    else "enable unchanged"
                )
                + ", "
                + (
                    f"auto_scope {1 if leg_follow_auto_scope else 0}"
                    if leg_follow_auto_scope is not None
                    else "auto_scope unchanged"
                )
                + ", "
                + (
                    f"machine_wide {1 if leg_follow_machine_wide else 0}"
                    if leg_follow_machine_wide is not None
                    else "machine_wide unchanged"
                )
                + ", "
                + (
                    f"roll_deadband {args.leg_follow_roll_deadband:g} rad"
                    if args.leg_follow_roll_deadband is not None
                    else "roll_deadband unchanged"
                )
                + ", "
                + (
                    f"episode_timeout {args.leg_follow_episode_timeout_ms:g} ms"
                    if args.leg_follow_episode_timeout_ms is not None
                    else "episode_timeout unchanged"
                )
                + ", "
                + (
                    f"scope_confirm {args.leg_follow_scope_confirm_ms:g} ms"
                    if args.leg_follow_scope_confirm_ms is not None
                    else "scope_confirm unchanged"
                )
                + ", "
                + (
                    f"rate {args.leg_follow_rate:g} m/s"
                    if args.leg_follow_rate is not None
                    else "rate unchanged"
                )
                + ", "
                + (
                    f"return {args.leg_follow_return_rate:g} m/s"
                    if args.leg_follow_return_rate is not None
                    else "return unchanged"
                )
            )
        if marker.enabled:
            print("markers: c=contact s=slip_start e=slip_end r=recover b=baseline q=stop")
        print(f"capturing {args.duration:.1f} s ...")
        time.sleep(0.2)
        # Drop any pre-capture handshake gap before counting link stalls, so a
        # stall report always refers to the recorded window.
        if hasattr(reader, "mark_capture_start"):
            reader.mark_capture_start()
        start = time.monotonic()

        with contextlib.ExitStack() as stack:
            fast_handle = None
            slow_handle = None
            if record_fast:
                fast_handle = stack.enter_context(
                    fast_path.open(
                        "w", newline="", encoding="utf-8", buffering=1024 * 1024
                    )
                )
            if record_slow:
                slow_handle = stack.enter_context(
                    slow_path.open(
                        "w", newline="", encoding="utf-8", buffering=1024 * 1024
                    )
                )
            event_handle = stack.enter_context(
                event_path.open("w", newline="", encoding="utf-8")
            )
            fast_writer = (
                csv.DictWriter(fast_handle, fieldnames=FAST_CSV_FIELDS)
                if fast_handle is not None
                else None
            )
            slow_writer = (
                csv.DictWriter(slow_handle, fieldnames=SLOW_CSV_FIELDS)
                if slow_handle is not None
                else None
            )
            event_writer = csv.DictWriter(
                event_handle,
                fieldnames=["host_time_s", "fast_t_ms", "slow_t_ms", "event"],
            )
            if fast_writer is not None:
                fast_writer.writeheader()
            if slow_writer is not None:
                slow_writer.writeheader()
            event_writer.writeheader()

            while True:
                now = time.monotonic()
                elapsed = now - start
                if elapsed >= args.duration:
                    break

                for kind, payload in demux.feed(reader.read_bytes()):
                    if kind == "fast":
                        if not record_fast:
                            continue
                        fast = payload
                        if fast["t_ms"] == fast["t_ms"]:
                            last_fast_t_ms = fast["t_ms"]
                            fast_times_ms.append(last_fast_t_ms)
                            fast_loop_counts.append(fast["loop_count"])
                        row = {"host_time_s": elapsed, **fast}
                        fast_writer.writerow(row)
                        continue

                    if not record_slow:
                        continue
                    parsed = parse_vmcvoa_line(payload, "format")
                    if parsed is None:
                        if payload:
                            unparsed_lines += 1
                        continue
                    _profile, sample = parsed
                    if sample.mcu_time_ms == sample.mcu_time_ms:
                        last_slow_t_ms = sample.mcu_time_ms
                        slow_times_ms.append(last_slow_t_ms)
                    # Read the detector settings back out of telemetry so the
                    # summary proves what the firmware actually applied rather
                    # than only what was requested.
                    if sample.slip_accel_threshold == sample.slip_accel_threshold:
                        applied_threshold = sample.slip_accel_threshold
                    if sample.slip_enable == sample.slip_enable:
                        applied_gate = sample.slip_enable
                    if sample.leg_follow_offset_left == sample.leg_follow_offset_left:
                        applied_leg_follow_left = sample.leg_follow_offset_left
                    if sample.leg_follow_offset_right == sample.leg_follow_offset_right:
                        applied_leg_follow_right = sample.leg_follow_offset_right
                    row = _sample_to_row(sample)
                    row["host_time_s"] = elapsed
                    slow_writer.writerow(row)

                event, stop_requested = marker.read()
                if event is not None:
                    entry = {
                        "host_time_s": elapsed,
                        "fast_t_ms": last_fast_t_ms,
                        "slow_t_ms": last_slow_t_ms,
                        "event": event,
                    }
                    events.append(entry)
                    event_writer.writerow(entry)
                    event_handle.flush()
                    stamp_text = (
                        f"fast {last_fast_t_ms:.1f} ms"
                        if record_fast
                        else f"slow {last_slow_t_ms:.1f} ms"
                    )
                    print(f"marker {event} at host {elapsed:.3f}s {stamp_text}")
                if stop_requested:
                    break

                if now - last_flush >= 1.0:
                    # Do not flush the high-rate CSVs from this loop.  A
                    # synchronous flush can stall long enough for the 64 kB
                    # RTT buffer to overflow, which looks exactly like a
                    # multi-second telemetry gap.  The event log is tiny and
                    # must remain immediately durable for labeling.
                    event_handle.flush()
                    last_flush = now
                time.sleep(0.0005)
    except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
        print(f"capture failed: {exc}", file=sys.stderr)
        return 3
    finally:
        if reader is not None and not args.no_restore:
            try:
                if send_with_ack(
                    reader,
                    disable_payload(),
                    "VMC_BALANCE_PARAMS OK",
                ):
                    print("streams restored; fast disabled, slip gate action off")
                else:
                    print(
                        "warning: firmware did not acknowledge the disable command",
                        file=sys.stderr,
                    )
            except Exception as exc:  # noqa: BLE001 - best effort restore
                print(f"could not disable fast stream: {exc}", file=sys.stderr)
        if reader is not None:
            reader.close()
        marker.close()

    summary = summarize_capture(
        fast_times_ms,
        slow_times_ms,
        events,
        float(fast_period_ms),
        float(active_slow_period_ms),
        fast_loop_counts,
        args.mode,
        unparsed_lines,
        int(getattr(reader, "restarts", 0)),
        [
            {"at_s": round(start_time, 3), "duration_s": round(duration, 3)}
            for start_time, duration in getattr(reader, "stalls", [])
        ],
        demux_stats={
            "resync_bytes": demux.resync_bytes,
            "bad_frames": demux.bad_frames,
            "fast_frames": demux.fast_frames,
            "slow_lines": demux.slow_lines,
        },
        slip_requested={
            "threshold": args.slip_threshold,
            "confirm_ms": args.slip_confirm_ms,
            "gate_active": bool(args.slip_gate),
        },
        slip_applied={
            "threshold": applied_threshold,
            "gate_active": applied_gate,
        },
        leg_follow_requested={
            "enable": leg_follow_enable,
            "machine_wide": leg_follow_machine_wide,
            "auto_scope": leg_follow_auto_scope,
            "roll_deadband": args.leg_follow_roll_deadband,
            "episode_timeout_ms": args.leg_follow_episode_timeout_ms,
            "scope_confirm_ms": args.leg_follow_scope_confirm_ms,
            "rate": args.leg_follow_rate,
            "return_rate": args.leg_follow_return_rate,
        },
        leg_follow_applied={
            "offset_left": applied_leg_follow_left,
            "offset_right": applied_leg_follow_right,
        },
    )
    summary["label"] = args.label
    summary["recorded_fast"] = record_fast
    summary["recorded_slow"] = record_slow
    summary["fast_period_ms"] = fast_period_ms
    summary["slow_period_ms"] = active_slow_period_ms
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print()
    print(f"mode:         {args.mode}")
    if record_fast:
        _print_stream("fast", summary["fast"])
    if record_slow:
        _print_stream("slow", summary["slow"])
    print(f"events:       {len(events)}")
    if summary["unparsed_lines"]:
        print(f"unparsed RTT lines: {summary['unparsed_lines']}")
    demux = summary.get("demux", {})
    if demux.get("bad_frames") or demux.get("resync_bytes"):
        print(
            f"framing:      {demux.get('bad_frames', 0)} bad frames, "
            f"{demux.get('resync_bytes', 0)} resync bytes skipped"
        )
    if summary["reader_restarts"]:
        print(f"J-Link reconnects:  {summary['reader_restarts']}")
    requested = summary.get("slip_requested", {})
    applied = summary.get("slip_applied", {})
    if requested.get("threshold") is not None or requested.get("confirm_ms") is not None:
        line = "slip detector:"
        if applied.get("threshold") is not None:
            line += f" threshold applied {applied['threshold']:g}"
        line += f" gate {'on' if applied.get('gate_active') else 'off'}"
        print(line)
    if summary["link_stalls"]:
        total = sum(item["duration_s"] for item in summary["link_stalls"])
        print(
            f"link stalls:  {len(summary['link_stalls'])} "
            f"({total:.1f}s silent total; "
            f"longest {max(i['duration_s'] for i in summary['link_stalls']):.1f}s)"
        )
        for item in summary["link_stalls"][:10]:
            print(
                f"   host {item['at_s']:7.2f}s silenced for "
                f"{item['duration_s']:.2f}s"
            )
    if record_fast:
        print(f"Fast CSV:     {fast_path}")
    if record_slow:
        print(f"Slow CSV:     {slow_path}")
    print(f"Events:       {event_path}")
    print(f"Summary:      {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
