#!/usr/bin/env python3
"""Capture VMCVOFA through SEGGER tools instead of pylink.

The pylink path reads RTT through ``JLINK_RTTERMINAL_Read``.  Repeated
captures on this machine sometimes only recovered ~76-81% of the expected
VMCVOFA frames even at 10 Hz, which is too little load for bandwidth to be the
cause.  This script isolates the host read path with two non-pylink sources:

``gdbserver`` (default)
    JLinkGDBServerCLExe relays RTT channel 0 to a TCP telnet port, and a
    plain socket reads it.  A batch ``arm-none-eabi-gdb`` attaches only to
    resume the core; it never touches RTT.

``logger``
    SEGGER's JLinkRTTLogger.  Measured on this J-Link (V9.48) it fails with
    "RTT Control Block not found" even though the block is present and valid
    at ``_SEGGER_RTT``, so it is kept only for comparison.

Both sources are read-only: they cannot send the RTT parameter payloads that
enable VMCFAST, so the fast stream must already be off (the ``slow``
single-channel case).  Any VMCFAST lines that do appear are counted and
reported rather than silently dropped.

A capture can begin with stale RTT data buffered before the host attached.
That segment usually ends in a multi-second gap, so analysis keeps the longest
contiguous segment and reports what it discarded.

Usage:
    cd /Users/zhouzhanen/chuantui/aboard
    python3 tools/rtt_logger_capture.py --duration 60
    python3 tools/rtt_logger_capture.py --duration 60 --source logger
    python3 tools/rtt_logger_capture.py --duration 60 --label gdbserver_60s

Analyse an existing raw log without touching hardware:
    python3 tools/rtt_logger_capture.py --input logs/gdbserver_rtt_.../raw.log
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import statistics
import subprocess
import sys
import time
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.rtt_kalman_capture import (  # noqa: E402
    CSV_FIELDS as SLOW_CSV_FIELDS,
    DEFAULT_DEVICE,
    DEFAULT_JLINK_SPEED,
    _sample_to_row,
    parse_vmcvoa_line,
)

DEFAULT_LOGGER = "/Applications/SEGGER/JLink/JLinkRTTLogger"
DEFAULT_GDB_SERVER = "/Applications/SEGGER/JLink/JLinkGDBServerCLExe"
DEFAULT_GDB = "arm-none-eabi-gdb"
DEFAULT_GDB_PORT = 2331
DEFAULT_RTT_CHANNEL = 0
SLOW_PERIOD_MS = 20.0
# A gap this long separates the stale pre-attach RTT segment from the live one.
SEGMENT_GAP_MS = 1000.0


def _finite(value: float) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(value)


def coverage_summary(
    times_ms: list[float], target_period_ms: float
) -> dict[str, Any]:
    """Coverage with the same monotonic filter used by slip_capture."""
    finite: list[float] = []
    invalid = 0
    for value in times_ms:
        if not _finite(value) or (finite and value <= finite[-1]):
            invalid += 1
            continue
        finite.append(value)
    result: dict[str, Any] = {
        "samples": len(times_ms),
        "invalid_samples": invalid,
    }
    if len(finite) < 2:
        return result
    intervals = [b - a for a, b in zip(finite, finite[1:]) if b > a]
    span_s = (finite[-1] - finite[0]) * 0.001
    # Measure loss against the device's own cadence, not the requested period.
    # HAL_GetTick gates telemetry on "elapsed >= period", so a 20 ms request
    # lands on 21 ms ticks.  Counting every 21 ms interval as a shortfall would
    # report ~5% loss on a stream that lost nothing.  A lost frame instead
    # shows up as a gap that is a multiple of the observed unit.
    # Tolerate one tick of HAL quantisation (21 ms for a 20 ms request) while
    # still catching uniform downsampling: take the smaller of the observed
    # cadence and the requested period.
    observed_ms = float(round(statistics.median(intervals))) if intervals else 0.0
    unit_ms = min(observed_ms, target_period_ms) if observed_ms > 0.0 else target_period_ms
    if unit_ms <= 0.0:
        unit_ms = target_period_ms
    lost = sum(max(0, int(round(value / unit_ms)) - 1) for value in intervals)
    total = len(finite) + lost
    result.update(
        span_s=span_s,
        rate_hz=(len(finite) - 1) / span_s if span_s > 0.0 else 0.0,
        median_period_ms=statistics.median(intervals) if intervals else 0.0,
        max_period_ms=max(intervals) if intervals else 0.0,
        observed_unit_ms=unit_ms,
        estimated_missing_frames=lost,
        coverage_pct=100.0 * len(finite) / total if total > 0 else 0.0,
        gap_count=sum(
            1 for value in intervals if int(round(value / unit_ms)) > 1
        ),
        interval_histogram={
            str(int(round(value))): count
            for value, count in Counter(intervals).most_common(8)
        },
    )
    return result


def parse_log(path: Path) -> dict[str, Any]:
    """Split a raw JLinkRTTLogger log into VMCVOFA / VMCFAST / other lines."""
    slow_samples: list[dict[str, Any]] = []
    slow_times: list[float] = []
    fast_lines = 0
    other_lines = 0
    total_lines = 0

    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for raw in handle:
            line = raw.strip()
            if not line:
                continue
            total_lines += 1
            if line.startswith("VMCFAST"):
                fast_lines += 1
                continue
            parsed = parse_vmcvoa_line(line, "format")
            if parsed is None:
                other_lines += 1
                continue
            _profile, sample = parsed
            # A truncated line still "parses" into a profile with every
            # channel invalid.  Only count it as a real frame when the
            # firmware timestamp is present.
            if not _finite(sample.mcu_time_ms):
                other_lines += 1
                continue
            slow_samples.append(sample)
            slow_times.append(sample.mcu_time_ms)

    return {
        "total_lines": total_lines,
        "vmcvoa_lines": len(slow_samples),
        "vmcfast_lines": fast_lines,
        "other_lines": other_lines,
        "slow_samples": slow_samples,
        "slow_times_ms": slow_times,
    }


def write_slow_csv(path: Path, samples: list[Any]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=SLOW_CSV_FIELDS)
        writer.writeheader()
        for sample in samples:
            row = _sample_to_row(sample)
            row["host_time_s"] = ""
            writer.writerow(row)


def start_logger(
    logger: str, device: str, speed: int, channel: int, path: Path
) -> Any:
    return subprocess.Popen(
        [
            logger,
            "-Device",
            device,
            "-If",
            "SWD",
            "-Speed",
            str(speed),
            "-RTTChannel",
            str(channel),
            str(path),
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def longest_segment_indices(times_ms: list[float]) -> tuple[int, int, int]:
    """Return (start, end, session_count) of the longest contiguous run.

    The end index is exclusive.  A telemetry capture attaches to a live RTT
    buffer, so it often begins with a short stale segment written before the
    host connected; that segment ends with a gap far larger than the nominal
    period.
    """
    if not times_ms:
        return 0, 0, 0
    starts = [0]
    for index in range(1, len(times_ms)):
        if times_ms[index] - times_ms[index - 1] > SEGMENT_GAP_MS:
            starts.append(index)
    bounds = list(zip(starts, starts[1:] + [len(times_ms)]))
    best_start, best_end = max(bounds, key=lambda pair: pair[1] - pair[0])
    return best_start, best_end, len(bounds)


class GdbServerRttReader:
    """Read and write RTT channel 0 through a J-Link GDB server telnet port.

    ``JLinkGDBServerCLExe`` relays the target's RTT channel 0 to a TCP telnet
    port; the relay is bidirectional, so parameter payloads can be written to
    the same socket that carries the telemetry back.  A batch
    ``arm-none-eabi-gdb`` attaches only to resume the core - it never reads
    RTT itself.

    Compared with pylink's ``JLINK_RTTERMINAL_Read`` path, which recovered
    only 76-81% of frames even at 10 Hz on this machine, this path measured
    100% coverage with no missing frame over 58.7 s.
    """

    def __init__(
        self,
        server: str = DEFAULT_GDB_SERVER,
        device: str = DEFAULT_DEVICE,
        speed: int = DEFAULT_JLINK_SPEED,
        gdb: str = DEFAULT_GDB,
        gdb_port: int = DEFAULT_GDB_PORT,
        rtt_port: int = 19021,
        connect_timeout: float = 15.0,
        server_log: Path | None = None,
    ) -> None:
        self.server = server
        self.device = device
        self.speed = speed
        self.gdb = gdb
        self.gdb_port = gdb_port
        self.rtt_port = rtt_port
        self.connect_timeout = connect_timeout
        self.server_log = server_log
        self.restarts = 0
        # Stalls are reported relative to reader construction so the numbers
        # are usable as capture offsets instead of raw monotonic clock values.
        self._t0 = time.monotonic()
        # Wall-clock stretches with no telemetry at all.  A long one overflows
        # the 64 kB RTT buffer and permanently loses the frames written while
        # the host was not draining it.
        self.stalls: list[tuple[float, float]] = []
        self._last_data_time: float | None = None
        self._capture_started = False
        self._server_proc: subprocess.Popen[bytes] | None = None
        self._gdb_proc: subprocess.Popen[bytes] | None = None
        self._socket: socket.socket | None = None
        self._buffer = bytearray()
        self._start()

    def _start(self) -> None:
        command = [
            self.server,
            "-device",
            self.device,
            "-if",
            "SWD",
            "-speed",
            str(self.speed),
            "-RTTTelnetPort",
            str(self.rtt_port),
            "-silent",
            "-nogui",
        ]
        if self.server_log is not None:
            self._server_log_handle = self.server_log.open("w", encoding="utf-8")
        else:
            self._server_log_handle = subprocess.DEVNULL
        self._server_proc = subprocess.Popen(
            command,
            stdout=self._server_log_handle,
            stderr=subprocess.STDOUT,
        )

        deadline = time.monotonic() + self.connect_timeout
        while time.monotonic() < deadline:
            if self._server_proc.poll() is not None:
                raise RuntimeError("JLinkGDBServerCLExe exited during startup")
            try:
                with socket.create_connection(
                    ("127.0.0.1", self.gdb_port), timeout=0.5
                ):
                    break
            except OSError:
                time.sleep(0.2)
        else:
            self.close()
            raise TimeoutError("GDB server did not open its port")

        # Batch GDB only resumes the core; it never reads RTT.
        self._gdb_proc = subprocess.Popen(
            [
                self.gdb,
                "-q",
                "-batch",
                "-ex",
                "set confirm off",
                "-ex",
                f"target remote :{self.gdb_port}",
                "-ex",
                "continue",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(1.5)

        self._socket = socket.create_connection(
            ("127.0.0.1", self.rtt_port), timeout=5.0
        )
        self._socket.settimeout(0.05)
        self._buffer = bytearray()
        self.wait_for_telemetry()

    def wait_for_telemetry(self, timeout_s: float | None = None) -> bool:
        """Block until the target is actually emitting RTT frames.

        Attaching with batch GDB resumes the core, but it takes roughly two
        seconds to take effect.  Commands sent while the core is still halted
        are never processed by the firmware, so wait for real telemetry before
        letting a caller write to the RTT down buffer.
        """
        deadline = time.monotonic() + (
            self.connect_timeout if timeout_s is None else timeout_s
        )
        while time.monotonic() < deadline:
            for line in self.read_lines():
                if line.startswith("VMCVOFA") or line.startswith("VMCFAST"):
                    return True
            time.sleep(0.01)
        raise TimeoutError(
            "no RTT telemetry after attaching; the core may still be halted"
        )

    def read_bytes(self) -> bytes:
        """Return whatever raw bytes arrived, or b"" when nothing is pending.

        VMCFAST is binary, so callers that need it must work on bytes; the
        line-oriented API below is only a convenience for the ASCII stream.
        """
        if self._socket is None:
            return b""
        try:
            chunk = self._socket.recv(65536)
        except (socket.timeout, TimeoutError):
            return b""
        except OSError:
            return b""
        if not chunk:
            return b""
        now = time.monotonic()
        if self._last_data_time is not None:
            silent = now - self._last_data_time
            # Only report stalls inside the capture window.  Attaching, the
            # GDB server's own target init and the enable handshake all happen
            # before the recording starts and are not telemetry loss.
            if silent > 1.0 and self._capture_started:
                self.stalls.append(
                    (self._last_data_time - self._t0, silent)
                )
        self._last_data_time = now
        return chunk

    def mark_capture_start(self) -> None:
        """Start counting link stalls and timestamps from now."""
        self.stalls.clear()
        self._last_data_time = time.monotonic()
        self._t0 = self._last_data_time
        self._capture_started = True

    def read_lines(self) -> list[str]:
        lines: list[str] = []
        self._buffer.extend(self.read_bytes())
        while b"\n" in self._buffer:
            raw, _, remainder = self._buffer.partition(b"\n")
            self._buffer = bytearray(remainder)
            lines.append(raw.decode("utf-8", errors="replace").strip())
        return lines

    def send(self, line: str) -> None:
        if self._socket is None:
            raise RuntimeError("RTT telnet socket is not open")
        payload = (line.rstrip("\n") + "\n").encode("utf-8")
        try:
            self._socket.sendall(payload)
        except OSError as exc:
            raise RuntimeError(f"RTT send failed: {exc}") from exc

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None
        for proc in (self._gdb_proc, self._server_proc):
            if proc is None:
                continue
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except Exception:  # noqa: BLE001 - best effort shutdown
                    proc.kill()
                    try:
                        proc.wait(timeout=3)
                    except Exception:  # noqa: BLE001
                        pass
        self._gdb_proc = None
        self._server_proc = None
        handle = getattr(self, "_server_log_handle", None)
        if handle not in (None, subprocess.DEVNULL):
            try:
                handle.close()
            except OSError:
                pass
        self._server_log_handle = None


def capture_gdbserver(
    server: str,
    device: str,
    speed: int,
    gdb: str,
    gdb_port: int,
    rtt_port: int,
    duration: float,
    raw_path: Path,
) -> int:
    """Relay RTT through JLinkGDBServerCLExe and a resuming batch GDB."""
    reader = None
    try:
        reader = GdbServerRttReader(
            server=server,
            device=device,
            speed=speed,
            gdb=gdb,
            gdb_port=gdb_port,
            rtt_port=rtt_port,
        )
        start = time.monotonic()
        with raw_path.open("wb") as handle:
            while time.monotonic() - start < duration:
                for line in reader.read_lines():
                    handle.write((line + "\n").encode("utf-8"))
        return 0
    except (OSError, RuntimeError, TimeoutError) as exc:
        print(f"gdbserver capture failed: {exc}", file=sys.stderr)
        return 3
    finally:
        if reader is not None:
            reader.close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument(
        "--source",
        choices=("gdbserver", "logger"),
        default="gdbserver",
        help=(
            "gdbserver: JLinkGDBServerCLExe + plain socket (works); "
            "logger: JLinkRTTLogger (cannot locate the control block here)"
        ),
    )
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--speed", type=int, default=DEFAULT_JLINK_SPEED)
    parser.add_argument("--channel", type=int, default=DEFAULT_RTT_CHANNEL)
    parser.add_argument("--logger", default=DEFAULT_LOGGER)
    parser.add_argument("--gdb-server", default=DEFAULT_GDB_SERVER)
    parser.add_argument("--gdb", default=DEFAULT_GDB)
    parser.add_argument("--gdb-port", type=int, default=DEFAULT_GDB_PORT)
    parser.add_argument("--rtt-port", type=int, default=19021)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "logs")
    parser.add_argument("--label", default="")
    parser.add_argument(
        "--input",
        type=Path,
        default=None,
        help="analyse an existing raw logger file instead of capturing",
    )
    args = parser.parse_args()

    if args.input is not None:
        raw_path = args.input
        out_dir = raw_path.parent
        label = args.label or raw_path.stem
        print(f"analysing {raw_path}")
    else:
        if args.duration <= 0.0:
            parser.error("--duration must be positive")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = args.output_dir / f"rttlogger_{stamp}"
        out_dir.mkdir(parents=True, exist_ok=True)
        raw_path = out_dir / "raw.log"
        label = args.label or stamp

        if args.source == "gdbserver":
            if not Path(args.gdb_server).exists():
                print(
                    f"JLinkGDBServerCLExe not found at {args.gdb_server}",
                    file=sys.stderr,
                )
                return 4
            print(
                f"capturing via JLinkGDBServerCLExe RTT telnet port "
                f"{args.rtt_port}: {args.device} @ {args.speed} kHz, "
                f"{args.duration:.1f} s"
            )
            status = capture_gdbserver(
                args.gdb_server,
                args.device,
                args.speed,
                args.gdb,
                args.gdb_port,
                args.rtt_port,
                args.duration,
                raw_path,
            )
            if status != 0:
                return status
        else:
            if not Path(args.logger).exists():
                print(
                    f"JLinkRTTLogger not found at {args.logger}", file=sys.stderr
                )
                return 4
            print(
                f"starting JLinkRTTLogger: {args.device} @ {args.speed} kHz, "
                f"channel {args.channel}, {args.duration:.1f} s"
            )
            proc = start_logger(
                args.logger, args.device, args.speed, args.channel, raw_path
            )
            try:
                deadline = time.monotonic() + args.duration
                while time.monotonic() < deadline:
                    if proc.poll() is not None:
                        print(
                            f"JLinkRTTLogger exited early with code "
                            f"{proc.returncode}",
                            file=sys.stderr,
                        )
                        break
                    time.sleep(0.1)
            finally:
                if proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=3)
                    except Exception:  # noqa: BLE001 - best effort shutdown
                        proc.kill()
                        proc.wait(timeout=3)

    if not raw_path.exists() or raw_path.stat().st_size == 0:
        print(f"no data written to {raw_path}", file=sys.stderr)
        return 3

    parsed = parse_log(raw_path)
    all_times = parsed["slow_times_ms"]
    seg_start, seg_end, session_count = longest_segment_indices(all_times)
    summary = coverage_summary(all_times[seg_start:seg_end], SLOW_PERIOD_MS)
    summary["sessions_in_log"] = session_count
    summary["discarded_samples"] = len(all_times) - (seg_end - seg_start)
    summary["label"] = label
    summary["raw_log"] = str(raw_path)
    summary["total_lines"] = parsed["total_lines"]
    summary["vmcvoa_lines"] = parsed["vmcvoa_lines"]
    summary["vmcfast_lines"] = parsed["vmcfast_lines"]
    summary["other_lines"] = parsed["other_lines"]

    csv_path = out_dir / "kalman_samples.csv"
    summary_path = out_dir / "summary.json"
    write_slow_csv(csv_path, parsed["slow_samples"][seg_start:seg_end])
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    print()
    print(f"label:        {label}")
    print(f"raw bytes:    {raw_path.stat().st_size}")
    print(f"total lines:  {summary['total_lines']}")
    print(f"VMCVOFA lines:{summary['vmcvoa_lines']}")
    print(f"VMCFAST lines:{summary['vmcfast_lines']}")
    print(f"other lines:  {summary['other_lines']}")
    if summary["sessions_in_log"] > 1:
        print(
            f"segments:     {summary['sessions_in_log']} "
            f"(kept the longest; discarded {summary['discarded_samples']} "
            "pre-attach samples)"
        )
    if "coverage_pct" in summary:
        print(f"span:         {summary['span_s']:.2f} s")
        print(f"rate:         {summary['rate_hz']:.2f} Hz")
        print(
            f"coverage:     {summary['coverage_pct']:.2f}% "
            f"({summary['estimated_missing_frames']} frames missing)"
        )
        print(
            f"max gap:      {summary['max_period_ms']:.0f} ms"
        )
        print(f"intervals:    {summary['interval_histogram']}")
    print(f"CSV:          {csv_path}")
    print(f"Summary:      {summary_path}")
    return 0 if summary.get("coverage_pct", 0.0) >= 95.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
