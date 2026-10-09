#!/usr/bin/env python3
"""Tests for the dual-stream slip capture helper."""

from __future__ import annotations

import json
import re
import struct
import unittest
from pathlib import Path

from tools.slip_capture import (
    FAST_FLAGS,
    FAST_FIELDS,
    FAST_FRAME_SIZE,
    FAST_MODE_SLOW_PERIOD_MS,
    FAST_SCALARS,
    FAST_SYNC,
    FAST_VERSION,
    RttDemux,
    _enable_payload,
    crc16_ccitt,
    disable_payload,
    enable_payload,
    parse_fast_frame,
    summarize_capture,
)

FIRMWARE = (
    Path(__file__).resolve().parents[1] / "App" / "Src" / "vmc_lqr_rtt.c"
).read_text(encoding="utf-8")


def _macro_entries(name: str) -> list[tuple[str, int]]:
    start = FIRMWARE.index(f"#define {name}(V)")
    end = FIRMWARE.index("\n\n", start)
    block = FIRMWARE[start:end]
    return [
        (match.group(1), int(match.group(2)))
        for match in re.finditer(r"V\((\w+),\s*(\d+),", block)
    ]


def _macro_flag_names() -> list[str]:
    start = FIRMWARE.index("#define VMC_LQR_RTT_FAST_FLAGS(V)")
    end = FIRMWARE.index("\n\n", start)
    block = FIRMWARE[start:end]
    return [match.group(1) for match in re.finditer(r"V\((\w+),", block)]


def build_frame(**overrides: float) -> bytes:
    """Mirror the firmware encoder so the parser can be tested standalone."""
    scalars = {name: 0.0 for name, _ in FAST_SCALARS}
    flags = {name: 0.0 for name in FAST_FLAGS}
    scalars.update({k: v for k, v in overrides.items() if k in scalars})
    flags.update({k: v for k, v in overrides.items() if k in flags})

    body = bytearray()
    body += FAST_SYNC
    body += bytes([FAST_VERSION, FAST_FRAME_SIZE])
    body += struct.pack("<I", int(overrides.get("seq", 0)))
    body += struct.pack("<I", int(overrides.get("t_ms", 0)))
    body += struct.pack("<I", int(overrides.get("loop_count", 0)))
    body += struct.pack("<H", int(overrides.get("loop_dt_us", 0)))
    for name in FAST_FLAGS:
        body += bytes([int(flags[name]) & 0xFF])
    for name, scale in FAST_SCALARS:
        value = scalars[name] * scale if scale else scalars[name]
        body += struct.pack("<h", max(-32768, min(32767, int(round(value)))))
    assert len(body) == FAST_FRAME_SIZE - 2
    body += struct.pack("<H", crc16_ccitt(bytes(body)))
    assert len(body) == FAST_FRAME_SIZE
    return bytes(body)


class FirmwareTableTest(unittest.TestCase):
    def test_scale_table_matches_firmware(self) -> None:
        self.assertEqual(_macro_entries("VMC_LQR_RTT_FAST_SCALARS"), FAST_SCALARS)

    def test_flag_table_matches_firmware(self) -> None:
        self.assertEqual(_macro_flag_names(), FAST_FLAGS)

    def test_frame_size_matches_firmware(self) -> None:
        match = re.search(
            r"#define VMC_LQR_RTT_FAST_FRAME_SIZE\s+(\d+)U", FIRMWARE
        )
        self.assertIsNotNone(match)
        assert match is not None
        self.assertEqual(int(match.group(1)), FAST_FRAME_SIZE)
        self.assertEqual(
            4 + 4 + 4 + 4 + 2 + len(FAST_FLAGS) + 2 * len(FAST_SCALARS) + 2,
            FAST_FRAME_SIZE,
        )

    def test_csv_columns_cover_every_wire_field(self) -> None:
        for name in ("t_ms", "seq", "loop_count", "loop_dt_us"):
            self.assertIn(name, FAST_FIELDS)
        for name in FAST_FLAGS:
            self.assertIn(name, FAST_FIELDS)
        for name, _scale in FAST_SCALARS:
            self.assertIn(name, FAST_FIELDS)


class FastFrameTest(unittest.TestCase):
    def test_round_trip(self) -> None:
        frame = build_frame(
            seq=1234,
            t_ms=1677613,
            loop_count=538127,
            loop_dt_us=2000,
            enabled=1,
            dm_state=0,
            body_pitch=-0.04,
            body_pitch_rate=0.017,
            kf_dx=2.058,
            wheel_left_speed=-0.019,
            wheel_left_torque=0.14,
            wheel_left_current_output=442,
            slip_metric=0.237,
            slip_left=0,
            airborne_joint_torque_max=12.329,
        )
        sample = parse_fast_frame(frame)
        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual(sample["seq"], 1234)
        self.assertEqual(sample["t_ms"], 1677613)
        self.assertEqual(sample["loop_count"], 538127)
        self.assertEqual(sample["loop_dt_us"], 2000)
        self.assertEqual(sample["enabled"], 1)
        self.assertAlmostEqual(sample["body_pitch"], -0.04, places=4)
        self.assertAlmostEqual(sample["kf_dx"], 2.058, places=2)
        self.assertAlmostEqual(sample["wheel_left_current_output"], 442)
        self.assertAlmostEqual(sample["airborne_joint_torque_max"], 12.33, places=2)

    def test_crc_rejects_corruption(self) -> None:
        frame = bytearray(build_frame(seq=7, wheel_left_speed=1.5))
        frame[40] ^= 0xFF
        self.assertIsNone(parse_fast_frame(bytes(frame)))

    def test_rejects_wrong_length_and_version(self) -> None:
        self.assertIsNone(parse_fast_frame(build_frame()[:-1]))
        frame = bytearray(build_frame())
        frame[2] = 99
        self.assertIsNone(parse_fast_frame(bytes(frame)))

    def test_int16_saturation_is_visible(self) -> None:
        # 400 rad/s cannot fit int16 at scale 100 either; the firmware
        # saturates, which is why the scale must clear the real range.
        sample = parse_fast_frame(build_frame(wheel_left_speed=400.0))
        assert sample is not None
        self.assertAlmostEqual(sample["wheel_left_speed"], 327.67, places=2)

    def test_wheel_speed_has_headroom_for_real_slip(self) -> None:
        """An M3508 reaches ~60 rad/s at 9000 rpm; the encoding must not clip."""
        for name in ("wheel_left_speed", "wheel_right_speed"):
            scale = dict(FAST_SCALARS)[name]
            self.assertGreaterEqual(32767 / scale, 100.0)
        sample = parse_fast_frame(build_frame(wheel_left_speed=-60.0))
        assert sample is not None
        self.assertAlmostEqual(sample["wheel_left_speed"], -60.0, places=2)

    def test_crc_matches_known_vector(self) -> None:
        # CRC-16/CCITT-FALSE("123456789") == 0x29B1
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)


class DemuxTest(unittest.TestCase):
    def test_splits_mixed_stream(self) -> None:
        demux = RttDemux()
        frame = build_frame(seq=5, t_ms=1000)
        slow = b"VMCVOFA," + b"0," * 136 + b"\n"
        events = demux.feed(slow + frame + slow)
        kinds = [kind for kind, _ in events]
        self.assertEqual(kinds, ["slow", "fast", "slow"])
        self.assertEqual(demux.fast_frames, 1)
        self.assertEqual(demux.slow_lines, 2)
        self.assertEqual(demux.bad_frames, 0)
        self.assertEqual(demux.resync_bytes, 0)

    def test_reassembles_split_frame(self) -> None:
        demux = RttDemux()
        frame = build_frame(seq=9, t_ms=2000)
        self.assertEqual(demux.feed(frame[:10]), [])
        events = demux.feed(frame[10:])
        self.assertEqual([kind for kind, _ in events], ["fast"])

    def test_resyncs_after_garbage(self) -> None:
        demux = RttDemux()
        frame = build_frame(seq=3, t_ms=1500)
        events = demux.feed(b"\x00\x11garbage" + frame)
        self.assertEqual([kind for kind, _ in events], ["fast"])
        self.assertGreater(demux.resync_bytes, 0)

    def test_counts_corrupt_frame_and_keeps_next(self) -> None:
        demux = RttDemux()
        bad = bytearray(build_frame(seq=1))
        bad[45] ^= 0x5A
        good = build_frame(seq=2)
        events = demux.feed(bytes(bad) + good)
        self.assertEqual(len(events), 1)
        self.assertEqual(demux.bad_frames, 1)
        self.assertEqual(events[0][1]["seq"], 2)


class PayloadTest(unittest.TestCase):
    def test_enable_payload_keeps_raw_gate_off(self) -> None:
        payload = json.loads(enable_payload(4, 20))
        self.assertEqual(payload["fast_enable"], 1)
        self.assertEqual(payload["fast_period_ms"], 4)
        self.assertEqual(payload["telem_period_ms"], 20)
        self.assertEqual(payload["slip_enable"], 0)

    def test_disable_payload(self) -> None:
        payload = json.loads(disable_payload())
        self.assertEqual(payload["fast_enable"], 0)

    def test_fast_only_mode_disables_full_frame_context(self) -> None:
        payload = json.loads(_enable_payload("fast", 4, 20))
        self.assertEqual(payload["fast_enable"], 1)
        self.assertEqual(payload["fast_period_ms"], 4)
        self.assertEqual(payload["telem_period_ms"], FAST_MODE_SLOW_PERIOD_MS)
        self.assertEqual(payload["slip_enable"], 0)

    def test_slow_only_mode_turns_fast_stream_off(self) -> None:
        payload = json.loads(_enable_payload("slow", 4, 20))
        self.assertEqual(payload["fast_enable"], 0)
        self.assertEqual(payload["telem_period_ms"], 20)
        self.assertNotIn("fast_period_ms", payload)
        self.assertEqual(payload["slip_enable"], 0)

    def test_both_mode_keeps_both_streams(self) -> None:
        payload = json.loads(_enable_payload("both", 4, 20))
        self.assertEqual(payload["fast_enable"], 1)
        self.assertEqual(payload["fast_period_ms"], 4)
        self.assertEqual(payload["telem_period_ms"], 20)

    def test_runtime_threshold_is_sent_without_a_reflash(self) -> None:
        payload = json.loads(_enable_payload("both", 4, 20, 36.0, 10.0))
        self.assertEqual(payload["slip_accel"], 36.0)
        self.assertEqual(payload["slip_confirm_ms"], 10.0)
        # A parameter sweep must not silently switch the filter gate on.
        self.assertEqual(payload["slip_enable"], 0)

    def test_gate_is_opt_in(self) -> None:
        off = json.loads(_enable_payload("both", 4, 20, 36.0))
        self.assertEqual(off["slip_enable"], 0)
        on = json.loads(_enable_payload("both", 4, 20, 36.0, 10.0, True))
        self.assertEqual(on["slip_enable"], 1)

    def test_omitted_slip_params_are_not_sent(self) -> None:
        payload = json.loads(_enable_payload("both", 4, 20))
        self.assertNotIn("slip_accel", payload)
        self.assertNotIn("slip_confirm_ms", payload)

    def test_runtime_threshold_works_in_every_mode(self) -> None:
        for mode in ("both", "fast", "slow"):
            payload = json.loads(_enable_payload(mode, 4, 20, 36.0))
            self.assertEqual(payload["slip_accel"], 36.0)

    def test_leg_follow_params_are_sent_at_runtime(self) -> None:
        payload = json.loads(
            _enable_payload(
                "both",
                4,
                20,
                leg_follow_enable=True,
                leg_follow_machine_wide=True,
                leg_follow_rate=0.2,
                leg_follow_return_rate=0.05,
            )
        )
        self.assertEqual(payload["leg_follow_enable"], 1)
        self.assertEqual(payload["leg_follow_machine_wide"], 1)
        self.assertEqual(payload["leg_follow_rate"], 0.2)
        self.assertEqual(payload["leg_follow_return_rate"], 0.05)
        # The type tag must stay vmc_balance_params so the firmware accepts it.
        self.assertEqual(payload["type"], "vmc_balance_params")

    def test_leg_follow_off_is_sent_as_zero(self) -> None:
        payload = json.loads(
            _enable_payload("both", 4, 20, leg_follow_enable=False)
        )
        self.assertEqual(payload["leg_follow_enable"], 0)

    def test_omitted_leg_follow_params_are_not_sent(self) -> None:
        payload = json.loads(_enable_payload("both", 4, 20))
        for key in (
            "leg_follow_enable",
            "leg_follow_machine_wide",
            "leg_follow_auto_scope",
            "leg_follow_roll_deadband",
            "leg_follow_episode_timeout_ms",
            "leg_follow_rate",
            "leg_follow_return_rate",
        ):
            self.assertNotIn(key, payload)

    def test_auto_scope_and_deadband_are_sent_at_runtime(self) -> None:
        """The scope is chosen from roll now, so a contrast run must be able
        to pin it back to a fixed scope without a reflash."""
        payload = json.loads(
            _enable_payload(
                "both",
                4,
                20,
                leg_follow_auto_scope=False,
                leg_follow_machine_wide=True,
                leg_follow_roll_deadband=0.08,
                leg_follow_episode_timeout_ms=5000.0,
            )
        )
        self.assertEqual(payload["leg_follow_auto_scope"], 0)
        self.assertEqual(payload["leg_follow_machine_wide"], 1)
        self.assertEqual(payload["leg_follow_roll_deadband"], 0.08)
        self.assertEqual(payload["leg_follow_episode_timeout_ms"], 5000.0)


class SummaryTest(unittest.TestCase):
    def test_summary_counts_dropped_fast_frames(self) -> None:
        times = [0.0, 4.0, 8.0, 20.0, 24.0]
        summary = summarize_capture(
            times,
            [0.0, 20.0],
            [],
            4.0,
            20.0,
            fast_loop_counts=[0.0, 2.0, 4.0, 10.0, 12.0],
        )
        self.assertEqual(summary["fast"]["samples"], len(times))
        self.assertEqual(summary["fast"]["gap_count"], 1)
        self.assertEqual(summary["fast"]["largest_gap_ms"], 12.0)
        self.assertEqual(summary["fast"]["observed_unit_ms"], 4.0)
        self.assertEqual(summary["fast"]["estimated_missing_frames"], 2)
        self.assertLess(summary["fast"]["coverage_pct"], 100.0)
        self.assertAlmostEqual(summary["fast"]["control_loop_rate_hz"], 500.0)

    def test_summary_records_mode(self) -> None:
        summary = summarize_capture([], [], [], 4.0, 20.0, None, "slow")
        self.assertEqual(summary["mode"], "slow")
        self.assertEqual(summary["fast"]["samples"], 0)

    def test_summary_ignores_broken_timestamp(self) -> None:
        # A truncated RTT line can decode as 0.089 ms after a real 643384 ms
        # sample.  It must not create a fake 643 s gap.
        times = [0.0, 20.0, 40.0, 0.089, 60.0]
        summary = summarize_capture([], times, [], 4.0, 20.0, None, "slow")
        slow = summary["slow"]
        self.assertEqual(slow["invalid_samples"], 1)
        self.assertEqual(slow["largest_gap_ms"], 20.0)

    def test_summary_reports_link_diagnostics(self) -> None:
        summary = summarize_capture(
            [], [], [], 4.0, 20.0, None, "slow",
            unparsed_lines=7, reader_restarts=2,
            stalls=[{"at_s": 1.0, "duration_s": 5.0}],
            demux_stats={"bad_frames": 3, "resync_bytes": 11},
        )
        self.assertEqual(summary["unparsed_lines"], 7)
        self.assertEqual(summary["reader_restarts"], 2)
        self.assertEqual(len(summary["link_stalls"]), 1)
        self.assertEqual(summary["demux"]["bad_frames"], 3)

    def test_summary_records_requested_and_applied_slip_settings(self) -> None:
        summary = summarize_capture(
            [], [], [], 4.0, 20.0, None, "both",
            slip_requested={"threshold": 36.0, "confirm_ms": 10.0,
                            "gate_active": False},
            slip_applied={"threshold": 36.0, "gate_active": 0.0},
        )
        self.assertEqual(summary["slip_requested"]["threshold"], 36.0)
        self.assertEqual(summary["slip_applied"]["threshold"], 36.0)


if __name__ == "__main__":
    unittest.main()
