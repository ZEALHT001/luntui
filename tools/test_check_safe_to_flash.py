#!/usr/bin/env python3
"""Tests for the pre-flash safety check.

Flashing a controller that is still driving the legs resets it into an
undefined state and the robot drops. The check has to be conservative in the
right direction, and it has to look at signals the enable gate actually
controls.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.check_safe_to_flash import (  # noqa: E402
    ACTIVE_FRAME_FRACTION,
    JOINT_TORQUE_MAX_NM,
    JOINT_TORQUE_ACTIVE_NM,
    RC_SOURCE_NONE,
    RC_SW_DOWN,
    RC_STATE_MIN_FRAMES,
    WHEEL_CURRENT_MAX_RAW,
    WHEEL_CURRENT_ACTIVE_RAW,
    Probe,
    _activity,
)
from tools.rtt_kalman_capture import KalmanSample  # noqa: E402


def _sample(**updates) -> KalmanSample:
    return KalmanSample(host_time_s=0.0, kf_x=0.0, kf_dx=0.0, **updates)


class SafeToFlashTest(unittest.TestCase):
    def test_disabled_machine_reads_idle(self) -> None:
        """Values measured on the real robot while it was switched off."""
        busy = _activity(
            _sample(
                airborne_joint_torque_min=0.013,
                airborne_joint_torque_max=0.065,
                left_wheel_current=0.0,
                right_wheel_current=0.0,
            )
        )
        self.assertLess(busy["measured joint torque"], JOINT_TORQUE_ACTIVE_NM)
        self.assertLess(busy["wheel output current"], WHEEL_CURRENT_ACTIVE_RAW)

    def test_enabled_machine_reads_active(self) -> None:
        """Values measured while the robot stood on its own legs."""
        busy = _activity(
            _sample(
                airborne_joint_torque_min=12.7,
                airborne_joint_torque_max=21.3,
                left_wheel_current=1862.0,
                right_wheel_current=-1780.0,
            )
        )
        self.assertGreater(busy["measured joint torque"], JOINT_TORQUE_ACTIVE_NM)
        self.assertGreater(busy["wheel output current"], WHEEL_CURRENT_ACTIVE_RAW)

    def test_joint_torque_alone_is_enough(self) -> None:
        """A held robot may command no wheel current but still be enabled."""
        busy = _activity(
            _sample(airborne_joint_torque_max=12.7, left_wheel_current=0.0)
        )
        self.assertGreater(busy["measured joint torque"], JOINT_TORQUE_ACTIVE_NM)

    def test_demand_channels_are_not_used(self) -> None:
        """The LQR demand is computed even when disabled, so it must be ignored.

        left_tp_cmd and left_wheel_torque_raw both read large on a switched-off
        machine; treating them as activity would block every safe flash.
        """
        busy = _activity(
            _sample(left_tp_cmd=17.0, left_wheel_torque_raw=15.5)
        )
        self.assertLess(busy["measured joint torque"], JOINT_TORQUE_ACTIVE_NM)
        self.assertLess(busy["wheel output current"], WHEEL_CURRENT_ACTIVE_RAW)

    def test_corrupt_frames_are_discarded(self) -> None:
        """A half-parsed RTT line must not look like a running controller.

        One real capture reported a peak wheel current of 259150 against a
        16000 hardware limit; believing it would block every flash.
        """
        busy = _activity(
            _sample(
                airborne_joint_torque_max=JOINT_TORQUE_MAX_NM * 4,
                left_wheel_current=WHEEL_CURRENT_MAX_RAW * 16,
                right_wheel_current=-WHEEL_CURRENT_MAX_RAW * 16,
            )
        )
        self.assertEqual(busy["measured joint torque"], 0.0)
        self.assertEqual(busy["wheel output current"], 0.0)

    def test_a_single_frame_is_not_enough(self) -> None:
        """Judging on one frame is unsafe in both directions.

        An isolated glitch read 2.2 N.m on a switched-off machine. The verdict
        is taken over the fraction of active frames, so one sample among
        hundreds must not decide it.
        """
        self.assertLess(1.0 / 300.0, ACTIVE_FRAME_FRACTION)
        # ...while a genuinely live controller is active nearly all the time:
        # the recorded standing and jump captures measured 57-100% of frames.
        self.assertLess(ACTIVE_FRAME_FRACTION, 0.5)


class ProbeFailureTest(unittest.TestCase):
    def test_unreachable_probe_returns_could_not_tell(self) -> None:
        """A probe that cannot be opened must not crash or pass.

        The caller has to treat it as unsafe and ask the operator, so the
        exit status is 2 (same as no telemetry), never a traceback.
        """
        from tools import check_safe_to_flash as module

        with mock.patch.object(
            module,
            "JLinkSubprocessReader",
            side_effect=RuntimeError("Cannot connect to the probe/programmer."),
        ):
            with mock.patch.object(sys, "argv", ["check_safe_to_flash.py"]):
                self.assertEqual(module.main(), 2)


def _probe(count: int, **updates) -> Probe:
    probe = Probe()
    for _ in range(count):
        probe.observe(_sample(**updates))
    return probe


class StaleBufferTest(unittest.TestCase):
    """The J-Link hands over whatever was buffered, so a capture usually starts
    with frames from an earlier session.  Those describe a different moment and
    must not vote on whether it is safe to reset now."""

    def test_buffered_prefix_is_dropped(self) -> None:
        probe = Probe()
        # An earlier session: the robot was up and driving.
        for index in range(40):
            probe.observe(
                _sample(
                    mcu_time_ms=float(index * 20),
                    remote_s1=1.0,
                    rc_source=3.0,
                    airborne_joint_torque_max=18.0,
                    left_wheel_current=15000.0,
                )
            )
        # The live session: remote off, everything idle.
        for index in range(100):
            probe.observe(
                _sample(
                    mcu_time_ms=500000.0 + float(index * 20),
                    rc_source=RC_SOURCE_NONE,
                )
            )

        self.assertEqual(probe.dropped_stale_frames, 40)
        self.assertEqual(probe.frames, 100)
        self.assertFalse(probe.busy())
        self.assertTrue(probe.rc_offline())
        self.assertEqual(probe.s1_frames, 0)

    def test_a_continuous_stream_is_not_dropped(self) -> None:
        probe = Probe()
        for index in range(100):
            probe.observe(
                _sample(
                    mcu_time_ms=float(index * 20),
                    rc_source=RC_SOURCE_NONE,
                )
            )
        self.assertEqual(probe.dropped_stale_frames, 0)
        self.assertEqual(probe.frames, 100)


class RemoteStateTest(unittest.TestCase):
    """The operator's stop command is positive evidence, and is enough alone."""

    def test_channel4_stop_is_accepted_even_with_active_outputs(self) -> None:
        """Channel 4 UP forces every output to zero, so flashing is safe.

        The output readings are deliberately left active here: the point is
        that the operator's command is authoritative, not that the outputs
        happen to agree.
        """
        probe = _probe(
            100,
            remote_s1=RC_SW_DOWN,
            rc_source=3.0,
            airborne_joint_torque_max=18.0,
            left_wheel_current=15000.0,
        )
        self.assertTrue(probe.s1_down())
        self.assertTrue(probe.busy())

    def test_channel4_enabled_is_rejected(self) -> None:
        probe = _probe(100, remote_s1=1.0, rc_source=3.0)
        self.assertFalse(probe.s1_down())
        self.assertTrue(probe.switched_on())

    def test_channel4_down_position_counts_as_enabled(self) -> None:
        """Synthesized MID means channel 4 is DOWN and enabled."""
        probe = _probe(100, remote_s1=3.0, rc_source=3.0)
        self.assertFalse(probe.s1_down())
        self.assertTrue(probe.switched_on())

    def test_remote_offline_is_accepted(self) -> None:
        probe = _probe(100, rc_source=RC_SOURCE_NONE)
        self.assertTrue(probe.rc_offline())
        self.assertFalse(probe.switched_on())

    def test_remote_offline_is_accepted_even_with_active_outputs(self) -> None:
        """Offline means the firmware is forcing zero; stale readings must not
        block a legitimate flash."""
        probe = _probe(
            100,
            rc_source=RC_SOURCE_NONE,
            airborne_joint_torque_max=21.3,
            left_wheel_current=15000.0,
        )
        self.assertTrue(probe.rc_offline())

    def test_remote_online_and_not_stopped_blocks(self) -> None:
        probe = _probe(100, remote_s1=1.0, rc_source=3.0)
        self.assertFalse(probe.rc_offline())
        self.assertTrue(probe.switched_on())

    def test_missing_s1_field_leaves_both_paths_available(self) -> None:
        """This branch's layout has no switch; offline and outputs still work."""
        probe = _probe(100, rc_source=1.0)
        self.assertEqual(probe.s1_frames, 0)
        self.assertFalse(probe.s1_down())
        self.assertFalse(probe.switched_on())
        self.assertFalse(probe.rc_offline())
        self.assertFalse(probe.busy())

    def test_a_single_stale_frame_does_not_decide(self) -> None:
        """The operator's command must hold across the window."""
        probe = _probe(100, rc_source=3.0, remote_s1=1.0)
        # One frame reporting the remote absent / channel-4 stop must not flip it.
        probe.observe(_sample(rc_source=RC_SOURCE_NONE, remote_s1=RC_SW_DOWN))
        self.assertFalse(probe.rc_offline())
        self.assertFalse(probe.s1_down())

    def test_a_very_short_window_cannot_open_the_gate(self) -> None:
        """A fraction of a handful of frames is not evidence.

        One frame reporting the remote absent must not clear the machine on
        its own; the output evidence still has to agree.
        """
        probe = _probe(3, rc_source=RC_SOURCE_NONE)
        self.assertLess(probe.frames, RC_STATE_MIN_FRAMES)
        self.assertFalse(probe.rc_offline())

        probe = _probe(3, remote_s1=RC_SW_DOWN, rc_source=3.0)
        self.assertFalse(probe.s1_down())


if __name__ == "__main__":
    unittest.main()
