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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.check_safe_to_flash import (  # noqa: E402
    ACTIVE_FRAME_FRACTION,
    JOINT_TORQUE_MAX_NM,
    JOINT_TORQUE_ACTIVE_NM,
    WHEEL_CURRENT_MAX_RAW,
    WHEEL_CURRENT_ACTIVE_RAW,
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


if __name__ == "__main__":
    unittest.main()
