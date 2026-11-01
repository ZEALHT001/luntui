#!/usr/bin/env python3
"""Regression checks that automatic platform descent is fully removed."""

from __future__ import annotations

import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "App" / "Src" / "vmc_probe.c"
HEADER = ROOT / "App" / "Inc" / "vmc_probe.h"
RTT = ROOT / "App" / "Src" / "vmc_lqr_rtt.c"
UI = ROOT / "tools" / "lqr_tuner" / "web" / "index.html"
WEB_MAIN = ROOT / "tools" / "lqr_tuner" / "web" / "src" / "main.js"
SERVER = ROOT / "tools" / "lqr_tuner" / "server.py"


class DescentRemovalTest(unittest.TestCase):
    def test_firmware_has_no_descent_state_or_gate(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")
        header = HEADER.read_text(encoding="utf-8")
        for token in (
            "vmc_descent_",
            "VMC_DESCENT_",
            "descent_override",
            "descent_force_profile",
        ):
            with self.subTest(token=token):
                self.assertNotIn(token, source)
                self.assertNotIn(token, header)

    def test_runtime_tuning_and_ui_are_removed(self) -> None:
        rtt = RTT.read_text(encoding="utf-8")
        index = UI.read_text(encoding="utf-8")
        main = WEB_MAIN.read_text(encoding="utf-8")
        server = SERVER.read_text(encoding="utf-8")
        for token in ("descent_enable", "descentExtend", "descentContact"):
            with self.subTest(token=token):
                self.assertNotIn(token, rtt)
                self.assertNotIn(token, index)
                self.assertNotIn(token, main)
                self.assertNotIn(token, server)

    def test_telemetry_slots_remain_zero_for_wire_compatibility(self) -> None:
        rtt = RTT.read_text(encoding="utf-8")
        self.assertIn("fields 177..186 are reserved compatibility slots", rtt)
        self.assertIn("reserved < 10U", rtt)


if __name__ == "__main__":
    unittest.main()
