#!/usr/bin/env python3
"""Regression test for web-calculated and compiled LQR gain consistency."""

from __future__ import annotations

import re
import unittest
from pathlib import Path

import numpy as np

from tools.lqr_tuner.server import _calc_response, _read_firmware_defaults


ROOT = Path(__file__).resolve().parents[2]
GAIN_ASSIGNMENT_RE = re.compile(
    r"k\[(\d+)\] = \(\(L_0 \* ([^ ]+) \+ t2 \* ([^)]+)\) "
    r"\+ t3 \* ([^)]+)\) \+ ([^;]+);"
)


def compiled_gain_at_length(leg_length: float) -> np.ndarray:
    source = (ROOT / "App" / "Src" / "vmc_lqr.c").read_text(encoding="utf-8")
    gains = np.zeros(12)
    found: set[int] = set()

    for match in GAIN_ASSIGNMENT_RE.finditer(source):
        index = int(match.group(1))
        linear, quadratic, cubic, constant = map(float, match.groups()[1:])
        gains[index] = (
            linear * leg_length
            + quadratic * leg_length**2
            + cubic * leg_length**3
            + constant
        )
        found.add(index)

    if found != set(range(12)):
        raise AssertionError(f"expected 12 compiled gains, found indices {sorted(found)}")
    return gains


class ModelConsistencyTest(unittest.TestCase):
    def test_web_gain_matches_compiled_schedule(self) -> None:
        defaults = _read_firmware_defaults()
        self.assertTrue(defaults["q_r_found"])

        for leg_length in np.arange(0.12, 0.3851, 0.005):
            response = _calc_response(
                {
                    "preset": "abk",
                    "leg_length": float(leg_length),
                    "q": defaults["q"],
                    "r": defaults["r"],
                }
            )
            web_gain = np.asarray(response["k_flat_col_major"])
            compiled_gain = compiled_gain_at_length(float(leg_length))
            max_error = float(np.max(np.abs(web_gain - compiled_gain)))
            self.assertLess(
                max_error,
                0.7,
                f"gain mismatch at leg length {leg_length:.3f} m",
            )


if __name__ == "__main__":
    unittest.main()
