#!/usr/bin/env python3
"""Regression tests for firmware-default serialization."""

from __future__ import annotations

import unittest

from tools.lqr_tuner.server import BALANCE_PARAM_DEFAULT_MACROS, _format_c_literal


class FirmwareDefaultFormattingTest(unittest.TestCase):
    def test_millisecond_defaults_keep_numeric_values(self) -> None:
        for key, (_macro, value_type) in BALANCE_PARAM_DEFAULT_MACROS.items():
            if not key.endswith("_ms"):
                continue
            with self.subTest(key=key):
                self.assertEqual(value_type, "uint_value")
                self.assertEqual(_format_c_literal(60.0, value_type), "60U")
                self.assertEqual(_format_c_literal(200.0, value_type), "200U")


if __name__ == "__main__":
    unittest.main()
