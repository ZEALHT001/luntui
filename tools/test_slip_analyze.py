#!/usr/bin/env python3
"""Tests for the slip feature evaluation helper."""

from __future__ import annotations

import unittest

from tools.slip_analyze import evaluate, find_runs, percentiles


class PercentileTest(unittest.TestCase):
    def test_percentiles_ignore_nan_and_are_ordered(self) -> None:
        values = [float("nan")] + [float(i) for i in range(101)]
        p50, p99 = percentiles(values, (0.5, 0.99))
        self.assertEqual(p50, 50.0)
        self.assertEqual(p99, 99.0)

    def test_empty_input_is_zero(self) -> None:
        self.assertEqual(percentiles([], (0.5,)), [0.0])


class RunTest(unittest.TestCase):
    def test_finds_contiguous_runs(self) -> None:
        times = [0.0, 4.0, 8.0, 12.0, 16.0, 20.0]
        values = [0.0, 10.0, 10.0, 0.0, 10.0, 0.0]
        runs = find_runs(times, values, 5.0, refractory_ms=1.0)
        self.assertEqual(runs, [(4.0, 8.0), (16.0, 16.0)])

    def test_refractory_merges_nearby_runs(self) -> None:
        # Two bursts 400 ms apart are one event when the refractory is 700 ms.
        times = [0.0, 100.0, 200.0, 500.0, 600.0, 900.0]
        values = [10.0, 0.0, 0.0, 10.0, 0.0, 0.0]
        runs = find_runs(times, values, 5.0, refractory_ms=700.0)
        self.assertEqual(runs, [(0.0, 500.0)])
        # A shorter refractory keeps them apart.
        runs = find_runs(times, values, 5.0, refractory_ms=200.0)
        self.assertEqual(runs, [(0.0, 0.0), (500.0, 500.0)])


class EvaluateTest(unittest.TestCase):
    def test_matches_event_with_reaction_lag(self) -> None:
        # The disturbance precedes the keypress, so a detection 600 ms before
        # the marker must still count as a hit.
        events = [10000.0]
        runs = [(9400.0, 9500.0)]
        hit, false = evaluate(runs, events, lag_back_ms=1200.0, lag_fwd_ms=200.0)
        self.assertEqual((hit, false), (1, 0))

    def test_detection_outside_window_is_false_positive(self) -> None:
        events = [10000.0]
        runs = [(20000.0, 20100.0)]
        hit, false = evaluate(runs, events, lag_back_ms=1200.0, lag_fwd_ms=200.0)
        self.assertEqual((hit, false), (0, 1))

    def test_two_detections_in_one_event_count_once(self) -> None:
        events = [10000.0]
        runs = [(9500.0, 9600.0), (9800.0, 9900.0)]
        hit, false = evaluate(runs, events, lag_back_ms=1200.0, lag_fwd_ms=200.0)
        self.assertEqual((hit, false), (1, 0))

    def test_unmarked_event_reported_as_false_positive(self) -> None:
        events = [10000.0, 50000.0]
        runs = [(9500.0, 9600.0), (30000.0, 30100.0)]
        hit, false = evaluate(runs, events, lag_back_ms=1200.0, lag_fwd_ms=200.0)
        self.assertEqual((hit, false), (1, 1))


if __name__ == "__main__":
    unittest.main()
