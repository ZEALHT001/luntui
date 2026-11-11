#!/usr/bin/env python3
"""Sweep the MPC tuning knobs on the host closed-loop testbed.

Builds tools/test_vmc_mpc.c once against the committed model, then runs it over
a grid of runtime parameters and prints the closed-loop metrics:

    tail        position peak-to-peak over the last 500 steps
    sat         share of steps sitting on the torque-increment rate limit
    du_rms      RMS torque increment
    wheel_rms   RMS wheel torque

The plant in this test is the model itself, so the numbers are optimistic and
they miss real stiction: the 2026-11-11 hardware baseline was a 105 mm slow
cycle against ~14 mm here.  Use this to screen for instability and saturation,
not to predict absolute hardware behaviour, and confirm anything on the robot.

    python3 scripts/mpc_weight_sweep.py
"""

from __future__ import annotations

import itertools
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT / "build" / "sweep_test"


def build() -> bool:
    result = subprocess.run(
        [
            "cc", "-std=c11", "-O2", "-I", str(ROOT / "App/Inc"),
            str(ROOT / "tools/test_vmc_mpc.c"),
            str(ROOT / "App/Src/vmc_mpc.c"),
            str(ROOT / "App/Src/vmc_mpc_model.c"),
            "-lm", "-o", str(BIN),
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode:
        print(result.stderr.strip()[:400], file=sys.stderr)
    return result.returncode == 0


def run(args: list[str]) -> tuple[str, str]:
    result = subprocess.run([str(BIN), *args], capture_output=True, text=True)
    metrics = ""
    for line in result.stdout.splitlines():
        if line.startswith("metrics:"):
            metrics = line.split("metrics:", 1)[1].strip()
    verdict = "PASS" if result.returncode == 0 else "FAIL"
    return metrics, verdict


def main() -> int:
    if not build():
        return 2
    print(f"{'abs_w':>6} {'rate':>5}  {'metrics':<52} verdict")
    baseline, verdict = run([])
    print(f"{0.0:6.2f} {0.20:5.2f}  {baseline:<52} {verdict}")
    for abs_w, rate in itertools.product(
        (0.10, 0.25, 0.50, 1.00), (0.20, 0.50, 1.00)
    ):
        metrics, verdict = run(
            ["--abs-wheel", f"{abs_w}", "--abs-leg", "0",
             "--rate-wheel", f"{rate}"]
        )
        if not metrics:
            continue
        print(f"{abs_w:6.2f} {rate:5.2f}  {metrics:<52} {verdict}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
