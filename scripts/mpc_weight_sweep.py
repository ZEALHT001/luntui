#!/usr/bin/env python3
"""Sweep the MPC input-increment weight on the host closed-loop testbed.

Rebuilds tools/test_vmc_mpc.c against a freshly generated model for each
weight and prints the closed-loop metrics.  No hardware and no flashing, so a
weight can be screened before it ever reaches the machine.  The plant is the
model against itself, so the numbers are optimistic; use them to rank weights,
not to predict absolute hardware behaviour.
"""
from __future__ import annotations
import subprocess, sys
from dataclasses import replace
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import derive_vmc_lqr_abk as gen  # noqa: E402

MPC_Q = np.diag([1.0, 100.0, 400.0, 30.0, 5000.0, 1.0, 0.05])
LENGTHS = np.linspace(0.180, 0.385, num=9)
PRESET = replace(gen.PRESETS["abk"], q=MPC_Q)
TMP = ROOT / "build" / "sweep_model.c"
BIN = ROOT / "build" / "sweep_test"

def run(weight: tuple[float, float]) -> str:
    gen.MPC_DELTA_WEIGHT = np.array(weight)
    TMP.write_text(gen.emit_mpc_model(PRESET, LENGTHS, 0.003), encoding="utf-8")
    build = subprocess.run(
        ["cc", "-std=c11", "-O2", "-I", str(ROOT / "App/Inc"),
         str(ROOT / "tools/test_vmc_mpc.c"), str(ROOT / "App/Src/vmc_mpc.c"),
         str(TMP), "-lm", "-o", str(BIN)],
        capture_output=True, text=True)
    if build.returncode:
        return f"BUILD FAIL {build.stderr.strip()[:120]}"
    out = subprocess.run([str(BIN)], capture_output=True, text=True)
    for line in out.stdout.splitlines():
        if line.startswith("metrics:"):
            return line
    return out.stdout.strip()[-120:] or out.stderr.strip()[-120:]

for weight in [
    (10.0, 0.5), (4.0, 0.2), (2.0, 0.1), (1.0, 0.05),
    (0.25, 0.0125), (0.05, 0.0025),
]:
    print(f"delta_weight={weight}: {run(weight)}")
