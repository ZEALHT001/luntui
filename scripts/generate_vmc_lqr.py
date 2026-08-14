#!/usr/bin/env python3
"""
Generate gain-scheduled VMC LQR C code without MATLAB.

Two input modes are supported:

1. Model JSON:
   Solves continuous-time LQR for each leg length from A/B/Q/R matrices,
   fits polynomial K_i(L), and emits C code.

2. K CSV:
   Fits polynomial K_i(L) from already computed/sample-measured gains.

Examples:
  python3 generate_vmc_lqr.py --input lqr_models.json --mode model --out vmc_lqr_generated.c
  python3 generate_vmc_lqr.py --input lqr_samples.csv --mode csv --out vmc_lqr_generated.c

Dependencies:
  python3 -m pip install numpy scipy
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.linalg import solve_continuous_are


def continuous_lqr(a: np.ndarray, b: np.ndarray, q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Return K for u = -Kx using continuous-time LQR."""
    p = solve_continuous_are(a, b, q, r)
    return np.linalg.solve(r, b.T @ p)


def load_model_json(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """
    Expected JSON format:
    {
      "samples": [
        {"length": 0.18, "A": [[...]], "B": [[...]], "Q": [[...]], "R": [[...]]},
        ...
      ]
    }
    """
    payload = json.loads(path.read_text())
    lengths = []
    gains = []

    for sample in payload["samples"]:
        a = np.array(sample["A"], dtype=float)
        b = np.array(sample["B"], dtype=float)
        q = np.array(sample["Q"], dtype=float)
        r = np.array(sample["R"], dtype=float)
        k = continuous_lqr(a, b, q, r)

        lengths.append(float(sample["length"]))
        gains.append(k.reshape(-1))

    return np.array(lengths, dtype=float), np.vstack(gains)


def load_k_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """
    Expected CSV columns:
      length,k0,k1,k2,...,k11
    """
    lengths = []
    gains = []

    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            lengths.append(float(row["length"]))
            k_cols = [name for name in row if name and name.lower().startswith("k")]
            k_cols.sort(key=lambda name: int(name[1:]))
            gains.append([float(row[name]) for name in k_cols])

    return np.array(lengths, dtype=float), np.array(gains, dtype=float)


def fit_polynomials(lengths: np.ndarray, gains: np.ndarray, degree: int) -> np.ndarray:
    if len(lengths) < degree + 1:
        raise ValueError(f"Need at least {degree + 1} samples to fit degree {degree}.")

    coeffs = []
    for i in range(gains.shape[1]):
        coeffs.append(np.polyfit(lengths, gains[:, i], degree))
    return np.vstack(coeffs)


def c_expr(coeff: Iterable[float], variable: str = "L_0") -> str:
    coeff = list(coeff)
    degree = len(coeff) - 1
    terms = []
    for idx, value in enumerate(coeff):
        power = degree - idx
        if power == 0:
            term = f"{value:.17g}"
        elif power == 1:
            term = f"({value:.17g} * {variable})"
        else:
            term = f"({value:.17g} * pow_{power})"
        terms.append(term)
    return " + ".join(terms)


def emit_c(coeffs: np.ndarray, header_name: str) -> str:
    degree = coeffs.shape[1] - 1
    lines = [
        f'#include "{header_name}"',
        "",
        "void vmc_lqr_update_gain(double leg_length, double k[12])",
        "{",
        "  double L_0 = leg_length;",
    ]

    for power in range(2, degree + 1):
        if power == 2:
            lines.append("  double pow_2 = L_0 * L_0;")
        else:
            lines.append(f"  double pow_{power} = pow_{power - 1} * L_0;")

    lines.append("")

    for i, coeff in enumerate(coeffs):
        lines.append(f"  k[{i}] = {c_expr(coeff)};")

    lines.append("}")
    lines.append("")
    return "\n".join(lines)


def emit_csv_template(path: Path, k_count: int = 12) -> None:
    headers = ["length"] + [f"k{i}" for i in range(k_count)]
    rows = [
        [0.18] + [0.0] * k_count,
        [0.28] + [0.0] * k_count,
        [0.38] + [0.0] * k_count,
    ]

    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, help="Input JSON/CSV file.")
    parser.add_argument("--mode", choices=("model", "csv"), help="Input mode.")
    parser.add_argument("--degree", type=int, default=3, help="Polynomial degree.")
    parser.add_argument("--out", type=Path, default=Path("vmc_lqr_generated.c"))
    parser.add_argument("--header", default="vmc_lqr.h")
    parser.add_argument("--csv-template", type=Path, help="Write a K CSV template and exit.")
    args = parser.parse_args()

    if args.csv_template:
        emit_csv_template(args.csv_template)
        return

    if args.input is None or args.mode is None:
        parser.error("--input and --mode are required unless --csv-template is used.")

    if args.mode == "model":
        lengths, gains = load_model_json(args.input)
    else:
        lengths, gains = load_k_csv(args.input)

    if gains.shape[1] != 12:
        raise ValueError(f"Expected 12 LQR gains per sample, got {gains.shape[1]}.")

    coeffs = fit_polynomials(lengths, gains, args.degree)
    args.out.write_text(emit_c(coeffs, args.header))


if __name__ == "__main__":
    main()
