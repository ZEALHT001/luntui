#!/usr/bin/env python3
"""Local LQR tuning server.

Receives Q/R weights from the Vite frontend, computes the scheduled VMC LQR
gain matrix using the existing ABK model script, and can send the selected
gain over SEGGER RTT as one JSON line.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))

from derive_vmc_lqr_abk import (  # noqa: E402
    PRESETS,
    emit_c,
    fit_schedule,
    k_at_length,
)


STATE_NAMES = ["theta", "dtheta", "x", "dx", "phi", "dphi"]
INPUT_NAMES = ["T", "Tp"]
DEFAULT_RTT_CLIENT = "/Applications/SEGGER/JLink/JLinkRTTClient"


def _diag(values: list[float], expected: int, name: str) -> np.ndarray:
    if len(values) != expected:
        raise ValueError(f"{name} expects {expected} values, got {len(values)}")
    return np.diag([float(v) for v in values])


def _preset_from_request(data: dict[str, Any]):
    preset_name = str(data.get("preset", "abk"))
    if preset_name not in PRESETS:
        raise ValueError(f"unknown preset: {preset_name}")

    preset = PRESETS[preset_name]
    if "q" in data:
        preset = replace(preset, q=_diag(data["q"], 6, "q"))
    if "r" in data:
        preset = replace(preset, r=_diag(data["r"], 2, "r"))
    return preset_name, preset


def _calc_response(data: dict[str, Any]) -> dict[str, Any]:
    preset_name, preset = _preset_from_request(data)
    leg_length = float(data.get("leg_length", 0.28))
    leg_min = float(data.get("leg_min", 0.03))
    leg_max = float(data.get("leg_max", 0.50))
    leg_step = float(data.get("leg_step", 0.005))
    degree = int(data.get("degree", 3))

    k = k_at_length(preset, leg_length)
    lengths, gains, coeffs = fit_schedule(preset, leg_min, leg_max, leg_step, degree)
    c_source = emit_c(coeffs)

    flat_col_major = k.reshape(-1, order="F")
    payload = {
        "type": "vmc_lqr_k",
        "preset": preset_name,
        "leg_length": leg_length,
        "state_order": STATE_NAMES,
        "input_order": INPUT_NAMES,
        "k_flat_col_major": [float(v) for v in flat_col_major],
    }

    result = {
        "preset": preset_name,
        "state_order": STATE_NAMES,
        "input_order": INPUT_NAMES,
        "q": [float(v) for v in np.diag(preset.q)],
        "r": [float(v) for v in np.diag(preset.r)],
        "leg_length": leg_length,
        "k_matrix": [[float(v) for v in row] for row in k],
        "k_flat_col_major": payload["k_flat_col_major"],
        "schedule": {
            "leg_min": leg_min,
            "leg_max": leg_max,
            "leg_step": leg_step,
            "degree": degree,
            "sample_count": int(len(lengths)),
            "coeffs": [[float(v) for v in row] for row in coeffs],
            "k_at_min": [[float(v) for v in row] for row in gains[0].reshape(2, 6)],
            "k_at_max": [[float(v) for v in row] for row in gains[-1].reshape(2, 6)],
        },
        "rtt_payload": json.dumps(payload, separators=(",", ":")),
        "c_source": c_source,
    }

    if bool(data.get("send_rtt", False)):
        result["rtt"] = send_rtt_line(
            result["rtt_payload"],
            client_path=str(data.get("rtt_client_path", DEFAULT_RTT_CLIENT)),
            timeout_s=float(data.get("rtt_timeout_s", 3.0)),
            dry_run=bool(data.get("dry_run", False)),
        )
    return result


def send_rtt_line(
    line: str,
    *,
    client_path: str = DEFAULT_RTT_CLIENT,
    timeout_s: float = 3.0,
    dry_run: bool = False,
) -> dict[str, Any]:
    frame = line.rstrip("\n") + "\n"
    if dry_run:
        return {"ok": True, "dry_run": True, "frame": frame}

    client = Path(client_path)
    if not client.exists():
        return {
            "ok": False,
            "error": f"RTT client not found: {client_path}",
            "frame": frame,
        }

    try:
        proc = subprocess.run(
            [str(client)],
            input=frame,
            text=True,
            capture_output=True,
            timeout=timeout_s,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        return {
            "ok": False,
            "error": f"RTT client timed out after {timeout_s}s",
            "stdout": exc.stdout or "",
            "stderr": exc.stderr or "",
            "frame": frame,
        }

    return {
        "ok": proc.returncode == 0,
        "returncode": proc.returncode,
        "stdout": proc.stdout,
        "stderr": proc.stderr,
        "frame": frame,
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "LQRTuner/0.1"

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "content-type")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("content-length", "0"))
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_OPTIONS(self) -> None:  # noqa: N802
        self._send_json(200, {"ok": True})

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/api/defaults":
            preset = PRESETS["abk"]
            self._send_json(
                200,
                {
                    "presets": sorted(PRESETS),
                    "default_preset": "abk",
                    "state_order": STATE_NAMES,
                    "input_order": INPUT_NAMES,
                    "q": [float(v) for v in np.diag(preset.q)],
                    "r": [float(v) for v in np.diag(preset.r)],
                    "leg_length": 0.28,
                    "rtt_client_path": DEFAULT_RTT_CLIENT,
                },
            )
            return
        self._send_json(404, {"ok": False, "error": "not found"})

    def do_POST(self) -> None:  # noqa: N802
        try:
            data = self._read_json()
            if self.path == "/api/calc":
                self._send_json(200, {"ok": True, **_calc_response(data)})
                return
            if self.path == "/api/send-rtt":
                line = str(data.get("payload", ""))
                if not line:
                    raise ValueError("payload is required")
                self._send_json(
                    200,
                    {
                        "ok": True,
                        "rtt": send_rtt_line(
                            line,
                            client_path=str(data.get("rtt_client_path", DEFAULT_RTT_CLIENT)),
                            timeout_s=float(data.get("rtt_timeout_s", 3.0)),
                            dry_run=bool(data.get("dry_run", False)),
                        ),
                    },
                )
                return
            self._send_json(404, {"ok": False, "error": "not found"})
        except Exception as exc:  # noqa: BLE001
            self._send_json(400, {"ok": False, "error": str(exc)})

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"{self.address_string()} - {fmt % args}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"LQR tuner API listening on http://{args.host}:{args.port}")
    httpd.serve_forever()


if __name__ == "__main__":
    main()
