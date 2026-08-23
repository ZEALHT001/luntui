#!/usr/bin/env python3
"""Local LQR tuning server.

Receives Q/R weights from the Vite frontend, computes the scheduled VMC LQR
gain matrix using the existing ABK model script, and can send the selected
gain over SEGGER RTT as one JSON line.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
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
DEFAULT_RTT_HOST = "127.0.0.1"
DEFAULT_RTT_PORT = 19021
DEFAULT_VOFA_HOST = "127.0.0.1"
DEFAULT_VOFA_PORT = 1347
DEFAULT_BALANCE_PARAMS = {
    "theta_target": 0.0,
    "pitch_target": -0.05,
    "phi_deadband": 0.0,
    "tp_output_enable": 1.0,
    "wheel_output_user_enable": 1.0,
    "theta_deadband": 0.0,
    "lqr_dtheta_gain": 1.0,
    "lqr_dtheta_deadband": 0.0,
    "lqr_dtheta_limit": 10.0,
    "tp_phi_gain": 1.0,
    "tp_x_gain": 1.0,
    "dpitch_lpf_alpha": 1.0,
    "x_feedback_gain": -1.0,
    "dx_feedback_gain": -1.0,
    "x_feedback_limit": 20.0,
    "dx_feedback_limit": 30.0,
    "speed_control_enable": 1.0,
    "speed_target_max": 2.0,
    "speed_ramp_rate": 3.0,
    "speed_deadband": 0.02,
    "stop_speed_threshold": 0.08,
    "x_hold_max_stretch": 0.15,
    "speed_pitch_gain": 0.0,
    "speed_pitch_limit": 0.12,
    "speed_pitch_ramp_rate": 0.5,
    "kp_length": 2000.0,
    "kd_length": 100.0,
    "max_force": 500.0,
    "support_force": 0.0,
    "length_ramp_rate": 0.2,
    "kp_phi0": 50.0,
    "kd_phi0": 0.5,
    "max_phi0_torque": 80.0,
    "yaw_hold_enable": 1.0,
    "yaw_hold_kp": 2000.0,
    "yaw_hold_kd": 0.0,
    "yaw_hold_deadband": 0.01,
    "yaw_hold_current_limit": 5000.0,
    "turn_current_max": 1000.0,
}


def _rtt_host(data: dict[str, Any]) -> str:
    host = str(data.get("rtt_host") or "").strip()
    if not host or host.lower() in {"undefined", "null", "none"}:
        return DEFAULT_RTT_HOST
    return host


def _rtt_port(data: dict[str, Any]) -> int:
    try:
        port = int(data.get("rtt_port") or 0)
    except (TypeError, ValueError):
        port = 0
    if port <= 0 or port > 65535:
        return DEFAULT_RTT_PORT
    return port


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
            host=_rtt_host(data),
            port=_rtt_port(data),
            timeout_s=float(data.get("rtt_timeout_s", 1.0)),
            dry_run=bool(data.get("dry_run", False)),
        )
    return result


def _balance_params_payload(data: dict[str, Any]) -> str:
    params = dict(DEFAULT_BALANCE_PARAMS)
    params.update(data.get("params", {}))
    payload = {"type": "vmc_balance_params"}
    for key in DEFAULT_BALANCE_PARAMS:
        payload[key] = float(params[key])
    return json.dumps(payload, separators=(",", ":"))


def _send_balance_params_response(data: dict[str, Any]) -> dict[str, Any]:
    payload = _balance_params_payload(data)
    return {
        "payload": payload,
        "rtt": send_rtt_line(
            payload,
            host=_rtt_host(data),
            port=_rtt_port(data),
            timeout_s=float(data.get("rtt_timeout_s", 1.0)),
            dry_run=bool(data.get("dry_run", False)),
        ),
    }


def _safe_text(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


class PersistentRttConnection:
    def __init__(self) -> None:
        self._sock: socket.socket | None = None
        self._target: tuple[str, int] | None = None
        self._banner = ""
        self._lock = threading.Lock()

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._target = None
        self._banner = ""

    def _drain_banner(self, sock: socket.socket) -> str:
        return self._read_available(sock, quiet_window_s=0.30, max_wait_s=1.20)

    def _read_available(
        self,
        sock: socket.socket,
        *,
        quiet_window_s: float,
        max_wait_s: float,
        stop_tokens: tuple[str, ...] = (),
    ) -> str:
        deadline = time.monotonic() + max_wait_s
        quiet_deadline = time.monotonic() + quiet_window_s
        banner = ""
        sock.settimeout(min(quiet_window_s, 0.10))
        while time.monotonic() < deadline and time.monotonic() < quiet_deadline:
            try:
                chunk = sock.recv(512)
            except socket.timeout:
                continue
            if not chunk:
                break
            banner += chunk.decode("utf-8", errors="replace")
            if stop_tokens and any(token in banner for token in stop_tokens):
                break
            quiet_deadline = time.monotonic() + quiet_window_s
        return banner

    def _connect(self, host: str, port: int, timeout_s: float) -> str:
        self.close()
        sock = socket.create_connection((host, port), timeout=timeout_s)
        banner = self._drain_banner(sock)
        self._sock = sock
        self._target = (host, port)
        self._banner = banner
        return banner

    def _ensure_connected(self, host: str, port: int, timeout_s: float) -> tuple[socket.socket, str, bool]:
        if self._sock is None or self._target != (host, port):
            banner = self._connect(host, port, timeout_s)
            return self._sock, banner, True  # type: ignore[return-value]
        return self._sock, "", False

    def read_available(self, host: str, port: int, timeout_s: float = 0.2) -> str:
        with self._lock:
            try:
                sock, _, _ = self._ensure_connected(host, port, timeout_s)
                return self._read_available(
                    sock,
                    quiet_window_s=0.02,
                    max_wait_s=0.05,
                )
            except OSError:
                self.close()
                return ""

    def send(self, frame: str, host: str, port: int, timeout_s: float) -> dict[str, Any]:
        with self._lock:
            reconnect_count = 0
            last_error = ""
            banner = ""

            for attempt in range(2):
                try:
                    sock, connect_banner, connected_now = self._ensure_connected(host, port, timeout_s)
                    if connected_now:
                        reconnect_count += 1
                        banner += connect_banner

                    sock.settimeout(timeout_s)
                    stale_output = self._read_available(
                        sock,
                        quiet_window_s=0.05,
                        max_wait_s=0.15,
                        stop_tokens=("VMC_LQR_RTT", "VMC_BALANCE_PARAMS"),
                    )
                    sock.sendall(frame.encode("utf-8"))

                    ack = self._read_available(
                        sock,
                        quiet_window_s=0.08,
                        max_wait_s=max(timeout_s, 1.0),
                        stop_tokens=("VMC_LQR_RTT", "VMC_BALANCE_PARAMS"),
                    )

                    return {
                        "ok": True,
                        "persistent": True,
                        "reconnect_count": reconnect_count,
                        "server_banner": banner.strip(),
                        "stale_output": stale_output.strip(),
                        "target_ack": ack.strip(),
                        "target_applied": "VMC_LQR_RTT OK" in ack or "VMC_BALANCE_PARAMS OK" in ack,
                        "host": host,
                        "port": port,
                        "frame": frame,
                    }
                except OSError as exc:
                    last_error = str(exc)
                    self.close()
                    if attempt == 0:
                        continue

            return {
                "ok": False,
                "persistent": True,
                "error": f"RTT TCP send failed to {host}:{port}: {last_error}",
                "host": host,
                "port": port,
                "frame": frame,
            }


_RTT_CONNECTION = PersistentRttConnection()


class VofaTcpBridge:
    def __init__(
        self,
        *,
        vofa_host: str,
        vofa_port: int,
        rtt_host: str,
        rtt_port: int,
    ) -> None:
        self.vofa_host = vofa_host
        self.vofa_port = vofa_port
        self.rtt_host = rtt_host
        self.rtt_port = rtt_port
        self._clients: list[socket.socket] = []
        self._clients_lock = threading.Lock()
        self._line_buffer = ""
        self._stop = threading.Event()

    def start(self) -> None:
        threading.Thread(target=self._serve_clients, daemon=True).start()
        threading.Thread(target=self._poll_rtt, daemon=True).start()
        print(f"VOFA TCP bridge listening on tcp://{self.vofa_host}:{self.vofa_port}")

    def _serve_clients(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.vofa_host, self.vofa_port))
            server.listen(4)
            server.settimeout(0.5)
            while not self._stop.is_set():
                try:
                    client, addr = server.accept()
                except socket.timeout:
                    continue
                except OSError as exc:
                    print(f"VOFA TCP accept stopped: {exc}")
                    return
                client.setblocking(False)
                with self._clients_lock:
                    self._clients.append(client)
                print(f"VOFA client connected from {addr[0]}:{addr[1]}")

    def _broadcast(self, line: str) -> None:
        data = line.encode("utf-8", errors="replace")
        with self._clients_lock:
            alive: list[socket.socket] = []
            for client in self._clients:
                try:
                    client.sendall(data)
                    alive.append(client)
                except OSError:
                    try:
                        client.close()
                    except OSError:
                        pass
            self._clients = alive

    def _handle_rtt_text(self, text: str) -> None:
        self._line_buffer += text
        while "\n" in self._line_buffer:
            line, self._line_buffer = self._line_buffer.split("\n", 1)
            line = line.strip()
            if not line.startswith("VMCVOFA,"):
                continue
            self._broadcast(line[len("VMCVOFA,"):] + "\n")

    def _poll_rtt(self) -> None:
        while not self._stop.is_set():
            text = _RTT_CONNECTION.read_available(self.rtt_host, self.rtt_port, timeout_s=0.2)
            if text:
                self._handle_rtt_text(text)
            else:
                time.sleep(0.01)


def send_rtt_line(
    line: str,
    *,
    host: str = DEFAULT_RTT_HOST,
    port: int = DEFAULT_RTT_PORT,
    timeout_s: float = 1.0,
    dry_run: bool = False,
) -> dict[str, Any]:
    frame = line.rstrip("\n") + "\n"
    if dry_run:
        return {"ok": True, "dry_run": True, "frame": frame}
    return _RTT_CONNECTION.send(frame, host, port, timeout_s)


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
                    "rtt_host": DEFAULT_RTT_HOST,
                    "rtt_port": DEFAULT_RTT_PORT,
                    "balance_params": DEFAULT_BALANCE_PARAMS,
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
                            host=_rtt_host(data),
                            port=_rtt_port(data),
                            timeout_s=float(data.get("rtt_timeout_s", 1.0)),
                            dry_run=bool(data.get("dry_run", False)),
                        ),
                    },
                )
                return
            if self.path == "/api/send-params":
                self._send_json(200, {"ok": True, **_send_balance_params_response(data)})
                return
            self._send_json(404, {"ok": False, "error": "not found"})
        except Exception as exc:  # noqa: BLE001
            self._send_json(400, {"ok": False, "error": str(exc)})

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"{self.address_string()} - {fmt % args}")


def _is_same_backend_process(pid: int) -> bool:
    if pid == os.getpid():
        return False
    try:
        result = subprocess.run(
            ["ps", "-p", str(pid), "-o", "command="],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    command = result.stdout.strip()
    return str(Path(__file__).resolve()) in command or "tools/lqr_tuner/server.py" in command


def _list_listening_pids(port: int) -> list[int]:
    try:
        result = subprocess.run(
            ["lsof", "-nP", f"-iTCP:{port}", "-sTCP:LISTEN", "-t"],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    pids = []
    for line in result.stdout.splitlines():
        try:
            pids.append(int(line.strip()))
        except ValueError:
            pass
    return pids


def _stop_stale_backend_on_port(port: int) -> bool:
    stopped = False
    for pid in _list_listening_pids(port):
        if not _is_same_backend_process(pid):
            continue
        print(f"Stopping stale LQR tuner backend pid={pid} on port {port}")
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            continue
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                os.kill(pid, 0)
            except OSError:
                stopped = True
                break
            time.sleep(0.05)
        else:
            try:
                os.kill(pid, signal.SIGKILL)
                stopped = True
            except OSError:
                pass
    return stopped


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--rtt-host", default=DEFAULT_RTT_HOST)
    parser.add_argument("--rtt-port", type=int, default=DEFAULT_RTT_PORT)
    parser.add_argument("--vofa-host", default=DEFAULT_VOFA_HOST)
    parser.add_argument("--vofa-port", type=int, default=DEFAULT_VOFA_PORT)
    parser.add_argument("--no-vofa", action="store_true")
    args = parser.parse_args()

    _stop_stale_backend_on_port(args.port)
    if not args.no_vofa:
        _stop_stale_backend_on_port(args.vofa_port)
        VofaTcpBridge(
            vofa_host=args.vofa_host,
            vofa_port=args.vofa_port,
            rtt_host=args.rtt_host,
            rtt_port=args.rtt_port,
        ).start()
    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"LQR tuner API listening on http://{args.host}:{args.port}")
    httpd.serve_forever()


if __name__ == "__main__":
    main()
