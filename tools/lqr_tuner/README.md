# LQR Tuner

Local tuning UI for the wheel-leg VMC LQR schedule.

## Start Backend

```sh
cd /Users/zhouzhanen/chuantui
python3 aboard/tools/lqr_tuner/server.py --host 127.0.0.1 --port 8765
```

The backend uses the existing `aboard/scripts/derive_vmc_lqr_abk.py` model and
requires `numpy` and `scipy`.

The model already includes the drivetrain relationship used by the firmware
gain schedule. The tuner therefore applies the LQR wheel input directly and
must not multiply it by the gearbox ratio or efficiency again.

## Start Frontend

```sh
cd /Users/zhouzhanen/chuantui/aboard/tools/lqr_tuner/web
npm install
npm run dev
```

Open the Vite URL and keep the backend running.

## RTT Downlink

Start the VS Code Cortex-Debug `J-Link Debug STM32F427` or `J-Link Attach STM32F427` configuration first. The launch config starts the J-Link GDB server with RTT Telnet on `127.0.0.1:19021`. The Python backend opens one persistent TCP connection to that port and reuses it for K and balance-parameter updates; it does not launch `JLinkRTTClient`. If the debug session restarts, the backend will reconnect on the next send.

The server sends this payload shape through RTT channel 0:

```json
{"type":"vmc_lqr_k","preset":"abk","leg_length":0.28,"state_order":["theta","dtheta","x","dx","phi","dphi"],"input_order":["T","Tp"],"k_flat_col_major":[...]}
```

Firmware parsing is implemented in `aboard/App/Src/vmc_lqr_rtt.c`. A valid
payload enables runtime K override and copies the 12 column-major gains to both
left and right LQR matrices before every balance update.

To return to the compiled leg-length gain schedule, send either command:

```text
LQRCLEAR
```

or:

```json
{"type":"vmc_lqr_clear"}
```

Useful Live Watch fields are under `vmc_probe_state.lqr`: `rtt_override_active`,
`rtt_update_count`, `rtt_error_count`, `rtt_last_error`, and `rtt_last_update_ms`.

Run the model/firmware consistency regression from `aboard/` with:

```sh
python3 -m unittest tools/lqr_tuner/test_model_consistency.py -v
```
