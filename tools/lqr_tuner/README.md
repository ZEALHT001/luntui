# LQR Tuner

Local tuning UI for the wheel-leg VMC LQR schedule.

## Start Backend

```sh
cd /Users/zhouzhanen/chuantui
python3 aboard/tools/lqr_tuner/server.py --host 127.0.0.1 --port 8765
```

The backend uses the existing `aboard/scripts/derive_vmc_lqr_abk.py` model and
requires `numpy` and `scipy`.

## Start Frontend

```sh
cd /Users/zhouzhanen/chuantui/aboard/tools/lqr_tuner/web
npm install
npm run dev
```

Open the Vite URL and keep the backend running.

## RTT Downlink

The server sends one JSON line through SEGGER RTT:

```json
{"type":"vmc_lqr_k","preset":"abk","leg_length":0.28,"state_order":["theta","dtheta","x","dx","phi","dphi"],"input_order":["T","Tp"],"k_flat_col_major":[...]}
```

Default RTT client path:

```text
/Applications/SEGGER/JLink/JLinkRTTClient
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
