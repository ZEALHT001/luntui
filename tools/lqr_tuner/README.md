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

## Kalman Telemetry

`VMCVOFA` retains fields 0..98 for existing readers and appends the filter
diagnostics below:

| Field | Meaning | Encoding |
|---:|---|---|
| 81..84 | `q_v`, `q_a`, `r_wheel`, `r_imu` | fixed-point, nine decimal places |
| 99 | `HAL_GetTick()` | integer milliseconds |
| 100 | wheel-derived velocity measurement | value multiplied by 1000 |
| 101 | IMU acceleration measurement consumed by the filter | value multiplied by 1000 |
| 102 | velocity innovation before correction | value multiplied by 1000 |
| 103 | acceleration innovation before correction | value multiplied by 1000 |
| 104 | diagnostic slip-gate state | 0 or 1 |
| 105 | wheel-speed-minus-fused-speed slip metric | value multiplied by 1000 |
| 106 | resolved accelerometer sign convention | `+1`, `-1`, or `0` while calibrating |
| 107 | raw body-X acceleration | g, multiplied by 1000 |
| 108 | gravity-compensated body-X acceleration | g, multiplied by 1000 |
| 109 | accelerometer corrections applied | counter |
| 110 | forward/back stick (`ch[3]`) | raw channel value |
| 111 | `speed_target` from the stick | value multiplied by 1000 |
| 112 | `speed_target_ramped` handed to the LQR | value multiplied by 1000 |
| 113 | `dx_target` used by the position loop | value multiplied by 1000 |
| 114 | `dx_error` of the speed loop | value multiplied by 1000 |
| 115 | telemetry schema version | integer |
| 116..117 | slip-gate latch state, left and right | `0` or `1` |
| 118..119 | raw per-wheel speed fed to condition C | value multiplied by 1000 |
| 120 | condition-C acceleration threshold | value multiplied by 1000 |
| 121 | slip gate enabled at runtime | `0` or `1` |
| 122..124 | body roll, body pitch, virtual-leg theta | value multiplied by 1000 |
| 125 | body roll rate (`gyro[1]`) | value multiplied by 1000 |
| 126..127 | weakest and strongest leg joint load | value multiplied by 1000 |
| 128 | pitch captured when the airborne latch armed | value multiplied by 1000, `-999` when unarmed |
| 129 | body pitch rate consumed by the controller | value multiplied by 1000 |
| 130..131 | per-side leg load, left and right | value multiplied by 1000 |
| 132..133 | per-side airborne latch, left and right | `0` or `1` |

Fields 130..133 exist because the whole-machine latch cannot see a single
lifted leg: it keys on the average of all four joints, and the leg that is
still carrying the body holds that average high.  Fields 130/131 give each
leg's own load so the per-side decision can be replayed; fields 132/133 are
the latches, and each gates only its own wheel - the grounded side has to keep
balancing the machine.

Fields 126..129 exist because the airborne latch needs to be reproducible
offline.  Recording `20260927_173327` shows why the latch keys on the *mean*
leg load (field 88) and not on the strongest leg (field 127): held in the air
the weakest leg reads p50 0.49 N.m but the strongest reads p50 3.89 N.m,
because the leg controller keeps driving the joints while airborne.  An
"all four legs under threshold" test - `max` on field 127 - is therefore
satisfied on only 7 of 275 in-air frames, while the mean test is satisfied on
126 and still never trips while the machine rests on its legs (average load
stays above 10 N.m).  Field 128 is the attitude reference the pitch guard
compares against; field 129 is the pitch rate that guard tests.

All channels use integer ASCII output, so the firmware does not depend on
newlib-nano floating-point `printf` support.

`kf_accel_measurement` (field 101) is the gravity-compensated body-X linear
acceleration in m/s2.  The HI12 reports `acc_b` in g, so the firmware removes
the attitude-derived gravity projection and scales the remainder by
`VMC_GRAVITY_ACCEL`.  The accelerometer is gated to its own 200 Hz frame rate;
feeding one frame into five 1 kHz updates would otherwise make `r_imu`
effectively five times stronger than configured.

Field 115 exists because the field count alone cannot identify the layout:
Q/R switched from a three-decimal to a nine-decimal fixed point without adding
or removing any field.  Bump `VMC_LQR_RTT_SCHEMA_VERSION` and this table
together whenever the layout or an existing field's encoding changes.

Calibrate `r_wheel`/`r_imu` from the measured sensor noise rather than treating
them as tuning knobs.  Scaling Q and R by the same factor leaves the filter
gain unchanged, so an uncalibrated pair collapses the two knobs into one.
`tools/rtt_kalman_capture.py` measures the innovation variance and reports the
consistency ratio (NIS) that verifies the calibration.

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
