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

## Incremental MPC

The controller selector on the balance page exposes `mpc_enable` and
`mpc_shadow_enable`.  At boot both are `0` so the machine starts on the verified
stable2 LQR path without extra solver load.  Set `mpc_shadow_enable=1` only
after LQR behaviour has been checked; the solver then runs and publishes its
candidate wheel/leg torques every cycle while LQR remains the command source.
After solve time and output alignment have been validated, `mpc_enable=1`
switches the same prewarmed solver to command the wheel torque `T` and
virtual-leg torque `Tp`.

`compute_only_enable=1` is the bench timing mode.  LQR and shadow MPC still
execute when shadow mode is enabled, but wheel currents and all four joint
torques are forced to zero before the CAN layer.  This mode must be disabled
explicitly before any driving test.

The MPC owns the steady-state integral action.  While it is enabled, the old
disturbance observer is held at zero to prevent two integrators fighting.
Disable MPC with the same selector at any time; the next control cycle drops
back to LQR.

Forward motion keeps the verified stable2 Kalman filter: the average wheel
velocity and gravity-compensated IMU acceleration are fused into `kf_dx`, and
`kf_x` integrates that estimate.  The MPC shadow state consumes the same
`kf_dx` used by LQR.

## Start Frontend

```sh
cd /Users/zhouzhanen/chuantui/aboard/tools/lqr_tuner/web
npm install
npm run dev
```

Open the Vite URL and keep the backend running.

## RTT Downlink

Every value on the page is applied at runtime; nothing on this path needs a
reflash.  That includes the automatic jump gate, added to the page under
"自动跳跃门控":

| Field | Meaning |
| --- | --- |
| `jump_auto_speed_min` | Wheel-odometry body speed the machine must reach, m/s (`kf_dx`) |
| `jump_auto_dist_min_cm` | Lower laser bound, exclusive |
| `jump_auto_dist_max_cm` | Upper laser bound, exclusive |
| `jump_auto_confirm_ms` | How long all three conditions must hold before firing |

The firmware normalises the distance pair on write, so a payload that sets
`max` before `min` (the chunked sender does not guarantee order) cannot leave an
inverted window that silently disables the gate forever.  The launch still runs
through `vmc_jump_start_is_allowed()`, so attitude, feedback and enable are
unchanged by anything tuned here.

The macros of the same names in `App/Inc/vmc_probe.h` are only the power-on
defaults.  "Write firmware defaults" updates them through the same table, and
the write is idempotent when the values already match.

Start the VS Code Cortex-Debug `J-Link Debug STM32F427` or `J-Link Attach STM32F427` configuration first. The launch config starts the J-Link GDB server with RTT Telnet on `127.0.0.1:19021`. The Python backend opens one persistent TCP connection to that port and reuses it for K and balance-parameter updates; it does not launch `JLinkRTTClient`. If the debug session restarts, the backend will reconnect on the next send.

The server sends this payload shape through RTT channel 0:

```json
{"type":"vmc_lqr_k","preset":"abk","leg_length":0.28,"state_order":["theta","dtheta","x","dx","phi","dphi"],"input_order":["T","Tp"],"k_flat_col_major":[...]}
```

Firmware parsing is implemented in `aboard/App/Src/vmc_lqr_rtt.c`. A valid
payload enables runtime K override and copies the 12 column-major gains to both
left and right LQR matrices before every balance update.

## Motion-Estimator Telemetry

`VMCVOFA` retains fields 0..98 for existing readers and appends the legacy
filter diagnostics below.  The innovation fields remain in the format for
compatibility but are zero on this branch because forward motion is wheel-only.

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
| 134 | measured control period | microseconds |
| 135 | control-loop iteration counter | count |
| 136 | TF02-Pro laser distance | centimetres, `0` with connected `0` means no reading |
| 137 | TF02-Pro laser health flag | `0` or `1`, cleared after 100 ms without a frame |
| 138 | roll compensation offset | metres, the commanded differential / 2 |
| 139 | left leg length target | metres, including roll compensation, airborne 0.30 m override and MIN/MAX clamp |
| 140 | right leg length target | metres, including roll compensation, airborne 0.30 m override and MIN/MAX clamp |
| 141 | ground-following extension, left | metres, already included in field 139 |
| 142 | ground-following extension, right | metres, already included in field 140 |
| 143 | side the follow feature is extending | `0` left, `1` right, `2` none |
| 144 | unload episode in progress | `0` or `1` |
| 145 | side classification settled | `0` = still being timed out, `1` = decided |
| 146 | classification being timed out | same encoding as 143 |
| 147 | raw Y acceleration | g |
| 148 | raw Z acceleration | g |
| 149 | gravity-compensated Y acceleration | g |
| 150 | gravity-compensated Z acceleration | g |
| 151 | slip LQR posture scale applied, left | `1.0` = not scaled |
| 152 | slip LQR posture scale applied, right | `1.0` = not scaled |
| 153 | slip wheel-current scale applied, left | `1.0` = not scaled |
| 154 | slip wheel-current scale applied, right | `1.0` = not scaled |
| 155 | anti-splay error, left_theta - right_theta | radians, value multiplied by 1000 |
| 156 | anti-splay differential torque applied | N.m virtual-leg torque, left takes `-`, right `+` |
| 157 | automatic stair stage | `0` idle, `1` retract, `2` hold, `3` finished, `4` abort |
| 158 | stair abort reason | `0` none, `1` remote, `2` feedback, `3` get-up, `4` jump, `5` timeout, `6` airborne |
| 159 | stair ready detection | `0` or `1` |
| 160 | stair pre-impact armed | `0` or `1` |
| 161 | stair impact detected | `0` or `1` |
| 162..163 | stair trigger / abort counts | count |
| 164 | stair wheel speed magnitude | m/s |
| 165 | stair wheel-speed drop | m/s, legacy trigger diagnostic |
| 166 | stair wheel-speed acceleration | m/s2 |
| 167 | wheel-side theta target during stair | radians, set to the measured average while the phi0 PD owns the legs |
| 168..169 | per-leg stair impact, left and right | `0` or `1` |
| 170 | stair fast-speed confirmation | `0` or `1` |
| 171 | stair speed confirmation timer | ms |
| 172 | stair pre-impact confirmation timer | ms |
| 173 | stair impact confirmation timer | ms |
| 174 | remote source | `0` none, `3` local SBUS |
| 175 | master switch, synthesized from SBUS channel 4 | `1` up, `3` mid, `2` down |
| 176 | leg knob, synthesized from SBUS channel 9 | `1` high, `3` middle, `2` low |
| 177..186 | reserved compatibility fields | always `0` |
| 187 | MPC runtime enable | `0` or `1` |
| 188 | MPC active on the last control cycle | `0` or `1` |
| 189..190 | MPC solve-failure and saturation counters | count |
| 191..192 | MPC left/right wheel torque | N.m |
| 193..194 | MPC left/right virtual-leg torque | N.m |
| 195..196 | MPC left/right wheel torque increment | N.m per cycle |
| 197..198 | MPC left/right virtual-leg torque increment | N.m per cycle |
| 199 | MPC solve time for both sides | microseconds |
| 200 | compute-only bench mode | `1` means all final actuator outputs are zero |
| 201 | controller enabled | `0` or `1` |
| 202..203 | applied MPC wheel/Tp authority | `0..1` |
| 204..205 | raw QP wheel torque, left/right | N.m, before rate limiting |
| 206..207 | raw QP virtual-leg torque, left/right | N.m, before rate limiting |
| 208 | MPC rate-limit count | count |
| 209 | MPC torque-limit count | count |
| 210 | average leg length used by the MPC model | m |
| 211..216 | left MPC state: theta, dtheta, x, dx, phi, dphi | rad, rad/s, m, m/s, rad, rad/s |
| 217..222 | right MPC state: theta, dtheta, x, dx, phi, dphi | rad, rad/s, m, m/s, rad, rad/s |
| 223..224 | left previous MPC input, T and Tp | N.m |
| 225..226 | right previous MPC input, T and Tp | N.m |
| 227..246 | left pre-solve QP plan, ten `(T, Tp)` pairs | N.m |
| 247..266 | right pre-solve QP plan, ten `(T, Tp)` pairs | N.m |
| 267 | MPC integral of forward position error | m·s |
| 268 | MPC integral currently awake | `0` or `1` |
| 269 | MPC integral hold reason | enum, see below |
| 270 | MPC integral at its configured limit | `0` or `1` |

Fields 157..173 publish the automatic stair climb: stage and abort reason show
the state machine, ready/pre-impact/impact show how far the trigger progressed,
the per-leg flags show whether only one side saw the step, and the three timer
channels show whether a threshold was missed or merely held too briefly.
Fields 174..176 (schema 21) identify which remote produced the frame and the
synthesized master/leg-switch positions, so a REMOTE abort can be separated
from a channel-4 stop command or a source handover.

Fields 177..186 are retained only to preserve the historical recorder layout.
The former automatic platform descent feature has been removed and these
fields are now always zero.

Fields 187..199 were added with schema 23. They separate a controller-mode
problem from an actuator or estimator problem: `mpc_active` proves the new
controller actually produced the output, the counters expose solver failure
or saturation, and the torque/delta channels show whether the fallback LQR was
used and whether the MPC output is plausible. The solve-time channel is the
sum for both sides.

Fields 200..201 were added with schema 24 for bench timing. `compute_only=1`
means LQR and shadow MPC still execute but the wheel-current and joint-torque
commands are forced to zero before transmission.

Fields 202..209 were added with schema 25. They separate raw QP demand from the
rate-limited command, publish the actual wheel/Tp authority ramp, and distinguish
rate limiting from actuator torque saturation.

Fields 210..226 were added with schema 26. They record the exact plant input and
the previous incremental control passed to the QP, not values reconstructed
from the rounded LQR torque terms. This makes the wheel-output discrepancy
replayable offline and distinguishes a state/objective mismatch from a solver
or rate-limiter problem.

Fields 227..242 were added with schema 27. They record the exact warm-start
plan supplied to the fixed five-iteration ADMM solver, in public T/Tp sign
convention. Together with the state and previous input they make each sampled
QP result reproducible even when telemetry runs slower than the control loop.

Schema 28 expands the control horizon from four to ten steps, aligning the MPC
stage cost with the LQR model. The two ten-pair plan blocks now occupy fields
227..266.

Schema 29 appends the MPC position-error integrator. This state provides the
steady-state disturbance rejection that the original six-state controller did
not have.

Schema 30 appends integral gate diagnostics. Hold reason `0` means integrating,
`1..9` are hard reset causes (stick, release, manual prepose, get-up, stair,
jump, airborne, feedback timeout, fall re-arm), `10` is excessive speed, `11`
is excessive position error, `12` means MPC is not computing, `13` is a solver
failure, and `14` means speed is inside the wake range but has not stayed
there for the required continuous second.

Fields 155/156 (schema 18) publish the leg-synchronisation (anti-splay) loop.
`left_theta` and `right_theta` were already in the frame, but a difference of
two separately-reported channels cannot separate "the legs never diverged" from
"they diverged and the loop never fired".  Field 155 is the error the loop acts
on and field 156 the differential torque it applied; both are zero while the
feature is disabled, while the measured legs have not reached the long preset,
or while the get-up, jump, or stair state machines own the legs.  The measured-length gate avoids
running the loop during the preset ramp and during the latched short-leg state
after a stair climb.

Fields 151/152 (schema 16) and 153/154 (schema 17) are the slip response's
applied per-side scales - the LQR posture weight and the wheel current.  Both
are latch-dependent values that only exist inside the control cycle, so without
publishing them a recording cannot tell "the response never fired" from "it
fired and the effect was too small to see in the outputs".  That distinction
mattered: the first off/on comparison was only readable because 151/152 showed
the switch had actually engaged.

Fields 147..150 were added with schema 15, which publishes the whole
accelerometer vector.  Only X was carried before, and that left "the
accelerometer disagrees with the wheels" undiagnosable: the wrong axis, a
gravity projection that is not being removed, and a genuinely unmapped axis
all look identical from one channel.  Gravity sits on whichever axis is
vertical at rest, so the raw triplet identifies the mounting, and the paired
`_lin` values show whether the quaternion-based removal takes the same
projection back out.  This matters beyond diagnostics - condition C of the slip
detector subtracts |body acceleration| from |wheel acceleration|, and the one
slip action coasts the estimator on this same channel.

Fields 143..146 were added with schema 12..14.  What the feature acts on is
which side is LOW, classified from the roll direction over a confirmation
window and then held, so it is neither a setting the operator dialled in nor
something the instantaneous latches show.  Without publishing it a recording
cannot tell "the wrong leg is pushing" from "no leg is pushing yet"; roll is
field 122 and the per-side latches are 130..133.

Field 146 is what makes the confirmation window verifiable: it shows a
transient WAS seen and deliberately not believed.

NOTE field 143 changed meaning in schema 14 - it used to carry the
per-leg/whole-machine flag.  That is why the layout gained a field instead of
reusing its count: 146 fields (schemas 12/13) parses as `format-v17`, 147
(schema 14) as `format-v18`, and 151 (schema 15) is the current `format`.
143 fields parses as `format-v15`.

Fields 141..146 were added with schema 11 for the retired ground-following
extension. They remain zero-valued compatibility slots so existing captures and
field indices stay valid. The current controller never changes the leg target
for an unloaded side; it only gates that side's wheel from Shanhai FN.
The 141-field layout is still parsed as `format-v14`.

Fields 138..140 were added with schema 10.  Field 77 is the COMMON ramped
target before the differential is added, so it could not show whether the roll
compensator was running, whether its differential reached the legs, or whether
a target was pinned on the stroke limit.  Those three faults look identical
from the measured leg lengths alone, and the third one is exactly what happens
at short leg lengths: the retract side runs out of stroke at 0.120 m while the
extend side still has 200 mm to go, so the pair stops levelling and starts
raising the machine instead.

Use `tools/roll_comp_report.py <kalman_samples.csv>` to read those three
fields straight into a verdict.  The 138-field layout is still parsed as
`format-v13`.

Fields 136/137 were added with schema 9.  The automatic jump gate fires on the
laser reading strictly inside its configured window while channel 6 is centred
and the fused speed is at least its configured minimum, so without the distance
and the health flag a capture cannot
distinguish "the window was never satisfied" from "the laser was not answering"
from "the robot never reached speed".  The previous 136-field layout is still
parsed as `format-v12`.

Fields 130..133 exist because the whole-machine latch cannot see a single
lifted leg: it keys on the average of all four joints, and the leg that is
still carrying the body holds that average high.  Fields 130/131 give each
leg's own load so the per-side decision can be replayed; fields 132/133 are
the latches, and each gates only its own wheel - the grounded side has to keep
balancing the machine.

Fields 134/135 cover the control loop itself.  The loop is pinned to 2222 us
(450 Hz) by `VMC_CONTROL_PERIOD_US`; it used to free-run, and removing the
snprintf work once let it drift to ~750 Hz, which changes the discrete
realisation of every filter on a machine tuned at 450 Hz.  The measured period
still feeds the Kalman prediction and position integration, slip condition C,
and the command ramps.  Field 134 is the instantaneous period and field 135 an
iteration counter, so dividing its change by the change in field 99 gives the
true average rate over any window - host timestamps cannot, because RTT
batching quantises them.

Do not read field 134 as a representative sample of the loop.  Telemetry is
emitted once every ~9 iterations, and the transmitting iteration is
systematically the shortest: field 134 reads ~2000 us while the true mean over
all iterations is 2222 us (450.0 Hz).  Use field 135 for any rate or duration,
never field 134 alone.  This bit the analysis once - comparing the mean of the
sampled periods against the true mean looked like a 10% under-count, and led to
a wrong conclusion that the DWT timebase was dropping sleep time.  The check
that settles it is the telescoping identity: the firmware integrates
`kf_x += kf_dx * dt`, so `kf_x / integral(kf_dx dt)` computed on the MCU
timebase must come out 1.000; it does, at RMSE < 1 mm.

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
