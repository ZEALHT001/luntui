#ifndef VMC_PROBE_H
#define VMC_PROBE_H

#include "can.h"
#include "cmsis_os.h"
#include "vmc_mpc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VMC_WHEEL_LEFT_ID         0x202U
#define VMC_WHEEL_RIGHT_ID        0x201U
#define VMC_WHEEL_CMD_ID          0x200U
#define VMC_WHEEL_CMD_CAN         (&hcan1)
#define VMC_WHEEL_LEFT_CMD_DIRECTION  -1
#define VMC_WHEEL_RIGHT_CMD_DIRECTION  1

#define VMC_JOINT_RIGHT_BACK_FEEDBACK_ID   0x003U
#define VMC_JOINT_RIGHT_AHEAD_FEEDBACK_ID  0x001U
#define VMC_JOINT_LEFT_AHEAD_FEEDBACK_ID   0x007U
#define VMC_JOINT_LEFT_BACK_FEEDBACK_ID    0x005U

#define VMC_JOINT_RIGHT_BACK_CMD_ID        0x004U
#define VMC_JOINT_RIGHT_AHEAD_CMD_ID       0x002U
#define VMC_JOINT_LEFT_AHEAD_CMD_ID        0x008U
#define VMC_JOINT_LEFT_BACK_CMD_ID         0x006U

#define VMC_JOINT_RIGHT_AHEAD_CAN (&hcan1)
#define VMC_JOINT_LEFT_AHEAD_CAN  (&hcan2)
#define VMC_JOINT_RIGHT_BACK_CAN  (&hcan1)
#define VMC_JOINT_LEFT_BACK_CAN   (&hcan2)

#define VMC_ANGLE_T               8191.0f
#define VMC_DM_J8009P_PMAX        12.5f
#define VMC_DM_J8009P_PMIN       -12.5f
#define VMC_DM_J8009_VMAX         45.0f
#define VMC_DM_J8009_VMIN        -45.0f
#define VMC_DM_J8009_TMAX         54.0f
#define VMC_DM_J8009_TMIN        -54.0f
/* Leg torque is commanded in the virtual-leg domain (force along the leg, Tp
 * about the virtual-leg angle) and only afterwards mapped onto the two joint
 * motors.  vmc_leg_force() puts exactly half of Tp on each joint, and that
 * 0.5 coefficient is posture independent, so one joint saturates at
 * |Tp| = 2 * VMC_DM_J8009_TMAX = 108.  VMC_DM_J8009_TMAX is the per-joint
 * limit; VMC_LEG_TP_MAX is the matching virtual-leg limit.  Mixing the two
 * units is what used to cap each joint at 27 N.m.
 *
 * Kept as plain literals on purpose: App/Src parsers (the LQR tuner and its
 * tests) read these tokens as numbers and cannot evaluate macro arithmetic. */
#define VMC_LEG_TP_MAX            108.0f

#define VMC_LEG_LENGTH_DEFAULT     0.280f
/* Leg-length command travel, in metres.
 *
 * MIN was 0.120 until this was actually measured.  On the vehicle the leg
 * cannot physically get shorter than 0.160 m: commanded to 0.119 m it sat at
 * 0.161 +-1 mm for 40 s, which is a mechanical end stop, not a control
 * limitation.  A tape measure confirmed 0.160 m.  The old value let every
 * length target be driven 40 mm below the hard stop, which silently turned
 * "retract this leg" into "hold the stop" and made the roll compensator look
 * far weaker than it is - the target was pinned at MIN while the leg was
 * pinned against metal.
 *
 * The software minimum is now 0.180, avoiding the low-length oscillation
 * region and leaving 20 mm of safety margin above the
 * measured 0.160 hard stop.  MAX stays 0.385.
 *
 * CONSEQUENCE - the headroom each preset leaves is much smaller than it
 * looked, especially downward.  See the table at
 * VMC_ROLL_LENGTH_COMP_LIMIT_DEFAULT below, which is the same arithmetic. */
#define VMC_LEG_LENGTH_MIN         0.180f
#define VMC_LEG_LENGTH_MAX         0.385f
#define VMC_LEG_LENGTH_KP_DEFAULT  2000.0f
#define VMC_LEG_LENGTH_KD_DEFAULT  100.0f
/* Leg force ceiling, used by BOTH the stance leg-length loop and the jump.
 *
 * It has to clear what a jump needs: the machine is about 20 kg, so ~200 N of
 * total leg force only just holds it up and leaves no upward acceleration at
 * all.  At 200 the jump could extend the legs but never leave the ground.  A
 * normal stance only commands ~100-200 N per leg, so raising the ceiling does
 * not change how the robot stands - a limit that is never reached has no
 * effect - it only stops the jump from being clipped.
 *
 * Note the jump also applies its own, lower clamp (VMC_JUMP_FORCE_MAX_DEFAULT)
 * inside this one, so both have to be large enough. */
#define VMC_LEG_FORCE_MAX_DEFAULT  500.0f
#define VMC_LEG_LENGTH_PID_DIRECTION -1.0f
#define VMC_LEG_PHI0_TARGET_DEFAULT VMC_PHI0_VERTICAL_RAD
#define VMC_LEG_PHI0_KP_DEFAULT     50.0f
#define VMC_LEG_PHI0_KD_DEFAULT     0.5f
#define VMC_LEG_PHI0_TORQUE_MAX_DEFAULT 108.0f /* = VMC_LEG_TP_MAX */
#define VMC_LEG_PHI0_PID_DIRECTION  1.0f
#define VMC_LEG_SUPPORT_FORCE_DEFAULT 50.0f
/* Ceiling on the constant leg feedforward, used by both the standing support
 * term and the jump's extension feedforward.
 *
 * Raised from 120 so the jump can actually reach the tail of its stroke. The
 * jump needs >150 N of feedforward to matter there: the length loop is a
 * position PD whose force collapses with the remaining error, and measured
 * over one EXTEND it fell from ~380 N to 19 N, so the last third of the stroke
 * was braking. Standing only ever asks for VMC_LEG_SUPPORT_FORCE_DEFAULT
 * (120 N), which is unchanged, so this ceiling does not affect how the robot
 * stands. */
#define VMC_LEG_SUPPORT_FORCE_MAX     250.0f
#define VMC_LEG_SUPPORT_FORCE_DIRECTION -1.0f

/* SBUS channel 9 is a continuous leg-length knob.  Its -1 endpoint maps to
 * VMC_LEG_KNOB_MIN_DEFAULT, midpoint is the linear midpoint, and +1 maps to
 * LONG.  Units are metres;
 * VMC_LEG_LENGTH_MIN/MAX (0.180/0.385) still clamp the result.
 *
 * SHORT is also the length the self-righting sequence stands up on
 * (VMC_GETUP_STAND_LENGTH), so changing it moves the recovery pose too.
 *
 * SHORT sits at the software minimum; LONG leaves only 25 mm before
 * the maximum stroke. */
#define VMC_LEG_TARGET_SHORT      0.180f
#define VMC_LEG_TARGET_MID        0.260f
#define VMC_LEG_TARGET_LONG       0.360f
#define VMC_LEG_KNOB_MIN_DEFAULT  0.180f
#define VMC_LEG_GEAR_LOW          0U
#define VMC_LEG_GEAR_MID          1U
#define VMC_LEG_GEAR_HIGH         2U
/* Absorbs SBUS quantisation and knob jitter when comparing against the old
 * discrete targets.  One SBUS count is about 0.12 mm over this knob range. */
#define VMC_LEG_GEAR_THRESHOLD_TOL 0.0005f
/* Scaled by 0.45 to preserve the tuned behaviour; see VMC_CONTROL_DT. */
#define VMC_LEG_LENGTH_RAMP_RATE_DEFAULT 0.225f
#define VMC_JUMP_ENABLE_DEFAULT   1U
/* Allow a jump to be started while the machine is driving.
 *
 * With this enabled the wheels keep balancing through the on-ground
 * COMPRESS and EXTEND phases and are only blanked once the robot is actually
 * in the air (RETRACT/LAND).  Disabling it restores the original behaviour,
 * where the wheels are cut for the whole jump - which makes jumping only
 * viable from a standstill, because 300 ms of no balance control tips a
 * moving machine over before it leaves the ground. */
#define VMC_JUMP_MOVING_ENABLE_DEFAULT 1U
/* Crouch depth.  NOTE this target is currently NOT reached: COMPRESS uses the
 * standing gains and the standing support force, whose 120 N points toward
 * extension, so the net command during COMPRESS is still outward. Measured,
 * the leg moved 1 mm against a target 20 mm away, i.e. the phase is a 180 ms
 * pause rather than a crouch. Changing this number therefore has no effect
 * until COMPRESS is given the jump gains with no extension feedforward. */
#define VMC_JUMP_COMPRESS_LENGTH_DEFAULT 0.18f
#define VMC_JUMP_EXTEND_LENGTH_DEFAULT   0.38f
#define VMC_JUMP_RETRACT_LENGTH_DEFAULT  0.18f
#define VMC_JUMP_LAND_LENGTH_DEFAULT     0.23f
/* Leg-length gains used during the jump's EXTEND and RETRACT phases.
 *
 * KP is the dominant lever on jump height.  The legs are driven by a position
 * PD, so the force is proportional to the REMAINING error: measured over one
 * EXTEND the commanded force fell from 360 N to 48 N as the leg closed on its
 * target, and the extension rate never exceeded ~0.9 m/s.  Raising the force
 * LIMIT alone does nothing (the PD was not saturating); the gain itself has to
 * rise.  Simulated against the measured body mass, 3000 -> 4500 lifts the
 * take-off speed by roughly half.
 *
 * This is deliberately short of doubling: the joint torque scales with the
 * force, the measured peak was 36 N.m against a 54 N.m motor limit, so there
 * is only ~1.4x of headroom before the motors clip.  4500 with a 390 N force
 * cap stays inside it.  Verify the joint torque in telemetry
 * (airborne_joint_torque_max) after any further increase. */
#define VMC_JUMP_LENGTH_KP_DEFAULT       4500.0f
#define VMC_JUMP_LENGTH_KD_DEFAULT       180.0f
/* Push force used during the EXTEND and RETRACT phases of a jump.  This is
 * clamped by VMC_LEG_FORCE_MAX_DEFAULT above, so at 100 the jump inherited a
 * 100 N per-leg ceiling and could not leave the ground.  350 N per leg
 * (~1.75 g of net lift) is what the jump was written with. */
#define VMC_JUMP_FORCE_MAX_DEFAULT       390.0f
/* Constant extension force added on top of the jump length PD.
 *
 * Rationale for the parameter being here at all: the legs run a position PD,
 * so force is proportional to the REMAINING error and collapses as the leg
 * closes on its target.  Measured over one EXTEND (target 0.319 m) the
 * commanded force fell from ~380 N to 19 N across the last third of the
 * stroke, and the extension rate peaked at 1.05 m/s and then DECLINED - the
 * tail of the stroke was braking, because -kd*v eventually outweighs
 * kp*error.  A constant feedforward is the natural correction for exactly
 * that, and it has to be inside +/-VMC_LEG_SUPPORT_FORCE_MAX (now 250 N) to
 * pass the clamp at the call site.
 *
 * 80 N was tried and made no measurable difference: the leg-length trajectory
 * was identical to the 0 N baseline, because at that size the term is small
 * compared with the ~390 N the PD already commands at the start of the stroke,
 * and it is the TAIL that needs help.  180 N did reach the tail and took the
 * take-off speed from 0.91 to 1.37 m/s (a measured ~200 mm jump), so 240 N
 * continues in that direction while staying inside the 250 N clamp.
 *
 * WARNING - this term applies to EXTEND and RETRACT only (jump_force_profile).
 * COMPRESS instead runs on the STANDING gains and the standing support force,
 * and that support force (120 N) points the same way as the jump extension
 * force. Measured during COMPRESS with a 0.18 m target, the net command was
 * -78 N - still extending - so the leg travelled 1 mm and the crouch never
 * happened. Lowering the compress target on its own changes nothing.
 *
 * The earlier "80 N made it worse" reading was a measurement artifact: the
 * take-off speed had been taken over a +-1 sample window, which picks up a
 * single-sample spike, while later runs used +-2. Re-analysed on one window
 * the 80 N run was 0.93 m/s against a 0 N baseline of 0.91, i.e. the same.
 * Always compare jumps with one fixed window - tools/jump_report.py uses +-2
 * samples and reports the window it used. */
#define VMC_JUMP_SUPPORT_FORCE_DEFAULT   240.0f
/* Stage timeouts.  The stage machine advances on (elapsed >= timeout) OR
 * (leg length reached the stage target), so these are timeouts, not delays -
 * but they must still be long enough for the legs to physically travel.  They
 * were all left at 1 ms, which made every stage complete in a millisecond:
 * the target snapped 0.18 -> 0.25 -> 0.18 in about 3 ms and the machine never
 * had time to extend, so jumping was silently impossible.  Restored to the
 * values the jump feature was written with. */
#define VMC_JUMP_COMPRESS_TIMEOUT_MS_DEFAULT 180U
#define VMC_JUMP_EXTEND_TIMEOUT_MS_DEFAULT   160U
#define VMC_JUMP_RETRACT_TIMEOUT_MS_DEFAULT  140U
#define VMC_JUMP_LAND_TIMEOUT_MS_DEFAULT     600U
#define VMC_JUMP_COOLDOWN_MS_DEFAULT         1500U
/* Deliberately larger than the original 1200: the stage sum is 1040 ms, so
 * 1200 left the retract and land phases almost no margin.  1800 still trips
 * long before anything is damaged. */
#define VMC_JUMP_TOTAL_TIMEOUT_MS_DEFAULT    1800U
#define VMC_JUMP_ATTITUDE_LIMIT_DEFAULT      0.35f
#define VMC_JUMP_LAND_TORQUE_THRESHOLD_DEFAULT 3.0f
#define VMC_JUMP_LAND_CONFIRM_MS_DEFAULT     80U
#define VMC_JUMP_LENGTH_READY_TOL            0.015f

/* Automatic jump gate.
 *
 * In SBUS channel 6's middle position the jump fires by itself once the
 * following gates hold at once:
 *
 *   - channel 4 is DOWN (master enable);
 *   - channel 6 is centred (automatic-jump mode);
 *   - the machine is actually travelling at >= VMC_JUMP_AUTO_SPEED_MIN m/s;
 *   - the TF02 laser reads strictly inside the (min, max) window in cm.
 *
 * Speed is the FUSED body velocity (kf_dx, m/s), not the stick demand: the
 * requirement is that the robot is really moving, not that the operator asked
 * for it.  The laser must be fresh - tf02_connected drops after 100 ms without
 * a frame - so a stale echo can never arm a jump.
 *
 * A short confirmation is required before firing.  The laser returns one echo
 * per frame and the speed estimate can spike on a single sample, so one frame
 * must not be able to launch a jump; 20 ms is about nine control cycles at
 * 450 Hz.  The existing stage timeouts, attitude gate, feedback gate, abort
 * logic and landing cooldown all still apply unchanged.
 *
 * These are the POWER-ON defaults.  The live values live in vmc_control
 * (jump_auto_speed_min / jump_auto_dist_min_cm / jump_auto_dist_max_cm /
 * jump_auto_confirm_ms) and are tunable over RTT from the LQR tuner web UI. */
#define VMC_JUMP_AUTO_SPEED_MIN    1.5f   /* m/s */
#define VMC_JUMP_AUTO_DIST_MIN_CM  90.0f  /* exclusive */
#define VMC_JUMP_AUTO_DIST_MAX_CM  110.0f /* exclusive */
#define VMC_JUMP_AUTO_CONFIRM_MS   20U
/* A jump must never come from a single remote frame: SBUS carries no checksum,
 * so a corrupted payload can put any value on the jump channel, and the switch
 * passes through the AUTO band whenever it moves between positions.  The
 * selected mode has to hold this long before it can arm anything. */
#define VMC_JUMP_MODE_STABLE_MS    300U

/* Automatic stair climb (one-shot switch-and-leg-response trigger).
 *
 * Operator procedure: move SBUS channel 7 from disabled to enabled, select the
 * highest channel-9 leg-length gear, and drive into the step.  Once both legs
 * have settled at the long ready length, a simultaneous change in both leg
 * angles with joint load starts one action.  The switch must be returned to
 * disabled before another action can arm; holding it cannot retrigger.
 *
 * LIFT uses the measured rear-leg pose: legs at 0.248 m and body-relative
 * theta=90 deg (1.5708 rad).  RETRACT then settles to the final short-leg
 * theta=60 deg pose.
 * SETTLE holds the climbing pose, then the stair action explicitly hands the
 * chassis to the normal get-up sequence (BRACE -> SHRINK -> UPRIGHT -> READY).
 * This mirrors Shanhai's Chassis_Reset followed by StandFromGround instead of
 * returning a still-tilted chassis directly to the normal LQR.  Body-relative
 * theta is converted to absolute phi0 every cycle.  Wheel output is disabled
 * until get-up releases it.  The short-leg lock remains until the channel-9
 * knob is moved.
 *
 * This is deliberately NOT a jump.  The detector first confirms commanded
 * motion, then uses the previous detector logic: a confirmed pre-impact
 * window, a two-wheel LQR torque spike, and both absolute leg angles above
 * threshold.  Wheel-speed drop remains the fallback when torque detection is
 * disabled. */
#define VMC_STAIR_ENABLE_DEFAULT                   1U
#define VMC_STAIR_LIFT_LENGTH_DEFAULT              0.248f
#define VMC_STAIR_LIFT_THETA_DEFAULT               1.5707963f
#define VMC_STAIR_RETRACT_THETA_DEFAULT            1.0471976f
#define VMC_STAIR_RETRACT_LENGTH_MIN               0.180f
#define VMC_STAIR_RETRACT_LENGTH_DEFAULT           0.180f
#define VMC_STAIR_POSE_LENGTH_TOL_DEFAULT          0.060f
#define VMC_STAIR_POSE_THETA_TOL_DEFAULT           0.300f
#define VMC_STAIR_POSE_CONFIRM_MS_DEFAULT          100U
#define VMC_STAIR_LIFT_SUPPORT_FORCE_DEFAULT       20.0f
#define VMC_STAIR_RETRACT_SUPPORT_FORCE_DEFAULT    -14.0f
/* Legacy serialized target retained for client compatibility. */
#define VMC_STAIR_PHI0_TARGET_DEFAULT              2.500f
#define VMC_STAIR_LIFT_PHI0_RAMP_RATE_DEFAULT      4.0f
#define VMC_STAIR_PHI0_RAMP_RATE_DEFAULT           2.5f
#define VMC_STAIR_PHI0_KP_DEFAULT                  120.0f
#define VMC_STAIR_PHI0_KD_DEFAULT                  6.0f
/* Automatic trigger defaults. */
#define VMC_STAIR_SPEED_THRESHOLD_DEFAULT          0.35f
#define VMC_STAIR_COMMAND_SPEED_THRESHOLD_DEFAULT  0.18f
#define VMC_STAIR_READY_LENGTH_DEFAULT             0.330f
#define VMC_STAIR_READY_CONFIRM_MS_DEFAULT         200U
#define VMC_STAIR_SPEED_CONFIRM_MS_DEFAULT         25U
#define VMC_STAIR_PREIMPACT_CONFIRM_MS_DEFAULT     20U
#define VMC_STAIR_IMPACT_CONFIRM_MS_DEFAULT        15U
#define VMC_STAIR_PREIMPACT_DIFF_MAX_DEFAULT       0.0f
#define VMC_STAIR_IMPACT_DIFF_THRESHOLD_DEFAULT    0.40f
#define VMC_STAIR_IMPACT_TORQUE_THRESHOLD_DEFAULT  1.45f
#define VMC_STAIR_IMPACT_THETA_THRESHOLD_DEFAULT   0.19f
#define VMC_STAIR_LENGTH_RAMP_RATE_DEFAULT         0.180f
#define VMC_STAIR_RETRACT_TIMEOUT_MS_DEFAULT       2000U
#define VMC_STAIR_HOLD_MS_DEFAULT                  500U
/* Kept for RTT/tuner compatibility.  The former RECOVER stage was removed
 * because it drove the legs back toward level while the chassis was still
 * supported by the step, then released the wheels directly into the old LQR
 * position target. */
#define VMC_STAIR_RECOVER_MS_DEFAULT               500U
#define VMC_STAIR_TIMEOUT_MS_DEFAULT               6000U
#define VMC_STAIR_COOLDOWN_MS_DEFAULT              1200U
#define VMC_STAIR_RELEASE_RAMP_MS_DEFAULT          500U
#define VMC_STAIR_LENGTH_KP_DEFAULT                2000.0f
#define VMC_STAIR_LENGTH_KD_DEFAULT                100.0f
#define VMC_STAIR_FORCE_MAX_DEFAULT                500.0f
#define VMC_STAIR_SUPPORT_FORCE_DEFAULT            100.0f
#define VMC_STAIR_LENGTH_READY_TOL                 0.015f

#define VMC_ROLL_COMP_ENABLE_DEFAULT 1U
#define VMC_ROLL_TARGET_DEFAULT 0.0f
#define VMC_ROLL_REMOTE_TARGET_MAX_DEFAULT 0.50f
#define VMC_ROLL_REMOTE_DEADBAND_DEFAULT 0.03f
#define VMC_ROLL_LENGTH_COMP_GAIN_DEFAULT 0.4f
/* Roll length compensation ceiling, in metres of symmetric leg-length offset.
 *
 * The compensator drives the two targets symmetrically about the preset:
 *
 *     left_target  = clamp(ramped + offset)
 *     right_target = clamp(ramped - offset)
 *
 * so the largest offset that can NEVER be clipped is half the travel:
 *
 *     (VMC_LEG_LENGTH_MAX - VMC_LEG_LENGTH_MIN) / 2
 *         = (0.385 - 0.180) / 2 = 0.1025 m
 *
 * That is only reachable when the preset sits exactly at the travel centre
 * (0.2825 m).  Every real preset is off-centre, so the binding limit is the
 * SMALLER of the two headrooms, which is not always the retract side:
 *
 *     preset   retract   extend   symmetric ceiling
 *     SHORT     0 mm     205 mm    0 mm
 *     MID      80 mm     125 mm   80 mm
 *     LONG    180 mm      25 mm   25 mm
 *
 * SHORT and LONG are both limit-bound, just on opposite sides: SHORT cannot
 * retract and LONG cannot extend.  An offset larger than the ceiling is not
 * rejected, it is silently clipped per leg, and while it is being clipped the
 * pair stops levelling and starts raising the machine instead - so this is the
 * physical limit, not a tuning preference.  With the gain at 0.4 the clamp
 * engages past 0.1025 / 0.4 = 0.256 rad (14.7 deg) of roll, where it used to
 * saturate at 0.08 / 0.4 = 0.200 rad (11.5 deg).
 *
 * Plain literals on purpose: the LQR tuner and its tests read these tokens as
 * numbers and cannot evaluate macro arithmetic. */
#define VMC_ROLL_LENGTH_COMP_LIMIT_DEFAULT 0.1025f
#define VMC_ROLL_LENGTH_COMP_LIMIT_MAX     0.1025f
/* Scaled by 0.45 to preserve the tuned behaviour; see VMC_CONTROL_DT. */
#define VMC_ROLL_LENGTH_COMP_RAMP_RATE_DEFAULT 0.054f
/* Fallback control period in seconds, used before the first measurement and
 * if the cycle counter is unavailable.
 *
 * The loop is held to VMC_CONTROL_PERIOD_US, but the value that actually
 * feeds the controller is the measured period: the Kalman prediction and its
 * position integration, slip condition C's differentiator, and the command
 * ramps all call vmc_loop_dt().
 *
 * HISTORY - the loop used to be free-running.  osDelay(1) only set a floor,
 * so the real period was "however long the loop body takes, plus the sleep";
 * measured on the vehicle it ran at 450 Hz, i.e. 2.22 ms, so every term that
 * had assumed 1 ms was off by that factor.
 *
 * Measured period is therefore what actually feeds the controller: the Kalman
 * prediction and its position integration, slip condition C's differentiator,
 * and the command ramps all call vmc_loop_dt().  This constant is only the
 * fallback used before the first measurement and if the cycle counter is
 * unavailable.
 *
 * CONSEQUENCE FOR THE PER-SECOND CONSTANTS BELOW: while dt was wrong, every
 * "rate * dt" term really advanced at 0.001 / 0.00222 = 0.45 of its nominal
 * rate.  Now that dt is measured they all advance at their nominal rate, which
 * is 2.22x faster in real time than the machine was tuned for.  The disturbance
 * observer reacted to that with a limit cycle - its output pinned at the +/-0.1
 * clamp, which drove the balance loop and left the wheels dithering at ~1250 A
 * with NIS_v around 15 while standing still.  Rescaling its gain and leak by
 * 0.45 restored it exactly (NIS_v 0.8-1.7, wheels ~500 A).
 *
 * Every constant that multiplies dt and acts as an actuator command or a timer
 * has therefore been multiplied by 0.45 so the machine keeps the behaviour it
 * was tuned for.  They are written as plain literals rather than as
 * "original * scale" because the LQR tuner parses these macros, and do NOT
 * "fix" them back to round numbers: a round value here means the old
 * 2.22x-too-fast behaviour, which is what caused the observer limit cycle.
 * The estimator terms are deliberately NOT rescaled - they were wrong and are
 * now correct, which is the entire point of measuring dt. */
#define VMC_CONTROL_DT            0.001f
/* Sanity bounds for the measured control period, in seconds.  The lower bound
 * keeps a division by a spuriously small delta from blowing up condition C;
 * the upper bound keeps one stalled cycle from stepping the estimator across
 * a whole second. */
#define VMC_LOOP_DT_MIN           0.0001f
#define VMC_LOOP_DT_MAX           0.05f
/* Fixed control period, in microseconds (333 Hz).
 *
 * The loop is held to this period instead of running as fast as it can.
 * Every rate term in the controller multiplies the measured dt, so a faster
 * loop does not change the tuning in theory - but it does change the
 * discrete-time realisation of every filter.  The MPC model table is
 * discretised at VMC_MPC_MODEL_PERIOD and cannot absorb a measured dt, so the
 * pinned period and the model sample time have to agree.  They did not before
 * 2026-11-09: the period asked for 450 Hz while the solver pushed the body to
 * ~2.92 ms, and the MPC predicted a plant that was 31 % faster than the real
 * one.  Measured over 37.7 s, loop_count / mcu_time was 333.4 Hz.
 *
 * Pinning the period means a telemetry or formatting change cannot alter the
 * closed-loop response, and it keeps the budget honest: the current body
 * leaves ~85 us of slack, so any added per-cycle work shows up as a missed
 * deadline rather than a silent sample-time drift.
 *
 * FreeRTOS ticks at 1 kHz, so the sub-millisecond remainder is waited out by
 * polling TIM1 (1 MHz).  That costs up to one millisecond of idle time per
 * cycle; deterministic timing is worth more here than the idle time.  There
 * is no watchdog and no idle hook, so a busy control task is safe. */
#define VMC_CONTROL_PERIOD_US     3000U
/* Sleep only when a whole RTOS tick comfortably fits in the remaining time.
 * osDelay(1) returns somewhere between almost immediately and ~1 ms later
 * depending on where the call lands inside the tick, so a margin is needed to
 * avoid overshooting the deadline. */
#define VMC_CONTROL_SLEEP_MIN_US  1200
#define VMC_FEEDBACK_TIMEOUT_MS   100U
/* Measured wheel radius: 60 mm.  All wheel-speed -> body-speed conversions
 * use this value, so balance and speed-loop tuning must be revalidated after
 * a wheel change. */
#define VMC_WHEEL_RADIUS          0.060f
#define VMC_AIRBORNE_DETECT_ENABLE_DEFAULT 1U
/* Shanhai ground detection.
 *
 * Each side is evaluated independently from its measured joint torques:
 *
 *   P   = F*cos(theta) + Tp*sin(theta)/L0
 *   ddz = body vertical acceleration - ddL0*cos(theta)
 *         + 2*dL0*dtheta*sin(theta)
 *         + L0*ddtheta*sin(theta)
 *         + L0*dtheta^2*cos(theta)
 *   FN  = P + m_w*g + m_w*ddz
 *
 * FN below the threshold means that wheel is unloaded.  That wheel's torque is
 * cut, its leg target ramps to Shanhai's 0.30 m air length, and Tp keeps only
 * the theta/dtheta posture terms.  The channel-9 target is restored after the
 * short airborne-state release hold. */
#define VMC_AIRBORNE_WHEEL_SPEED_THRESHOLD_DEFAULT 4.0f
/* Vehicle calibration (2026-11-01):
 *   grounded FN p01/median: left 72.8/79.7 N, right 65.4/67.8 N
 *   airborne FN median:     left 22.6 N, right 23.5 N
 * Shanhai's 80 N value cuts both wheels in a normal crouch on this chassis.
 * 35 N gives the partial-lift case extra margin while remaining well above
 * the measured airborne median. */
#define VMC_AIRBORNE_JOINT_TORQUE_THRESHOLD_DEFAULT 35.0f
/* Whole-machine compatibility thresholds.  Wheel speed and pitch no longer
 * participate in ground detection. */
#define VMC_AIRBORNE_CONFIRM_MS_DEFAULT 20U
#define VMC_AIRBORNE_RELEASE_MS_DEFAULT 100U
#define VMC_AIRBORNE_PITCH_DEV_MAX_DEFAULT 0.2f
#define VMC_AIRBORNE_PITCH_RATE_MAX_DEFAULT 4.0f
#define VMC_AIRBORNE_PITCH_REF_HOLD_MS_DEFAULT 200U
#define VMC_AIRBORNE_PITCH_REFERENCE_NONE (-999.0f)
/* FN is filtered before the threshold comparison.  Entry and release use a
 * wide hysteresis band.  Wheel output is released immediately, while the leg
 * posture/0.30 m state has a short minimum hold to avoid state chatter. */
#define VMC_AIRBORNE_SIDE_ENABLE_DEFAULT 1U
#define VMC_AIRBORNE_SIDE_CONFIRM_MS_DEFAULT 30U
#define VMC_AIRBORNE_SIDE_RELEASE_MS_DEFAULT 100U
#define VMC_AIRBORNE_FN_RELEASE_RATIO_DEFAULT 1.70f
#define VMC_AIRBORNE_SIDE_HOLD_MS_DEFAULT 100U
#define VMC_AIRBORNE_LENGTH_RAMP_RATE_DEFAULT 0.30f
#define VMC_AIRBORNE_FN_FILTER_ALPHA_DEFAULT 0.18f
#define VMC_AIRBORNE_LEG_LENGTH_TARGET_DEFAULT 0.30f
#define VMC_WHEEL_MASS_KG                  0.40f
#define VMC_LEG_ACCEL_FILTER_ALPHA_DEFAULT 0.20f
#define VMC_AIRBORNE_SIDE_LEFT  0U
#define VMC_AIRBORNE_SIDE_RIGHT 1U
#define VMC_AIRBORNE_SIDE_COUNT 2U
#define VMC_PHI0_VERTICAL_RAD     1.5707963267948966f
#define VMC_BODY_PITCH_DIRECTION  1.0f
#define VMC_BODY_PITCH_RATE_DIRECTION 1.0f
#define VMC_BODY_MOTION_X_LIMIT_DEFAULT 100.0f

/* Standard gravity, used to turn the HI12 acceleration (unit g) into m/s2 for
 * the body-motion estimator. */
#define VMC_GRAVITY_ACCEL 9.80665f
/* Gravity-compensation sign resolution.
 *
 * The protocol documents acc_b as unit G but not its sign, and the two
 * conventions in the wild (body-Z up vs body-Z down) need opposite
 * corrections.  Rather than assume one, the sign is measured: the raw
 * reading's component along the attitude-derived gravity direction averages
 * to +1 g or -1 g.  The gravity contribution keeps that sign even while the
 * vehicle moves, so a slow average converges regardless of motion. */
#define VMC_GRAVITY_SIGN_ALPHA  0.002f
#define VMC_GRAVITY_SIGN_MARGIN 0.5f

/* Body-motion Kalman filter defaults.  q_v/q_a are the process-noise terms
 * for velocity and acceleration; r_wheel/r_imu are the wheel-speed and IMU
 * acceleration measurement-noise terms.
 *
 * R is calibrated from the measured sensor noise, not chosen as a tuning
 * knob, so that Q and R stay independently meaningful.  Feeding a rescaled
 * Q/R pair would leave the filter gain unchanged and make the two knobs
 * degenerate.  Measured on the vehicle:
 *   wheel velocity measurement variance  1.12e-4 (m/s)^2
 *   IMU linear acceleration variance      8.34e-3 (m/s2)^2
 *
 * The accelerometer is gated to its own 200 Hz frame rate, so r_imu is the
 * weight per IMU sample rather than per 1 kHz control cycle.
 *
 * q_v sets the wheel-channel bandwidth; q_a sets the IMU weight.  A q_v sweep
 * measured on the vehicle gave:
 *   q_v       tau_wheel   NIS_v   high-frequency noise passed
 *   1.12e-7    29.0 ms     9.4    0.26x
 *   4.0e-7     16.8 ms     4.3    0.41x   <- most consistent
 *   1.0e-6     11.0 ms     4.8    0.62x
 *   4.0e-6      5.8 ms  1771      0.94x   <- control loop oscillated
 * The default is the most statistically consistent setting that stayed
 * stable.  Raising q_v further removes almost all smoothing and destabilises
 * the velocity feedback loop. */
#define VMC_BODY_MOTION_KF_Q_V_DEFAULT      4.0e-7f
#define VMC_BODY_MOTION_KF_Q_A_DEFAULT      2.0721e-5f
#define VMC_BODY_MOTION_KF_R_WHEEL_DEFAULT  1.1176e-4f
#define VMC_BODY_MOTION_KF_R_IMU_DEFAULT    8.3363e-3f
/* Diagnostic threshold for wheel-speed runaway relative to the previous
 * fused velocity, in m/s.  This mirrors USTB/Nanhang condition B. */
#define VMC_BODY_MOTION_KF_SLIP_VEL_DIFF_DEFAULT 1.2f

/* Slip gating, following the USTB/Nanhang sliding-observation scheme.  The
 * upstream formulation is `slip = (A OR B) AND C`; this implementation
 * triggers on C alone, with A and B wired as optional extra triggers:
 *
 *   A: the wheel-derived yaw rate disagrees with the IMU yaw rate
 *   B: a wheel's speed disagrees with the estimated body speed
 *   C: a wheel is accelerating harder than the body
 *
 * C is the detector and triggers on its own; A and B are optional extra
 * triggers.  Framing C behind them does not work: B compares a wheel against a
 * velocity fused from that same wheel, so it is self-limiting - the filter has
 * already followed the slipping wheel and the gap stays small.  Measured on
 * the vehicle, a landing transient drove C to 56 m/s2 while the B gap never
 * exceeded 0.76 m/s against its 1.2 m/s threshold, so the latch never engaged.
 *
 * C must use the RAW wheel speed.  Differentiating the leg-corrected body
 * speed instead turns commanded leg swing and leg-length servo motion into
 * apparent wheel acceleration, that is, into false slip.
 *
 * A clamped velocity measurement alone cannot reject a slipping wheel: the
 * filter follows whatever it is fed, so a slaved metric is self-limiting and
 * only ever reports a fraction of the true error.  While a latch is active the
 * wheel measurement is therefore replaced by the filter's own prediction (the
 * velocity innovation collapses) and r_wheel is widened so the estimate coasts
 * on the inertial prediction instead of chasing the spinning wheel.
 *
 * Each side latches independently.  Release needs the metric to fall to
 * RELEASE_RATIO of the trigger level AND the minimum hold to elapse.  The
 * optional timeout is only a safety net. */
/* Disabled by default.  The mechanism works and its inputs are verified, but
 * no test has yet demonstrated a benefit, and one showed a cost: during a
 * lift/drop the excursion is brief, so replacing the wheel measurement with
 * the prediction freezes the estimate at its last value while the wheel has
 * already returned to zero.  Replayed on that recording the hard gate gave
 * peak 1.54 / rms 0.351 against 1.49 / 0.280 ungated.  Gating suits a
 * sustained slide - a wheel spinning while the body stays put - where coasting
 * is the correct response; enable it for that case only, and re-validate
 * before making it the default.  Note also that the airborne detector cuts
 * wheel torque on the same evidence, so a low-friction surface can trip that
 * protection first. */
#define VMC_BODY_MOTION_KF_SLIP_ENABLE_DEFAULT    0U
/* Condition C threshold in m/s2.
 *
 * Condition C differentiates against the MEASURED control period, so this is
 * a true m/s2 figure.
 *
 * RE-DERIVED after the fore-aft axis fix, and the old value is void.  16 was
 * calibrated while dd_x - the body-acceleration term of condition C - was
 * still reading the LATERAL accelerometer channel, so the metric it was tuned
 * against was not the metric the firmware computes now.
 *
 * Two labelled runs on the corrected axis, 219 s of driving and 19
 * operator-marked slips, captured at 225 Hz on the binary VMCFAST stream with
 * zero dropped frames:
 *
 *   logs/slip_20261011_012244  132 s, 10 marked
 *   logs/slip_20261011_012936   87 s,  9 marked
 *
 * The separation is clean and does not depend on the analysis window, which
 * was checked because the operator's keypress lags the real peak by 0.4-2.4 s
 * here (a 1.2 s window truncates real peaks and manufactures false positives):
 *
 *   marked event peaks                     40.9, 43.2 ... 79.6
 *   highest unmarked excursion             36.99  (a 5 ms spike, see below)
 *   highest unmarked outside every window  24.90
 *
 * Sweep over thresholds, scoring one detection per marked event:
 *
 *   threshold   recall   extra detections
 *      25       19/19         1
 *      33       19/19         1     <- this value
 *      37       17/19         0
 *
 * 33 keeps every marked event.  37 buys the single extra detection back at the
 * cost of two real ones, which is the wrong trade.  Also verified by
 * leave-one-out (10/10) and by counting detections without using any event
 * window at all - at 25 and above the runs match the marked events one-to-one.
 *
 * The one extra detection is a 5 ms spike that peaks at 36.99, ABOVE the
 * weakest genuine event (35.76), so no threshold separates it - amplitude is
 * not the discriminating axis.  Its duration is: 5 ms against 13 ms for the
 * weakest marked event.  It is left in deliberately rather than pushed out
 * with a longer confirmation, because the available gap is only about 10-13 ms
 * and widening confirm_ms to reach it trades real recall on the short bursts
 * described below.
 *
 * Older runs recorded 9.0 and then 16.0.  Prefer changing this at runtime over
 * RTT (see tools/slip_capture.py --slip-threshold) and re-derive from a fresh
 * labelled run if the machine, surface or speed changes. */
#define VMC_BODY_MOTION_KF_SLIP_ACCEL_DEFAULT     33.0f
/* Condition A: wheel track in metres and yaw-rate threshold in rad/s.  The
 * track width has not been measured, so A stays disabled until yaw_track is
 * set to a real value. */
#define VMC_BODY_MOTION_KF_SLIP_YAW_TRACK_DEFAULT 0.0f
#define VMC_BODY_MOTION_KF_SLIP_YAW_DEFAULT       0.9f
/* Safety-net release for a latch whose metric sits inside the hysteresis band,
 * in ms. 0 never force-releases. */
#define VMC_BODY_MOTION_KF_SLIP_HOLD_MS_DEFAULT   200U
/* Trigger confirmation time in ms.
 *
 * Real slips are SHORT.  On the runs behind the threshold above, the longest
 * CONTIGUOUS stretch above 33 ms was 13-40 ms for the nineteen marked events,
 * and the metric typically fires twice per event (entry and recovery) about
 * 100-120 ms apart rather than staying high throughout.  A 60 ms confirmation
 * - an early default - therefore sat out entire real events.
 *
 * 5 ms is one to two control frames at 450 Hz: enough to reject a
 * single-sample transient, short enough to admit a 13 ms burst.  It is kept
 * even though the one extra detection in those runs (5 ms wide, see the
 * threshold comment) could be excluded by 12 ms, because the gap between 5 ms
 * and the weakest genuine burst is only about 10-13 ms and this parameter
 * trades real recall away fastest.
 *
 * Threshold and confirmation are a pair; do not move one without re-running
 * tools/slip_analyze.py --sweep. */
#define VMC_BODY_MOTION_KF_SLIP_CONFIRM_MS_DEFAULT 5U

/* MPC is available at runtime but dormant at boot.  Keep the verified stable2
 * LQR path bit-for-bit in command until shadow performance and output
 * alignment have been validated on this firmware. */
#define VMC_MPC_SHADOW_ENABLE_DEFAULT   0U
#define VMC_MPC_ENABLE_DEFAULT          0U
#define VMC_MPC_WHEEL_DELTA_MAX_DEFAULT 0.20f
#define VMC_MPC_LEG_DELTA_MAX_DEFAULT   2.00f
#define VMC_MPC_WHEEL_AUTHORITY_DEFAULT 0.0f
#define VMC_MPC_LEG_AUTHORITY_DEFAULT   0.0f
#define VMC_MPC_AUTHORITY_RAMP_RATE_DEFAULT 0.5f
/* Boot with the verified LQR actuator path enabled.  Bench timing can still
 * force every final command to zero through compute_only_enable at runtime. */
#define VMC_COMPUTE_ONLY_DEFAULT        0U
/* Slip LQR weight switching.
 *
 * The reference this comes from is the Nanhang sentry wheel-leg firmware
 * (开源参考/南航/SENTRY_LEG).  Its LQR is Fitting_K[4][10] - four outputs
 * (left wheel, right wheel, left joint, right joint) by ten states
 * (x, dx, yaw, dyaw, leg0 theta, leg0 dtheta, leg1 theta, leg1 dtheta,
 * body theta, body dtheta) - and on a slip it does:
 *
 *     Fitting_K[i][2] = Fitting_K[i][3] = 0;      // drop the YAW columns
 *     Fitting_K[i][4] *= 2;  Fitting_K[i][6] *= 2; // double the LEG ANGLE
 *                                                  // columns (both legs)
 *
 * for row i of the slipping side.  Note it is the leg ANGLE, not the rate,
 * and note the columns dropped are YAW - not wheel speed.  (An earlier note
 * in this workstream said "cut the wheel-speed terms"; that was wrong and is
 * corrected here.  Nanhang's yaw comes from the IMU gyro and the gimbal
 * encoder, not from wheel odometry.)
 *
 * THE YAW HALF DOES NOT TRANSFER.  This firmware's LQR has no yaw state at
 * all: its states are theta, dtheta, x, dx, phi, dphi, and heading is held by
 * a separate yaw_hold loop.  Nothing here to zero, so only the posture half is
 * implemented.  If the yaw_hold loop should also soften during a slip, that is
 * a separate decision - do not fold it in silently.
 *
 * The posture half maps cleanly.  theta is this firmware's leg angle
 * (left_theta / right_theta, relative to the body) and it feeds both outputs:
 * k[0]/k[1] are the wheel and virtual-leg gains on theta, k[2]/k[3] the same
 * on dtheta.  Scaling the ANGLE gains reproduces the reference; the RATE
 * scales default to 1.0 to match it exactly.
 *
 * CAVEAT worth knowing before tuning: the reference doubles only the position
 * term, which drops the loop's damping ratio by about sqrt(2).  If the machine
 * rings after a slip, scale the rate term too - that is why it is a separate
 * parameter rather than being hard-coded to 1.0.
 *
 * ON by default as of the first on/off comparison run, which showed the
 * intended effect with no limit being reached: over 21 vs 23 slip events the
 * leg-angle disturbance fell to 78% and the roll excursion to 70% of the
 * unswitched values, the joint torque command rose 49% (7.5 -> 11.2 of a
 * 108 limit) and nothing saturated.  Two runs is thin evidence, so it stays
 * switchable over RTT. */
#define VMC_SLIP_LQR_ENABLE_DEFAULT        1U
#define VMC_SLIP_LQR_THETA_SCALE_DEFAULT   2.0f
#define VMC_SLIP_LQR_DTHETA_SCALE_DEFAULT  1.0f

/* Slip current scaling - the second half of the reference response.
 *
 * The same function that does the weight switching also does:
 *
 *     if (flag->slip_flag[i]) joint_m[i]->A = joint_m[i]->A * 0.6f;
 *
 * on the FINAL motor current for the slipping side, after it has been limited
 * and after yaw has been folded in - so it scales the whole wheel output, not
 * just the balance term.  In this firmware the equivalent point is
 * vmc_balance_apply_wheel_output(), where left/right_current have just been
 * assembled from the balance command, the operator's manual current and the
 * yaw_hold / gimbal-follow current.
 *
 * READ THIS BEFORE TUNING.  This is the part of the reference response that
 * fights the machine rather than rebalancing it.  The wheel current IS the
 * balancing authority on a wheel-leg balancer: cutting it 40% gives up 40% of
 * the authority to arrest a pitch excursion, and it does so precisely while
 * the machine is slipping, i.e. while it is already disturbed.  The reference
 * machine does the same thing, so it is not a mistake - but it is the term
 * most likely to be the wrong trade on a different machine, and it is why the
 * scale is a parameter rather than a constant.
 *
 * Set it to 1.0 to keep the weight switching but drop the current reduction.
 * It is gated by slip_lqr_enable, which is the master switch for the whole
 * slip response. */
#define VMC_SLIP_WHEEL_CURRENT_SCALE_DEFAULT 0.6f

/* Raw joint positions captured with both legs vertical down at 385 mm.
 *
 * These are zero offsets, not gains.  joint_model_angle() applies
 *   model = REF_MODEL + DIR * (raw - REF_RAW)
 * so re-zeroing a leg means solving REF_RAW = raw - (target - REF_MODEL)/DIR
 * for the raw position the motor reports at the reference pose.  Leave DIR
 * alone unless the linkage itself was rebuilt: DIR sets the sign of motion,
 * not the zero.
 *
 * HISTORY - the right_ahead motor (id 2) used to re-establish its multi-turn
 * reference at power-on instead of holding an absolute zero.  Measured at the
 * same physical pose on three separate power-ups it read -2.5378, -1.8507 and
 * -2.5519 rad, stepping by 39.4 and 40.2 degrees - one motor-rotor revolution
 * through the ~9:1 reduction.  That is exactly why re-calibrating REF_RAW from
 * a single reading is a trap: it bakes in whichever turn happened to come up.
 *
 * That instability is NOT present on the current motor.  After the zero was
 * stored, all four joints read byte-identical positions across a power cycle
 * (right_ahead +2.387000 before and after, p95 spread 0.0000 rad on all
 * four), so the constants could be corrected safely.
 *
 * The values below were re-derived on 2026-10-05 from a pose that was
 * physically locked with both legs in the SAME configuration.  That
 * measurement found the legs disagreeing by 19 mm of computed length
 * (phi1 +3.64 deg, phi4 -5.31 deg) while the machine stood level to 0.86 deg
 * of roll.  A real 19 mm leg-length difference would have to appear as about
 * 7 deg of roll, since the leg-length controller cannot remove roll - so the
 * disagreement was calibration, not pose.  With these constants the two legs
 * agree to 0.0 mm in that pose.
 *
 * If a leg zero ever moves again, re-derive BOTH constants from a locked,
 * genuinely symmetric pose instead of editing one number by hand. */
#define VMC_REF_RAW_LEFT_BACK      2.51313968f
#define VMC_REF_RAW_LEFT_AHEAD    -3.20170302f
/* Values that are correct when the right_ahead motor comes up in its NOMINAL
 * turn (derived from a fixture-locked symmetric pose; see the history above).
 *
 * DO NOT patch these to chase a bad power-up. This pair has already been
 * "corrected" for an offset reference three times, and every correction was
 * invalid on the next power-up:
 *
 *   - The motor re-establishes its multi-turn reference at power-on and the
 *     result is NOT repeatable. On four observed power-ups of the current
 *     motor the right_ahead reading at one physical pose was -2.5378, -1.8507,
 *     -2.5519 (steps of one rotor turn) and, after a jump, one full turn away
 *     again.
 *   - A compensation derived from one of those states is a -36 to -39 deg
 *     error on any other, which the machine cannot balance. Measured directly:
 *     after a jump needed (+39.4 deg) and the next power-up needed (-36.5 deg)
 *     on the same joint - i.e. the reference had moved back.
 *
 * So when the legs disagree, the useful question is not "what constant fixes
 * it" but "where is the motor's reference this time". Measure it, and if it is
 * nominal these constants are right. The durable fix is on the motor side:
 * store the zero with the DM tool and confirm it survives several power cycles,
 * and check the joint supply for a droop under landing current (the failure
 * appeared at the instant of touchdown, when wheel and joint current peak
 * together). */
#define VMC_REF_RAW_RIGHT_AHEAD    -2.630482f
#define VMC_REF_RAW_RIGHT_BACK     0.495861f

/* VMC model angles at the same 385 mm reference pose. */
#define VMC_REF_MODEL_PHI1         2.20948918f
#define VMC_REF_MODEL_PHI4         0.93210348f

/*
 * Motor-to-model angle directions.
 * If the measured length increases while the physical leg retracts, flip the
 * corresponding leg direction pair instead of changing the reference raw angle.
 */
#define VMC_DIR_LEFT_BACK         -1.0f
#define VMC_DIR_LEFT_AHEAD        -1.0f
#define VMC_DIR_RIGHT_AHEAD        1.0f
#define VMC_DIR_RIGHT_BACK         1.0f

#define VMC_DM_STATUS_NOT_ENABLED  0x00U
#define VMC_DM_STATUS_CAN_TIMEOUT  0x0DU

#define VMC_DM_STATE_CONTROL       0U
#define VMC_DM_STATE_INIT          1U
#define VMC_DM_STATE_RECOVER       2U
#define VMC_DM_STATE_DISABLE       3U

#define VMC_GETUP_STATE_IDLE       0U
#define VMC_GETUP_STATE_TURN       1U
#define VMC_GETUP_STATE_SHRINK     2U
#define VMC_GETUP_STATE_READY      3U
#define VMC_GETUP_STATE_LEG_RECOVER 4U
#define VMC_GETUP_STATE_BRACE      5U
#define VMC_GETUP_STATE_UPRIGHT    6U
#define VMC_GETUP_STATE_BODY_ALIGN 7U
#define VMC_GETUP_STATE_BODY_FLIP  8U
#define VMC_GETUP_STATE_BODY_FAN   9U
#define VMC_GETUP_STATE_BODY_FAN_SYNC 10U
#define VMC_GETUP_STATE_FOLDED_RAISE 11U
#define VMC_GETUP_STATE_FOLDED_LOWER 12U

#define VMC_JUMP_STAGE_IDLE        0U
#define VMC_JUMP_STAGE_COMPRESS    1U
#define VMC_JUMP_STAGE_EXTEND      2U
#define VMC_JUMP_STAGE_RETRACT     3U
#define VMC_JUMP_STAGE_LAND        4U
#define VMC_JUMP_STAGE_FINISHED    5U
#define VMC_JUMP_STAGE_ABORT       6U

#define VMC_JUMP_ABORT_NONE        0U
#define VMC_JUMP_ABORT_REMOTE      1U
#define VMC_JUMP_ABORT_FEEDBACK    2U
#define VMC_JUMP_ABORT_GETUP       3U
#define VMC_JUMP_ABORT_ATTITUDE    4U
#define VMC_JUMP_ABORT_TIMEOUT     5U

#define VMC_STAIR_STAGE_IDLE       0U
#define VMC_STAIR_STAGE_LIFT       1U
#define VMC_STAIR_STAGE_RETRACT    2U
#define VMC_STAIR_STAGE_FINISHED   3U
#define VMC_STAIR_STAGE_ABORT      4U
#define VMC_STAIR_STAGE_SETTLE     5U
#define VMC_STAIR_STAGE_RECOVER    6U

#define VMC_STAIR_ABORT_NONE       0U
#define VMC_STAIR_ABORT_REMOTE     1U
#define VMC_STAIR_ABORT_FEEDBACK   2U
#define VMC_STAIR_ABORT_GETUP      3U
#define VMC_STAIR_ABORT_JUMP       4U
#define VMC_STAIR_ABORT_TIMEOUT    5U
#define VMC_STAIR_ABORT_AIRBORNE   6U
#define VMC_STAIR_ABORT_ATTITUDE   7U

#define VMC_GETUP_START_PITCH_RAD  0.80f
/* Every get-up entry requires the operator's channel-9 leg-length target to be
 * at or below this value before the sequence is allowed to start. */
#define VMC_GETUP_SAFE_LENGTH_MAX  0.220f
#define VMC_GETUP_TURN_LENGTH      0.300f
#define VMC_GETUP_STAND_LENGTH     VMC_LEG_TARGET_SHORT
/* Keep the posture loop deliberately slow and weak while the chassis is on
 * the ground.  Current + PHI0_LEAD is a velocity command: it stays inside the
 * get-up torque ceiling instead of asking the normal 108 N.m posture loop for
 * a step response every cycle. */
#define VMC_GETUP_PHI0_LEAD_RAD    0.060f
#define VMC_GETUP_FAN_PHI0_LEAD_RAD 0.250f
#define VMC_GETUP_PHI0_KP          70.0f
#define VMC_GETUP_PHI0_KD          4.0f
#define VMC_GETUP_FAN_PHI0_KP      70.0f
#define VMC_GETUP_FAN_PHI0_KD      2.0f
#define VMC_GETUP_PHI0_TORQUE_MAX  36.0f
#define VMC_GETUP_LENGTH_KP        300.0f
#define VMC_GETUP_LENGTH_KD        25.0f
#define VMC_GETUP_FORCE_MAX        180.0f
#define VMC_GETUP_SUPPORT_FORCE    30.0f
#define VMC_GETUP_FAN_SUPPORT_FORCE 20.0f
#define VMC_GETUP_LENGTH_RAMP_RATE 0.180f
#define VMC_GETUP_THETA_MIN_RAD    0.500f
#define VMC_GETUP_THETA_MAX_RAD    1.400f
#define VMC_GETUP_PITCH_READY_RAD  0.300f
#define VMC_GETUP_LENGTH_READY_TOL 0.030f
#define VMC_GETUP_LEG_BAD_NEG_MIN_RAD -3.140f
#define VMC_GETUP_LEG_BAD_NEG_MAX_RAD -0.790f
#define VMC_GETUP_LEG_BAD_POS_MIN_RAD  1.800f
#define VMC_GETUP_LEG_BAD_POS_MAX_RAD  3.140f
#define VMC_GETUP_LEG_RECOVER_TARGET_RAD 1.300f
#define VMC_GETUP_LEG_RECOVER_TOL_RAD    0.100f
/* Direction used for the theta bad-zone recovery path.  Flip this sign to
 * choose the other physical winding around the same safe theta target. */
#define VMC_GETUP_LEG_RECOVER_DIRECTION -1.0f
/* Shanhai-style pre-positioning for a fully inverted chassis.  The two body
 * pitch directions use different rear-link targets and opposite flip
 * directions instead of one generic TURN path. */
#define VMC_GETUP_BODY_START_PITCH_RAD   1.5707963f
#define VMC_GETUP_BODY_POS_TARGET_RAD   -2.3561945f
#define VMC_GETUP_BODY_NEG_TARGET_RAD   -1.5707963f
#define VMC_GETUP_BODY_TARGET_TOL_RAD    0.1745329f
#define VMC_GETUP_BODY_INNER_ANGLE_RAD  -0.3490659f
#define VMC_GETUP_BODY_EXIT_PITCH_RAD    0.1745329f
#define VMC_GETUP_BODY_ALIGN_CONFIRM_MS  30U
#define VMC_GETUP_BODY_FLIP_CONFIRM_MS   50U
#define VMC_GETUP_LEG_RECOVER_CONFIRM_MS  50U
/* Shanhai-style inverted-body windmill.  Body roll near +/-180 degrees is not
 * covered by the pitch-only BODY_ALIGN path.  Drive both phi0 joints with a
 * real velocity command, keep the legs long, and wait until the body returns
 * near level before retracting. */
#define VMC_GETUP_BODY_FAN_START_ROLL_RAD  1.900f
#define VMC_GETUP_BODY_FAN_SYNC_LENGTH     0.385f
#define VMC_GETUP_BODY_FAN_SYNC_LENGTH_MIN 0.340f
#define VMC_GETUP_BODY_FAN_SYNC_LENGTH_TOL_M 0.025f
#define VMC_GETUP_BODY_FAN_SYNC_LENGTH_RATE 0.100f
#define VMC_GETUP_BODY_FAN_SYNC_THETA_TOL_RAD 0.120f
#define VMC_GETUP_BODY_FAN_SYNC_EARLY_PHASE_TOL_RAD 0.120f
#define VMC_GETUP_BODY_FAN_SYNC_RATE_RADPS 2.000f
#define VMC_GETUP_BODY_FAN_SYNC_PHASE_KP 2.000f
#define VMC_GETUP_BODY_FAN_SYNC_TORQUE_FF 20.0f
#define VMC_GETUP_BODY_FAN_SYNC_BREAKAWAY_FULL_SPEED_RADPS 0.150f
#define VMC_GETUP_BODY_FAN_SYNC_BREAKAWAY_EXIT_SPEED_RADPS 0.250f
#define VMC_GETUP_BODY_FAN_SYNC_TORQUE_FF_RISE_RATE 600.0f
#define VMC_GETUP_BODY_FAN_SYNC_TORQUE_FF_FALL_RATE 2400.0f
#define VMC_GETUP_BODY_FAN_SYNC_TORQUE_MAX 36.0f
#define VMC_GETUP_BODY_FAN_SYNC_SUPPORT_FORCE 0.0f
#define VMC_GETUP_BODY_FAN_SYNC_CONFIRM_MS 100U
#define VMC_GETUP_BODY_FAN_SPEED_RADPS     3.000f
#define VMC_GETUP_BODY_FAN_SPEED_KP        10.0f
#define VMC_GETUP_BODY_FAN_SPEED_DEADBAND_RADPS 0.050f
#define VMC_GETUP_BODY_FAN_SPEED_TORQUE_MAX 15.0f
#define VMC_GETUP_BODY_FAN_TORQUE_MAX      19.0f
#define VMC_GETUP_BODY_FAN_PHI0_KP         0.0f
#define VMC_GETUP_BODY_FAN_PHI0_KD         0.0f
#define VMC_GETUP_BODY_FAN_ENTRY_THETA_RAD 0.0f
#define VMC_GETUP_BODY_FAN_STALL_RATE_RADPS 0.100f
#define VMC_GETUP_BODY_FAN_STALL_ROLL_RATE_RADPS 0.050f
#define VMC_GETUP_BODY_FAN_STALL_STARTUP_MS 250U
#define VMC_GETUP_BODY_FAN_STALL_CONFIRM_MS 300U
#define VMC_GETUP_BODY_FAN_GRAVITY_GAIN    10.0f
#define VMC_GETUP_BODY_FAN_GRAVITY_ANGLE_RAD 0.3490659f
#define VMC_GETUP_BODY_FAN_TORQUE_FF_RISE_RATE 800.0f
#define VMC_GETUP_BODY_FAN_TORQUE_FF_FALL_RATE 2400.0f
#define VMC_GETUP_BODY_FAN_EXIT_RAD        0.1745329f
#define VMC_GETUP_BODY_FAN_CONFIRM_MS      50U
#define VMC_GETUP_BODY_FAN_TIMEOUT_MS      30000U
/* If the body did not invert and both virtual legs are already on this side,
 * a folded phi4 is just the compact linkage pose.  Shrink directly instead of
 * driving another full turn through LEG_RECOVER. */
#define VMC_GETUP_SHRINK_READY_THETA_MIN_RAD 0.650f
#define VMC_GETUP_SHRINK_READY_THETA_MAX_RAD 1.550f
/* The sequence must not hand the chassis back to the normal balance loop from
 * the broad TURN window.  UPRIGHT ramps theta to zero first, then waits for a
 * quiet, level pose before READY releases wheel and LQR-joint output. */
#define VMC_GETUP_UPRIGHT_PHI0_RATE_RADPS 1.200f
#define VMC_GETUP_UPRIGHT_THETA_TOL_RAD   0.150f
/* The supported chassis settles around 0.30 rad pitch when the legs are at
 * the short stand length.  Requiring 0.18 kept a mechanically completed
 * recovery in UPRIGHT until timeout. */
#define VMC_GETUP_UPRIGHT_PITCH_RAD       0.350f
#define VMC_GETUP_UPRIGHT_DTHETA_MAX_RADPS 0.350f
#define VMC_GETUP_UPRIGHT_DPITCH_MAX_RADPS 0.350f
#define VMC_GETUP_BRACE_MS                250U
#define VMC_GETUP_POSE_CONFIRM_MS         100U
#define VMC_GETUP_LENGTH_CONFIRM_MS       100U
#define VMC_GETUP_SHRINK_THETA_TARGET_RAD 0.800f
#define VMC_GETUP_SHRINK_THETA_RATE_RADPS 0.500f
#define VMC_GETUP_READY_CONFIRM_MS        1U
#define VMC_GETUP_TIMEOUT_MS              10000U
/* Shallow folded fall (pitch below the full-fall threshold).  This path does
 * not reuse TURN/SHRINK.  It first extends to 0.20 m while an LQR theta row
 * drives the virtual legs directly to vertical (theta = 0), then retracts to
 * the 0.18 m hardware floor and reuses UPRIGHT/READY.  Shanhai asks for
 * 0.15 m after alignment, but this chassis must not go below 0.18 m. */
#define VMC_GETUP_PARTIAL_PITCH_MIN_RAD       0.080f
#define VMC_GETUP_PARTIAL_ROLL_MAX_RAD        0.350f
#define VMC_GETUP_PARTIAL_ENTRY_LENGTH_MAX    0.230f
#define VMC_GETUP_PARTIAL_ENTRY_THETA_MIN_RAD 0.250f
#define VMC_GETUP_PARTIAL_ENTRY_THETA_MAX_RAD 1.550f
#define VMC_GETUP_PARTIAL_LOWER_LENGTH         0.180f
#define VMC_GETUP_PARTIAL_PHI0_KP              50.0f
#define VMC_GETUP_PARTIAL_CONFIRM_MS           1U
#define VMC_GETUP_PARTIAL_TIMEOUT_MS           5000U

/* Phi0 range protection is intentionally disabled.  These diagnostic state
 * values remain for debugger/telemetry compatibility only. */
#define VMC_PHI0_PROTECT_NONE      0U
#define VMC_PHI0_PROTECT_LEFT      1U
#define VMC_PHI0_PROTECT_RIGHT     2U
#define VMC_PHI0_PROTECT_BOTH      3U

#define VMC_FEEDBACK_TIMEOUT_NONE        0x00U
#define VMC_FEEDBACK_TIMEOUT_RIGHT_BACK  0x01U
#define VMC_FEEDBACK_TIMEOUT_RIGHT_AHEAD 0x02U
#define VMC_FEEDBACK_TIMEOUT_LEFT_AHEAD  0x04U
#define VMC_FEEDBACK_TIMEOUT_LEFT_BACK   0x08U
#define VMC_FEEDBACK_TIMEOUT_WHEEL_LEFT  0x10U
#define VMC_FEEDBACK_TIMEOUT_WHEEL_RIGHT 0x20U

#define VMC_WHEEL_CURRENT_MAX_DEFAULT 16000
#define VMC_BALANCE_WHEEL_CURRENT_LIMIT_DEFAULT 15000
/* The LQR wheel input and its runtime limit are 3508 motor-shaft torque (N.m).
 * The model already reflects the drivetrain; tuning must not apply the
 * reduction ratio and efficiency a second time. */
#define VMC_BALANCE_WHEEL_GEAR_RATIO_DEFAULT 15.7647f
#define VMC_BALANCE_WHEEL_GEAR_EFFICIENCY_DEFAULT 0.7f
#define VMC_BALANCE_WHEEL_TORQUE_LIMIT_DEFAULT 5.5f
#define VMC_BALANCE_WHEEL_TORQUE_CONSTANT 0.3f
#define VMC_BALANCE_C620_MAX_CURRENT_A 20.0f
#define VMC_BALANCE_C620_RAW_MAX 16384.0f
#define VMC_BALANCE_WHEEL_TORQUE_TO_CURRENT 2730.6666f /* motor N.m -> C620 raw */
#define VMC_BALANCE_WHEEL_CURRENT_GAIN_DEFAULT 1.0f
#define VMC_BALANCE_TP_LIMIT_DEFAULT 108.0f /* = VMC_LEG_TP_MAX */
#define VMC_BALANCE_TP_PHI_GAIN_DEFAULT 1.0f
#define VMC_BALANCE_TP_X_GAIN_DEFAULT 1.0f
#define VMC_BALANCE_TP_OUTPUT_ENABLE_DEFAULT 1U
/* Leg synchronisation, i.e. the anti-splay loop.
 *
 * The left and right virtual legs each run their own controller against the
 * same theta target, so nothing in the six-state model ties
 * (left_theta - right_theta) to zero.  Body roll appears as a COMMON-MODE
 * theta error; a pure left/right difference is unobserved by every feedback
 * term and is only weakly corrected by the per-side LQR theta rows
 * (K[1] = 5.49 at 0.360 m).  With the wheels unloaded, or at long leg lengths
 * where the gravity de-stabilising term is largest, the two legs therefore
 * drift apart and the machine splays and falls sideways.
 *
 * This adds a pure differential term to the two joint-torque commands:
 *
 *     e        = left_theta_error - right_theta_error
 *     edot     = left_d_theta     - right_d_theta
 *     Tp_sync  = clamp(kp * e + kd * edot, +/- leg_sync_torque_limit)
 *     left_leg  -= Tp_sync
 *     right_leg += Tp_sync
 *
 * The common mode (the average) is untouched, so balance and the theta
 * equilibrium are unaffected.  The term is added in the virtual-leg torque
 * (Tp) domain, so the limit shares the +/-2*VMC_DM_J8009_TMAX range with
 * tp_limit.
 *
 * Only a channel-9 setting at/above the old long-leg target applies this term.
 * At mid and short knob positions the loop is forced to zero so the established
 * standing tune there is unchanged.
 *
 * The applied term is independent of tp_output_enable: it is summed into the
 * same joint-torque command as both the LQR Tp and the phi0 PD, so it keeps
 * working when the balance loop is switched back to the phi0 gains.
 *
 * Gains: with e = left-right the differential inertia is about half one
 * leg's inertia about the hip, J_e ~ 0.042 kg.m2, so kp sets how stiffly the
 * legs are tied and kd damps that differential mode.  A pure-P tie (kd = 0)
 * will ring at a stiff kp, so add damping before raising kp far.  The limit
 * bounds the worst-case leg-vs-leg torque so a bad theta estimate cannot
 * command the full joint range. */
#define VMC_LEG_SYNC_ENABLE_DEFAULT 1U
#define VMC_LEG_SYNC_KP_DEFAULT 60.0f
#define VMC_LEG_SYNC_KD_DEFAULT 0.0f
#define VMC_LEG_SYNC_TORQUE_LIMIT_DEFAULT 12.0f
#define VMC_BALANCE_WHEEL_OUTPUT_USER_ENABLE_DEFAULT 1U
/* Public tuning names:
 * theta_target controls the wheel-leg theta equilibrium.
 * pitch_target controls the body pitch/phi equilibrium.
 *
 * The struct still uses the older pitch_offset/phi_offset field names for
 * compatibility with existing watch expressions and RTT payloads.
 */
#define VMC_BALANCE_THETA_TARGET_DEFAULT 0.05f
#define VMC_BALANCE_PITCH_TARGET_DEFAULT 0.0f
#define VMC_BALANCE_PITCH_OFFSET_DEFAULT VMC_BALANCE_THETA_TARGET_DEFAULT
#define VMC_BALANCE_PHI_OFFSET_DEFAULT VMC_BALANCE_PITCH_TARGET_DEFAULT
#define VMC_BALANCE_DX_FEEDBACK_ENABLE_DEFAULT 1U
#define VMC_BALANCE_THETA_DEADBAND_DEFAULT 0.0f // 死区
#define VMC_BALANCE_PHI_DEADBAND_DEFAULT 0.0f // 死区 0.010f
#define VMC_BALANCE_X_FEEDBACK_ENABLE_DEFAULT 1U
#define VMC_BALANCE_X_FEEDBACK_GAIN_DEFAULT -1.0f
#define VMC_BALANCE_DX_FEEDBACK_GAIN_DEFAULT -1.0f
#define VMC_BALANCE_X_FEEDBACK_LIMIT_DEFAULT 1.0f
#define VMC_BALANCE_DX_FEEDBACK_LIMIT_DEFAULT 4.0f
/* Standing hold: after this long with the stick released the velocity
 * feedback gain ramps to the static value. */
#define VMC_BALANCE_DX_GAIN_STATIC_DEFAULT   -2.0f
#define VMC_BALANCE_DX_STATIC_DELAY_MS       1000.0f
/* Upper knee of the standing gain schedule: at or above this speed the gain is
 * the normal one, at a quarter of it the strong gain is fully engaged, and
 * between the two the gain is interpolated so the residual ring after a
 * disturbance still sees extra damping. */
#define VMC_BALANCE_DX_STATIC_SPEED_LIMIT    0.20f
#define VMC_BALANCE_DX_STATIC_SPEED_LOW      0.10f
/* The strong gain must never be live while the legs are still extending or the
 * body is settling after enable: the leg-length ramp alone runs at 0.225 m/s
 * and takes about a second from folded, and the enable edge otherwise carried
 * the previous -2.0 straight into it. */
#define VMC_BALANCE_DX_STATIC_SETTLE_MS      3000U
/* Gain slew, per second, so the hand-off cannot step the leg target. */
#define VMC_BALANCE_DX_GAIN_RAMP_RATE        4.0f
/* Shedding the strong gain has to be quicker than adding it: a disturbance
 * must fall back to the normal gain within a few control cycles. */
#define VMC_BALANCE_DX_GAIN_RELEASE_RATE     20.0f
#define VMC_BALANCE_SPEED_CONTROL_ENABLE_DEFAULT 1U
#define VMC_BALANCE_SPEED_TARGET_MAX_DEFAULT 4.0f
/* Scaled by 0.45 to preserve the tuned behaviour; see VMC_CONTROL_DT. */
#define VMC_BALANCE_SPEED_RAMP_RATE_DEFAULT 2.25f
#define VMC_BALANCE_SPIN_SPEED_RAMP_RATE_DEFAULT 4.5f
#define VMC_BALANCE_SPEED_DEADBAND_DEFAULT 0.02f
#define VMC_BALANCE_STOP_SPEED_THRESHOLD_DEFAULT 0.08f
/* Braking-only Pitch feedforward gain (rad per m/s^2). */
#define VMC_BALANCE_SPEED_PITCH_GAIN_DEFAULT 0.0f
#define VMC_BALANCE_SPEED_PITCH_LIMIT_DEFAULT 0.18f
/* Scaled by 0.45 to preserve the tuned behaviour; see VMC_CONTROL_DT. */
#define VMC_BALANCE_SPEED_PITCH_RAMP_RATE_DEFAULT 0.3f
/* Anchor (x hold) capture window.
 *
 * This is a drift fix, not a tuning knob.  At 0.05 the position loop asked for
 * less than the wheels' Coulomb friction could deliver, so the machine could
 * not pull itself back: it crept in one direction indefinitely (measured
 * metres).  At 0.25 the loop commands several times the friction threshold and
 * the same test holds to a few centimetres.  Raising it further buys nothing -
 * the residual limit cycle is closed-loop dynamics, not authority. */
#define VMC_BALANCE_X_HOLD_MAX_STRETCH_DEFAULT 0.25f
#define VMC_BALANCE_DTHETA_LPF_ALPHA_DEFAULT 0.1f
#define VMC_BALANCE_LQR_DTHETA_GAIN_DEFAULT 1.0f
#define VMC_BALANCE_LQR_DTHETA_DEADBAND_DEFAULT 0.0f
#define VMC_BALANCE_LQR_DTHETA_LIMIT_DEFAULT 10.0f
#define VMC_BALANCE_DPITCH_LPF_ALPHA_DEFAULT 1.0f
#define VMC_BALANCE_DISTURBANCE_OBSERVER_ENABLE_DEFAULT 1U
/* Re-tuned on the 60 mm / 0.4 kg wheels (2026-10-01 RTT sweep).  The gains are
 * a leaky velocity-error integrator, so the pair sets both the steady-state
 * authority (gain/leak) and the response time (1/leak); these cannot be tuned
 * independently.  Measured on the vehicle:
 *   0.27/0.045  stable but the residual drift was never cancelled
 *   1.00/0.200  time constant 5 s - the machine oscillated and had to be cut
 *   0.28/0.060  gain/leak 4.7 - slow 8..10 s limit cycle grew over 100 s
 *   0.20/0.060  stable over a 180 s capture; this is the shipped pair
 * Do not raise gain without re-running a long standstill capture. */
#define VMC_BALANCE_DISTURBANCE_OBSERVER_GAIN_DEFAULT 0.2f
#define VMC_BALANCE_DISTURBANCE_OBSERVER_LEAK_DEFAULT 0.06f
#define VMC_BALANCE_DISTURBANCE_OBSERVER_LIMIT_DEFAULT 0.1f
#define VMC_BALANCE_DISTURBANCE_OBSERVER_SPEED_LIMIT_DEFAULT 0.6f
/* Divides by 0.45 instead of multiplying: it is a time, so restoring the old
 * wait in real seconds means a LARGER number.  Scaled for the same reason as
 * the gain above; see VMC_CONTROL_DT. */
#define VMC_BALANCE_DISTURBANCE_OBSERVER_DELAY_DEFAULT 1.11f
#define VMC_BALANCE_DISTURBANCE_OBSERVER_PITCH_GAIN_DEFAULT 0.0f
#define VMC_BALANCE_DISTURBANCE_OBSERVER_THETA_GAIN_DEFAULT -1.0f
#define VMC_RC_CH_DEADBAND 20
#define VMC_RC_WHEEL_FWD_CURRENT_MAX 2000
#define VMC_RC_WHEEL_TURN_CURRENT_MAX 1000
/* Scaled by 0.45 to preserve the tuned behaviour; see VMC_CONTROL_DT. */
#define VMC_RC_YAW_TARGET_RATE_MAX_DEFAULT 1.8f
#define VMC_RC_YAW_SPIN_RATE_MIN_DEFAULT 0.0f
#define VMC_RC_YAW_SPIN_RATE_MAX_DEFAULT 9.0f
/* Manual pre-positioning is entered only when CH7 is already on before CH4
 * enables the controller. The right vertical stick moves both virtual legs at
 * this rate. CH9 offsets both captured lengths relative to its entry position,
 * preserving the initial left/right length difference within the safe range. */
#define VMC_MANUAL_PREPOSE_PHI0_RATE_RADPS 1.0f
#define VMC_MANUAL_PREPOSE_PHI0_MIN_RAD    -6.2831853f
#define VMC_MANUAL_PREPOSE_PHI0_MAX_RAD    6.2831853f
#define VMC_MANUAL_PREPOSE_LENGTH_MIN      0.180f
#define VMC_MANUAL_PREPOSE_LENGTH_MAX      0.380f
#define VMC_SPIN_PROJECTION_POWER_DEFAULT 5.0f
#define VMC_SPIN_LINE_PATH_OFFSET_DEG_DEFAULT -75.0f
#define VMC_YAW_HOLD_ENABLE_DEFAULT 1U
#define VMC_YAW_HOLD_KP_DEFAULT 10000.0f
#define VMC_YAW_HOLD_KD_DEFAULT 1000.0f
#define VMC_YAW_HOLD_DEADBAND_DEFAULT 0.01f
#define VMC_YAW_HOLD_CURRENT_LIMIT_DEFAULT 12000
#define VMC_YAW_HOLD_DIRECTION 1.0f
#define VMC_GIMBAL_FOLLOW_ENABLE_DEFAULT 0U
#define VMC_GIMBAL_FOLLOW_FRONT_ECD_DEFAULT 0U
#define VMC_GIMBAL_FOLLOW_FRONT_VALID_DEFAULT 0U
#define VMC_GIMBAL_FOLLOW_KP_DEFAULT 8.0f
#define VMC_GIMBAL_FOLLOW_KD_DEFAULT 4.0f
#define VMC_GIMBAL_FOLLOW_DEADBAND_DEFAULT 15.0f
#define VMC_GIMBAL_FOLLOW_CURRENT_LIMIT_DEFAULT 4000
#define VMC_GIMBAL_FOLLOW_DIRECTION_DEFAULT 1.0f

typedef struct {
  int16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
  float torque;
  float speed_rads;
  float angle;
  int32_t round;
  float last_angle;
  uint32_t last_update_ms;
} vmc_wheel_motor_t;

typedef struct {
  uint8_t id;
  uint8_t status;
  uint8_t t_mos;
  uint8_t t_rotor;
  float position;
  float speed;
  float torque;
  uint32_t last_update_ms;
} vmc_joint_motor_t;

typedef struct {
  double phi1;
  double phi4;
  double d_phi1;
  double d_phi4;
  double length;
  double phi0;
  double d_length;
  double d_phi0;
  double dd_length;
  double dd_theta;
  double previous_d_length;
  double previous_d_theta;
  uint8_t acceleration_valid;
  double length_error;
  double length_p_output;
  double length_d_output;
  double length_pid_output;
  double phi0_error;
  double phi0_p_output;
  double phi0_d_output;
  double phi0_pid_output;
  double support_force_output;
  double force_command;
  double torque_phi0_command;
  double joint_torque_ahead;
  double joint_torque_back;
} vmc_leg_state_t;

typedef struct {
  uint8_t enabled;
  uint8_t compute_only;
  float target_length;
  uint8_t jump_enable;
  uint8_t jump_moving_enable;
  uint8_t jump_stage;
  uint8_t jump_armed;
  uint8_t jump_abort_reason;
  uint32_t jump_stage_start_ms;
  uint32_t jump_start_ms;
  uint32_t jump_last_finish_ms;
  uint32_t jump_stage_ms;
  uint32_t jump_land_contact_ms;
  uint32_t jump_trigger_count;
  uint32_t jump_abort_count;
  float jump_target_length;
  float jump_compress_length;
  float jump_extend_length;
  float jump_retract_length;
  float jump_land_length;
  float jump_kp_length;
  float jump_kd_length;
  float jump_max_force;
  float jump_support_force;
  uint32_t jump_compress_timeout_ms;
  uint32_t jump_extend_timeout_ms;
  uint32_t jump_retract_timeout_ms;
  uint32_t jump_land_timeout_ms;
  uint32_t jump_cooldown_ms;
  uint32_t jump_total_timeout_ms;
  float jump_attitude_limit;
  float jump_land_torque_threshold;
  uint32_t jump_land_confirm_ms;
  /* Automatic jump gate, runtime tunable over RTT (see vmc_jump_auto_ready).
   * The macros above are only the power-on defaults. */
  float jump_auto_speed_min;    /* m/s   */
  float jump_auto_dist_min_cm;  /* cm    */
  float jump_auto_dist_max_cm;  /* cm    */
  uint32_t jump_auto_confirm_ms;
  /* Automatic stair climb.  See VMC_STAIR_ENABLE_DEFAULT for the one-shot
   * arm, loaded two-leg change trigger, and abort conditions. */
  uint8_t stair_enable;
  uint8_t stair_stage;
  uint8_t stair_abort_reason;
  /* Latched after a trigger: keep the legs at stair_retract_length while the
   * physical stair switch remains enabled.  Returning that switch to disabled
   * releases the lock and restores the current channel-9 knob target. */
  uint8_t stair_length_override_active;
  /* Legacy impact-detector telemetry.  Kept at zero for wire compatibility. */
  uint8_t stair_fast_detected;
  uint8_t stair_impact_detected;
  uint8_t stair_ready_detected;
  uint8_t stair_preimpact_armed;
  uint8_t stair_left_impact;
  uint8_t stair_right_impact;
  uint32_t stair_stage_start_ms;
  uint32_t stair_start_ms;
  uint32_t stair_last_finish_ms;
  uint32_t stair_stage_ms;
  uint32_t stair_trigger_count;
  uint32_t stair_abort_count;
  uint32_t stair_ready_ms;
  uint32_t stair_speed_ms;
  uint32_t stair_preimpact_ms;
  uint32_t stair_impact_ms;
  uint32_t stair_ready_confirm_ms;
  uint32_t stair_speed_confirm_ms;
  uint32_t stair_preimpact_confirm_ms;
  uint32_t stair_impact_confirm_ms;
  uint32_t stair_retract_timeout_ms;
  uint32_t stair_pose_confirm_ms;
  uint32_t stair_pose_confirm_elapsed_ms;
  uint32_t stair_hold_ms;
  uint32_t stair_recover_ms;
  uint32_t stair_timeout_ms;
  uint32_t stair_cooldown_ms;
  float stair_lift_length;
  float stair_retract_length;
  float stair_lift_theta;
  float stair_retract_theta;
  float stair_pose_length_tol;
  float stair_pose_theta_tol;
  float stair_theta_ramped;
  float stair_phi0_target;
  float stair_phi0_ramped;
  float stair_lift_phi0_ramp_rate;
  float stair_phi0_ramp_rate;
  float stair_phi0_kp;
  float stair_phi0_kd;
  float stair_speed_threshold;
  float stair_command_speed_threshold;
  float stair_ready_length;
  float stair_preimpact_diff_max;
  float stair_impact_diff_threshold;
  float stair_impact_torque_threshold;
  float stair_impact_theta_threshold;
  float stair_speed_abs;
  float stair_speed_diff;
  float stair_accel;
  float stair_kp_length;
  float stair_kd_length;
  float stair_length_ramp_rate;
  float stair_max_force;
  float stair_lift_support_force;
  float stair_retract_support_force;
  float stair_support_force;
  float ramped_target_length;
  float length_ramp_rate;
  float kp_length;
  float kd_length;
  float target_phi0;
  float kp_phi0;
  float kd_phi0;
  float max_phi0_torque;
  float max_force;
  float support_force;
  uint8_t roll_comp_enable;
  float roll_target;
  float roll_remote_target_max;
  float roll_remote_deadband;
  float roll_comp_gain;
  float roll_comp_limit;
  float roll_comp_ramp_rate;
  float roll_comp_offset;
  float left_target_length;
  float right_target_length;
  uint8_t getup_state;
  int8_t getup_turn_direction;
  int8_t getup_body_flip_direction;
  int8_t getup_left_direction;
  int8_t getup_right_direction;
  uint8_t getup_left_done;
  uint8_t getup_right_done;
  uint8_t getup_leg_align_complete;
  uint8_t getup_partial_armed;
  uint8_t getup_partial_active;
  uint8_t manual_prepose_armed;
  uint8_t manual_prepose_active;
  uint8_t manual_prepose_exit_length_check_active;
  float manual_prepose_phi0_left;
  float manual_prepose_phi0_right;
  float manual_prepose_knob_position;
  float manual_prepose_base_length_left;
  float manual_prepose_base_length_right;
  float manual_prepose_length_left;
  float manual_prepose_length_right;
  uint32_t getup_left_confirm_ms;
  uint32_t getup_right_confirm_ms;
  float getup_left_hold_length;
  float getup_right_hold_length;
  float getup_left_hold_theta;
  float getup_right_hold_theta;
  float getup_body_fan_sync_theta_left;
  float getup_body_fan_sync_theta_right;
  float getup_body_fan_sync_phi0_target;
  uint8_t getup_body_fan_sync_complete;
  uint8_t getup_body_fan_sync_fold_side;
  uint8_t getup_body_fan_sync_prelift_active;
  uint8_t getup_body_fan_sync_targets_ready;
  uint8_t getup_body_fan_sync_initial_phase_close;
  uint8_t getup_body_fan_sync_leader_side;
  uint32_t getup_body_fan_sync_prelift_confirm_ms;
  uint8_t getup_body_fan_escape_active;
  uint8_t getup_body_fan_retry_used;
  uint32_t getup_body_fan_stall_ms;
  uint8_t getup_lqr_release_pending;
  float getup_shrink_target_theta_left;
  float getup_shrink_target_theta_right;
  float getup_target_phi0_left;
  float getup_target_phi0_right;
  float getup_upright_phi0_left;
  float getup_upright_phi0_right;
  float getup_phi0_feedforward_left;
  float getup_phi0_feedforward_right;
  uint32_t getup_state_ms;
  uint32_t getup_confirm_ms;
  uint8_t ramp_active;
} vmc_control_t;

typedef struct {
  uint8_t enabled;
  int16_t left_current;
  int16_t right_current;
  int16_t max_current;
  int16_t manual_left_current;
  int16_t manual_right_current;
  int16_t manual_forward_current;
  int16_t manual_turn_current;
  int16_t yaw_hold_current;
  int16_t gimbal_follow_current;
  int16_t left_current_output;
  int16_t right_current_output;
} vmc_wheel_control_t;

typedef struct {
  uint8_t enabled;
  uint8_t wheel_output_enable;
  uint8_t wheel_output_user_enable;
  uint8_t tp_output_enable;
  uint8_t dx_feedback_enable;
  uint8_t x_feedback_enable;
  uint8_t safety_allows_output;
  uint8_t yaw_hold_enable;
  uint8_t yaw_hold_active;
  uint8_t gimbal_follow_enable;
  uint8_t gimbal_follow_active;
  uint8_t gimbal_follow_front_valid;
  uint8_t roll_remote_active;
  float pitch_offset;
  float phi_offset;
  float roll_remote_input;
  float theta_deadband;
  float phi_deadband;
  float x_offset;
  float x_limit;
  float x_feedback_gain;
  float dx_feedback_gain;
  float x_feedback_limit;
  float dx_feedback_limit;
  uint8_t speed_control_enable;
  float speed_target;
  float speed_target_ramped;
  float speed_target_max;
  float speed_ramp_rate;
  float spin_speed_ramp_rate;
  float speed_deadband;
  float stop_speed_threshold;
  float speed_pitch_gain;
  float speed_pitch_limit;
  float speed_pitch_ramp_rate;
  float speed_pitch_offset;
  uint8_t speed_stop_hold_active;
  float x_hold_max_stretch;
  float dtheta_lpf_alpha;
  float lqr_dtheta_gain;
  float lqr_dtheta_deadband;
  float lqr_dtheta_limit;
  float dpitch_lpf_alpha;
  uint8_t disturbance_observer_enable;
  float disturbance_observer_gain;
  float disturbance_observer_leak;
  float disturbance_observer_limit;
  float disturbance_observer_output;
  float disturbance_observer_residual;
  float disturbance_observer_speed_limit;
  float disturbance_observer_delay;
  float disturbance_observer_pitch_gain;
  float disturbance_observer_theta_gain;
  float disturbance_observer_neutral_time;
  uint8_t disturbance_observer_active;
  float yaw_target;
  float yaw_hold_kp;
  float yaw_hold_kd;
  float yaw_hold_deadband;
  float yaw_target_rate_max;
  float yaw_spin_rate;
  float yaw_spin_rate_max;
  float spin_line_origin_yaw;
  float spin_line_yaw;
  float spin_line_path_offset_deg;
  float spin_line_input_forward;
  float spin_line_input_lateral;
  float spin_line_input_magnitude;
  float spin_line_phase_error;
  float spin_line_phase_lead_rad;
  float spin_line_projection_power;
  float spin_line_gate;
  uint8_t spin_mode_enabled;
  uint8_t spin_line_heading_valid;
  int16_t yaw_hold_current_limit;
  uint16_t gimbal_follow_front_ecd;
  float gimbal_follow_kp;
  float gimbal_follow_kd;
  float gimbal_follow_deadband;
  float gimbal_follow_direction;
  int16_t gimbal_follow_current_limit;
  int16_t turn_current_max;
  int16_t wheel_current_limit;
  /* Legacy member names are kept for RTT/LiveWatch compatibility.  Their
   * units are 3508 motor-shaft torque (N.m), never wheel/axle torque. */
  float wheel_torque_limit;
  float wheel_torque_to_current;
  float wheel_current_gain;
  uint8_t airborne_detect_enable;
  /* Compatibility names for existing RTT clients.  The operational threshold
   * is airborne_joint_torque_threshold, now Shanhai's FN cutoff in newtons. */
  float airborne_wheel_speed_threshold;
  float airborne_joint_torque_threshold;
  uint32_t airborne_confirm_ms;
  uint32_t airborne_release_ms;
  float airborne_pitch_dev_max;
  float airborne_pitch_rate_max;
  uint8_t airborne_side_enable;
  uint32_t airborne_side_confirm_ms;
  uint32_t airborne_side_release_ms;
  float kalman_q_v;
  float kalman_q_a;
  float kalman_r_wheel;
  float kalman_r_imu;
  uint8_t kalman_slip_enable;
  float kalman_slip_accel;
  float kalman_slip_vel_diff;
  float kalman_slip_yaw_track;
  float kalman_slip_yaw;
  uint32_t kalman_slip_hold_ms;
  uint32_t kalman_slip_confirm_ms;
  uint8_t mpc_enable;
  uint8_t mpc_active;
  uint8_t mpc_shadow_enable;
  uint8_t mpc_shadow_active;
  uint32_t mpc_failure_count;
  uint32_t mpc_saturation_count;
  uint32_t mpc_rate_limit_count;
  uint32_t mpc_torque_limit_count;
  float mpc_wheel_authority;
  float mpc_leg_authority;
  float mpc_wheel_authority_applied;
  float mpc_leg_authority_applied;
  float mpc_authority_ramp_rate;
  float mpc_delta_weight_wheel;
  float mpc_delta_weight_leg;
  float mpc_abs_weight_wheel;
  float mpc_abs_weight_leg;
  float mpc_wheel_delta_max;
  float mpc_leg_delta_max;
  /* Velocity-feedback gain used after the stick has been released for a
   * while.  The standing limit cycle shrinks monotonically with more velocity
   * feedback, but the same gain makes the machine leaden while driving. */
  float dx_feedback_gain_static;
  float dx_static_delay_ms;
  /* Lower knee of the standing gain schedule.  Below this speed the gain is
   * the full static one; raising it pulls more of the low-speed band into full
   * damping, which settles faster but moves closer to where the strong gain
   * stops being stable. */
  float dx_static_speed_low;
  /* The strong gain is only safe near zero speed: at 0.3 m/s and above the
   * same gain turns the leg-target path into positive feedback and the machine
   * rings instead of recovering. */
  float dx_static_speed_limit;
  float mpc_left_wheel_torque;
  float mpc_right_wheel_torque;
  float mpc_left_tp_command;
  float mpc_right_tp_command;
  float mpc_left_wheel_delta;
  float mpc_right_wheel_delta;
  float mpc_left_tp_delta;
  float mpc_right_tp_delta;
  float mpc_left_wheel_raw;
  float mpc_right_wheel_raw;
  float mpc_left_tp_raw;
  float mpc_right_tp_raw;
  float mpc_leg_length;
  float mpc_x_integral;
  uint8_t mpc_integral_awake;
  uint8_t mpc_integral_hold_reason;
  uint8_t mpc_integral_saturated;
  float mpc_left_state_theta;
  float mpc_left_state_dtheta;
  float mpc_left_state_x;
  float mpc_left_state_dx;
  float mpc_left_state_phi;
  float mpc_left_state_dphi;
  float mpc_right_state_theta;
  float mpc_right_state_dtheta;
  float mpc_right_state_x;
  float mpc_right_state_dx;
  float mpc_right_state_phi;
  float mpc_right_state_dphi;
  float mpc_left_previous_wheel;
  float mpc_left_previous_tp;
  float mpc_right_previous_wheel;
  float mpc_right_previous_tp;
  /* Absolute command trajectories reconstructed from the incremental plan. */
  float mpc_left_plan[VMC_MPC_DECISION_COUNT];
  float mpc_right_plan[VMC_MPC_DECISION_COUNT];
  uint32_t mpc_solve_us;
  /* Slip LQR weight switching.  While one side's slip latch is set, that
   * side's leg-angle feedback is scaled up so the posture term dominates the
   * control output instead of the (unreliable) wheel/velocity terms.  See
   * VMC_SLIP_LQR_THETA_SCALE_DEFAULT. */
  uint8_t slip_lqr_enable;
  float slip_lqr_theta_scale;
  float slip_lqr_dtheta_scale;
  /* The scale actually applied last control cycle, per side.  Published so a
   * recording can prove the response fired. */
  float slip_lqr_applied_left;
  float slip_lqr_applied_right;
  /* Wheel-current scale applied per side (1.0 = not scaled). */
  float slip_wheel_current_scale;
  float slip_curr_applied_left;
  float slip_curr_applied_right;
  float tp_limit;
  float tp_phi_gain;
  float tp_x_gain;
  /* Anti-splay (leg-synchronisation) loop.  See
   * VMC_LEG_SYNC_ENABLE_DEFAULT.  leg_sync_torque is the term computed last
   * cycle: the left leg has -leg_sync_torque and the right +leg_sync_torque
   * folded into its joint-torque command, so the pair drives
   * left_theta - right_theta toward zero without moving their average. */
  uint8_t leg_sync_enable;
  float leg_sync_kp;
  float leg_sync_kd;
  float leg_sync_torque_limit;
  double leg_sync_error;
  double leg_sync_rate_error;
  double leg_sync_torque;
  float lqr_theta;
  float lqr_d_theta;
  float lqr_d_theta_raw;
  float theta_error_raw;
  float theta_error;
  float left_lqr_theta_error_raw;
  float right_lqr_theta_error_raw;
  float left_lqr_theta_error;
  float right_lqr_theta_error;
  float left_lqr_d_theta_raw;
  float right_lqr_d_theta_raw;
  float left_lqr_d_theta;
  float right_lqr_d_theta;
  float left_lqr_d_theta_limited;
  float right_lqr_d_theta_limited;
  float phi_error_raw;
  float phi_error;
  float dpitch_raw;
  float dpitch_lpf;
  float x_error;
  float dx_error;
  float dx_target;
  float dx_raw;
  double yaw_error_raw;
  double yaw_error;
  double yaw_rate_error;
  double gimbal_follow_error_raw;
  double gimbal_follow_error;
  double gimbal_follow_p_output;
  double gimbal_follow_d_output;
  double gimbal_follow_output;
  float left_wheel_theta_term;
  float left_wheel_dtheta_term;
  float left_wheel_x_term;
  float left_wheel_dx_term;
  float left_wheel_phi_term;
  float left_wheel_dphi_term;
  float left_tp_theta_term;
  float left_tp_dtheta_term;
  float left_tp_x_term;
  float left_tp_dx_term;
  float right_wheel_theta_term;
  float right_wheel_dtheta_term;
  float right_wheel_x_term;
  float right_wheel_dx_term;
  float right_wheel_phi_term;
  float right_wheel_dphi_term;
  float right_tp_theta_term;
  float right_tp_dtheta_term;
  float right_tp_x_term;
  float right_tp_dx_term;
  /* Legacy field names; all values below are motor-shaft torque in N.m. */
  float wheel_torque_raw;
  float left_wheel_torque_raw;
  float right_wheel_torque_raw;
  float left_wheel_torque_limited;
  float right_wheel_torque_limited;
  float left_tp_raw;
  float right_tp_raw;
  int16_t left_wheel_current_cmd;
  int16_t right_wheel_current_cmd;
  int16_t left_wheel_current_send;
  int16_t right_wheel_current_send;
  float left_tp_cmd;
  float right_tp_cmd;
} vmc_balance_t;

typedef struct {
  float roll;
  float pitch;
  float yaw;
  float pitch_rate;
  float wx;
  float wy;
  float wz;
  /* HI12 acc_b raw reading, unit g (not m/s2). */
  float ax;
  float ay;
  float az;
  /* Same reading with the gravity reaction removed, unit g. */
  float ax_lin;
  float ay_lin;
  float az_lin;
  /* Gravity-compensated acceleration projected onto world +Z, m/s2. */
  float world_az_lin;
  /* Resolved accelerometer sign convention: 0 = not yet known, else +1/-1. */
  int8_t gravity_sign;
  uint8_t imu_connected;
  uint8_t imu_valid;
} vmc_body_state_t;

typedef struct {
  double left_theta;
  double left_d_theta;
  double right_theta;
  double right_d_theta;
  double theta;
  double d_theta;
  double phi;
  double d_phi;
  double left_wheel_w;
  double right_wheel_w;
  double wheel_dx;
  double wheel_dx_gap;
  double left_body_dx;
  double right_body_dx;
  double d_x_raw;
  /* Gravity-compensated body-X acceleration in m/s2, fed to the estimator. */
  double dd_x;
  /* Diagnostics for the body-motion Kalman filter.  kf_wheel_measurement and
   * kf_accel_measurement are the measurement values available to the
   * estimator.  kf_accel_measurement is recomputed every control cycle from
   * the newest HI12 frame and reads zero only while body.gravity_sign is
   * still 0 (the accelerometer is withheld from the update).  The innovations
   * are evaluated before each corresponding correction step. */
  double kf_wheel_measurement;
  double kf_accel_measurement;
  double kf_innovation_v;
  double kf_innovation_a;
  /* Accelerometer corrections actually applied.  Telemetry is emitted at
   * 50 Hz while the accelerometer runs at 200 Hz and both are locked to the
   * 1 kHz control tick, so the 20 ms telemetry period can alias against the
   * 5 ms IMU period.  This counter proves the fusion rate independently. */
  uint32_t kf_accel_update_count;
  /* Slip-gate diagnostics.  kf_slip is 1 while either side's wheel
   * measurement is being rejected in favour of the prediction; the per-side
   * flags and the raw wheel speeds let the gate's inputs be verified from a
   * recording.  kf_slip_metric is the larger side's condition-C magnitude in
   * m/s2: |wheel acceleration| - |IMU body acceleration|. */
  double kf_slip;
  double kf_slip_metric;
  double kf_slip_left;
  double kf_slip_right;
  /* Raw detector evidence.  These latch regardless of whether the gate is
   * allowed to modify the Kalman input, so a capture can study the detector
   * without the detector changing the data it is evaluating. */
  double slip_raw_latched;
  double slip_raw_left;
  double slip_raw_right;
  double kf_wheel_left_raw;
  double kf_wheel_right_raw;
  double kf_x;
  double kf_dx;
  double kf_acc;
  double leg_length_avg;
} vmc_motion_state_t;

typedef struct {
  float left_k[12];
  float right_k[12];
  float rtt_k[12];
  uint8_t rtt_override_active;
  uint8_t rtt_last_error;
  uint32_t rtt_update_count;
  uint32_t rtt_error_count;
  uint32_t rtt_last_update_ms;
  uint32_t rtt_rx_count;
} vmc_lqr_state_t;

typedef struct {
  vmc_wheel_motor_t wheel_left;
  vmc_wheel_motor_t wheel_right;
  vmc_joint_motor_t joint_right_back;
  vmc_joint_motor_t joint_right_ahead;
  vmc_joint_motor_t joint_left_ahead;
  vmc_joint_motor_t joint_left_back;
  vmc_leg_state_t left_leg;
  vmc_leg_state_t right_leg;
  vmc_body_state_t body;
  vmc_motion_state_t motion;
  vmc_lqr_state_t lqr;
  vmc_balance_t balance;
  uint32_t can_rx_count;
  uint32_t can1_rx_count;
  uint32_t can2_rx_count;
  uint32_t unknown_rx_count;
  uint32_t last_can1_rx_id;
  uint32_t last_can2_rx_id;
  uint32_t mit_zero_tx_count;
  uint32_t mit_zero_tx_fail_count;
  uint32_t mit_enable_tx_count;
  uint32_t mit_enable_tx_fail_count;
  uint32_t dm_init_tx_count;
  uint32_t dm_init_tx_fail_count;
  uint32_t dm_recover_tx_count;
  uint32_t dm_recover_tx_fail_count;
  uint32_t dm_disable_tx_count;
  uint32_t dm_disable_tx_fail_count;
  uint32_t wheel_tx_count;
  uint32_t wheel_tx_fail_count;
  uint8_t dm_state;
  uint8_t dm_tx_phase;
  uint8_t phi0_protect_latched;
  uint8_t phi0_protect_source;
  uint8_t feedback_timeout_active;
  uint8_t feedback_timeout_source;
  uint8_t airborne_active;
  uint8_t airborne_candidate;
  uint32_t airborne_candidate_ms;
  uint32_t airborne_release_candidate_ms;
  float airborne_wheel_speed;
  float airborne_joint_torque;
  /* Slow telemetry keeps the measured joint-torque extremes for flash-safety
   * and legacy analysis.  The fast stream reuses these two wire slots for
   * per-side FN so the 450 Hz support-force signal remains observable. */
  float airborne_joint_torque_min;
  float airborne_joint_torque_max;
  /* Retired attitude-guard fields, retained at their original offsets. */
  float airborne_pitch_reference;
  uint8_t airborne_pitch_reference_valid;
  uint32_t airborne_pitch_reference_hold_ms;
  /* Per-side Shanhai FN (newtons, despite the legacy field name) and its wheel
   * latch, indexed by VMC_AIRBORNE_SIDE_LEFT/RIGHT.  Each latch gates only its
   * own wheel; airborne_active is the compatibility AND of both sides. */
  float airborne_side_torque[VMC_AIRBORNE_SIDE_COUNT];
  uint8_t airborne_side_active[VMC_AIRBORNE_SIDE_COUNT];
  uint8_t airborne_side_candidate[VMC_AIRBORNE_SIDE_COUNT];
  uint32_t airborne_side_candidate_ms[VMC_AIRBORNE_SIDE_COUNT];
  uint32_t airborne_side_release_candidate_ms[VMC_AIRBORNE_SIDE_COUNT];
  float airborne_side_fn_filtered[VMC_AIRBORNE_SIDE_COUNT];
  uint8_t airborne_side_fn_filter_initialized[VMC_AIRBORNE_SIDE_COUNT];
  uint8_t airborne_side_posture_active[VMC_AIRBORNE_SIDE_COUNT];
  uint32_t airborne_side_posture_ms[VMC_AIRBORNE_SIDE_COUNT];
  uint8_t airborne_side_length_returning[VMC_AIRBORNE_SIDE_COUNT];
  /* Measured control-loop timing.  loop_dt_s is the period handed to every
   * time-based term in the controller, loop_dt_us the same value in the
   * telemetry's integer encoding, and loop_count an iteration counter so a
   * recording can recover the true average rate over any window. */
  float loop_dt_s;
  uint32_t loop_dt_us;
  uint32_t loop_count;
} vmc_probe_state_t;

extern volatile vmc_probe_state_t vmc_probe_state;
extern volatile vmc_control_t vmc_control;
extern volatile vmc_wheel_control_t vmc_wheel_control;
extern volatile vmc_balance_t vmc_balance;
extern volatile uint8_t vmc_getup_fully_fallen;

void vmc_probe_task(void const *argument);
void vmc_can_init(void);
void vmc_can_poll_joints_zero(void);
void vmc_dm_state_machine_update(void);
void vmc_update_leg_state(void);
void vmc_control_init(void);
void vmc_control_update(void);
void vmc_wheel_control_init(void);
void vmc_wheel_control_update(void);
void vmc_balance_init(void);
void vmc_balance_update(void);

#ifdef __cplusplus
}
#endif

#endif
