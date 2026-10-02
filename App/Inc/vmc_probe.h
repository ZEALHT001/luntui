#ifndef VMC_PROBE_H
#define VMC_PROBE_H

#include "can.h"
#include "cmsis_os.h"

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
#define VMC_LEG_LENGTH_MIN         0.120f
#define VMC_LEG_LENGTH_MAX         0.385f
#define VMC_LEG_LENGTH_KP_DEFAULT  2000.0f
#define VMC_LEG_LENGTH_KD_DEFAULT  100.0f
#define VMC_LEG_FORCE_MAX_DEFAULT  200.0f
#define VMC_LEG_LENGTH_PID_DIRECTION -1.0f
#define VMC_LEG_PHI0_TARGET_DEFAULT VMC_PHI0_VERTICAL_RAD
#define VMC_LEG_PHI0_KP_DEFAULT     50.0f
#define VMC_LEG_PHI0_KD_DEFAULT     0.5f
#define VMC_LEG_PHI0_TORQUE_MAX_DEFAULT 108.0f /* = VMC_LEG_TP_MAX */
#define VMC_LEG_PHI0_PID_DIRECTION  1.0f
#define VMC_LEG_SUPPORT_FORCE_DEFAULT 120.0f
#define VMC_LEG_SUPPORT_FORCE_MAX     120.0f
#define VMC_LEG_SUPPORT_FORCE_DIRECTION -1.0f
#define VMC_LEG_TARGET_SHORT      0.180f
#define VMC_LEG_TARGET_MID        0.230f
#define VMC_LEG_TARGET_LONG       0.280f
/* Scaled by 0.45 to preserve the tuned behaviour; see VMC_CONTROL_DT. */
#define VMC_LEG_LENGTH_RAMP_RATE_DEFAULT 0.225f
#define VMC_JUMP_ENABLE_DEFAULT   1U
#define VMC_JUMP_COMPRESS_LENGTH_DEFAULT 0.18f
#define VMC_JUMP_EXTEND_LENGTH_DEFAULT   0.25f
#define VMC_JUMP_RETRACT_LENGTH_DEFAULT  0.18f
#define VMC_JUMP_LAND_LENGTH_DEFAULT     0.18f
#define VMC_JUMP_LENGTH_KP_DEFAULT       3000.0f
#define VMC_JUMP_LENGTH_KD_DEFAULT       100.0f
#define VMC_JUMP_FORCE_MAX_DEFAULT       100.0f
#define VMC_JUMP_SUPPORT_FORCE_DEFAULT   0.0f
#define VMC_JUMP_COMPRESS_TIMEOUT_MS_DEFAULT 1U
#define VMC_JUMP_EXTEND_TIMEOUT_MS_DEFAULT   1U
#define VMC_JUMP_RETRACT_TIMEOUT_MS_DEFAULT  1U
#define VMC_JUMP_LAND_TIMEOUT_MS_DEFAULT     1U
#define VMC_JUMP_COOLDOWN_MS_DEFAULT         1U
#define VMC_JUMP_TOTAL_TIMEOUT_MS_DEFAULT    1U
#define VMC_JUMP_ATTITUDE_LIMIT_DEFAULT      0.35f
#define VMC_JUMP_LAND_TORQUE_THRESHOLD_DEFAULT 3.0f
#define VMC_JUMP_LAND_CONFIRM_MS_DEFAULT     1U
#define VMC_JUMP_LENGTH_READY_TOL            0.015f
#define VMC_ROLL_COMP_ENABLE_DEFAULT 1U
#define VMC_ROLL_TARGET_DEFAULT 0.0f
#define VMC_ROLL_LENGTH_COMP_GAIN_DEFAULT 0.4f
#define VMC_ROLL_LENGTH_COMP_LIMIT_DEFAULT 0.08f
/* Scaled by 0.45 to preserve the tuned behaviour; see VMC_CONTROL_DT. */
#define VMC_ROLL_LENGTH_COMP_RAMP_RATE_DEFAULT 0.054f
/* Nominal control period in seconds.
 *
 * The loop is NOT fixed-rate.  osDelay(1) only sets a floor, so the real
 * period is "however long the loop body takes, plus the sleep".  Measured on
 * the vehicle the loop ran at 450 Hz, i.e. 2.22 ms, so every term that had
 * assumed 1 ms was off by that factor.
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
#define VMC_FEEDBACK_TIMEOUT_MS   100U
/* Measured wheel radius: 60 mm.  All wheel-speed -> body-speed conversions
 * use this value, so balance and speed-loop tuning must be revalidated after
 * a wheel change. */
#define VMC_WHEEL_RADIUS          0.060f
#define VMC_AIRBORNE_DETECT_ENABLE_DEFAULT 1U
/* An unloaded wheel accelerates quickly while the joints carry almost no
 * load.  Keep the thresholds conservative so normal balancing corrections do
 * not trip the latch. */
#define VMC_AIRBORNE_WHEEL_SPEED_THRESHOLD_DEFAULT 4.0f
#define VMC_AIRBORNE_JOINT_TORQUE_THRESHOLD_DEFAULT 2.0f
#define VMC_AIRBORNE_CONFIRM_MS_DEFAULT 1U
#define VMC_AIRBORNE_RELEASE_MS_DEFAULT 1U
/* "Whole machine off the ground" also has to look still, so the latch cannot
 * sit on a body the pilot is carrying or throwing.  Measured on the recorded
 * runs: pitch moved by at most 0.012 rad in the 200 ms before every latch
 * onset (bench lift, landing and low-friction driving alike), so the guard is
 * deliberately loose and does not sharpen the trigger.  It is there for the
 * swing that is not in the recordings yet - a tumbling body moves several
 * tenths of a rad over that span. */
#define VMC_AIRBORNE_PITCH_DEV_MAX_DEFAULT 0.2f
#define VMC_AIRBORNE_PITCH_RATE_MAX_DEFAULT 4.0f
/* How long the armed pitch reference survives after the wheel-speed condition
 * stops holding.  Without this the reference would be re-armed on the very
 * cycle that triggers the latch, and the comparison would always read zero.
 * 200 ms covers several control-confirmation windows so a body that swings
 * away inside that span still blocks the latch. */
#define VMC_AIRBORNE_PITCH_REF_HOLD_MS_DEFAULT 200U
/* Telemetry sentinel for "no attitude reference armed".  It has to be outside
 * the reachable pitch range so a recording cannot confuse an unarmed guard
 * with a reference captured at zero pitch. */
#define VMC_AIRBORNE_PITCH_REFERENCE_NONE (-999.0f)
/* Single-leg variant.  The whole-machine test above keys on the average of all
 * four joints, so it cannot see one leg being lifted: the other leg still
 * carries the body and holds that average well above the threshold.  Each side
 * therefore latches on its own, and only the lifted side's wheel is gated -
 * the grounded side is still needed to balance.
 *
 * The torque threshold is the same 2 N.m used by the whole-machine test, but
 * the confirmation is far longer.  Ground margin is wide - over the projectile
 * run the lowest single joint read 3.67 N.m and a side average is higher than
 * its own lowest joint - so the long confirmation is what keeps one bump from
 * cutting a wheel that is still on the ground. */
#define VMC_AIRBORNE_SIDE_ENABLE_DEFAULT 1U
#define VMC_AIRBORNE_SIDE_CONFIRM_MS_DEFAULT 60U
#define VMC_AIRBORNE_SIDE_RELEASE_MS_DEFAULT 200U
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
 * Condition C now differentiates against the MEASURED control period, so this
 * is a true m/s2 figure.  It was previously tuned while the differentiator
 * assumed 1 ms against a loop that actually ran at ~450 Hz, which inflated
 * every metric by about 2.2x.  The default is rescaled by that factor so the
 * same events trigger as before: a landing transient that spins the wheel from
 * 0 to 2.9 m/s in ~200 ms is a true 14.5 m/s2, which the old code reported as
 * ~32 m/s2 while steady driving stays far below it either way.
 *
 * The rescale is an equivalence, not a re-measurement.  Any slip_accel value
 * carried over from before this change should be divided by ~2.2, and the
 * threshold should be re-derived from a fresh projectile run. */
#define VMC_BODY_MOTION_KF_SLIP_ACCEL_DEFAULT     9.0f
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
 * Condition C differentiates the wheel speed, so any short mechanical
 * transient - a landing impact, an encoder glitch, a single-sample current
 * step - shows up in it as an isolated spike rather than a sustained level.
 * Measured on the vehicle those transients produced contiguous runs of only
 * 1-2 telemetry frames (20-40 ms) above threshold, while a genuine slide
 * persists for hundreds of milliseconds.  Requiring the metric to stay above
 * the threshold rejects the spikes without meaningfully delaying a real
 * detection. */
#define VMC_BODY_MOTION_KF_SLIP_CONFIRM_MS_DEFAULT 60U

/* Raw joint positions captured with both legs vertical down at 385 mm.
 *
 * These are zero offsets, not gains.  joint_model_angle() applies
 *   model = REF_MODEL + DIR * (raw - REF_RAW)
 * so re-zeroing a leg means solving REF_RAW = raw - (target - REF_MODEL)/DIR
 * for the raw position the motor reports at the reference pose.  Leave DIR
 * alone unless the linkage itself was rebuilt: DIR sets the sign of motion,
 * not the zero.
 *
 * WARNING - the right_ahead motor (id 2) does NOT report a stable absolute
 * position across power cycles.  Measured at the same physical pose on three
 * separate power-ups it read -2.5378, -1.8507 and -2.5519 rad.  The steps are
 * 39.4 and 40.2 degrees, i.e. one motor-rotor revolution through the ~9:1
 * reduction (360/9 = 40.0 deg): the multi-turn reference is re-established at
 * power-on instead of being absolute.  The joint angle is therefore only
 * reliable modulo ~40 degrees and NO firmware offset can fix that.
 *
 * Consequences: do not re-calibrate REF_RAW from a single reading - it will
 * bake in whichever turn happened to come up and break on the next power-on.
 * The values below are the long-standing ones; with them the right leg tracks
 * the left to ~1.3 deg at the reference pose.  Fix the motor (set and store
 * its zero, and check the encoder connector) rather than the constants. */
#define VMC_REF_RAW_LEFT_BACK      2.51313968f
#define VMC_REF_RAW_LEFT_AHEAD    -3.20170302f
#define VMC_REF_RAW_RIGHT_AHEAD    -2.53776646f
#define VMC_REF_RAW_RIGHT_BACK     0.432402611f

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

#define VMC_GETUP_START_PITCH_RAD  0.80f
#define VMC_GETUP_TURN_LENGTH      0.300f
#define VMC_GETUP_STAND_LENGTH     VMC_LEG_TARGET_SHORT
#define VMC_GETUP_PHI0_STEP        0.200f
#define VMC_GETUP_THETA_MIN_RAD    0.500f
#define VMC_GETUP_THETA_MAX_RAD    1.400f
#define VMC_GETUP_PITCH_READY_RAD  0.300f
#define VMC_GETUP_LENGTH_READY_TOL 0.080f
#define VMC_GETUP_LEG_BAD_NEG_MIN_RAD -3.140f
#define VMC_GETUP_LEG_BAD_NEG_MAX_RAD -0.790f
#define VMC_GETUP_LEG_BAD_POS_MIN_RAD  2.000f
#define VMC_GETUP_LEG_BAD_POS_MAX_RAD  3.140f
#define VMC_GETUP_LEG_RECOVER_TARGET_RAD 1.100f
#define VMC_GETUP_LEG_RECOVER_TOL_RAD    0.100f

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
#define VMC_BALANCE_SPEED_CONTROL_ENABLE_DEFAULT 1U
#define VMC_BALANCE_SPEED_TARGET_MAX_DEFAULT 4.0f
/* Scaled by 0.45 to preserve the tuned behaviour; see VMC_CONTROL_DT. */
#define VMC_BALANCE_SPEED_RAMP_RATE_DEFAULT 2.25f
#define VMC_BALANCE_SPEED_DEADBAND_DEFAULT 0.02f
#define VMC_BALANCE_STOP_SPEED_THRESHOLD_DEFAULT 0.08f
/* Braking-only Pitch feedforward gain (rad per m/s^2). */
#define VMC_BALANCE_SPEED_PITCH_GAIN_DEFAULT 0.0f
#define VMC_BALANCE_SPEED_PITCH_LIMIT_DEFAULT 0.18f
/* Scaled by 0.45 to preserve the tuned behaviour; see VMC_CONTROL_DT. */
#define VMC_BALANCE_SPEED_PITCH_RAMP_RATE_DEFAULT 0.3f
#define VMC_BALANCE_X_HOLD_MAX_STRETCH_DEFAULT 0.05f
#define VMC_BALANCE_DTHETA_LPF_ALPHA_DEFAULT 0.1f
#define VMC_BALANCE_LQR_DTHETA_GAIN_DEFAULT 1.0f
#define VMC_BALANCE_LQR_DTHETA_DEADBAND_DEFAULT 0.0f
#define VMC_BALANCE_LQR_DTHETA_LIMIT_DEFAULT 10.0f
#define VMC_BALANCE_DPITCH_LPF_ALPHA_DEFAULT 1.0f
#define VMC_BALANCE_DISTURBANCE_OBSERVER_ENABLE_DEFAULT 0U
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
#define VMC_RC_YAW_TARGET_RATE_MAX_DEFAULT 0.9f
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
  float target_length;
  uint8_t jump_enable;
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
  float roll_comp_gain;
  float roll_comp_limit;
  float roll_comp_ramp_rate;
  float roll_comp_offset;
  float left_target_length;
  float right_target_length;
  uint8_t getup_state;
  int8_t getup_turn_direction;
  uint8_t getup_lqr_release_pending;
  float getup_shrink_target_theta_left;
  float getup_shrink_target_theta_right;
  float getup_target_phi0_left;
  float getup_target_phi0_right;
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
  float pitch_offset;
  float phi_offset;
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
  float tp_limit;
  float tp_phi_gain;
  float tp_x_gain;
  double lqr_theta;
  double lqr_d_theta;
  double lqr_d_theta_raw;
  double theta_error_raw;
  double theta_error;
  double left_lqr_theta_error_raw;
  double right_lqr_theta_error_raw;
  double left_lqr_theta_error;
  double right_lqr_theta_error;
  double left_lqr_d_theta_raw;
  double right_lqr_d_theta_raw;
  double left_lqr_d_theta;
  double right_lqr_d_theta;
  double left_lqr_d_theta_limited;
  double right_lqr_d_theta_limited;
  double phi_error_raw;
  double phi_error;
  double dpitch_raw;
  double dpitch_lpf;
  double x_error;
  double dx_error;
  double dx_target;
  double dx_raw;
  double yaw_error_raw;
  double yaw_error;
  double yaw_rate_error;
  double gimbal_follow_error_raw;
  double gimbal_follow_error;
  double gimbal_follow_p_output;
  double gimbal_follow_d_output;
  double gimbal_follow_output;
  double left_wheel_theta_term;
  double left_wheel_dtheta_term;
  double left_wheel_x_term;
  double left_wheel_dx_term;
  double left_wheel_phi_term;
  double left_wheel_dphi_term;
  double left_tp_theta_term;
  double left_tp_dtheta_term;
  double left_tp_x_term;
  double left_tp_dx_term;
  double right_wheel_theta_term;
  double right_wheel_dtheta_term;
  double right_wheel_x_term;
  double right_wheel_dx_term;
  double right_wheel_phi_term;
  double right_wheel_dphi_term;
  double right_tp_theta_term;
  double right_tp_dtheta_term;
  double right_tp_x_term;
  double right_tp_dx_term;
  /* Legacy field names; all values below are motor-shaft torque in N.m. */
  double wheel_torque_raw;
  double left_wheel_torque_raw;
  double right_wheel_torque_raw;
  double left_wheel_torque_limited;
  double right_wheel_torque_limited;
  double left_tp_raw;
  double right_tp_raw;
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
  double kf_wheel_left_raw;
  double kf_wheel_right_raw;
  double kf_x;
  double kf_dx;
  double kf_acc;
  double leg_length_avg;
} vmc_motion_state_t;

typedef struct {
  double left_k[12];
  double right_k[12];
  double rtt_k[12];
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
  /* The same "all four unloaded" test now needs the weakest and the strongest
   * joint, not their mean.  Keeping the mean for compatibility with older
   * recordings, the extremes are what decide the latch. */
  float airborne_joint_torque_min;
  float airborne_joint_torque_max;
  /* Pitch captured when the wheel-speed condition first armed.  The latch
   * requires the body not to have swung away from that value. */
  float airborne_pitch_reference;
  uint8_t airborne_pitch_reference_valid;
  uint32_t airborne_pitch_reference_hold_ms;
  /* Per-side "this leg is off the ground" latch, indexed by
   * VMC_AIRBORNE_SIDE_LEFT/RIGHT.  Unlike airborne_active above - which is the
   * whole-machine condition and gates both wheels - these gate only their own
   * wheel, so a lifted leg stops spinning while the grounded one keeps
   * balancing. */
  float airborne_side_torque[VMC_AIRBORNE_SIDE_COUNT];
  uint8_t airborne_side_active[VMC_AIRBORNE_SIDE_COUNT];
  uint8_t airborne_side_candidate[VMC_AIRBORNE_SIDE_COUNT];
  uint32_t airborne_side_candidate_ms[VMC_AIRBORNE_SIDE_COUNT];
  uint32_t airborne_side_release_candidate_ms[VMC_AIRBORNE_SIDE_COUNT];
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
