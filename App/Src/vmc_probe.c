#include "vmc_probe.h"

#include "board_comm.h"
#include "hi12_imu.h"
#include "remote_control.h"
#include "sbus_remote.h"
#include "tf02_pro.h"
#include "vmc_lqr.h"
#include "vmc_lqr_rtt.h"
#include "vmc_math.h"

#include <math.h>
#include <string.h>

volatile vmc_probe_state_t vmc_probe_state;
volatile vmc_control_t vmc_control;
volatile vmc_wheel_control_t vmc_wheel_control;
volatile vmc_balance_t vmc_balance;
volatile uint8_t vmc_getup_fully_fallen;
static double vmc_lqr_d_theta_lpf;
static double vmc_lqr_left_d_theta_lpf;
static double vmc_lqr_right_d_theta_lpf;
static double vmc_dpitch_lpf;
static double vmc_speed_dx_prev;
static double vmc_speed_accel_lpf;
static uint8_t vmc_speed_accel_initialized;
static uint8_t vmc_remote_source_last;
static float vmc_yaw_last_wrapped;
static float vmc_yaw_unwrapped;
static uint8_t vmc_yaw_unwrap_initialized;
static uint32_t vmc_airborne_last_update_ms;
static uint32_t vmc_jump_last_update_ms;
static uint32_t vmc_jump_auto_hold_ms;
static uint8_t vmc_stair_auto_armed;
static uint8_t vmc_stair_preimpact_armed;
static double vmc_stair_preimpact_speed_ref;
static uint8_t vmc_getup_reset_balance_on_ready;
static uint8_t vmc_jump_manual_latched;
static uint8_t vmc_spin_mode_last;
/* Gravity-compensation sign calibration; see VMC_GRAVITY_SIGN_ALPHA. */
static float vmc_gravity_dot_lpf;
static int8_t vmc_gravity_sign;
/* The accelerometer runs at 200 Hz while this task runs several times faster.
 * Feeding one frame into every consecutive update would make r_imu effectively
 * stronger than configured by the ratio of the two rates, so the acceleration
 * update is gated on a new HI12 frame. */
static uint32_t vmc_last_imu_rx_count;
/* Control-loop timing.  See VMC_CONTROL_DT: the loop is not fixed-rate, so the
 * period is measured each cycle and fed to every time-based term.
 *
 * The timebase is TIM1, which the firmware already uses as the HAL tick source:
 * its counter clock is 1 MHz with a 1 ms period, so TIM1->CNT is the number of
 * microseconds into the current millisecond.  Deriving the period from the same
 * timer as the tick means dt can never disagree with the recorded MCU
 * timestamps by construction, and the prescaler is computed from the same clock
 * configuration, so a mis-set SystemCoreClock cannot make the two diverge.
 * DWT->CYCCNT would also work (it was verified to telescope to the same elapsed
 * time) but depends on the debug block and on SystemCoreClock. */
static uint32_t vmc_loop_us_last;
static float vmc_loop_dt_s;
static uint32_t vmc_loop_deadline_us;
/* Static-hold damping state.  The applied gain is kept separate from the
 * configured one so the hand-off between them can be ramped. */
static uint32_t vmc_dx_static_hold_ms;
static float vmc_dx_gain_applied = VMC_BALANCE_DX_FEEDBACK_GAIN_DEFAULT;
/* A fall detected while the robot was enabled requires channel 4 to be moved
 * UP to emergency-stop and then DOWN again before control can re-enable. */
static uint8_t vmc_fall_rearm_required;
static uint8_t vmc_fall_rearm_armed;
static uint8_t vmc_fall_monitor_armed;
static uint8_t vmc_fall_prev_fully_fallen;

static void clear_leg_command(volatile vmc_leg_state_t *leg);
static uint8_t vmc_feedback_timeout_source(void);
static void vmc_update_remote_target_length(void);
static double vmc_remote_leg_length_target(void);
static uint8_t vmc_remote_leg_length_gear(void);
static uint8_t vmc_getup_safe_length_ready(void);
static uint8_t vmc_manual_prepose_exit_length_ready(void);
static uint8_t vmc_jump_is_active(void);
static uint8_t vmc_jump_is_active_or_aborted(void);
static uint8_t vmc_jump_blocks_wheel_output(void);
static uint8_t vmc_jump_joint_lqr_active(void);
static uint8_t vmc_jump_overrides_target(void);
static double vmc_jump_stage_target_length(void);
static void vmc_jump_update(void);
static uint8_t vmc_stair_is_active(void);
static uint8_t vmc_stair_overrides_target(void);
static double vmc_stair_release_scale(void);
static double vmc_stair_target_length(void);
static void vmc_stair_update(void);
static uint8_t vmc_getup_is_active(void);
static uint8_t vmc_getup_pose_ready(void);
static uint8_t vmc_getup_upright_ready(void);
static uint8_t vmc_getup_partial_folded_is_required(double pitch);
static int8_t vmc_getup_body_mode(double pitch);
static int8_t vmc_getup_body_align_direction(double phi4, int8_t body_mode);
static uint8_t vmc_getup_body_fan_is_required(void);
static uint8_t vmc_getup_body_fan_sync_ready(void);
static void vmc_getup_body_fan_sync_targets(double *left_target,
                                            double *right_target);
static uint8_t vmc_getup_direct_shrink_is_required(void);
static uint8_t vmc_getup_next_precondition_state(void);
static uint8_t vmc_getup_enter_brace(int8_t turn_direction);
static void vmc_getup_enter_stage(uint8_t state);
static void vmc_getup_abort(void);
static void vmc_update_getup_state(void);
static double vmc_spin_phase_lead_rad(double spin_rate);
static double vmc_spin_projection_gain(double phase_error, double phase_lead,
                                       double power);
static void vmc_update_remote_roll_target(void);
static void vmc_update_remote_mode(void);
static void vmc_update_body_state(void);
static void vmc_update_gravity_compensation(void);
static void vmc_update_motion_state(void);
static void vmc_update_airborne_state(void);
static double vmc_loop_dt(void);
static void vmc_update_lqr_gain(void);
static uint8_t vmc_safety_allows_output(void);
static uint8_t vmc_manual_prepose_allows_wheel_output(void);
static int16_t vmc_update_yaw_hold_current(void);
static int16_t vmc_update_gimbal_follow_current(void);
static void vmc_balance_apply_wheel_output(void);
static void vmc_body_motion_kf_reset(void);
static void vmc_body_motion_kf_update(double measured_dx, double measured_acc,
                                      uint8_t acceleration_valid,
                                      uint8_t slip_active);
static HAL_StatusTypeDef can_send_wheel_current(int16_t left_current,
                                                int16_t right_current);

static float dm_uint_to_float(uint16_t x, float x_min, float x_max, uint8_t bits)
{
  const float span = x_max - x_min;
  const float max_int = (float)((1UL << bits) - 1UL);
  return ((float)x) * span / max_int + x_min;
}

static float clamp_float(float value, float min_value, float max_value)
{
  if (value > max_value) {
    return max_value;
  }
  if (value < min_value) {
    return min_value;
  }
  return value;
}

static uint32_t clamp_u32(uint32_t value, uint32_t min_value, uint32_t max_value)
{
  if (value > max_value) {
    return max_value;
  }
  if (value < min_value) {
    return min_value;
  }
  return value;
}

static double body_motion_kf_x[2];
static double body_motion_kf_p[4];
static uint8_t body_motion_kf_initialized;

/* Slip gate for the two-state estimator.
 *
 * Condition C differentiates the wheel speed and compares it against the IMU
 * body acceleration, so both signals are low-passed first; the C620 reports
 * wheel speed as an integer RPM, which makes a raw sample-to-sample difference
 * far too coarse to threshold directly.  Each side latches on its own. */
/* NOTE: these two alphas are applied once per control cycle, not per unit
 * time, so the filter's real time constant is alpha * (period / 0.223) and
 * therefore scales with the loop rate.  They were picked by ear against the
 * real rate, so they are left alone here; making them dt-aware would be a
 * separate change and would need its own re-tune. */
#define VMC_BODY_MOTION_KF_SLIP_WHEEL_ALPHA 0.20
#define VMC_BODY_MOTION_KF_SLIP_ACCEL_ALPHA 0.20
/* Release hysteresis and minimum hold.  Releasing on "the wheel speed fell
 * back below the value recorded on entry" only holds for an accelerating
 * slip; a decelerating or locked wheel drops below that value on the very
 * next sample, so the latch flapped at sampling rate and re-admitted the
 * slipping wheel every few milliseconds. */
#define VMC_BODY_MOTION_KF_SLIP_RELEASE_RATIO 0.5
#define VMC_BODY_MOTION_KF_SLIP_MIN_HOLD_MS   50U
/* Weight changes while a latch is active.  The wheel measurement is replaced
 * by the prediction, so this mostly bounds how much the near-zero innovation
 * may still move the state; widening r_wheel keeps the wheel from regaining
 * authority through the covariance. */
#define VMC_BODY_MOTION_KF_SLIP_Q_V_SCALE     0.1
#define VMC_BODY_MOTION_KF_SLIP_Q_A_SCALE     2.0
#define VMC_BODY_MOTION_KF_SLIP_R_WHEEL_SCALE 90.0

static double body_motion_kf_slip_left_raw_lpf;
static double body_motion_kf_slip_right_raw_lpf;
static double body_motion_kf_slip_left_accel_lpf;
static double body_motion_kf_slip_right_accel_lpf;
static double body_motion_kf_slip_imu_accel_lpf;
static uint32_t body_motion_kf_slip_left_ms;
static uint32_t body_motion_kf_slip_right_ms;
static uint32_t body_motion_kf_slip_left_above_ms;
static uint32_t body_motion_kf_slip_right_above_ms;
static uint8_t body_motion_kf_slip_left;
static uint8_t body_motion_kf_slip_right;
static uint8_t body_motion_kf_slip_initialized;

static double clamp_double(double value, double min_value, double max_value)
{
  if (value > max_value) {
    return max_value;
  }
  if (value < min_value) {
    return min_value;
  }
  return value;
}

/* Convert an angle target to the representation nearest the measured angle.
 * phi0 is reported on [-pi, pi], so an absolute target near the wrap boundary
 * otherwise looks like a full-turn position error. */
static double vmc_angle_nearest_to(double target, double reference)
{
  return reference +
         (double)vmc_rad_format((float)(target - reference));
}

static double vmc_angle_forward_to(double target, double reference,
                                   int8_t direction)
{
  const double signed_direction = (direction < 0) ? -1.0 : 1.0;
  double delta = (double)vmc_rad_format((float)(target - reference)) *
                 signed_direction;

  if (delta < 0.0) {
    delta += 2.0 * (double)VMC_PI;
  }
  return reference + signed_direction * delta;
}

static void vmc_body_motion_kf_slip_reset(double wheel_left_raw_speed,
                                          double wheel_right_raw_speed,
                                          double imu_accel)
{
  body_motion_kf_slip_left_raw_lpf = wheel_left_raw_speed;
  body_motion_kf_slip_right_raw_lpf = wheel_right_raw_speed;
  body_motion_kf_slip_left_accel_lpf = 0.0;
  body_motion_kf_slip_right_accel_lpf = 0.0;
  body_motion_kf_slip_imu_accel_lpf = imu_accel;
  body_motion_kf_slip_left_ms = 0U;
  body_motion_kf_slip_right_ms = 0U;
  body_motion_kf_slip_left_above_ms = 0U;
  body_motion_kf_slip_right_above_ms = 0U;
  body_motion_kf_slip_left = 0U;
  body_motion_kf_slip_right = 0U;
  body_motion_kf_slip_initialized = 1U;
}

/* One side's latch.  Returns 1 while that side's wheel measurement should be
 * rejected.
 *
 * The metric must stay above the threshold for `confirm_ms` before the latch
 * engages.  Condition C differentiates the wheel speed, so the leg/frame
 * assembly's anti-phase left/right oscillation also produces large values;
 * that mode appears as isolated 20-40 ms spikes, whereas a real slide persists.
 * Confirmation is what tells them apart. */
static uint8_t vmc_body_motion_kf_slip_latch_update(uint8_t *latch,
                                                    uint32_t *latch_ms,
                                                    uint32_t *above_ms,
                                                    double metric,
                                                    double threshold,
                                                    double release_threshold,
                                                    uint32_t dt_ms,
                                                    uint32_t confirm_ms,
                                                    uint32_t now,
                                                    uint32_t min_hold_ms,
                                                    uint32_t hold_ms)
{
  uint32_t latched_ms;

  if (metric >= threshold) {
    if (*above_ms < confirm_ms) {
      *above_ms += dt_ms;
    }
    if (*latch == 0U && *above_ms >= confirm_ms) {
      *latch = 1U;
      *latch_ms = now;
    }
    return *latch;
  }

  *above_ms = 0U;
  if (*latch == 0U) {
    return 0U;
  }

  /* Never release before the minimum hold: the metric can dip between two
   * samples of a genuine slide. */
  latched_ms = now - *latch_ms;
  if (latched_ms < min_hold_ms) {
    return 1U;
  }
  if (metric < release_threshold ||
      (hold_ms != 0U && latched_ms >= hold_ms)) {
    *latch = 0U;
    return 0U;
  }
  return 1U;
}

/*
 * Slip detector, following the USTB/Nanhang structure:
 *
 *   slip = (condition A OR condition B) AND condition C
 *
 * Returns 1 while either side is latched.  metric_out receives the larger of
 * the two sides' condition-C magnitude, in m/s2.
 */
static uint8_t vmc_body_motion_kf_slip_gate(double wheel_left_body_speed,
                                            double wheel_right_body_speed,
                                            double wheel_left_raw_speed,
                                            double wheel_right_raw_speed,
                                            double imu_accel,
                                            double body_speed,
                                            double yaw_rate_imu,
                                            uint8_t accel_valid,
                                            double dt,
                                            double *metric_out)
{
  const double accel_threshold =
      clamp_double((double)vmc_balance.kalman_slip_accel, 0.0, 1.0e4);
  const double velocity_threshold =
      clamp_double((double)vmc_balance.kalman_slip_vel_diff, 0.0, 1.0e3);
  const double yaw_track =
      clamp_double((double)vmc_balance.kalman_slip_yaw_track, 0.0, 2.0);
  const double yaw_threshold =
      clamp_double((double)vmc_balance.kalman_slip_yaw, 0.0, 1.0e3);
  const uint32_t hold_ms =
      clamp_u32(vmc_balance.kalman_slip_hold_ms, 0U, 5000U);
  const uint32_t confirm_ms =
      clamp_u32(vmc_balance.kalman_slip_confirm_ms, 0U, 1000U);
  const uint32_t now = HAL_GetTick();
  const double wheel_left_previous = body_motion_kf_slip_left_raw_lpf;
  const double wheel_right_previous = body_motion_kf_slip_right_raw_lpf;
  double metric_left;
  double metric_right;
  uint8_t frame = 0U;

  if (metric_out != NULL) {
    *metric_out = 0.0;
  }
  if (dt <= 0.0 || metric_out == NULL) {
    return 0U;
  }

  /* Condition C only ever looks at the raw wheel speed. */
  body_motion_kf_slip_left_raw_lpf +=
      VMC_BODY_MOTION_KF_SLIP_WHEEL_ALPHA *
      (wheel_left_raw_speed - body_motion_kf_slip_left_raw_lpf);
  body_motion_kf_slip_right_raw_lpf +=
      VMC_BODY_MOTION_KF_SLIP_WHEEL_ALPHA *
      (wheel_right_raw_speed - body_motion_kf_slip_right_raw_lpf);
  /* The wheel differentiator runs on every control cycle, so dt is the
   * MEASURED period (see VMC_CONTROL_DT).  That is what makes the threshold a
   * real m/s2 figure: assuming 1 ms while the loop actually ran at ~450 Hz
   * inflated every metric by about 2.2x.  The IMU term is only refreshed when
   * a new frame arrived; holding it in between keeps one accel sample from
   * being counted once per control cycle, which is the same aliasing the
   * estimator's own gate exists to avoid. */
  if (accel_valid != 0U) {
    body_motion_kf_slip_imu_accel_lpf +=
        VMC_BODY_MOTION_KF_SLIP_ACCEL_ALPHA *
        (imu_accel - body_motion_kf_slip_imu_accel_lpf);
  }
  body_motion_kf_slip_left_accel_lpf +=
      VMC_BODY_MOTION_KF_SLIP_ACCEL_ALPHA *
      (((body_motion_kf_slip_left_raw_lpf - wheel_left_previous) / dt) -
       body_motion_kf_slip_left_accel_lpf);
  body_motion_kf_slip_right_accel_lpf +=
      VMC_BODY_MOTION_KF_SLIP_ACCEL_ALPHA *
      (((body_motion_kf_slip_right_raw_lpf - wheel_right_previous) / dt) -
       body_motion_kf_slip_right_accel_lpf);

  /* Condition C: a wheel accelerating harder than the body is the core slip
   * signature.  Magnitudes are compared so the check does not depend on the
   * sign convention of either channel. */
  metric_left = fabs(body_motion_kf_slip_left_accel_lpf) -
                fabs(body_motion_kf_slip_imu_accel_lpf);
  metric_right = fabs(body_motion_kf_slip_right_accel_lpf) -
                 fabs(body_motion_kf_slip_imu_accel_lpf);
  *metric_out = (metric_left > metric_right) ? metric_left : metric_right;

  /* Condition A: the wheel-derived yaw rate disagrees with the IMU yaw rate.
   * Only usable once the wheel track has been measured. */
  if (yaw_track > 0.0 && yaw_threshold > 0.0) {
    const double yaw_rate_est =
        (wheel_right_body_speed - wheel_left_body_speed) / yaw_track;
    if (fabs(fabs(yaw_rate_est) - fabs(yaw_rate_imu)) >= yaw_threshold) {
      frame = 1U;
    }
  }
  /* Condition B: a wheel runs away from the estimated body speed. */
  if (velocity_threshold > 0.0 &&
      (fabs(wheel_left_body_speed) - fabs(body_speed) >= velocity_threshold ||
       fabs(wheel_right_body_speed) - fabs(body_speed) >= velocity_threshold)) {
    frame = 1U;
  }

  if (accel_threshold <= 0.0) {
    body_motion_kf_slip_left = 0U;
    body_motion_kf_slip_right = 0U;
    return 0U;
  }

  /* This runs on every control cycle; only the IMU term above is gated.  The
   * confirmation accumulator therefore advances by the control period. */
  const uint32_t gate_dt_ms =
      (uint32_t)clamp_double(dt * 1000.0, 1.0, 100.0);

  /* Condition C triggers on its own.  Framing it behind A or B was measured to
   * never fire: condition B compares a wheel against a fused velocity that is
   * itself derived from that wheel, so it is self-limiting - the filter has
   * already followed the slipping wheel and the gap stays small.  Measured on
   * the vehicle, a landing transient drove condition C to 56 m/s2 while the
  * B gap never exceeded 0.76 m/s against a 1.2 m/s threshold, so the latch
  * stayed open through the whole event.  Condition C is already a differencing
  * test (wheel acceleration minus body acceleration), so it does not need
  * them; A and B remain wired as optional extra triggers for the case where a
  * measured wheel track enables A.
   *
   * `frame` therefore only ever adds a trigger, never suppresses one. */
  if (frame != 0U) {
    metric_left += accel_threshold;
    metric_right += accel_threshold;
  }

  {
    const double release_threshold =
        accel_threshold * VMC_BODY_MOTION_KF_SLIP_RELEASE_RATIO;

    (void)vmc_body_motion_kf_slip_latch_update(
        &body_motion_kf_slip_left, &body_motion_kf_slip_left_ms,
        &body_motion_kf_slip_left_above_ms, metric_left,
        accel_threshold, release_threshold, gate_dt_ms, confirm_ms, now,
        VMC_BODY_MOTION_KF_SLIP_MIN_HOLD_MS, hold_ms);
    (void)vmc_body_motion_kf_slip_latch_update(
        &body_motion_kf_slip_right, &body_motion_kf_slip_right_ms,
        &body_motion_kf_slip_right_above_ms, metric_right,
        accel_threshold, release_threshold, gate_dt_ms, confirm_ms, now,
        VMC_BODY_MOTION_KF_SLIP_MIN_HOLD_MS, hold_ms);
  }

  return (body_motion_kf_slip_left != 0U || body_motion_kf_slip_right != 0U)
             ? 1U
             : 0U;
}

static double apply_soft_deadband(double value, double deadband)
{
  deadband = clamp_double(deadband, 0.0, 0.20);
  if (value > deadband) {
    return value - deadband;
  }
  if (value < -deadband) {
    return value + deadband;
  }
  return 0.0;
}

static double apply_zero_deadband(double value, double deadband)
{
  deadband = clamp_double(deadband, 0.0, 10.0);
  return (fabs(value) <= deadband) ? 0.0 : value;
}

static double vmc_body_fan_speed_torque(double target_speed,
                                        double measured_speed,
                                        double leg_length,
                                        double phi0,
                                        double speed_kp,
                                        double speed_torque_limit)
{
  const double gravity_gain =
      clamp_double(VMC_GETUP_BODY_FAN_GRAVITY_GAIN, 0.0, 50.0);
  const double speed_deadband = clamp_double(
      VMC_GETUP_BODY_FAN_SPEED_DEADBAND_RADPS, 0.0, 10.0);
  speed_kp = clamp_double(speed_kp, 0.0, 100.0);
  speed_torque_limit =
      clamp_double(speed_torque_limit, 0.0, VMC_LEG_TP_MAX);
  const double speed_error =
      apply_zero_deadband(target_speed - measured_speed, speed_deadband);
  const double speed_torque =
      clamp_double(speed_kp * speed_error,
                   -speed_torque_limit,
                   speed_torque_limit);
  const double gravity_torque =
      -gravity_gain * leg_length *
      cos(phi0 - (double)VMC_GETUP_BODY_FAN_GRAVITY_ANGLE_RAD);

  return speed_torque + gravity_torque;
}

static double vmc_getup_body_fan_sync_torque_feedforward(
    double target_speed,
    double measured_speed,
    double speed_limit)
{
  const double direction = (target_speed < 0.0) ? -1.0 : 1.0;
  const double full_speed = clamp_double(
      VMC_GETUP_BODY_FAN_SYNC_BREAKAWAY_FULL_SPEED_RADPS, 0.0, 10.0);
  const double exit_speed = clamp_double(
      VMC_GETUP_BODY_FAN_SYNC_BREAKAWAY_EXIT_SPEED_RADPS,
      full_speed + 0.001,
      10.0);
  const double measured_speed_abs = fabs(measured_speed);
  double breakaway_scale;
  double speed_error;

  if (target_speed == 0.0 || speed_limit <= 0.0) {
    return 0.0;
  }
  if (measured_speed_abs <= full_speed) {
    breakaway_scale = 1.0;
  } else if (measured_speed_abs >= exit_speed) {
    breakaway_scale = 0.0;
  } else {
    breakaway_scale =
        (exit_speed - measured_speed_abs) / (exit_speed - full_speed);
  }
  speed_error = (target_speed - measured_speed) * direction;
  if (speed_error <= 0.0) {
    return 0.0;
  }
  return direction *
         clamp_double(VMC_GETUP_BODY_FAN_SYNC_TORQUE_FF, 0.0, 50.0) *
         clamp_double(speed_error / speed_limit, 0.0, 1.0) *
         breakaway_scale;
}

static double apply_encoder_deadband(double value, double deadband)
{
  deadband = clamp_double(deadband, 0.0, 4096.0);
  if (value > deadband) {
    return value - deadband;
  }
  if (value < -deadband) {
    return value + deadband;
  }
  return 0.0;
}

static int16_t vmc_gimbal_encoder_error(uint16_t ecd, uint16_t front_ecd)
{
  /* Shortest signed distance on the 13-bit encoder ring. */
  const int32_t wrapped = ((int32_t)ecd - (int32_t)front_ecd + 4096) & 0x1FFF;
  return (int16_t)(wrapped - 4096);
}

static int16_t clamp_i16(int16_t value, int16_t min_value, int16_t max_value)
{
  if (value > max_value) {
    return max_value;
  }
  if (value < min_value) {
    return min_value;
  }
  return value;
}

static float clamp_float_abs(float value, float max_abs)
{
  if (max_abs < 0.0f) {
    max_abs = -max_abs;
  }
  return clamp_float(value, -max_abs, max_abs);
}

static int16_t rc_channel_to_current(int16_t ch, int16_t max_current)
{
  if (ch > -VMC_RC_CH_DEADBAND && ch < VMC_RC_CH_DEADBAND) {
    return 0;
  }
  return (int16_t)((int32_t)ch * max_current / RC_CHANNAL_ERROR_VALUE);
}

static double rc_channel_to_unit(int16_t ch)
{
  if (ch > -VMC_RC_CH_DEADBAND && ch < VMC_RC_CH_DEADBAND) {
    return 0.0;
  }
  return clamp_double((double)ch / (double)RC_CHANNAL_ERROR_VALUE, -1.0, 1.0);
}

static void vmc_body_motion_kf_reset(void)
{
  body_motion_kf_x[0] = 0.0;
  body_motion_kf_x[1] = 0.0;
  body_motion_kf_p[0] = 1.0;
  body_motion_kf_p[1] = 0.0;
  body_motion_kf_p[2] = 0.0;
  body_motion_kf_p[3] = 1.0;
  body_motion_kf_initialized = 1U;
  vmc_probe_state.motion.kf_x = 0.0;
  vmc_probe_state.motion.kf_dx = 0.0;
  vmc_probe_state.motion.kf_acc = 0.0;
  vmc_probe_state.motion.kf_wheel_measurement = 0.0;
  vmc_probe_state.motion.kf_accel_measurement = 0.0;
  vmc_probe_state.motion.kf_innovation_v = 0.0;
  vmc_probe_state.motion.kf_innovation_a = 0.0;
  vmc_probe_state.motion.kf_slip = 0.0;
  vmc_probe_state.motion.kf_slip_metric = 0.0;
  vmc_probe_state.motion.kf_slip_left = 0.0;
  vmc_probe_state.motion.kf_slip_right = 0.0;
  vmc_probe_state.motion.slip_raw_latched = 0.0;
  vmc_probe_state.motion.slip_raw_left = 0.0;
  vmc_probe_state.motion.slip_raw_right = 0.0;
  vmc_probe_state.motion.kf_wheel_left_raw = 0.0;
  vmc_probe_state.motion.kf_wheel_right_raw = 0.0;
}

static void vmc_body_motion_kf_update(double measured_dx, double measured_acc,
                                      uint8_t acceleration_valid,
                                      uint8_t slip_active)
{
  const double dt = vmc_loop_dt();
  double q_v = clamp_double((double)vmc_balance.kalman_q_v, 0.0, 1.0e12);
  double q_a = clamp_double((double)vmc_balance.kalman_q_a, 0.0, 1.0e12);
  double r_wheel = clamp_double((double)vmc_balance.kalman_r_wheel,
                                1.0e-9, 1.0e12);
  const double r_imu = clamp_double((double)vmc_balance.kalman_r_imu,
                                    1.0e-9, 1.0e12);
  double velocity_measurement = measured_dx;
  double x0;
  double x1;
  double p00;
  double p01;
  double p10;
  double p11;
  double y;
  double s;
  double k0;
  double k1;
  double hp0;
  double hp1;
  const double previous_velocity = body_motion_kf_x[0];

  if (body_motion_kf_initialized == 0U) {
    vmc_body_motion_kf_reset();
  }

  if (slip_active != 0U) {
    /* Reject the slipping wheel.  Feeding the filter its own velocity back
     * collapses the velocity innovation, and the widened weights let the
     * estimate coast on the inertial prediction.  Zeroing the innovation is
     * what actually rejects the measurement; the rescaling keeps the wheel
     * from regaining authority through the covariance. */
    q_v *= VMC_BODY_MOTION_KF_SLIP_Q_V_SCALE;
    q_a *= VMC_BODY_MOTION_KF_SLIP_Q_A_SCALE;
    r_wheel *= VMC_BODY_MOTION_KF_SLIP_R_WHEEL_SCALE;
    velocity_measurement = body_motion_kf_x[0];
  }

  /* Report the raw wheel measurement, not the gated one.  kf_slip says whether
   * it was rejected; keeping the raw value here is what makes the gate's
   * threshold verifiable from a recording. */
  vmc_probe_state.motion.kf_wheel_measurement = measured_dx;
  /* Report the measurement the estimator has available.  This is recomputed
   * every control cycle from the newest HI12 frame, so it must not be zeroed
   * on cycles where no new frame arrived: that would make the 50 Hz telemetry
   * under-report the accelerometer purely because of sampling phase. */
  vmc_probe_state.motion.kf_accel_measurement =
      (vmc_probe_state.body.gravity_sign != 0) ? measured_acc : 0.0;
  if (acceleration_valid != 0U) {
    vmc_probe_state.motion.kf_accel_update_count++;
  }

  x0 = previous_velocity + dt * body_motion_kf_x[1];
  x1 = body_motion_kf_x[1];
  p00 = body_motion_kf_p[0] + dt * (body_motion_kf_p[2] + body_motion_kf_p[1]) +
        dt * dt * body_motion_kf_p[3] + q_v;
  p01 = body_motion_kf_p[1] + dt * body_motion_kf_p[3];
  p10 = body_motion_kf_p[2] + dt * body_motion_kf_p[3];
  p11 = body_motion_kf_p[3] + q_a;

  y = velocity_measurement - x0;
  vmc_probe_state.motion.kf_innovation_v = y;
  s = p00 + r_wheel;
  k0 = p00 / s;
  k1 = p10 / s;
  hp0 = p00;
  hp1 = p01;
  x0 += k0 * y;
  x1 += k1 * y;
  p00 -= k0 * hp0;
  p01 -= k0 * hp1;
  p10 -= k1 * hp0;
  p11 -= k1 * hp1;

  if (acceleration_valid != 0U) {
    y = measured_acc - x1;
    vmc_probe_state.motion.kf_innovation_a = y;
    s = p11 + r_imu;
    k0 = p01 / s;
    k1 = p11 / s;
    hp0 = p10;
    hp1 = p11;
    x0 += k0 * y;
    x1 += k1 * y;
    p00 -= k0 * hp0;
    p01 -= k0 * hp1;
    p10 -= k1 * hp0;
    p11 -= k1 * hp1;
  }

  body_motion_kf_x[0] = x0;
  body_motion_kf_x[1] = x1;
  body_motion_kf_p[0] = p00;
  body_motion_kf_p[1] = p01;
  body_motion_kf_p[2] = p10;
  body_motion_kf_p[3] = p11;

  vmc_probe_state.motion.kf_dx = x0;
  vmc_probe_state.motion.kf_acc = x1;
  vmc_probe_state.motion.kf_x = clamp_double(
      vmc_probe_state.motion.kf_x + x0 * dt,
      -clamp_double(vmc_balance.x_limit, 0.05, VMC_BODY_MOTION_X_LIMIT_DEFAULT),
      clamp_double(vmc_balance.x_limit, 0.05, VMC_BODY_MOTION_X_LIMIT_DEFAULT));
}

static double joint_model_angle(float raw_position, float raw_reference,
                                float model_reference, float direction)
{
  return vmc_rad_format(model_reference + direction * (raw_position - raw_reference));
}

static double joint_model_speed(float raw_speed, float direction)
{
  return direction * raw_speed;
}

static uint16_t dm_float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
  const float span = x_max - x_min;
  const float max_int = (float)((1UL << bits) - 1UL);

  x = clamp_float(x, x_min, x_max);

  return (uint16_t)((x - x_min) * max_int / span);
}

static void update_wheel_angle(vmc_wheel_motor_t *motor)
{
  if ((float)(motor->ecd - motor->last_ecd) > (VMC_ANGLE_T / 2.0f)) {
    motor->round--;
  } else if ((float)(motor->ecd - motor->last_ecd) < -(VMC_ANGLE_T / 2.0f)) {
    motor->round++;
  }

  motor->angle = ((float)motor->round * VMC_ANGLE_T + (float)motor->ecd) /
                 VMC_ANGLE_T * 2.0f * VMC_PI / 268.0f * 17.0f;
}

static void parse_wheel(vmc_wheel_motor_t *motor, const uint8_t data[8])
{
  motor->last_ecd = motor->ecd;
  motor->temperate = data[6];
  motor->ecd = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  motor->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  motor->given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  /* C620/M3508 current command: +/-16384 corresponds to +/-20 A and
   * 0.3 N.m/A.  This is motor-shaft torque, matching the LQR input and the
   * command-side coefficient VMC_BALANCE_WHEEL_TORQUE_TO_CURRENT (2730.6666).
   * The LQR model already reflects the drivetrain relationship. */
  motor->torque = (float)motor->given_current / 16384.0f * 20.0f * 0.3f;
  motor->speed_rads = (float)motor->speed_rpm * VMC_PI / 30.0f / 268.0f * 17.0f;
  update_wheel_angle(motor);
  motor->last_angle = motor->angle;
  motor->last_update_ms = HAL_GetTick();
}

static void parse_joint(vmc_joint_motor_t *motor, const uint8_t data[8])
{
  const uint16_t position = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
  const uint16_t speed = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
  const uint16_t torque = (uint16_t)(((uint16_t)(data[4] & 0x0FU) << 8) | data[5]);

  motor->id = data[0] & 0x0FU;
  motor->status = (data[0] >> 4) & 0x0FU;
  motor->t_mos = data[6];
  motor->t_rotor = data[7];
  motor->position = dm_uint_to_float(position, VMC_DM_J8009P_PMIN, VMC_DM_J8009P_PMAX, 16);
  motor->speed = dm_uint_to_float(speed, VMC_DM_J8009_VMIN, VMC_DM_J8009_VMAX, 12);
  motor->torque = dm_uint_to_float(torque, VMC_DM_J8009_TMIN, VMC_DM_J8009_TMAX, 12);
  motor->last_update_ms = HAL_GetTick();
}

static void parse_can_frame(CAN_HandleTypeDef *hcan, uint32_t std_id, const uint8_t data[8])
{
  uint8_t handled = 1U;

  if (hcan->Instance == CAN1) {
    vmc_probe_state.can1_rx_count++;
    vmc_probe_state.last_can1_rx_id = std_id;
  } else if (hcan->Instance == CAN2) {
    vmc_probe_state.can2_rx_count++;
    vmc_probe_state.last_can2_rx_id = std_id;
  }

  switch (std_id) {
    case VMC_WHEEL_LEFT_ID:
      parse_wheel((vmc_wheel_motor_t *)&vmc_probe_state.wheel_left, data);
      break;
    case VMC_WHEEL_RIGHT_ID:
      parse_wheel((vmc_wheel_motor_t *)&vmc_probe_state.wheel_right, data);
      break;
    case VMC_JOINT_RIGHT_BACK_FEEDBACK_ID:
      parse_joint((vmc_joint_motor_t *)&vmc_probe_state.joint_right_back, data);
      break;
    case VMC_JOINT_RIGHT_AHEAD_FEEDBACK_ID:
      parse_joint((vmc_joint_motor_t *)&vmc_probe_state.joint_right_ahead, data);
      break;
    case VMC_JOINT_LEFT_AHEAD_FEEDBACK_ID:
      parse_joint((vmc_joint_motor_t *)&vmc_probe_state.joint_left_ahead, data);
      break;
    case VMC_JOINT_LEFT_BACK_FEEDBACK_ID:
      parse_joint((vmc_joint_motor_t *)&vmc_probe_state.joint_left_back, data);
      break;
    default:
      handled = 0U;
      break;
  }

  if (handled != 0U) {
    vmc_probe_state.can_rx_count++;
  } else {
    vmc_probe_state.unknown_rx_count++;
  }
}

static HAL_StatusTypeDef can_send_joint_mit(CAN_HandleTypeDef *hcan, uint32_t std_id,
                                            float position, float speed,
                                            float kp, float kd, float torque)
{
  CAN_TxHeaderTypeDef header;
  uint8_t data[8] = {0};
  uint32_t mailbox;
  const uint16_t p_uint = dm_float_to_uint(position, VMC_DM_J8009P_PMIN,
                                           VMC_DM_J8009P_PMAX, 16);
  const uint16_t v_uint = dm_float_to_uint(speed, VMC_DM_J8009_VMIN,
                                           VMC_DM_J8009_VMAX, 12);
  const uint16_t kp_uint = dm_float_to_uint(kp, 0.0f, 500.0f, 12);
  const uint16_t kd_uint = dm_float_to_uint(kd, 0.0f, 5.0f, 12);
  const uint16_t t_uint = dm_float_to_uint(torque, VMC_DM_J8009_TMIN,
                                           VMC_DM_J8009_TMAX, 12);

  header.StdId = std_id;
  header.ExtId = 0;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;
  header.TransmitGlobalTime = DISABLE;

  data[0] = (uint8_t)(p_uint >> 8);
  data[1] = (uint8_t)(p_uint & 0xFFU);
  data[2] = (uint8_t)(v_uint >> 4);
  data[3] = (uint8_t)(((v_uint & 0x0FU) << 4) | ((kp_uint >> 8) & 0x0FU));
  data[4] = (uint8_t)(kp_uint & 0xFFU);
  data[5] = (uint8_t)(kd_uint >> 4);
  data[6] = (uint8_t)(((kd_uint & 0x0FU) << 4) | ((t_uint >> 8) & 0x0FU));
  data[7] = (uint8_t)(t_uint & 0xFFU);

  return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

static HAL_StatusTypeDef can_send_joint_torque(CAN_HandleTypeDef *hcan, uint32_t std_id,
                                               double torque)
{
  return can_send_joint_mit(hcan, std_id, 0.0f, 0.0f, 0.0f, 0.0f, (float)torque);
}

static HAL_StatusTypeDef can_send_joint_special(CAN_HandleTypeDef *hcan, uint32_t std_id,
                                                uint8_t command)
{
  CAN_TxHeaderTypeDef header;
  uint8_t data[8] = {
      0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, command};
  uint32_t mailbox;

  header.StdId = std_id;
  header.ExtId = 0;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;
  header.TransmitGlobalTime = DISABLE;

  return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}

static HAL_StatusTypeDef can_send_joint_init(CAN_HandleTypeDef *hcan, uint32_t std_id)
{
  return can_send_joint_special(hcan, std_id, 0xFCU);
}

static HAL_StatusTypeDef can_send_joint_enable(CAN_HandleTypeDef *hcan, uint32_t std_id)
{
  return can_send_joint_special(hcan, std_id, 0xFBU);
}

static HAL_StatusTypeDef can_send_joint_disable(CAN_HandleTypeDef *hcan, uint32_t std_id)
{
  return can_send_joint_special(hcan, std_id, 0xFDU);
}

static HAL_StatusTypeDef can_send_wheel_current(int16_t left_current,
                                                int16_t right_current)
{
  CAN_TxHeaderTypeDef header;
  uint8_t data[8] = {0};
  uint32_t mailbox;

  header.StdId = VMC_WHEEL_CMD_ID;
  header.ExtId = 0;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;
  header.TransmitGlobalTime = DISABLE;

  data[0] = (uint8_t)((uint16_t)right_current >> 8);
  data[1] = (uint8_t)((uint16_t)right_current & 0xFFU);
  data[2] = (uint8_t)((uint16_t)left_current >> 8);
  data[3] = (uint8_t)((uint16_t)left_current & 0xFFU);

  return HAL_CAN_AddTxMessage(VMC_WHEEL_CMD_CAN, &header, data, &mailbox);
}

void vmc_can_init(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  filter.FilterBank = 0;
  (void)HAL_CAN_ConfigFilter(&hcan1, &filter);

  filter.FilterBank = 14;
  (void)HAL_CAN_ConfigFilter(&hcan2, &filter);

  (void)HAL_CAN_Start(&hcan1);
  (void)HAL_CAN_Start(&hcan2);
  (void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  (void)HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

static void update_tx_counters(HAL_StatusTypeDef status)
{
  if (status == HAL_OK) {
    vmc_probe_state.mit_zero_tx_count++;
  } else {
    vmc_probe_state.mit_zero_tx_fail_count++;
  }
}

static void update_init_tx_counters(HAL_StatusTypeDef status)
{
  if (status == HAL_OK) {
    vmc_probe_state.dm_init_tx_count++;
    vmc_probe_state.mit_enable_tx_count++;
  } else {
    vmc_probe_state.dm_init_tx_fail_count++;
    vmc_probe_state.mit_enable_tx_fail_count++;
  }
}

static void update_recover_tx_counters(HAL_StatusTypeDef status)
{
  if (status == HAL_OK) {
    vmc_probe_state.dm_recover_tx_count++;
  } else {
    vmc_probe_state.dm_recover_tx_fail_count++;
  }
}

static void update_disable_tx_counters(HAL_StatusTypeDef status)
{
  if (status == HAL_OK) {
    vmc_probe_state.dm_disable_tx_count++;
  } else {
    vmc_probe_state.dm_disable_tx_fail_count++;
  }
}

static uint8_t vmc_joint_state_is_any(uint8_t status)
{
  return (vmc_probe_state.joint_right_back.status == status ||
          vmc_probe_state.joint_right_ahead.status == status ||
          vmc_probe_state.joint_left_ahead.status == status ||
          vmc_probe_state.joint_left_back.status == status)
             ? 1U
             : 0U;
}

static uint8_t vmc_feedback_is_timeout(uint32_t now, uint32_t last_update_ms)
{
  return (last_update_ms == 0U || (now - last_update_ms) > VMC_FEEDBACK_TIMEOUT_MS)
             ? 1U
             : 0U;
}

static uint8_t vmc_feedback_timeout_source(void)
{
  const uint32_t now = HAL_GetTick();
  uint8_t source = VMC_FEEDBACK_TIMEOUT_NONE;

  if (vmc_feedback_is_timeout(now, vmc_probe_state.joint_right_back.last_update_ms) != 0U) {
    source |= VMC_FEEDBACK_TIMEOUT_RIGHT_BACK;
  }
  if (vmc_feedback_is_timeout(now, vmc_probe_state.joint_right_ahead.last_update_ms) != 0U) {
    source |= VMC_FEEDBACK_TIMEOUT_RIGHT_AHEAD;
  }
  if (vmc_feedback_is_timeout(now, vmc_probe_state.joint_left_ahead.last_update_ms) != 0U) {
    source |= VMC_FEEDBACK_TIMEOUT_LEFT_AHEAD;
  }
  if (vmc_feedback_is_timeout(now, vmc_probe_state.joint_left_back.last_update_ms) != 0U) {
    source |= VMC_FEEDBACK_TIMEOUT_LEFT_BACK;
  }
  if (vmc_feedback_is_timeout(now, vmc_probe_state.wheel_left.last_update_ms) != 0U) {
    source |= VMC_FEEDBACK_TIMEOUT_WHEEL_LEFT;
  }
  if (vmc_feedback_is_timeout(now, vmc_probe_state.wheel_right.last_update_ms) != 0U) {
    source |= VMC_FEEDBACK_TIMEOUT_WHEEL_RIGHT;
  }

  return source;
}

static void vmc_update_remote_target_length(void)
{
  const uint32_t now = HAL_GetTick();

  if (remote_control.valid == 0U || remote_control.rx_count == 0U ||
      (now - remote_control.last_update_ms) > RC_LOST_TIMEOUT_MS) {
    return;
  }

  if (vmc_control.stair_length_override_active != 0U) {
    /* Keep the stair pose at 18 cm for the entire enabled-switch interval.
     * This covers LIFT/RETRACT/SETTLE, the following get-up, and normal
     * standing after READY. */
    if (vmc_stair_is_active() != 0U ||
        remote_control_stair_enabled() != 0U) {
      vmc_control.target_length = (float)vmc_stair_target_length();
      return;
    }

    /* The physical stair switch is back in its disabled position. */
    vmc_control.stair_length_override_active = 0U;
  }

  vmc_control.target_length = (float)vmc_remote_leg_length_target();
}

static double vmc_remote_leg_length_target(void)
{
  const double position = clamp_double(
      (double)remote_control_leg_length_position(), -1.0, 1.0);
  return (double)VMC_LEG_KNOB_MIN_DEFAULT +
         (position + 1.0) * 0.5 *
             ((double)VMC_LEG_TARGET_LONG - (double)VMC_LEG_KNOB_MIN_DEFAULT);
}

static uint8_t vmc_remote_leg_length_gear(void)
{
  const double target = vmc_remote_leg_length_target();

  if (target + (double)VMC_LEG_GEAR_THRESHOLD_TOL >=
      (double)VMC_LEG_TARGET_LONG) {
    return VMC_LEG_GEAR_HIGH;
  }
  if (target + (double)VMC_LEG_GEAR_THRESHOLD_TOL >=
      (double)VMC_LEG_TARGET_MID) {
    return VMC_LEG_GEAR_MID;
  }
  return VMC_LEG_GEAR_LOW;
}

/* Every get-up trigger goes through this check before entering BRACE.  The
 * knob mapping alone is not enough in manual pre-positioning because CH9 is
 * relative there: both measured legs must also already be short. */
static uint8_t vmc_getup_safe_length_ready(void)
{
  const uint32_t now = HAL_GetTick();

  if (remote_control.valid == 0U || remote_control.rx_count == 0U ||
      remote_control_source != RC_SOURCE_SBUS ||
      (now - remote_control.last_update_ms) > RC_LOST_TIMEOUT_MS) {
    return 0U;
  }

  return (vmc_remote_leg_length_target() <=
              (double)VMC_GETUP_SAFE_LENGTH_MAX &&
          vmc_probe_state.left_leg.length <=
              (double)VMC_GETUP_SAFE_LENGTH_MAX &&
          vmc_probe_state.right_leg.length <=
              (double)VMC_GETUP_SAFE_LENGTH_MAX)
             ? 1U
             : 0U;
}

static uint8_t vmc_manual_prepose_exit_length_ready(void)
{
  return (vmc_probe_state.left_leg.length <=
              (double)VMC_GETUP_SAFE_LENGTH_MAX &&
          vmc_probe_state.right_leg.length <=
              (double)VMC_GETUP_SAFE_LENGTH_MAX)
             ? 1U
             : 0U;
}

static double vmc_step_towards(double current, double target, double step)
{
  if (current < target) {
    current += step;
    return (current > target) ? target : current;
  }
  if (current > target) {
    current -= step;
    return (current < target) ? target : current;
  }
  return current;
}

static double vmc_getup_phi0_feedforward_slew(double previous,
                                              double target,
                                              double rise_rate,
                                              double fall_rate)
{
  const uint8_t falling =
      (fabs(target) < fabs(previous) || previous * target < 0.0) ? 1U : 0U;
  const double rate =
      falling != 0U ? clamp_double(fall_rate, 0.0, 10000.0)
                    : clamp_double(rise_rate, 0.0, 10000.0);

  return vmc_step_towards(previous, target, rate * vmc_loop_dt());
}

static float vmc_average_joint_torque_abs(void)
{
  return (fabsf(vmc_probe_state.joint_left_ahead.torque) +
          fabsf(vmc_probe_state.joint_left_back.torque) +
          fabsf(vmc_probe_state.joint_right_ahead.torque) +
          fabsf(vmc_probe_state.joint_right_back.torque)) *
         0.25f;
}

/* Weakest and strongest leg load.  The airborne latch asks whether *every*
 * leg is unloaded, so the average is not enough: one slipping side can drag
 * the mean below the threshold while the other side still carries the body.
 * "All four unloaded" is max <= threshold, and max also says whether the body
 * has been picked up again (release). */
static void vmc_joint_torque_abs_extremes(float *minimum, float *maximum)
{
  float values[4];
  float lo;
  float hi;
  uint8_t i;

  values[0] = fabsf(vmc_probe_state.joint_left_ahead.torque);
  values[1] = fabsf(vmc_probe_state.joint_left_back.torque);
  values[2] = fabsf(vmc_probe_state.joint_right_ahead.torque);
  values[3] = fabsf(vmc_probe_state.joint_right_back.torque);
  lo = values[0];
  hi = values[0];
  for (i = 1U; i < 4U; i++) {
    if (values[i] < lo) {
      lo = values[i];
    }
    if (values[i] > hi) {
      hi = values[i];
    }
  }
  if (minimum != 0) {
    *minimum = lo;
  }
  if (maximum != 0) {
    *maximum = hi;
  }
}

static uint8_t vmc_jump_is_active(void)
{
  return (vmc_control.jump_stage == VMC_JUMP_STAGE_COMPRESS ||
          vmc_control.jump_stage == VMC_JUMP_STAGE_EXTEND ||
          vmc_control.jump_stage == VMC_JUMP_STAGE_RETRACT ||
          vmc_control.jump_stage == VMC_JUMP_STAGE_LAND)
             ? 1U
             : 0U;
}

/* True for every stage of a jump, plus the aborted-while-still-selected case.
 *
 * The leg torque path and the airborne detector gate on THIS: while a jump
 * runs the legs are driven by the jump length profile, and the detector must
 * stay out of the way rather than reacting to the leg motion. */
static uint8_t vmc_jump_is_active_or_aborted(void)
{
  if (vmc_jump_is_active() != 0U) {
    return 1U;
  }
  return (vmc_control.jump_stage == VMC_JUMP_STAGE_ABORT &&
          remote_control.valid != 0U &&
          remote_control_jump_mode() != RC_JUMP_DISABLED)
             ? 1U
             : 0U;
}

/* Wheel blanking, which is deliberately NARROWER than the predicate above.
 *
 * Only the airborne part of a jump (RETRACT/LAND) cuts the wheels.  Cutting
 * them for the whole jump - as the original code did - also kills the balance
 * loop for the 300 ms of COMPRESS and EXTEND while the robot is still on the
 * ground, so a jump started at any speed tips the machine over before it can
 * ever leave the ground.  That is what made jumping stationary-only.  Keeping
 * the wheels alive through the on-ground phases is also strictly better for a
 * standing jump: the robot stays balanced during the crouch.
 *
 * VMC_JUMP_MOVING_ENABLE_DEFAULT = 0 restores the old whole-jump blanking.
 *
 * NOTE the aborted-while-still-selected latch deliberately does NOT appear here, in
 * either mode.  It used to, and the effect was that an aborted jump left the
 * wheels dead until the pilot moved the jump switch back through disabled: the balance loop
 * never came back, so the machine simply fell over on landing.  A jump aborts
 * for ordinary reasons - the landing pushes |pitch| past the attitude limit,
 * or one CAN frame is missed - so this happened intermittently and looked like
 * the robot "sometimes not restarting its wheels".  The latch still gates the
 * leg target and the re-arm, which is all it should do; a balancing robot must
 * not have its wheels held off while the operator is mid-manoeuvre.  Real
 * flight is covered independently by vmc_probe_state.airborne_active. */
static uint8_t vmc_jump_blocks_wheel_output(void)
{
  if (vmc_control.jump_moving_enable == 0U) {
    return vmc_jump_is_active();
  }
  return vmc_jump_joint_lqr_active();
}

static uint8_t vmc_jump_joint_lqr_active(void)
{
  /* After the kick, match the airborne detector: wheels stay blocked while
   * LQR Tp holds the virtual leg angle against world vertical. */
  return (vmc_control.jump_stage == VMC_JUMP_STAGE_RETRACT ||
          vmc_control.jump_stage == VMC_JUMP_STAGE_LAND)
             ? 1U
             : 0U;
}

static uint8_t vmc_jump_overrides_target(void)
{
  if (vmc_jump_is_active() != 0U) {
    return 1U;
  }
  return (vmc_control.jump_stage == VMC_JUMP_STAGE_ABORT &&
          remote_control.valid != 0U &&
          remote_control_jump_mode() != RC_JUMP_DISABLED)
             ? 1U
             : 0U;
}

static void vmc_clear_airborne_latch(void)
{
  uint8_t side;

  vmc_probe_state.airborne_active = 0U;
  vmc_probe_state.airborne_candidate = 0U;
  vmc_probe_state.airborne_candidate_ms = 0U;
  vmc_probe_state.airborne_release_candidate_ms = 0U;
  vmc_probe_state.airborne_pitch_reference_valid = 0U;
  for (side = 0U; side < VMC_AIRBORNE_SIDE_COUNT; side++) {
    vmc_probe_state.airborne_side_active[side] = 0U;
    vmc_probe_state.airborne_side_candidate[side] = 0U;
    vmc_probe_state.airborne_side_candidate_ms[side] = 0U;
    vmc_probe_state.airborne_side_release_candidate_ms[side] = 0U;
    vmc_probe_state.airborne_side_fn_filtered[side] = 0.0f;
    vmc_probe_state.airborne_side_fn_filter_initialized[side] = 0U;
    vmc_probe_state.airborne_side_posture_active[side] = 0U;
    vmc_probe_state.airborne_side_posture_ms[side] = 0U;
    vmc_probe_state.airborne_side_length_returning[side] = 0U;
  }
}

static double vmc_jump_stage_target_length(void)
{
  switch (vmc_control.jump_stage) {
    case VMC_JUMP_STAGE_COMPRESS:
      return clamp_double(vmc_control.jump_compress_length,
                          VMC_LEG_LENGTH_MIN,
                          VMC_LEG_LENGTH_MAX);
    case VMC_JUMP_STAGE_EXTEND:
      return clamp_double(vmc_control.jump_extend_length,
                          VMC_LEG_LENGTH_MIN,
                          VMC_LEG_LENGTH_MAX);
    case VMC_JUMP_STAGE_RETRACT:
      return clamp_double(vmc_control.jump_retract_length,
                          VMC_LEG_LENGTH_MIN,
                          VMC_LEG_LENGTH_MAX);
    case VMC_JUMP_STAGE_LAND:
    case VMC_JUMP_STAGE_FINISHED:
    case VMC_JUMP_STAGE_ABORT:
      return clamp_double(vmc_control.jump_land_length,
                          VMC_LEG_LENGTH_MIN,
                          VMC_LEG_LENGTH_MAX);
    default:
      return clamp_double(vmc_control.target_length,
                          VMC_LEG_LENGTH_MIN,
                          VMC_LEG_LENGTH_MAX);
  }
}

static void vmc_jump_enter_stage(uint8_t stage, uint32_t now)
{
  vmc_control.jump_stage = stage;
  vmc_control.jump_stage_start_ms = now;
  vmc_control.jump_stage_ms = 0U;
  if (stage != VMC_JUMP_STAGE_LAND) {
    vmc_control.jump_land_contact_ms = 0U;
  }
  vmc_control.jump_target_length = (float)vmc_jump_stage_target_length();
}

static void vmc_jump_finish(uint8_t stage, uint8_t reason, uint32_t now)
{
  if (stage == VMC_JUMP_STAGE_ABORT) {
    vmc_control.jump_abort_count++;
    vmc_control.jump_abort_reason = reason;
  } else {
    vmc_control.jump_abort_reason = VMC_JUMP_ABORT_NONE;
  }
  vmc_control.jump_armed = 0U;
  vmc_jump_auto_hold_ms = 0U;
  vmc_control.jump_last_finish_ms = now;
  vmc_control.jump_land_contact_ms = 0U;
  /* Drop the airborne latch the jump itself asserted.
   *
   * While RETRACT/LAND run, vmc_jump_joint_lqr_active() forces airborne_active
   * to 1 to keep the wheels quiet, which means that flag is an artefact of the
   * jump rather than a measurement.  Only the "landing confirmed by joint
   * torque" path used to clear it; when the LAND phase instead expired on its
   * timeout the latch stayed set and the wheels stayed cut until the normal
   * detector released it - which needs 500 ms of sustained leg load and never
   * happens if the robot lands softly.  Clear it for every exit so the
   * detector starts from a clean state and can decide for itself. */
  vmc_clear_airborne_latch();
  vmc_jump_enter_stage(stage, now);
}

static uint8_t vmc_jump_start_is_allowed(void)
{
  const float attitude_limit =
      clamp_float(vmc_control.jump_attitude_limit, 0.05f, 1.00f);
  const uint8_t imu_ready =
      (vmc_probe_state.body.imu_valid != 0U &&
       vmc_probe_state.body.imu_connected != 0U)
          ? 1U
          : 0U;

  return (vmc_control.jump_enable != 0U &&
          vmc_control.enabled != 0U &&
          vmc_control.manual_prepose_active == 0U &&
          vmc_control.manual_prepose_exit_length_check_active == 0U &&
          imu_ready != 0U &&
          vmc_probe_state.feedback_timeout_active == 0U &&
          vmc_getup_is_active() == 0U &&
          vmc_fall_rearm_required == 0U &&
          fabsf(vmc_probe_state.body.pitch) <= attitude_limit &&
          fabsf(vmc_probe_state.body.roll) <= attitude_limit)
             ? 1U
             : 0U;
}

static uint8_t vmc_jump_abort_reason(void)
{
  const uint8_t imu_ready =
      (vmc_probe_state.body.imu_valid != 0U &&
       vmc_probe_state.body.imu_connected != 0U)
          ? 1U
          : 0U;

  if (vmc_control.jump_enable == 0U ||
      remote_control.valid == 0U || remote_control.rx_count == 0U ||
      (HAL_GetTick() - remote_control.last_update_ms) > RC_LOST_TIMEOUT_MS ||
      remote_control_allows_joint_enable() == 0U) {
    return VMC_JUMP_ABORT_REMOTE;
  }
  if (vmc_probe_state.feedback_timeout_active != 0U) {
    return VMC_JUMP_ABORT_FEEDBACK;
  }
  if (vmc_getup_is_active() != 0U || vmc_fall_rearm_required != 0U) {
    return VMC_JUMP_ABORT_GETUP;
  }
  /* Attitude is deliberately NOT an abort condition once the jump is running.
   *
   * It is still required before the jump starts (vmc_jump_start_is_allowed),
   * which is enough to refuse a launch from a bad pose.  During and after the
   * jump it has to be dropped: landing pitches the body hard - measured
   * transients of 0.46 rad (26 deg) against the 0.35 rad limit - and that is
   * ordinary landing nod, not loss of control.  Aborting on it marked roughly
   * half of otherwise-good jumps as failures and disturbed the stage timing of
   * the next one (a following jump launched with only 57 mm of push instead of
   * 145-176 mm).
   *
   * IMU health is still checked: without a valid attitude the controller
   * cannot balance at all, so losing it must stop the jump regardless. */
  if (imu_ready == 0U) {
    return VMC_JUMP_ABORT_ATTITUDE;
  }
  return VMC_JUMP_ABORT_NONE;
}

/* Automatic jump gate: are the two measurable conditions satisfied?
 *
 * Channel 6 mode is checked by the caller, because it also controls manual
 * jumps and the disabled state.
 *
 * Speed is the FUSED body velocity rather than the stick demand: the operator
 * asked for the robot to be actually travelling at 1 m/s, not for the stick to
 * be asking for it.  kf_dx is signed by travel direction, so the magnitude is
 * what matters.
 *
 * The laser has to be fresh.  tf02_connected is cleared after 100 ms without a
 * frame, and a stale echo must never arm a jump - tf02_distance_cm would
 * otherwise hold whatever the last valid reading happened to be.  The window
 * is exclusive at both ends, matching "小于 100 大于 80". */
static uint8_t vmc_jump_auto_ready(void)
{
  const double speed = fabs(vmc_probe_state.motion.kf_dx);
  const double distance_cm = (double)tf02_distance_cm;
  /* Read from vmc_control rather than the macros: these are runtime tunable
   * over RTT so the gate can be re-tuned from the web UI without a reflash. */
  const double speed_min = (double)vmc_control.jump_auto_speed_min;
  const double dist_min = (double)vmc_control.jump_auto_dist_min_cm;
  const double dist_max = (double)vmc_control.jump_auto_dist_max_cm;

  if (tf02_connected == 0U) {
    return 0U;
  }
  if (speed < speed_min) {
    return 0U;
  }
  if (distance_cm <= dist_min || distance_cm >= dist_max) {
    return 0U;
  }
  return 1U;
}

static void vmc_jump_update(void)
{
  const uint32_t now = HAL_GetTick();
  uint32_t dt_ms = 0U;
  const uint8_t jump_mode = remote_control_jump_mode();
  const uint32_t cooldown_ms =
      clamp_u32(vmc_control.jump_cooldown_ms, 0U, 5000U);
  const uint32_t total_timeout_ms =
      clamp_u32(vmc_control.jump_total_timeout_ms, 100U, 5000U);

  if (vmc_jump_last_update_ms != 0U) {
    dt_ms = now - vmc_jump_last_update_ms;
    if (dt_ms > 100U) {
      dt_ms = 100U;
    }
  }
  vmc_jump_last_update_ms = now;

  if (vmc_control.manual_prepose_active != 0U ||
      vmc_control.manual_prepose_exit_length_check_active != 0U) {
    vmc_control.jump_armed = 0U;
    vmc_jump_auto_hold_ms = 0U;
    if (vmc_jump_is_active() != 0U) {
      vmc_jump_finish(VMC_JUMP_STAGE_ABORT, VMC_JUMP_ABORT_REMOTE, now);
    }
    return;
  }

  if (vmc_control.jump_stage_start_ms != 0U) {
    vmc_control.jump_stage_ms = now - vmc_control.jump_stage_start_ms;
  } else {
    vmc_control.jump_stage_ms = 0U;
  }
  vmc_control.jump_target_length = (float)vmc_jump_stage_target_length();

  if (vmc_control.jump_stage == VMC_JUMP_STAGE_IDLE) {
    const uint8_t cooled_down =
        ((now - vmc_control.jump_last_finish_ms) >= cooldown_ms) ? 1U : 0U;
    const uint8_t auto_now =
        (cooled_down != 0U && jump_mode == RC_JUMP_AUTO &&
         vmc_jump_auto_ready() != 0U)
            ? 1U
            : 0U;
    const uint8_t manual_now =
        (cooled_down != 0U && jump_mode == RC_JUMP_MANUAL &&
         vmc_jump_manual_latched == 0U)
            ? 1U
            : 0U;

    if (jump_mode != RC_JUMP_MANUAL) {
      vmc_jump_manual_latched = 0U;
    }

    vmc_control.jump_abort_reason = VMC_JUMP_ABORT_NONE;

    /* Require the conditions to hold continuously for a short window: the
     * laser is one echo per frame and the speed estimate can spike on a single
     * sample, so no single frame may launch a jump.
     *
     * jump_armed is kept as the telemetry-visible "about to fire" indication:
     * 1 means auto mode is selected, the robot is at speed, the laser is in the
     * window and the confirmation has elapsed. */
    if (auto_now != 0U) {
      if (vmc_jump_auto_hold_ms < vmc_control.jump_auto_confirm_ms) {
        vmc_jump_auto_hold_ms += dt_ms;
      }
    } else {
      vmc_jump_auto_hold_ms = 0U;
    }
    vmc_control.jump_armed =
        (auto_now != 0U && vmc_jump_auto_hold_ms >= vmc_control.jump_auto_confirm_ms)
            ? 1U
            : 0U;

    if ((vmc_control.jump_armed != 0U || manual_now != 0U) &&
        vmc_jump_start_is_allowed() != 0U) {
      vmc_control.jump_trigger_count++;
      vmc_control.jump_start_ms = now;
      vmc_jump_auto_hold_ms = 0U;
      if (manual_now != 0U) {
        vmc_jump_manual_latched = 1U;
      }
      vmc_jump_enter_stage(VMC_JUMP_STAGE_COMPRESS, now);
    }
    return;
  }

  if (vmc_control.jump_stage == VMC_JUMP_STAGE_FINISHED ||
      vmc_control.jump_stage == VMC_JUMP_STAGE_ABORT) {
    const uint8_t cooled_down =
        ((now - vmc_control.jump_last_finish_ms) >= cooldown_ms) ? 1U : 0U;

    vmc_control.jump_target_length = (float)vmc_jump_stage_target_length();
    if ((jump_mode == RC_JUMP_DISABLED || jump_mode == RC_JUMP_MANUAL) &&
        cooled_down != 0U) {
      vmc_control.jump_stage = VMC_JUMP_STAGE_IDLE;
      vmc_control.jump_stage_start_ms = 0U;
      vmc_control.jump_stage_ms = 0U;
      vmc_control.jump_abort_reason = VMC_JUMP_ABORT_NONE;
    }
    /* Not armed here: with the automatic gate the jump fires from IDLE as soon
     * as the three conditions hold, so "armed" is computed there. */
    vmc_control.jump_armed = 0U;
    vmc_jump_auto_hold_ms = 0U;
    return;
  }

  {
    const uint8_t abort_reason = vmc_jump_abort_reason();
    const double leg_length_avg =
        (vmc_probe_state.left_leg.length + vmc_probe_state.right_leg.length) *
        0.5;
    const double ready_tol = VMC_JUMP_LENGTH_READY_TOL;

    if (abort_reason != VMC_JUMP_ABORT_NONE) {
      vmc_jump_finish(VMC_JUMP_STAGE_ABORT, abort_reason, now);
      return;
    }
    if ((now - vmc_control.jump_start_ms) >= total_timeout_ms) {
      vmc_jump_finish(VMC_JUMP_STAGE_ABORT, VMC_JUMP_ABORT_TIMEOUT, now);
      return;
    }

    switch (vmc_control.jump_stage) {
      case VMC_JUMP_STAGE_COMPRESS:
        if (vmc_control.jump_stage_ms >=
                clamp_u32(vmc_control.jump_compress_timeout_ms, 1U, 2000U) ||
            leg_length_avg <=
                ((double)clamp_float(vmc_control.jump_compress_length,
                                     VMC_LEG_LENGTH_MIN,
                                     VMC_LEG_LENGTH_MAX) +
                 ready_tol)) {
          vmc_jump_enter_stage(VMC_JUMP_STAGE_EXTEND, now);
        }
        break;
      case VMC_JUMP_STAGE_EXTEND:
        if (vmc_control.jump_stage_ms >=
                clamp_u32(vmc_control.jump_extend_timeout_ms, 1U, 2000U) ||
            leg_length_avg >=
                ((double)clamp_float(vmc_control.jump_extend_length,
                                     VMC_LEG_LENGTH_MIN,
                                     VMC_LEG_LENGTH_MAX) -
                 ready_tol)) {
          vmc_jump_enter_stage(VMC_JUMP_STAGE_RETRACT, now);
        }
        break;
      case VMC_JUMP_STAGE_RETRACT:
        if (vmc_control.jump_stage_ms >=
                clamp_u32(vmc_control.jump_retract_timeout_ms, 1U, 2000U) ||
            leg_length_avg <=
                ((double)clamp_float(vmc_control.jump_retract_length,
                                     VMC_LEG_LENGTH_MIN,
                                     VMC_LEG_LENGTH_MAX) +
                 ready_tol)) {
          vmc_jump_enter_stage(VMC_JUMP_STAGE_LAND, now);
        }
        break;
      case VMC_JUMP_STAGE_LAND: {
        const float land_torque_threshold =
            clamp_float(vmc_control.jump_land_torque_threshold, 0.1f, 54.0f);
        const uint32_t land_confirm_ms =
            clamp_u32(vmc_control.jump_land_confirm_ms, 0U, 1000U);

        if (vmc_average_joint_torque_abs() >= land_torque_threshold) {
          if (vmc_control.jump_land_contact_ms < land_confirm_ms) {
            vmc_control.jump_land_contact_ms += dt_ms;
          }
        } else {
          vmc_control.jump_land_contact_ms = 0U;
        }

        if (vmc_control.jump_stage_ms >=
                clamp_u32(vmc_control.jump_land_timeout_ms, 1U, 3000U) ||
            vmc_control.jump_land_contact_ms >= land_confirm_ms) {
          /* vmc_jump_finish() clears the airborne latch for every exit. */
          vmc_jump_finish(VMC_JUMP_STAGE_FINISHED,
                          VMC_JUMP_ABORT_NONE,
                          now);
        }
        break;
      }
      default:
        vmc_jump_finish(VMC_JUMP_STAGE_ABORT, VMC_JUMP_ABORT_TIMEOUT, now);
        break;
    }
  }
}

static uint8_t vmc_stair_is_active(void)
{
  return (vmc_control.stair_stage == VMC_STAIR_STAGE_LIFT ||
          vmc_control.stair_stage == VMC_STAIR_STAGE_RETRACT ||
          vmc_control.stair_stage == VMC_STAIR_STAGE_SETTLE ||
          vmc_control.stair_stage == VMC_STAIR_STAGE_RECOVER)
             ? 1U
             : 0U;
}

static uint8_t vmc_stair_overrides_target(void)
{
  return vmc_stair_is_active();
}

static uint8_t vmc_stair_blocks_wheel_output(void)
{
  return vmc_stair_is_active();
}

static double vmc_stair_release_scale(void)
{
  if (vmc_control.stair_stage != VMC_STAIR_STAGE_FINISHED ||
      vmc_control.stair_last_finish_ms == 0U) {
    return 1.0;
  }
  return clamp_double(
      (double)(HAL_GetTick() - vmc_control.stair_last_finish_ms) /
          (double)VMC_STAIR_RELEASE_RAMP_MS_DEFAULT,
      0.0,
      1.0);
}

static double vmc_stair_target_length(void)
{
  const double target =
      (vmc_control.stair_stage == VMC_STAIR_STAGE_LIFT)
          ? (double)vmc_control.stair_lift_length
          : (double)vmc_control.stair_retract_length;

  const double target_min =
      (vmc_control.stair_stage == VMC_STAIR_STAGE_LIFT)
          ? (double)VMC_LEG_LENGTH_MIN
          : (double)VMC_STAIR_RETRACT_LENGTH_MIN;

  return clamp_double(target,
                      target_min,
                      VMC_LEG_LENGTH_MAX);
}

static uint8_t vmc_stair_start_is_allowed(void)
{
  return (vmc_control.stair_enable != 0U &&
          vmc_control.enabled != 0U &&
          vmc_control.manual_prepose_active == 0U &&
          vmc_control.manual_prepose_exit_length_check_active == 0U &&
          vmc_balance.wheel_output_enable != 0U &&
          vmc_fall_rearm_required == 0U &&
          vmc_probe_state.feedback_timeout_active == 0U &&
          vmc_probe_state.airborne_active == 0U &&
          vmc_getup_is_active() == 0U &&
          vmc_jump_is_active_or_aborted() == 0U &&
          remote_control_stair_enabled() != 0U &&
          vmc_remote_leg_length_gear() == VMC_LEG_GEAR_HIGH &&
          vmc_probe_state.body.imu_valid != 0U &&
          vmc_probe_state.body.imu_connected != 0U &&
          remote_control_allows_joint_enable() != 0U)
             ? 1U
             : 0U;
}

static uint8_t vmc_stair_pose_ready(double target_length,
                                    double target_theta)
{
  const double length_tol =
      clamp_double(vmc_control.stair_pose_length_tol, 0.005, 0.15);
  const double theta_tol =
      clamp_double(vmc_control.stair_pose_theta_tol, 0.05, 0.60);

  return (fabs((double)vmc_probe_state.left_leg.length - target_length) <=
              length_tol &&
          fabs((double)vmc_probe_state.right_leg.length - target_length) <=
              length_tol &&
          fabs((double)vmc_rad_format((float)(
              vmc_probe_state.motion.left_theta - target_theta))) <=
              theta_tol &&
          fabs((double)vmc_rad_format((float)(
              vmc_probe_state.motion.right_theta - target_theta))) <=
              theta_tol)
             ? 1U
             : 0U;
}

static uint8_t vmc_stair_abort_reason(void)
{
  if (vmc_control.stair_enable == 0U ||
      remote_control_allows_joint_enable() == 0U) {
    return VMC_STAIR_ABORT_REMOTE;
  }
  if (vmc_probe_state.feedback_timeout_active != 0U) {
    return VMC_STAIR_ABORT_FEEDBACK;
  }
  if (vmc_getup_is_active() != 0U || vmc_fall_rearm_required != 0U) {
    return VMC_STAIR_ABORT_GETUP;
  }
  if (vmc_jump_is_active_or_aborted() != 0U) {
    return VMC_STAIR_ABORT_JUMP;
  }
  if (fabs((double)vmc_probe_state.body.pitch) >
          (double)VMC_GETUP_START_PITCH_RAD ||
      fabs((double)vmc_probe_state.body.roll) >
          (double)VMC_GETUP_BODY_FAN_START_ROLL_RAD) {
    return VMC_STAIR_ABORT_ATTITUDE;
  }
  return VMC_STAIR_ABORT_NONE;
}

static void vmc_stair_enter_stage(uint8_t stage, uint32_t now)
{
  vmc_control.stair_stage = stage;
  vmc_control.stair_stage_start_ms = now;
  vmc_control.stair_stage_ms = 0U;
  vmc_control.stair_pose_confirm_elapsed_ms = 0U;
  if (stage == VMC_STAIR_STAGE_LIFT) {
    vmc_control.stair_start_ms = now;
    vmc_control.stair_theta_ramped = vmc_balance.pitch_offset;
    vmc_control.stair_phi0_ramped =
        (float)((vmc_probe_state.left_leg.phi0 +
                 vmc_probe_state.right_leg.phi0) * 0.5);
    vmc_control.stair_length_override_active = 1U;
    vmc_control.target_length = (float)vmc_stair_target_length();
  }
}

static void vmc_stair_reset_detector(void)
{
  vmc_stair_preimpact_armed = 0U;
  vmc_stair_preimpact_speed_ref = 0.0;
  vmc_control.stair_speed_ms = 0U;
  vmc_control.stair_preimpact_ms = 0U;
  vmc_control.stair_impact_ms = 0U;
  vmc_control.stair_preimpact_armed = 0U;
  vmc_control.stair_left_impact = 0U;
  vmc_control.stair_right_impact = 0U;
}

static void vmc_stair_finish(uint8_t stage, uint8_t reason, uint32_t now)
{
  if (stage == VMC_STAIR_STAGE_ABORT) {
    vmc_control.stair_abort_count++;
    vmc_control.stair_abort_reason = reason;
  } else {
    vmc_control.stair_abort_reason = VMC_STAIR_ABORT_NONE;
  }
  vmc_control.stair_last_finish_ms = now;
  vmc_stair_auto_armed = 0U;
  vmc_stair_reset_detector();
  vmc_control.stair_ready_ms = 0U;
  vmc_control.stair_ready_detected = 0U;
  vmc_control.stair_impact_detected = 0U;
  vmc_stair_enter_stage(stage, now);
}

/* A stair finish is a controller handoff, not a return to the old balance
 * controller.  Drop every position/velocity integral that was accumulated
 * against the pre-step ground frame, then let get-up own the legs until it has
 * rebuilt a quiet stand pose. */
static void vmc_stair_reset_balance_for_getup(void)
{
  vmc_body_motion_kf_reset();
  body_motion_kf_slip_initialized = 0U;
  vmc_speed_dx_prev = 0.0;
  vmc_speed_accel_lpf = 0.0;
  vmc_speed_accel_initialized = 0U;
  vmc_dpitch_lpf = 0.0;
  vmc_lqr_d_theta_lpf = 0.0;
  vmc_lqr_left_d_theta_lpf = 0.0;
  vmc_lqr_right_d_theta_lpf = 0.0;

  vmc_balance.wheel_output_enable = 0U;
  vmc_balance.safety_allows_output = 0U;
  vmc_balance.x_offset = 0.0f;
  vmc_balance.speed_target = 0.0f;
  vmc_balance.speed_target_ramped = 0.0f;
  vmc_balance.speed_stop_hold_active = 0U;
  vmc_balance.speed_pitch_offset = 0.0f;
  vmc_balance.disturbance_observer_output = 0.0f;
  vmc_balance.disturbance_observer_residual = 0.0f;
  vmc_balance.disturbance_observer_neutral_time = 0.0f;
  vmc_balance.disturbance_observer_active = 0U;
  vmc_balance.yaw_target = vmc_probe_state.body.yaw;
  vmc_balance.yaw_hold_active = 0U;
  vmc_balance.gimbal_follow_active = 0U;
  vmc_control.roll_target = 0.0f;
  vmc_balance.roll_remote_input = 0.0f;
  vmc_balance.roll_remote_active = 0U;
  vmc_wheel_control.yaw_hold_current = 0;
  vmc_wheel_control.gimbal_follow_current = 0;
  vmc_wheel_control.left_current = 0;
  vmc_wheel_control.right_current = 0;
}

static void vmc_stair_handoff_to_getup(uint32_t now)
{
  const int8_t turn_direction =
      (vmc_probe_state.body.pitch < 0.0) ? 1 : -1;

  if (vmc_getup_safe_length_ready() == 0U) {
    return;
  }
  vmc_stair_finish(VMC_STAIR_STAGE_FINISHED, VMC_STAIR_ABORT_NONE, now);
  vmc_stair_reset_balance_for_getup();
  /* Reset once more as get-up reaches READY.  The KF necessarily runs while
   * the legs retract and swing upright, so its handoff-time zero is not the
   * horizontal position that should be held after the chassis is standing. */
  vmc_getup_reset_balance_on_ready = 1U;
  (void)vmc_getup_enter_brace(turn_direction);
}

static void vmc_stair_update(void)
{
  const uint32_t now = HAL_GetTick();
  const uint32_t cooldown_ms =
      clamp_u32(vmc_control.stair_cooldown_ms, 0U, 10000U);
  const uint8_t remote_frame_ready =
      (remote_control.valid != 0U &&
       remote_control.rx_count != 0U &&
       remote_control_source == RC_SOURCE_SBUS &&
       (now - remote_control.last_update_ms) <= RC_LOST_TIMEOUT_MS)
          ? 1U
          : 0U;
  const uint8_t stair_switch_enabled =
      (remote_frame_ready != 0U &&
       remote_control_channel(RC_STAIR_ENABLE) > RC_SWITCH_ON_THRESHOLD)
          ? 1U
          : 0U;
  const uint32_t ready_confirm_ms =
      clamp_u32(vmc_control.stair_ready_confirm_ms, 1U, 5000U);
  const uint32_t speed_confirm_ms =
      clamp_u32(vmc_control.stair_speed_confirm_ms, 1U, 1000U);
  const uint32_t preimpact_confirm_ms =
      clamp_u32(vmc_control.stair_preimpact_confirm_ms, 1U, 1000U);
  const uint32_t impact_confirm_ms =
      clamp_u32(vmc_control.stair_impact_confirm_ms, 1U, 200U);
  const uint32_t dt_ms =
      (uint32_t)(vmc_loop_dt() * 1000.0 + 0.5);
  const double ready_length =
      clamp_double(vmc_control.stair_ready_length,
                   VMC_LEG_LENGTH_MIN,
                   VMC_LEG_LENGTH_MAX);
  const float impact_torque_threshold =
      clamp_float(vmc_control.stair_impact_torque_threshold, 0.0f, 54.0f);
  const float impact_theta_threshold =
      clamp_float(vmc_control.stair_impact_theta_threshold, 0.0f, 3.15f);
  const float speed_threshold =
      clamp_float(vmc_control.stair_speed_threshold, 0.05f, 5.0f);
  const float command_speed_threshold =
      clamp_float(vmc_control.stair_command_speed_threshold, 0.0f, 3.0f);
  const float impact_speed_drop_threshold =
      clamp_float(vmc_control.stair_impact_diff_threshold, 0.05f, 10.0f);
  const double wheel_speed_abs =
      fabs(vmc_probe_state.motion.wheel_dx);
  const uint8_t drive_command =
      (fabs(vmc_balance.speed_target) >= command_speed_threshold) ? 1U : 0U;
  const uint8_t speed_now =
      (drive_command != 0U &&
       wheel_speed_abs >= (double)speed_threshold)
          ? 1U
          : 0U;
  const uint8_t high_gear =
      (vmc_remote_leg_length_gear() == VMC_LEG_GEAR_HIGH) ? 1U : 0U;
  const uint8_t ready_now =
      (vmc_probe_state.left_leg.length >= ready_length &&
       vmc_probe_state.right_leg.length >= ready_length)
          ? 1U
          : 0U;
  uint8_t left_leg_changed = 0U;
  uint8_t right_leg_changed = 0U;
  uint8_t impact_now;
  uint8_t start_allowed;

  if (vmc_control.manual_prepose_active != 0U ||
      vmc_control.manual_prepose_exit_length_check_active != 0U) {
    if (vmc_stair_is_active() != 0U) {
      vmc_stair_finish(VMC_STAIR_STAGE_ABORT, VMC_STAIR_ABORT_REMOTE, now);
    }
    vmc_stair_auto_armed = 0U;
    vmc_stair_reset_detector();
    vmc_control.stair_ready_ms = 0U;
    vmc_control.stair_ready_detected = 0U;
    vmc_control.stair_impact_detected = 0U;
    return;
  }

  if (vmc_control.stair_stage_start_ms != 0U) {
    vmc_control.stair_stage_ms = now - vmc_control.stair_stage_start_ms;
  } else {
    vmc_control.stair_stage_ms = 0U;
  }

  /* Channel 7 arms one automatic action.  It no longer fires on its own rising
   * edge: the highest gear and the loaded, two-leg angle response must follow.
   * A completed action consumes the arm, so leaving the switch on cannot
   * retrigger it. */
  if (remote_frame_ready == 0U || vmc_control.stair_enable == 0U) {
    vmc_stair_auto_armed = 0U;
    vmc_stair_reset_detector();
  } else if (stair_switch_enabled == 0U) {
    vmc_stair_auto_armed = 1U;
    vmc_stair_reset_detector();
  } else if (vmc_stair_is_active() != 0U ||
             vmc_getup_is_active() != 0U ||
             vmc_jump_is_active_or_aborted() != 0U) {
    vmc_stair_auto_armed = 0U;
    vmc_stair_reset_detector();
  }

  if (ready_now != 0U && high_gear != 0U) {
    if (vmc_control.stair_ready_ms < ready_confirm_ms) {
      vmc_control.stair_ready_ms += dt_ms;
      if (vmc_control.stair_ready_ms > ready_confirm_ms) {
        vmc_control.stair_ready_ms = ready_confirm_ms;
      }
    }
  } else {
    vmc_control.stair_ready_ms = 0U;
  }
  vmc_control.stair_ready_detected =
      (vmc_control.stair_ready_ms >= ready_confirm_ms) ? 1U : 0U;
  vmc_control.stair_left_impact = left_leg_changed;
  vmc_control.stair_right_impact = right_leg_changed;
  vmc_control.stair_speed_abs = (float)wheel_speed_abs;
  vmc_control.stair_speed_diff = 0.0f;
  vmc_control.stair_accel = 0.0f;
  vmc_control.stair_fast_detected =
      (vmc_control.stair_speed_ms >= speed_confirm_ms) ? 1U : 0U;
  vmc_control.stair_preimpact_armed = vmc_stair_preimpact_armed;

  if (vmc_control.stair_stage == VMC_STAIR_STAGE_FINISHED ||
      vmc_control.stair_stage == VMC_STAIR_STAGE_ABORT) {
    vmc_stair_auto_armed = 0U;
    vmc_control.stair_impact_ms = 0U;
    vmc_control.stair_impact_detected = 0U;
    if ((now - vmc_control.stair_last_finish_ms) >= cooldown_ms) {
      vmc_control.stair_stage = VMC_STAIR_STAGE_IDLE;
      vmc_control.stair_stage_start_ms = 0U;
      vmc_control.stair_stage_ms = 0U;
      vmc_control.stair_abort_reason = VMC_STAIR_ABORT_NONE;
    }
    return;
  }

  if (vmc_control.stair_stage == VMC_STAIR_STAGE_IDLE) {
    if ((now - vmc_control.stair_last_finish_ms) < cooldown_ms) {
      return;
    }
    start_allowed = vmc_stair_start_is_allowed();
    if (vmc_stair_auto_armed == 0U || start_allowed == 0U ||
        vmc_control.stair_ready_detected == 0U ||
        high_gear == 0U || drive_command == 0U) {
      vmc_stair_reset_detector();
    } else if (vmc_stair_preimpact_armed == 0U) {
      if (speed_now != 0U) {
        if (vmc_control.stair_speed_ms < speed_confirm_ms) {
          vmc_control.stair_speed_ms += dt_ms;
          if (vmc_control.stair_speed_ms > speed_confirm_ms) {
            vmc_control.stair_speed_ms = speed_confirm_ms;
          }
        }
      } else {
        vmc_control.stair_speed_ms = 0U;
      }

      if (vmc_control.stair_speed_ms >= speed_confirm_ms) {
        if (vmc_control.stair_preimpact_ms < preimpact_confirm_ms) {
          vmc_control.stair_preimpact_ms += dt_ms;
          if (vmc_control.stair_preimpact_ms > preimpact_confirm_ms) {
            vmc_control.stair_preimpact_ms = preimpact_confirm_ms;
          }
        }
      } else {
        vmc_control.stair_preimpact_ms = 0U;
      }

      if (vmc_control.stair_preimpact_ms >= preimpact_confirm_ms) {
        vmc_stair_preimpact_armed = 1U;
        vmc_stair_preimpact_speed_ref = wheel_speed_abs;
      }
    }

    if (vmc_stair_preimpact_armed != 0U) {
      const double left_theta = vmc_probe_state.motion.left_theta;
      const double right_theta = vmc_probe_state.motion.right_theta;
      const double wheel_speed_drop =
          vmc_stair_preimpact_speed_ref - wheel_speed_abs;

      vmc_control.stair_speed_diff = (float)wheel_speed_drop;
      left_leg_changed =
          (fabs(left_theta) >= (double)impact_theta_threshold)
              ? 1U
              : 0U;
      right_leg_changed =
          (fabs(right_theta) >= (double)impact_theta_threshold)
              ? 1U
              : 0U;
      vmc_control.stair_left_impact = left_leg_changed;
      vmc_control.stair_right_impact = right_leg_changed;
    }

    if (impact_torque_threshold > 0.0f &&
        impact_theta_threshold > 0.0f) {
      left_leg_changed =
          (fabs((double)vmc_balance.left_wheel_torque_raw) >=
               (double)impact_torque_threshold)
              ? left_leg_changed
              : 0U;
      right_leg_changed =
          (fabs((double)vmc_balance.right_wheel_torque_raw) >=
               (double)impact_torque_threshold)
              ? right_leg_changed
              : 0U;
      impact_now =
          (vmc_stair_preimpact_armed != 0U &&
           left_leg_changed != 0U &&
           right_leg_changed != 0U)
              ? 1U
              : 0U;
    } else {
      impact_now =
          (vmc_stair_preimpact_armed != 0U &&
           vmc_control.stair_speed_diff >= impact_speed_drop_threshold)
              ? 1U
              : 0U;
    }
    vmc_control.stair_left_impact = left_leg_changed;
    vmc_control.stair_right_impact = right_leg_changed;
    if (impact_now != 0U) {
      if (vmc_control.stair_impact_ms < impact_confirm_ms) {
        vmc_control.stair_impact_ms += dt_ms;
        if (vmc_control.stair_impact_ms > impact_confirm_ms) {
          vmc_control.stair_impact_ms = impact_confirm_ms;
        }
      }
    } else {
      vmc_control.stair_impact_ms = 0U;
    }
    vmc_control.stair_impact_detected =
        (vmc_control.stair_impact_ms >= impact_confirm_ms) ? 1U : 0U;
    if (vmc_control.stair_impact_detected != 0U) {
      vmc_control.stair_trigger_count++;
      vmc_control.stair_abort_reason = VMC_STAIR_ABORT_NONE;
      vmc_stair_auto_armed = 0U;
      vmc_stair_enter_stage(VMC_STAIR_STAGE_LIFT, now);
    }
    return;
  }

  {
    const uint8_t abort_reason = vmc_stair_abort_reason();
    const uint32_t total_timeout_ms =
        clamp_u32(vmc_control.stair_timeout_ms, 100U, 10000U);
    const uint32_t retract_timeout_ms =
        clamp_u32(vmc_control.stair_retract_timeout_ms, 50U, 2000U);
    const uint32_t hold_ms = clamp_u32(vmc_control.stair_hold_ms, 1U, 3000U);
    const uint32_t pose_confirm_ms =
        clamp_u32(vmc_control.stair_pose_confirm_ms, 1U, 1000U);
    const uint32_t dt_ms =
        (uint32_t)(vmc_loop_dt() * 1000.0 + 0.5);

    if (abort_reason != VMC_STAIR_ABORT_NONE) {
      vmc_stair_finish(VMC_STAIR_STAGE_ABORT, abort_reason, now);
      return;
    }
    if ((now - vmc_control.stair_start_ms) >= total_timeout_ms &&
        !(vmc_control.stair_stage == VMC_STAIR_STAGE_SETTLE &&
          vmc_control.stair_stage_ms >= hold_ms &&
          vmc_getup_safe_length_ready() == 0U)) {
      vmc_stair_finish(VMC_STAIR_STAGE_ABORT, VMC_STAIR_ABORT_TIMEOUT, now);
      return;
    }

    if (vmc_control.stair_stage == VMC_STAIR_STAGE_LIFT ||
        vmc_control.stair_stage == VMC_STAIR_STAGE_RETRACT) {
      const uint8_t lifting =
          (vmc_control.stair_stage == VMC_STAIR_STAGE_LIFT) ? 1U : 0U;
      const double target_length =
          lifting != 0U ? (double)vmc_control.stair_lift_length
                        : (double)vmc_control.stair_retract_length;
      const double target_theta =
          lifting != 0U ? (double)vmc_control.stair_lift_theta
                        : (double)vmc_control.stair_retract_theta;

      if (vmc_stair_pose_ready(target_length, target_theta) != 0U) {
        if (vmc_control.stair_pose_confirm_elapsed_ms < pose_confirm_ms) {
          vmc_control.stair_pose_confirm_elapsed_ms += dt_ms;
        }
      } else {
        vmc_control.stair_pose_confirm_elapsed_ms = 0U;
      }
      if (vmc_control.stair_pose_confirm_elapsed_ms >= pose_confirm_ms) {
        vmc_stair_enter_stage(
            lifting != 0U ? VMC_STAIR_STAGE_RETRACT
                          : VMC_STAIR_STAGE_SETTLE,
            now);
      } else if (vmc_control.stair_stage_ms >= retract_timeout_ms) {
        vmc_stair_finish(VMC_STAIR_STAGE_ABORT,
                         VMC_STAIR_ABORT_TIMEOUT,
                         now);
      }
    } else if (vmc_control.stair_stage == VMC_STAIR_STAGE_SETTLE) {
      if (vmc_control.stair_stage_ms >= hold_ms) {
        vmc_stair_handoff_to_getup(now);
      }
    } else {
      vmc_stair_finish(VMC_STAIR_STAGE_ABORT, VMC_STAIR_ABORT_TIMEOUT, now);
    }
  }
}

static uint8_t vmc_getup_is_active(void)
{
  return (vmc_control.getup_state == VMC_GETUP_STATE_BRACE ||
          vmc_control.getup_state == VMC_GETUP_STATE_TURN ||
          vmc_control.getup_state == VMC_GETUP_STATE_SHRINK ||
          vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER ||
          vmc_control.getup_state == VMC_GETUP_STATE_UPRIGHT ||
          vmc_control.getup_state == VMC_GETUP_STATE_BODY_ALIGN ||
          vmc_control.getup_state == VMC_GETUP_STATE_BODY_FLIP ||
          vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN ||
          vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN_SYNC ||
          vmc_control.getup_state == VMC_GETUP_STATE_FOLDED_LOWER ||
          vmc_control.getup_lqr_release_pending != 0U)
             ? 1U
             : 0U;
}

static uint8_t vmc_getup_theta_is_in_bad_zone(double theta)
{
  return ((theta > VMC_GETUP_LEG_BAD_NEG_MIN_RAD &&
           theta < VMC_GETUP_LEG_BAD_NEG_MAX_RAD) ||
          (theta > VMC_GETUP_LEG_BAD_POS_MIN_RAD &&
           theta < VMC_GETUP_LEG_BAD_POS_MAX_RAD))
             ? 1U
             : 0U;
}

static uint8_t vmc_getup_theta_is_on_shrink_side(double theta)
{
  return (theta > VMC_GETUP_SHRINK_READY_THETA_MIN_RAD &&
          theta < VMC_GETUP_SHRINK_READY_THETA_MAX_RAD)
             ? 1U
             : 0U;
}

static uint8_t vmc_getup_legs_ready_to_shrink(void)
{
  const double left_theta = vmc_probe_state.motion.left_theta;
  const double right_theta = vmc_probe_state.motion.right_theta;

  return (fabs(vmc_probe_state.body.pitch) <
              VMC_GETUP_START_PITCH_RAD &&
          vmc_getup_theta_is_on_shrink_side(left_theta) != 0U &&
          vmc_getup_theta_is_on_shrink_side(right_theta) != 0U)
             ? 1U
             : 0U;
}

static uint8_t vmc_getup_theta_recover_is_required(void)
{
  return (vmc_getup_theta_is_in_bad_zone(vmc_probe_state.motion.theta) != 0U ||
          vmc_getup_theta_is_in_bad_zone(vmc_probe_state.motion.left_theta) != 0U ||
          vmc_getup_theta_is_in_bad_zone(vmc_probe_state.motion.right_theta) != 0U)
             ? 1U
             : 0U;
}

static uint8_t vmc_getup_leg_recover_is_required(void)
{
  return (vmc_getup_legs_ready_to_shrink() == 0U &&
          vmc_getup_theta_recover_is_required() != 0U)
             ? 1U
             : 0U;
}

static uint8_t vmc_getup_direct_shrink_is_required(void)
{
  const double average_length =
      (vmc_probe_state.left_leg.length + vmc_probe_state.right_leg.length) *
      0.5;
  const uint8_t one_leg_on_shrink_side =
      (vmc_getup_theta_is_on_shrink_side(vmc_probe_state.motion.left_theta) !=
           0U ||
       vmc_getup_theta_is_on_shrink_side(vmc_probe_state.motion.right_theta) !=
           0U)
          ? 1U
          : 0U;

  return (fabs(vmc_probe_state.body.pitch) <
              VMC_GETUP_START_PITCH_RAD &&
          vmc_getup_theta_recover_is_required() == 0U &&
          one_leg_on_shrink_side != 0U &&
          average_length >
              (double)VMC_GETUP_STAND_LENGTH +
                  (double)VMC_GETUP_LENGTH_READY_TOL)
             ? 1U
             : 0U;
}

static int8_t vmc_getup_body_mode(double pitch)
{
  if (pitch > VMC_GETUP_BODY_START_PITCH_RAD && pitch < VMC_PI) {
    return 1;
  }
  if (pitch < -VMC_GETUP_BODY_START_PITCH_RAD && pitch > -VMC_PI) {
    return -1;
  }
  return 0;
}

static int8_t vmc_getup_body_align_direction(double phi4, int8_t body_mode)
{
  const double angle = vmc_rad_format((float)phi4);
  const double target =
      (body_mode > 0) ? VMC_GETUP_BODY_POS_TARGET_RAD
                      : VMC_GETUP_BODY_NEG_TARGET_RAD;

  if (fabs(vmc_rad_format((float)(angle - target))) <=
      VMC_GETUP_BODY_TARGET_TOL_RAD) {
    return 0;
  }
  if (body_mode > 0) {
    if (angle < 0.0 && angle > target + VMC_GETUP_BODY_TARGET_TOL_RAD) {
      return -1;
    }
    return 1;
  }
  if (angle < target - VMC_GETUP_BODY_TARGET_TOL_RAD && angle > -VMC_PI) {
    return 1;
  }
  return -1;
}

static uint8_t vmc_getup_body_fan_is_required(void)
{
  return (fabs(vmc_probe_state.body.roll) >
          (double)VMC_GETUP_BODY_FAN_START_ROLL_RAD)
             ? 1U
             : 0U;
}

static uint8_t vmc_getup_body_fan_sync_ready(void)
{
  const double left_theta = vmc_probe_state.motion.left_theta;
  const double right_theta = vmc_probe_state.motion.right_theta;

  if (vmc_control.getup_body_fan_sync_prelift_active != 0U ||
      vmc_control.getup_body_fan_sync_targets_ready == 0U) {
    return 0U;
  }

  if (vmc_control.getup_body_fan_escape_active == 0U) {
    if (fabs((double)vmc_rad_format((float)(
                 vmc_probe_state.left_leg.phi0 -
                 vmc_probe_state.right_leg.phi0))) >=
        VMC_GETUP_BODY_FAN_SYNC_THETA_TOL_RAD) {
      return 0U;
    }
  } else {
    const double left_target =
        (double)vmc_control.getup_body_fan_sync_theta_left;
    const double right_target =
        (double)vmc_control.getup_body_fan_sync_theta_right;

    if (fabs((double)vmc_rad_format(
                 (float)(left_theta - left_target))) >=
            VMC_GETUP_BODY_FAN_SYNC_THETA_TOL_RAD ||
        fabs((double)vmc_rad_format(
                 (float)(right_theta - right_target))) >=
            VMC_GETUP_BODY_FAN_SYNC_THETA_TOL_RAD) {
      return 0U;
    }
  }

  return (vmc_probe_state.left_leg.length >
              (double)VMC_GETUP_BODY_FAN_SYNC_LENGTH_MIN &&
          vmc_probe_state.right_leg.length >
              (double)VMC_GETUP_BODY_FAN_SYNC_LENGTH_MIN &&
          fabs(vmc_probe_state.left_leg.length -
               vmc_probe_state.right_leg.length) <
              (double)VMC_GETUP_BODY_FAN_SYNC_LENGTH_TOL_M)
             ? 1U
             : 0U;
}

static void vmc_getup_body_fan_sync_targets(double *left_target,
                                            double *right_target)
{
  const double left_theta = vmc_probe_state.motion.left_theta;
  const double right_theta = vmc_probe_state.motion.right_theta;
  const uint8_t left_bad = vmc_getup_theta_is_in_bad_zone(left_theta);
  const uint8_t right_bad = vmc_getup_theta_is_in_bad_zone(right_theta);
  double leader_theta;
  uint8_t leader_side = 0U;

  vmc_control.getup_body_fan_sync_phi0_target = 0.0f;
  if (vmc_control.getup_body_fan_sync_fold_side == 0U) {
    if (left_bad != 0U && right_bad == 0U) {
      vmc_control.getup_body_fan_sync_fold_side = 1U;
    } else if (right_bad != 0U && left_bad == 0U) {
      vmc_control.getup_body_fan_sync_fold_side = 2U;
    }
  }

  {
    const double direction =
        (vmc_control.getup_turn_direction < 0) ? -1.0 : 1.0;
    double left_phase = fmod(direction * left_theta,
                             2.0 * (double)VMC_PI);
    double right_phase = fmod(direction * right_theta,
                              2.0 * (double)VMC_PI);

    if (left_phase < 0.0) {
      left_phase += 2.0 * (double)VMC_PI;
    }
    if (right_phase < 0.0) {
      right_phase += 2.0 * (double)VMC_PI;
    }
    leader_side = (left_phase >= right_phase) ? 1U : 2U;
    leader_theta = (leader_side == 1U) ? left_theta : right_theta;
    vmc_control.getup_body_fan_sync_phi0_target =
        (leader_side == 1U) ? (float)vmc_probe_state.left_leg.phi0
                            : (float)vmc_probe_state.right_leg.phi0;
  }
  if (vmc_control.getup_body_fan_escape_active != 0U) {
    leader_theta = vmc_angle_forward_to(
        (double)VMC_GETUP_BODY_FAN_ENTRY_THETA_RAD,
        leader_theta,
        vmc_control.getup_turn_direction);
    leader_side = 0U;
  }
  vmc_control.getup_body_fan_sync_leader_side = leader_side;
  *left_target = leader_theta;
  *right_target = leader_theta;
}

static uint8_t vmc_getup_next_precondition_state(void)
{
  const double pitch = vmc_probe_state.body.pitch;

  if (vmc_getup_body_mode(pitch) != 0) {
    return VMC_GETUP_STATE_BODY_ALIGN;
  }
  if (vmc_getup_body_fan_is_required() != 0U) {
    return (vmc_control.getup_body_fan_sync_complete != 0U)
               ? VMC_GETUP_STATE_BODY_FAN
               : VMC_GETUP_STATE_BODY_FAN_SYNC;
  }
  if (fabs(pitch) > VMC_GETUP_START_PITCH_RAD) {
    return VMC_GETUP_STATE_TURN;
  }
  if (vmc_control.getup_leg_align_complete == 0U &&
      vmc_getup_theta_recover_is_required() != 0U) {
    return VMC_GETUP_STATE_LEG_RECOVER;
  }
  return VMC_GETUP_STATE_SHRINK;
}

static uint8_t vmc_getup_pose_ready(void)
{
  const double left_theta = vmc_probe_state.motion.left_theta;
  const double right_theta = vmc_probe_state.motion.right_theta;

  return (left_theta > VMC_GETUP_THETA_MIN_RAD &&
          left_theta < VMC_GETUP_THETA_MAX_RAD &&
          right_theta > VMC_GETUP_THETA_MIN_RAD &&
          right_theta < VMC_GETUP_THETA_MAX_RAD &&
          fabs(vmc_probe_state.body.pitch) < VMC_GETUP_PITCH_READY_RAD)
             ? 1U
             : 0U;
}

static uint8_t vmc_getup_upright_ready(void)
{
  return (fabs(vmc_probe_state.motion.left_theta) <
              VMC_GETUP_UPRIGHT_THETA_TOL_RAD &&
          fabs(vmc_probe_state.motion.right_theta) <
              VMC_GETUP_UPRIGHT_THETA_TOL_RAD &&
          fabs(vmc_probe_state.body.pitch) < VMC_GETUP_UPRIGHT_PITCH_RAD &&
          fabs(vmc_probe_state.motion.left_d_theta) <
              VMC_GETUP_UPRIGHT_DTHETA_MAX_RADPS &&
          fabs(vmc_probe_state.motion.right_d_theta) <
              VMC_GETUP_UPRIGHT_DTHETA_MAX_RADPS &&
          fabs(vmc_probe_state.body.pitch_rate) <
              VMC_GETUP_UPRIGHT_DPITCH_MAX_RADPS &&
          fabs(vmc_probe_state.left_leg.length - VMC_GETUP_STAND_LENGTH) <
              VMC_GETUP_LENGTH_READY_TOL &&
          fabs(vmc_probe_state.right_leg.length - VMC_GETUP_STAND_LENGTH) <
              VMC_GETUP_LENGTH_READY_TOL)
             ? 1U
             : 0U;
}

static uint8_t vmc_getup_partial_folded_is_required(double pitch)
{
  const double average_length =
      (vmc_probe_state.left_leg.length + vmc_probe_state.right_leg.length) *
      0.5;
  const double left_theta = vmc_probe_state.motion.left_theta;
  const double right_theta = vmc_probe_state.motion.right_theta;
  const uint8_t legs_folded =
      (fabs(left_theta) > VMC_GETUP_PARTIAL_ENTRY_THETA_MIN_RAD &&
       fabs(left_theta) < VMC_GETUP_PARTIAL_ENTRY_THETA_MAX_RAD &&
       fabs(right_theta) > VMC_GETUP_PARTIAL_ENTRY_THETA_MIN_RAD &&
       fabs(right_theta) < VMC_GETUP_PARTIAL_ENTRY_THETA_MAX_RAD &&
       left_theta * right_theta > 0.0)
          ? 1U
          : 0U;

  return (fabs(pitch) > VMC_GETUP_PARTIAL_PITCH_MIN_RAD &&
          fabs(pitch) < VMC_GETUP_START_PITCH_RAD &&
          fabs(vmc_probe_state.body.roll) <
              VMC_GETUP_PARTIAL_ROLL_MAX_RAD &&
          average_length < VMC_GETUP_PARTIAL_ENTRY_LENGTH_MAX &&
          legs_folded != 0U)
             ? 1U
             : 0U;
}

static uint8_t vmc_getup_enter_brace(int8_t turn_direction)
{
  if (vmc_getup_safe_length_ready() == 0U) {
    return 0U;
  }
  vmc_control.getup_state = VMC_GETUP_STATE_BRACE;
  vmc_control.getup_turn_direction = turn_direction;
  vmc_control.getup_body_flip_direction = 0;
  vmc_control.getup_left_direction = 0;
  vmc_control.getup_right_direction = 0;
  vmc_control.getup_left_done = 0U;
  vmc_control.getup_right_done = 0U;
  vmc_control.getup_leg_align_complete = 0U;
  vmc_control.getup_partial_active = 0U;
  vmc_control.getup_body_fan_sync_complete = 0U;
  vmc_control.getup_body_fan_sync_phi0_target = 0.0f;
  vmc_control.getup_body_fan_sync_fold_side = 0U;
  vmc_control.getup_body_fan_sync_prelift_active = 0U;
  vmc_control.getup_body_fan_sync_targets_ready = 0U;
  vmc_control.getup_body_fan_sync_initial_phase_close = 0U;
  vmc_control.getup_body_fan_sync_leader_side = 0U;
  vmc_control.getup_body_fan_sync_prelift_confirm_ms = 0U;
  vmc_control.getup_body_fan_escape_active = 0U;
  vmc_control.getup_body_fan_retry_used = 0U;
  vmc_control.getup_body_fan_stall_ms = 0U;
  vmc_control.getup_left_confirm_ms = 0U;
  vmc_control.getup_right_confirm_ms = 0U;
  vmc_control.getup_left_hold_length =
      (float)vmc_probe_state.left_leg.length;
  vmc_control.getup_right_hold_length =
      (float)vmc_probe_state.right_leg.length;
  vmc_control.getup_left_hold_theta =
      (float)vmc_probe_state.motion.left_theta;
  vmc_control.getup_right_hold_theta =
      (float)vmc_probe_state.motion.right_theta;
  vmc_control.getup_lqr_release_pending = 1U;
  vmc_control.getup_state_ms = 0U;
  vmc_control.getup_confirm_ms = 0U;
  vmc_control.getup_upright_phi0_left = vmc_probe_state.left_leg.phi0;
  vmc_control.getup_upright_phi0_right = vmc_probe_state.right_leg.phi0;
  vmc_control.getup_phi0_feedforward_left = 0.0f;
  vmc_control.getup_phi0_feedforward_right = 0.0f;
  return 1U;
}

static void vmc_getup_enter_stage(uint8_t state)
{
  vmc_control.getup_state = state;
  vmc_control.getup_state_ms = 0U;
  vmc_control.getup_confirm_ms = 0U;
  vmc_control.getup_left_direction = 0;
  vmc_control.getup_right_direction = 0;
  vmc_control.getup_left_done = 0U;
  vmc_control.getup_right_done = 0U;
  vmc_control.getup_left_confirm_ms = 0U;
  vmc_control.getup_right_confirm_ms = 0U;

  if (state == VMC_GETUP_STATE_TURN) {
    vmc_control.getup_left_direction = vmc_control.getup_turn_direction;
    vmc_control.getup_right_direction = vmc_control.getup_turn_direction;
  } else if (state == VMC_GETUP_STATE_BODY_FAN ||
             state == VMC_GETUP_STATE_BODY_FAN_SYNC) {
    vmc_control.getup_left_direction = vmc_control.getup_turn_direction;
    vmc_control.getup_right_direction = vmc_control.getup_turn_direction;
    if (state == VMC_GETUP_STATE_BODY_FAN_SYNC) {
      double left_sync_theta;
      double right_sync_theta;

      vmc_getup_body_fan_sync_targets(
          &left_sync_theta, &right_sync_theta);
      vmc_control.getup_body_fan_sync_initial_phase_close =
          (fabs((double)vmc_rad_format(
                    (float)(vmc_probe_state.motion.left_theta -
                            vmc_probe_state.motion.right_theta))) <=
           (double)VMC_GETUP_BODY_FAN_SYNC_EARLY_PHASE_TOL_RAD)
              ? 1U
              : 0U;
      vmc_control.getup_body_fan_sync_prelift_active =
          (vmc_control.getup_body_fan_sync_fold_side != 0U &&
           vmc_control.getup_body_fan_escape_active == 0U)
              ? 1U
              : 0U;
      vmc_control.getup_body_fan_sync_targets_ready =
          (vmc_control.getup_body_fan_sync_prelift_active == 0U) ? 1U : 0U;
      vmc_control.getup_body_fan_sync_theta_left =
          (float)left_sync_theta;
      vmc_control.getup_body_fan_sync_theta_right =
          (float)right_sync_theta;
      vmc_control.getup_upright_phi0_left =
          vmc_probe_state.left_leg.phi0;
      vmc_control.getup_upright_phi0_right =
          vmc_probe_state.right_leg.phi0;
    }
  } else if (state == VMC_GETUP_STATE_BODY_FLIP) {
    vmc_control.getup_left_direction =
        vmc_control.getup_body_flip_direction;
    vmc_control.getup_right_direction =
        vmc_control.getup_body_flip_direction;
  } else if (state == VMC_GETUP_STATE_LEG_RECOVER) {
    vmc_control.getup_left_hold_length =
        (float)vmc_probe_state.left_leg.length;
    vmc_control.getup_right_hold_length =
        (float)vmc_probe_state.right_leg.length;
    vmc_control.getup_left_hold_theta =
        (float)vmc_probe_state.motion.left_theta;
    vmc_control.getup_right_hold_theta =
        (float)vmc_probe_state.motion.right_theta;
    vmc_control.getup_left_done =
        (vmc_getup_theta_is_in_bad_zone(vmc_probe_state.motion.left_theta) ==
         0U)
            ? 1U
            : 0U;
    vmc_control.getup_right_done =
        (vmc_getup_theta_is_in_bad_zone(vmc_probe_state.motion.right_theta) ==
         0U)
            ? 1U
            : 0U;
    vmc_control.getup_leg_align_complete = 0U;
  } else if (state == VMC_GETUP_STATE_SHRINK) {
    vmc_control.getup_shrink_target_theta_left =
        (float)vmc_probe_state.motion.left_theta;
    vmc_control.getup_shrink_target_theta_right =
        (float)vmc_probe_state.motion.right_theta;
  }
}

static uint8_t vmc_getup_enter_partial_fold(void)
{
  if (vmc_control.getup_partial_armed == 0U ||
      vmc_getup_safe_length_ready() == 0U) {
    return 0U;
  }

  vmc_stair_reset_balance_for_getup();
  vmc_getup_enter_stage(VMC_GETUP_STATE_FOLDED_LOWER);
  vmc_control.getup_lqr_release_pending = 1U;
  vmc_control.getup_partial_armed = 0U;
  vmc_control.getup_partial_active = 1U;
  vmc_getup_reset_balance_on_ready = 1U;
  vmc_balance.wheel_output_enable = 0U;
  return 1U;
}

static void vmc_getup_abort(void)
{
  vmc_control.getup_state = VMC_GETUP_STATE_IDLE;
  vmc_control.getup_turn_direction = 0;
  vmc_control.getup_body_flip_direction = 0;
  vmc_control.getup_left_direction = 0;
  vmc_control.getup_right_direction = 0;
  vmc_control.getup_left_done = 0U;
  vmc_control.getup_right_done = 0U;
  vmc_control.getup_left_confirm_ms = 0U;
  vmc_control.getup_right_confirm_ms = 0U;
  vmc_control.getup_lqr_release_pending = 0U;
  vmc_getup_reset_balance_on_ready = 0U;
  vmc_control.getup_partial_armed = 0U;
  vmc_control.getup_partial_active = 0U;
  vmc_control.getup_state_ms = 0U;
  vmc_control.getup_confirm_ms = 0U;
  vmc_fall_rearm_required = 1U;
  vmc_fall_rearm_armed = 0U;
  vmc_balance.wheel_output_enable = 0U;
  clear_leg_command(&vmc_probe_state.left_leg);
  clear_leg_command(&vmc_probe_state.right_leg);
}

static void vmc_update_getup_state(void)
{
  const uint8_t joint_allowed = remote_control_allows_joint_enable();
  const uint8_t imu_ready =
      (vmc_probe_state.body.imu_valid != 0U &&
       vmc_probe_state.body.imu_connected != 0U)
          ? 1U
          : 0U;
  const double pitch = vmc_probe_state.body.pitch;
  const uint8_t fully_fallen =
      (imu_ready != 0U && fabs(pitch) > VMC_GETUP_START_PITCH_RAD) ? 1U : 0U;
  const uint8_t partial_folded =
      vmc_getup_partial_folded_is_required(pitch);
  const int8_t body_mode = vmc_getup_body_mode(pitch);
  const uint8_t pose_getup_allowed =
      (vmc_stair_is_active() == 0U) ? 1U : 0U;
  const uint32_t dt_ms =
      (uint32_t)(vmc_loop_dt() * 1000.0f) > 0U
          ? (uint32_t)(vmc_loop_dt() * 1000.0f)
          : 1U;
  const uint32_t stage_timeout_ms =
      (vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN)
          ? VMC_GETUP_BODY_FAN_TIMEOUT_MS
          : ((vmc_control.getup_state == VMC_GETUP_STATE_FOLDED_LOWER)
                 ? VMC_GETUP_PARTIAL_TIMEOUT_MS
                 : VMC_GETUP_TIMEOUT_MS);

  vmc_getup_fully_fallen = fully_fallen;

  if (vmc_control.manual_prepose_active != 0U) {
    /* Manual pre-positioning deliberately owns the joints and wheel command.
     * Keep the get-up machine out of the way until CH7 is returned to OFF. */
    vmc_control.getup_state = VMC_GETUP_STATE_IDLE;
    vmc_control.getup_turn_direction = 0;
    vmc_control.getup_body_flip_direction = 0;
    vmc_control.getup_left_direction = 0;
    vmc_control.getup_right_direction = 0;
    vmc_control.getup_left_done = 0U;
    vmc_control.getup_right_done = 0U;
    vmc_control.getup_lqr_release_pending = 0U;
    vmc_control.getup_partial_active = 0U;
    vmc_getup_reset_balance_on_ready = 0U;
    vmc_control.getup_state_ms = 0U;
    vmc_control.getup_confirm_ms = 0U;
    vmc_control.getup_left_confirm_ms = 0U;
    vmc_control.getup_right_confirm_ms = 0U;
    vmc_fall_prev_fully_fallen = fully_fallen;
    vmc_fall_monitor_armed = 0U;
    return;
  }

  if (vmc_control.manual_prepose_exit_length_check_active != 0U) {
    if (vmc_manual_prepose_exit_length_ready() != 0U) {
      vmc_control.manual_prepose_exit_length_check_active = 0U;
    } else {
      vmc_control.getup_state = VMC_GETUP_STATE_IDLE;
      vmc_control.getup_turn_direction = 0;
      vmc_control.getup_body_flip_direction = 0;
      vmc_control.getup_left_direction = 0;
      vmc_control.getup_right_direction = 0;
      vmc_control.getup_left_done = 0U;
      vmc_control.getup_right_done = 0U;
      vmc_control.getup_lqr_release_pending = 0U;
      vmc_control.getup_partial_active = 0U;
      vmc_getup_reset_balance_on_ready = 0U;
      vmc_control.getup_state_ms = 0U;
      vmc_control.getup_confirm_ms = 0U;
      vmc_control.getup_left_confirm_ms = 0U;
      vmc_control.getup_right_confirm_ms = 0U;
      vmc_fall_prev_fully_fallen = fully_fallen;
      vmc_fall_monitor_armed = 0U;
      return;
    }
  }

  if (remote_control.s[1] == RC_SW_DOWN) {
    vmc_fall_rearm_required = 0U;
    vmc_fall_rearm_armed = 1U;
    vmc_fall_monitor_armed = 0U;
    vmc_fall_prev_fully_fallen = fully_fallen;
    vmc_control.getup_partial_armed = 1U;
  }

  if (vmc_control.enabled == 0U || joint_allowed == 0U || imu_ready == 0U ||
      vmc_probe_state.feedback_timeout_active != 0U) {
    vmc_control.getup_state = VMC_GETUP_STATE_IDLE;
    vmc_control.getup_turn_direction = 0;
    vmc_control.getup_body_flip_direction = 0;
    vmc_control.getup_left_direction = 0;
    vmc_control.getup_right_direction = 0;
    vmc_control.getup_lqr_release_pending = 0U;
    vmc_getup_reset_balance_on_ready = 0U;
    vmc_control.getup_state_ms = 0U;
    vmc_control.getup_confirm_ms = 0U;
    vmc_control.getup_left_confirm_ms = 0U;
    vmc_control.getup_right_confirm_ms = 0U;
    return;
  }

  /* A normal upright enable consumes the shallow-fold opportunity.  The next
   * automatic attempt requires channel 4 to pass through STOP again, so a
   * walking transient cannot start a get-up later. */
  if (remote_control.s[1] != RC_SW_DOWN &&
      (vmc_control.getup_state == VMC_GETUP_STATE_IDLE ||
       vmc_control.getup_state == VMC_GETUP_STATE_READY) &&
      partial_folded == 0U) {
    vmc_control.getup_partial_armed = 0U;
  }

  /* Arm fall detection only after observing an upright running state.  This
   * deliberately ignores the case where channel 4 is already enabled while the chassis
   * was already lying down, allowing that explicit restart to perform one
   * get-up sequence. */
  if (fully_fallen == 0U &&
      (remote_control.s[1] == RC_SW_MID || remote_control.s[1] == RC_SW_UP)) {
    vmc_fall_monitor_armed = 1U;
  }

  /* Once re-armed by channel 4 UP, only channel 4 DOWN may start the
   * controller again.  The one-shot arm is consumed when starting recovery
   * from a fallen pose. */
  if (vmc_fall_rearm_armed != 0U && remote_control.s[1] == RC_SW_MID) {
    if (fully_fallen == 0U ||
        (vmc_control.getup_state != VMC_GETUP_STATE_IDLE &&
         vmc_control.getup_state != VMC_GETUP_STATE_READY)) {
      vmc_fall_rearm_armed = 0U;
    }
  }

  if (fully_fallen != 0U && vmc_fall_monitor_armed != 0U &&
      vmc_fall_prev_fully_fallen == 0U && vmc_fall_rearm_armed == 0U &&
      (remote_control.s[1] == RC_SW_MID || remote_control.s[1] == RC_SW_UP) &&
      (vmc_control.getup_state == VMC_GETUP_STATE_IDLE ||
       vmc_control.getup_state == VMC_GETUP_STATE_READY)) {
    vmc_fall_rearm_required = 1U;
    vmc_control.getup_state = VMC_GETUP_STATE_IDLE;
    vmc_control.getup_turn_direction = 0;
    vmc_control.getup_body_flip_direction = 0;
    vmc_control.getup_left_direction = 0;
    vmc_control.getup_right_direction = 0;
    vmc_fall_prev_fully_fallen = fully_fallen;
    return;
  }

  vmc_fall_prev_fully_fallen = fully_fallen;

  if (vmc_control.getup_state == VMC_GETUP_STATE_BRACE ||
      vmc_control.getup_state == VMC_GETUP_STATE_TURN ||
      vmc_control.getup_state == VMC_GETUP_STATE_SHRINK ||
      vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER ||
      vmc_control.getup_state == VMC_GETUP_STATE_UPRIGHT ||
      vmc_control.getup_state == VMC_GETUP_STATE_BODY_ALIGN ||
      vmc_control.getup_state == VMC_GETUP_STATE_BODY_FLIP ||
      vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN ||
      vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN_SYNC ||
      vmc_control.getup_state == VMC_GETUP_STATE_FOLDED_LOWER) {
    vmc_control.getup_state_ms += dt_ms;
    if (vmc_control.getup_state_ms >= stage_timeout_ms) {
      vmc_getup_abort();
      return;
    }
  }
  if ((vmc_control.getup_state == VMC_GETUP_STATE_IDLE ||
       vmc_control.getup_state == VMC_GETUP_STATE_READY) &&
      fully_fallen != 0U) {
    (void)vmc_getup_enter_brace((pitch < 0.0) ? 1 : -1);
  } else if ((vmc_control.getup_state == VMC_GETUP_STATE_IDLE ||
              vmc_control.getup_state == VMC_GETUP_STATE_READY) &&
             pose_getup_allowed != 0U &&
             vmc_getup_body_fan_is_required() != 0U) {
    (void)vmc_getup_enter_brace((pitch < 0.0) ? 1 : -1);
  } else if ((vmc_control.getup_state == VMC_GETUP_STATE_IDLE ||
              vmc_control.getup_state == VMC_GETUP_STATE_READY) &&
             pose_getup_allowed != 0U &&
             vmc_getup_leg_recover_is_required() != 0U) {
    (void)vmc_getup_enter_brace((pitch < 0.0) ? 1 : -1);
  } else if ((vmc_control.getup_state == VMC_GETUP_STATE_IDLE ||
              vmc_control.getup_state == VMC_GETUP_STATE_READY) &&
             pose_getup_allowed != 0U &&
             partial_folded != 0U) {
    (void)vmc_getup_enter_partial_fold();
  } else if ((vmc_control.getup_state == VMC_GETUP_STATE_IDLE ||
              vmc_control.getup_state == VMC_GETUP_STATE_READY) &&
             pose_getup_allowed != 0U &&
             vmc_getup_direct_shrink_is_required() != 0U) {
    (void)vmc_getup_enter_brace((pitch < 0.0) ? 1 : -1);
  } else if ((vmc_control.getup_state == VMC_GETUP_STATE_IDLE ||
              vmc_control.getup_state == VMC_GETUP_STATE_READY) &&
             vmc_control.getup_lqr_release_pending != 0U) {
    (void)vmc_getup_enter_brace(-1);
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_BRACE &&
      vmc_control.getup_state_ms >= VMC_GETUP_BRACE_MS) {
    uint8_t next_state = vmc_getup_next_precondition_state();

    if (next_state == VMC_GETUP_STATE_BODY_ALIGN) {
      vmc_control.getup_body_flip_direction = (pitch > 0.0) ? 1 : -1;
    } else if (next_state == VMC_GETUP_STATE_BODY_FAN ||
               next_state == VMC_GETUP_STATE_BODY_FAN_SYNC) {
      vmc_control.getup_turn_direction = (pitch < 0.0) ? 1 : -1;
    } else if (next_state == VMC_GETUP_STATE_TURN) {
      vmc_control.getup_turn_direction = (pitch < 0.0) ? 1 : -1;
    }
    vmc_getup_enter_stage(next_state);
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN_SYNC) {
    const uint8_t fan_required = vmc_getup_body_fan_is_required();

    if (vmc_control.getup_body_fan_sync_prelift_active != 0U) {
      const uint8_t lengths_ready =
          (vmc_probe_state.left_leg.length >
               (double)VMC_GETUP_BODY_FAN_SYNC_LENGTH_MIN &&
           vmc_probe_state.right_leg.length >
               (double)VMC_GETUP_BODY_FAN_SYNC_LENGTH_MIN &&
           fabs(vmc_probe_state.left_leg.length -
                vmc_probe_state.right_leg.length) <
               (double)VMC_GETUP_BODY_FAN_SYNC_LENGTH_TOL_M)
              ? 1U
              : 0U;

      if (lengths_ready != 0U &&
          vmc_control.getup_body_fan_sync_initial_phase_close != 0U) {
        vmc_control.getup_body_fan_sync_prelift_active = 0U;
        vmc_control.getup_body_fan_sync_targets_ready = 0U;
        vmc_control.getup_body_fan_sync_prelift_confirm_ms = 0U;
        vmc_control.getup_body_fan_sync_complete = 1U;
        vmc_control.getup_body_fan_escape_active = 0U;
        vmc_getup_enter_stage(VMC_GETUP_STATE_BODY_FAN);
        return;
      }

      if (lengths_ready != 0U) {
        if (vmc_control.getup_body_fan_sync_prelift_confirm_ms <
            VMC_GETUP_BODY_FAN_SYNC_CONFIRM_MS) {
          vmc_control.getup_body_fan_sync_prelift_confirm_ms += dt_ms;
        }
      } else {
        vmc_control.getup_body_fan_sync_prelift_confirm_ms = 0U;
      }
      if (vmc_control.getup_body_fan_sync_prelift_confirm_ms >=
          VMC_GETUP_BODY_FAN_SYNC_CONFIRM_MS) {
        double left_sync_theta;
        double right_sync_theta;

        vmc_control.getup_body_fan_sync_prelift_active = 0U;
        vmc_control.getup_body_fan_sync_targets_ready = 1U;
        vmc_control.getup_body_fan_sync_prelift_confirm_ms = 0U;
        vmc_getup_body_fan_sync_targets(
            &left_sync_theta, &right_sync_theta);
        vmc_control.getup_body_fan_sync_theta_left =
            (float)left_sync_theta;
        vmc_control.getup_body_fan_sync_theta_right =
            (float)right_sync_theta;
        vmc_control.getup_upright_phi0_left =
            vmc_probe_state.left_leg.phi0;
        vmc_control.getup_upright_phi0_right =
            vmc_probe_state.right_leg.phi0;
      }
    }

    if (vmc_getup_body_fan_sync_ready() != 0U) {
      if (vmc_control.getup_confirm_ms <
          VMC_GETUP_BODY_FAN_SYNC_CONFIRM_MS) {
        vmc_control.getup_confirm_ms += dt_ms;
      }
    } else {
      vmc_control.getup_confirm_ms = 0U;
    }
    if (vmc_control.getup_confirm_ms >=
        VMC_GETUP_BODY_FAN_SYNC_CONFIRM_MS) {
      vmc_control.getup_body_fan_sync_complete = 1U;
      vmc_control.getup_body_fan_escape_active = 0U;
      if (fan_required != 0U) {
        vmc_getup_enter_stage(VMC_GETUP_STATE_BODY_FAN);
      } else {
        uint8_t next_state;

        vmc_control.getup_turn_direction = 0;
        vmc_control.getup_left_direction = 0;
        vmc_control.getup_right_direction = 0;
        next_state = vmc_getup_next_precondition_state();
        if (next_state == VMC_GETUP_STATE_BODY_ALIGN) {
          vmc_control.getup_body_flip_direction =
              (pitch > 0.0) ? 1 : -1;
        } else if (next_state == VMC_GETUP_STATE_TURN) {
          vmc_control.getup_turn_direction =
              (pitch < 0.0) ? 1 : -1;
        }
        vmc_getup_enter_stage(next_state);
      }
    }
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN) {
    const uint8_t fan_stalled =
        (vmc_control.getup_state_ms >=
             VMC_GETUP_BODY_FAN_STALL_STARTUP_MS &&
         fabs(vmc_probe_state.left_leg.d_phi0) <
             VMC_GETUP_BODY_FAN_STALL_RATE_RADPS &&
         fabs(vmc_probe_state.right_leg.d_phi0) <
             VMC_GETUP_BODY_FAN_STALL_RATE_RADPS &&
         fabs(vmc_probe_state.body.wy) <
             VMC_GETUP_BODY_FAN_STALL_ROLL_RATE_RADPS)
            ? 1U
            : 0U;

    vmc_control.getup_left_direction =
        vmc_control.getup_turn_direction;
    vmc_control.getup_right_direction =
        vmc_control.getup_turn_direction;
    if (fan_stalled != 0U) {
      if (vmc_control.getup_body_fan_stall_ms <
          VMC_GETUP_BODY_FAN_STALL_CONFIRM_MS) {
        vmc_control.getup_body_fan_stall_ms += dt_ms;
      }
    } else {
      vmc_control.getup_body_fan_stall_ms = 0U;
    }
    if (vmc_control.getup_body_fan_stall_ms >=
        VMC_GETUP_BODY_FAN_STALL_CONFIRM_MS) {
      if (vmc_control.getup_body_fan_retry_used == 0U) {
        vmc_control.getup_body_fan_retry_used = 1U;
        vmc_control.getup_body_fan_escape_active = 1U;
        vmc_control.getup_body_fan_sync_complete = 0U;
        vmc_getup_enter_stage(VMC_GETUP_STATE_BODY_FAN_SYNC);
        return;
      }
      vmc_getup_abort();
      return;
    }
    if (fabs(vmc_probe_state.body.pitch) < VMC_GETUP_BODY_FAN_EXIT_RAD &&
        fabs(vmc_probe_state.body.roll) < VMC_GETUP_BODY_FAN_EXIT_RAD) {
      if (vmc_control.getup_confirm_ms < VMC_GETUP_BODY_FAN_CONFIRM_MS) {
        vmc_control.getup_confirm_ms += dt_ms;
      }
    } else {
      vmc_control.getup_confirm_ms = 0U;
    }
    if (vmc_control.getup_confirm_ms >= VMC_GETUP_BODY_FAN_CONFIRM_MS) {
      uint8_t next_state;

      vmc_control.getup_turn_direction = 0;
      vmc_control.getup_left_direction = 0;
      vmc_control.getup_right_direction = 0;
      next_state = vmc_getup_next_precondition_state();
      vmc_getup_enter_stage(next_state);
    }
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_BODY_ALIGN &&
      body_mode != 0) {
    const double left_angle =
        vmc_rad_format((float)vmc_probe_state.left_leg.phi4);
    const double right_angle =
        vmc_rad_format((float)vmc_probe_state.right_leg.phi4);
    const double left_target =
        (body_mode > 0) ? VMC_GETUP_BODY_POS_TARGET_RAD
                        : VMC_GETUP_BODY_NEG_TARGET_RAD;
    const double right_target = left_target;
    const uint8_t left_at_target =
        (fabs(vmc_rad_format((float)(left_angle - left_target))) <=
         VMC_GETUP_BODY_TARGET_TOL_RAD)
            ? 1U
            : 0U;
    const uint8_t right_at_target =
        (fabs(vmc_rad_format((float)(right_angle - right_target))) <=
         VMC_GETUP_BODY_TARGET_TOL_RAD)
            ? 1U
            : 0U;
    int8_t left_direction = vmc_getup_body_align_direction(left_angle, body_mode);
    int8_t right_direction =
        vmc_getup_body_align_direction(right_angle, body_mode);

    vmc_control.getup_body_flip_direction = body_mode;
    if (body_mode > 0) {
      if (left_angle > VMC_GETUP_BODY_INNER_ANGLE_RAD &&
          right_angle < VMC_GETUP_BODY_INNER_ANGLE_RAD &&
          right_at_target == 0U) {
        left_direction = 0;
      }
      if (right_angle > VMC_GETUP_BODY_INNER_ANGLE_RAD &&
          left_angle < VMC_GETUP_BODY_INNER_ANGLE_RAD &&
          left_at_target == 0U) {
        right_direction = 0;
      }
    }

    vmc_control.getup_left_direction = left_direction;
    vmc_control.getup_right_direction = right_direction;
    if (left_at_target != 0U) {
      if (vmc_control.getup_left_confirm_ms <
          VMC_GETUP_BODY_ALIGN_CONFIRM_MS) {
        vmc_control.getup_left_confirm_ms += dt_ms;
      }
      if (vmc_control.getup_left_confirm_ms >=
          VMC_GETUP_BODY_ALIGN_CONFIRM_MS) {
        vmc_control.getup_left_done = 1U;
      }
    } else {
      vmc_control.getup_left_confirm_ms = 0U;
      vmc_control.getup_left_done = 0U;
    }
    if (right_at_target != 0U) {
      if (vmc_control.getup_right_confirm_ms <
          VMC_GETUP_BODY_ALIGN_CONFIRM_MS) {
        vmc_control.getup_right_confirm_ms += dt_ms;
      }
      if (vmc_control.getup_right_confirm_ms >=
          VMC_GETUP_BODY_ALIGN_CONFIRM_MS) {
        vmc_control.getup_right_done = 1U;
      }
    } else {
      vmc_control.getup_right_confirm_ms = 0U;
      vmc_control.getup_right_done = 0U;
    }
    if (vmc_control.getup_left_done != 0U &&
        vmc_control.getup_right_done != 0U) {
      vmc_getup_enter_stage(VMC_GETUP_STATE_BODY_FLIP);
    }
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_BODY_ALIGN) {
    uint8_t next_state = vmc_getup_next_precondition_state();

    vmc_control.getup_body_flip_direction = 0;
    if (next_state == VMC_GETUP_STATE_TURN) {
      vmc_control.getup_turn_direction = (pitch < 0.0) ? 1 : -1;
    }
    vmc_getup_enter_stage(next_state);
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_BODY_FLIP) {
    vmc_control.getup_left_direction =
        vmc_control.getup_body_flip_direction;
    vmc_control.getup_right_direction =
        vmc_control.getup_body_flip_direction;
    if (fabs(pitch) < VMC_GETUP_BODY_EXIT_PITCH_RAD) {
      if (vmc_control.getup_confirm_ms < VMC_GETUP_BODY_FLIP_CONFIRM_MS) {
        vmc_control.getup_confirm_ms += dt_ms;
      }
    } else {
      vmc_control.getup_confirm_ms = 0U;
    }
    if (vmc_control.getup_confirm_ms >= VMC_GETUP_BODY_FLIP_CONFIRM_MS) {
      uint8_t next_state;

      vmc_control.getup_body_flip_direction = 0;
      next_state = vmc_getup_next_precondition_state();
      if (next_state == VMC_GETUP_STATE_TURN) {
        vmc_control.getup_turn_direction = (pitch < 0.0) ? 1 : -1;
      }
      vmc_getup_enter_stage(next_state);
    }
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_FOLDED_LOWER) {
    if (fabs(vmc_probe_state.left_leg.length - VMC_GETUP_STAND_LENGTH) <
            VMC_GETUP_LENGTH_READY_TOL &&
        fabs(vmc_probe_state.right_leg.length - VMC_GETUP_STAND_LENGTH) <
            VMC_GETUP_LENGTH_READY_TOL) {
      if (vmc_control.getup_confirm_ms < VMC_GETUP_PARTIAL_CONFIRM_MS) {
        vmc_control.getup_confirm_ms += dt_ms;
      }
    } else {
      vmc_control.getup_confirm_ms = 0U;
    }
    if (vmc_control.getup_confirm_ms >= VMC_GETUP_PARTIAL_CONFIRM_MS) {
      vmc_getup_enter_stage(VMC_GETUP_STATE_UPRIGHT);
      vmc_control.getup_upright_phi0_left =
          vmc_probe_state.left_leg.phi0;
      vmc_control.getup_upright_phi0_right =
          vmc_probe_state.right_leg.phi0;
    }
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER) {
    const double target_phi0 =
        (double)VMC_PHI0_VERTICAL_RAD + (double)pitch +
        (double)VMC_GETUP_LEG_RECOVER_TARGET_RAD;

    if (vmc_control.getup_left_done == 0U) {
      int8_t direction = 1;
      uint8_t at_target = 0U;
      const uint32_t confirm_time_ms = VMC_GETUP_LEG_RECOVER_CONFIRM_MS;

      if (vmc_getup_theta_is_in_bad_zone(
              vmc_probe_state.motion.left_theta) != 0U) {
        at_target =
            (fabs(vmc_rad_format((float)(
                      target_phi0 - vmc_probe_state.left_leg.phi0))) <=
             VMC_GETUP_LEG_RECOVER_TOL_RAD)
                ? 1U
                : 0U;
        direction = (int8_t)VMC_GETUP_LEG_RECOVER_DIRECTION;
      } else {
        at_target = 1U;
      }

      if (at_target != 0U) {
        vmc_control.getup_left_direction = 0;
        if (vmc_control.getup_left_confirm_ms < confirm_time_ms) {
          vmc_control.getup_left_confirm_ms += dt_ms;
        }
        if (vmc_control.getup_left_confirm_ms >= confirm_time_ms) {
          vmc_control.getup_left_done = 1U;
          vmc_control.getup_left_hold_length =
              (float)vmc_probe_state.left_leg.length;
          vmc_control.getup_left_hold_theta =
              (float)vmc_probe_state.motion.left_theta;
        }
      } else {
        vmc_control.getup_left_confirm_ms = 0U;
        vmc_control.getup_left_direction = direction;
      }
    } else {
      vmc_control.getup_left_direction = 0;
    }

    if (vmc_control.getup_right_done == 0U) {
      int8_t direction = 1;
      uint8_t at_target = 0U;
      const uint32_t confirm_time_ms = VMC_GETUP_LEG_RECOVER_CONFIRM_MS;

      if (vmc_getup_theta_is_in_bad_zone(
              vmc_probe_state.motion.right_theta) != 0U) {
        at_target =
            (fabs(vmc_rad_format((float)(
                      target_phi0 - vmc_probe_state.right_leg.phi0))) <=
             VMC_GETUP_LEG_RECOVER_TOL_RAD)
                ? 1U
                : 0U;
        direction = (int8_t)VMC_GETUP_LEG_RECOVER_DIRECTION;
      } else {
        at_target = 1U;
      }

      if (at_target != 0U) {
        vmc_control.getup_right_direction = 0;
        if (vmc_control.getup_right_confirm_ms < confirm_time_ms) {
          vmc_control.getup_right_confirm_ms += dt_ms;
        }
        if (vmc_control.getup_right_confirm_ms >= confirm_time_ms) {
          vmc_control.getup_right_done = 1U;
          vmc_control.getup_right_hold_length =
              (float)vmc_probe_state.right_leg.length;
          vmc_control.getup_right_hold_theta =
              (float)vmc_probe_state.motion.right_theta;
        }
      } else {
        vmc_control.getup_right_confirm_ms = 0U;
        vmc_control.getup_right_direction = direction;
      }
    } else {
      vmc_control.getup_right_direction = 0;
    }

    if (vmc_control.getup_left_done != 0U &&
        vmc_control.getup_right_done != 0U) {
      uint8_t next_state;

      vmc_control.getup_leg_align_complete = 1U;
      next_state = vmc_getup_next_precondition_state();
      if (next_state == VMC_GETUP_STATE_BODY_ALIGN) {
        vmc_control.getup_body_flip_direction = (pitch > 0.0) ? 1 : -1;
      } else if (next_state == VMC_GETUP_STATE_TURN) {
        vmc_control.getup_turn_direction = (pitch < 0.0) ? 1 : -1;
      }
      vmc_getup_enter_stage(next_state);
    }
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_TURN) {
    vmc_control.getup_left_direction = vmc_control.getup_turn_direction;
    vmc_control.getup_right_direction = vmc_control.getup_turn_direction;
    if (vmc_getup_pose_ready() != 0U) {
      if (vmc_control.getup_confirm_ms < VMC_GETUP_POSE_CONFIRM_MS) {
        vmc_control.getup_confirm_ms += dt_ms;
      }
    } else {
      vmc_control.getup_confirm_ms = 0U;
    }
    if (vmc_control.getup_confirm_ms >= VMC_GETUP_POSE_CONFIRM_MS) {
      vmc_getup_enter_stage(VMC_GETUP_STATE_SHRINK);
    }
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_SHRINK) {
    const double shrink_theta_step =
        clamp_double(VMC_GETUP_SHRINK_THETA_RATE_RADPS, 0.0, 2.0) *
        vmc_loop_dt();

    if (vmc_control.getup_shrink_target_theta_left >
        VMC_GETUP_SHRINK_THETA_TARGET_RAD) {
      vmc_control.getup_shrink_target_theta_left = (float)vmc_step_towards(
          vmc_control.getup_shrink_target_theta_left,
          VMC_GETUP_SHRINK_THETA_TARGET_RAD,
          shrink_theta_step);
    }
    if (vmc_control.getup_shrink_target_theta_right >
        VMC_GETUP_SHRINK_THETA_TARGET_RAD) {
      vmc_control.getup_shrink_target_theta_right = (float)vmc_step_towards(
          vmc_control.getup_shrink_target_theta_right,
          VMC_GETUP_SHRINK_THETA_TARGET_RAD,
          shrink_theta_step);
    }
    if (fabs(vmc_probe_state.left_leg.length - VMC_GETUP_STAND_LENGTH) <
            VMC_GETUP_LENGTH_READY_TOL &&
        fabs(vmc_probe_state.right_leg.length - VMC_GETUP_STAND_LENGTH) <
            VMC_GETUP_LENGTH_READY_TOL) {
      if (vmc_control.getup_confirm_ms < VMC_GETUP_LENGTH_CONFIRM_MS) {
        vmc_control.getup_confirm_ms += dt_ms;
      }
    } else {
      vmc_control.getup_confirm_ms = 0U;
    }
    if (vmc_control.getup_confirm_ms >= VMC_GETUP_LENGTH_CONFIRM_MS) {
      vmc_getup_enter_stage(VMC_GETUP_STATE_UPRIGHT);
      vmc_control.getup_upright_phi0_left =
          vmc_probe_state.left_leg.phi0;
      vmc_control.getup_upright_phi0_right =
          vmc_probe_state.right_leg.phi0;
    }
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_UPRIGHT) {
    if (vmc_getup_upright_ready() != 0U) {
      if (vmc_control.getup_confirm_ms < VMC_GETUP_READY_CONFIRM_MS) {
        vmc_control.getup_confirm_ms += dt_ms;
      }
    } else {
      vmc_control.getup_confirm_ms = 0U;
    }
    if (vmc_control.getup_confirm_ms >= VMC_GETUP_READY_CONFIRM_MS) {
      if (vmc_getup_reset_balance_on_ready != 0U) {
        vmc_stair_reset_balance_for_getup();
        vmc_getup_reset_balance_on_ready = 0U;
      }
      vmc_control.getup_state = VMC_GETUP_STATE_READY;
      vmc_control.getup_turn_direction = 0;
      vmc_control.getup_body_flip_direction = 0;
      vmc_control.getup_left_direction = 0;
      vmc_control.getup_right_direction = 0;
      vmc_control.getup_lqr_release_pending = 0U;
      vmc_control.getup_partial_active = 0U;
      vmc_control.getup_state_ms = 0U;
      vmc_control.getup_confirm_ms = 0U;
      vmc_balance.wheel_output_enable = 1U;
    }
  }
}

static void vmc_update_remote_roll_target(void)
{
  const uint8_t active =
      (remote_control_source == RC_SOURCE_SBUS &&
       remote_control_is_enabled() != 0U)
          ? 1U
          : 0U;
  float input = 0.0f;
  float target = 0.0f;

  if (active != 0U) {
    const float deadband =
        clamp_float(vmc_control.roll_remote_deadband, 0.0f, 0.50f);
    const float max_target =
        clamp_float(vmc_control.roll_remote_target_max, 0.0f, 0.50f);

    input = remote_control_channel(RC_ROLL);
    if (fabsf(input) <= deadband) {
      input = 0.0f;
    }
    /* body.roll is positive when the right side is high.  Channel 3 uses the
     * requested reversed direction: stick-left commands the opposite roll. */
    target = clamp_float(input * max_target, -0.50f, 0.50f);
  }

  vmc_control.roll_target = target;
  vmc_balance.roll_remote_input = input;
  vmc_balance.roll_remote_active = active;
}

static void vmc_update_manual_prepose_mode(void)
{
  const uint32_t now = HAL_GetTick();
  const uint8_t remote_ready =
      (remote_control.valid != 0U &&
       remote_control.rx_count != 0U &&
       remote_control_source == RC_SOURCE_SBUS &&
       (now - remote_control.last_update_ms) <= RC_LOST_TIMEOUT_MS)
          ? 1U
          : 0U;
  const uint8_t enabled = remote_control_is_enabled();
  const uint8_t stair_on =
      (remote_ready != 0U &&
       remote_control_channel(RC_STAIR_ENABLE) > RC_SWITCH_ON_THRESHOLD)
          ? 1U
          : 0U;

  if (remote_ready == 0U) {
    vmc_control.manual_prepose_armed = 0U;
    vmc_control.manual_prepose_active = 0U;
    return;
  }
  if (enabled == 0U) {
    vmc_control.manual_prepose_active = 0U;
    vmc_control.manual_prepose_armed = stair_on;
    return;
  }

  if (vmc_control.manual_prepose_active == 0U) {
    if (vmc_control.manual_prepose_armed != 0U && stair_on != 0U) {
      vmc_control.manual_prepose_active = 1U;
      vmc_control.manual_prepose_armed = 0U;
      vmc_control.manual_prepose_exit_length_check_active = 0U;
      vmc_control.manual_prepose_phi0_left =
          vmc_probe_state.left_leg.phi0;
      vmc_control.manual_prepose_phi0_right =
          vmc_probe_state.right_leg.phi0;
      vmc_control.manual_prepose_knob_position =
          remote_control_leg_length_position();
      vmc_control.manual_prepose_base_length_left = (float)clamp_double(
          vmc_probe_state.left_leg.length,
          VMC_MANUAL_PREPOSE_LENGTH_MIN,
          VMC_MANUAL_PREPOSE_LENGTH_MAX);
      vmc_control.manual_prepose_base_length_right = (float)clamp_double(
          vmc_probe_state.right_leg.length,
          VMC_MANUAL_PREPOSE_LENGTH_MIN,
          VMC_MANUAL_PREPOSE_LENGTH_MAX);
      vmc_control.manual_prepose_length_left =
          vmc_control.manual_prepose_base_length_left;
      vmc_control.manual_prepose_length_right =
          vmc_control.manual_prepose_base_length_right;
    } else if (stair_on == 0U) {
      vmc_control.manual_prepose_armed = 0U;
    }
  }

  if (vmc_control.manual_prepose_active == 0U) {
    return;
  }
  if (stair_on == 0U) {
    vmc_control.manual_prepose_exit_length_check_active = 1U;
    vmc_control.manual_prepose_active = 0U;
    return;
  }

  {
    const double knob_position = clamp_double(
        (double)remote_control_leg_length_position(), -1.0, 1.0);
    const double knob_delta =
        knob_position - (double)vmc_control.manual_prepose_knob_position;
    const double length_delta =
        knob_delta * 0.5 *
        ((double)VMC_MANUAL_PREPOSE_LENGTH_MAX -
         (double)VMC_MANUAL_PREPOSE_LENGTH_MIN);

    vmc_control.manual_prepose_length_left = (float)clamp_double(
        (double)vmc_control.manual_prepose_base_length_left + length_delta,
        VMC_MANUAL_PREPOSE_LENGTH_MIN,
        VMC_MANUAL_PREPOSE_LENGTH_MAX);
    vmc_control.manual_prepose_length_right = (float)clamp_double(
        (double)vmc_control.manual_prepose_base_length_right + length_delta,
        VMC_MANUAL_PREPOSE_LENGTH_MIN,
        VMC_MANUAL_PREPOSE_LENGTH_MAX);
  }

  {
    float input = remote_control_channel(RC_GIMBAL_X);
    const float deadband =
        (float)VMC_RC_CH_DEADBAND / (float)RC_CHANNAL_ERROR_VALUE;
    const double step =
        clamp_double(VMC_MANUAL_PREPOSE_PHI0_RATE_RADPS, 0.0, 3.0) *
        (double)input * vmc_loop_dt();

    if (fabsf(input) <= deadband) {
      input = 0.0f;
    }
    if (input != 0.0f) {
      /* Positive channel 2 is stick-forward. Increasing phi0 rotates the
       * virtual leg forward (counter-clockwise in the leg convention). */
      vmc_control.manual_prepose_phi0_left = (float)clamp_double(
          (double)vmc_control.manual_prepose_phi0_left + step,
          VMC_MANUAL_PREPOSE_PHI0_MIN_RAD,
          VMC_MANUAL_PREPOSE_PHI0_MAX_RAD);
      vmc_control.manual_prepose_phi0_right = (float)clamp_double(
          (double)vmc_control.manual_prepose_phi0_right + step,
          VMC_MANUAL_PREPOSE_PHI0_MIN_RAD,
          VMC_MANUAL_PREPOSE_PHI0_MAX_RAD);
    }
  }
}

static double vmc_spin_phase_lead_rad(double spin_rate)
{
  const double omega = clamp_double(spin_rate, 0.0, 9.0);
  double lead_deg;

  if (omega <= 1.0) {
    lead_deg = 0.0;
  } else if (omega <= 2.0) {
    lead_deg = 11.0 * (omega - 1.0);
  } else if (omega <= 3.0) {
    lead_deg = 11.0 + 9.0 * (omega - 2.0);
  } else if (omega <= 4.0) {
    lead_deg = 20.0 + 5.0 * (omega - 3.0);
  } else if (omega <= 5.0) {
    lead_deg = 25.0 + 5.0 * (omega - 4.0);
  } else if (omega <= 6.0) {
    lead_deg = 30.0 + 3.0 * (omega - 5.0);
  } else {
    lead_deg = 33.0 + 2.0 * (omega - 6.0);
  }

  return lead_deg * (double)VMC_PI / 180.0;
}

static double vmc_spin_projection_gain(double phase_error, double phase_lead,
                                       double power)
{
  const double phase = (double)vmc_rad_format(
      (float)(phase_error + phase_lead));
  const double cosine = cos(phase);
  const double exponent = clamp_double(power, 1.0, 8.0);

  if (cosine > 0.0) {
    return pow(cosine, exponent);
  }
  if (cosine < 0.0) {
    return -pow(-cosine, exponent);
  }
  return 0.0;
}

static void vmc_update_remote_mode(void)
{
  int16_t forward_current;
  int16_t manual_turn_current;
  double turn_input;
  double speed_target_max;
  double yaw_rate_max;
  const double dt = vmc_loop_dt();
  const uint32_t now = HAL_GetTick();
  const uint8_t spin_enabled = remote_control_spin_mode_enabled();
  const double knob_scale = (double)remote_control_spin_rate_scale();
  const double forward_speed_scale =
      (spin_enabled != 0U) ? 1.0 : knob_scale;
  const double spin_rate_max =
      clamp_double(vmc_balance.yaw_spin_rate_max, 0.0, 9.0);
  const double path_offset_rad =
      (double)clamp_float(vmc_balance.spin_line_path_offset_deg, -180.0f, 180.0f) *
      (double)VMC_PI / 180.0;
  const uint8_t imu_ready =
      (vmc_probe_state.body.imu_valid != 0U &&
       vmc_probe_state.body.imu_connected != 0U)
          ? 1U
          : 0U;
  const float spin_forward_input =
      remote_control_channel(RC_CHASSIS_Y);
  const float spin_left_input =
      -remote_control_channel(RC_CHASSIS_X);
  const float spin_input_magnitude = clamp_float(
      sqrtf(spin_forward_input * spin_forward_input +
            spin_left_input * spin_left_input),
      0.0f, 1.0f);
  const uint8_t spin_input_active =
      (spin_input_magnitude >
       ((float)VMC_RC_CH_DEADBAND / (float)RC_CHANNAL_ERROR_VALUE))
          ? 1U
          : 0U;
  uint8_t follow_requested = 0U;

  vmc_update_remote_roll_target();
  vmc_update_manual_prepose_mode();
  vmc_wheel_control.manual_left_current = 0;
  vmc_wheel_control.manual_right_current = 0;
  vmc_wheel_control.manual_forward_current = 0;
  vmc_wheel_control.manual_turn_current = 0;
  manual_turn_current = 0;
  vmc_balance.spin_mode_enabled = spin_enabled;
  vmc_balance.spin_line_input_forward = spin_forward_input;
  vmc_balance.spin_line_input_lateral = spin_left_input;
  vmc_balance.spin_line_input_magnitude = spin_input_magnitude;
  vmc_balance.yaw_spin_rate =
      (spin_enabled != 0U)
          ? (float)((double)VMC_RC_YAW_SPIN_RATE_MIN_DEFAULT +
                    knob_scale * (spin_rate_max -
                                  (double)VMC_RC_YAW_SPIN_RATE_MIN_DEFAULT))
          : 0.0f;
  vmc_balance.spin_line_phase_error = 0.0f;
  vmc_balance.spin_line_phase_lead_rad = 0.0f;
  vmc_balance.spin_line_gate = 0.0f;

  if (spin_enabled != vmc_spin_mode_last) {
    vmc_balance.yaw_target = vmc_probe_state.body.yaw;
    vmc_balance.yaw_hold_active = 0U;
    vmc_balance.spin_line_heading_valid = imu_ready;
    if (imu_ready != 0U) {
      vmc_balance.spin_line_origin_yaw = vmc_probe_state.body.yaw;
      vmc_balance.spin_line_yaw = vmc_rad_format(
          vmc_probe_state.body.yaw + (float)path_offset_rad);
    }
    vmc_spin_mode_last = spin_enabled;
  }

  if (spin_enabled != 0U && vmc_balance.spin_line_heading_valid == 0U &&
      imu_ready != 0U) {
    vmc_balance.spin_line_origin_yaw = vmc_probe_state.body.yaw;
    vmc_balance.spin_line_yaw = vmc_rad_format(
        vmc_probe_state.body.yaw + (float)path_offset_rad);
    vmc_balance.spin_line_heading_valid = 1U;
  }

  if (spin_enabled != 0U && spin_input_active != 0U) {
    const float direction_offset = atan2f(spin_left_input, spin_forward_input);
    vmc_balance.spin_line_yaw = vmc_rad_format(
        vmc_balance.spin_line_origin_yaw + direction_offset +
        (float)path_offset_rad);
  }

  if (remote_control_source != vmc_remote_source_last) {
    vmc_balance.yaw_target = vmc_probe_state.body.yaw;
    vmc_balance.yaw_hold_active = 0U;
    vmc_balance.gimbal_follow_active = 0U;
    vmc_wheel_control.gimbal_follow_current = 0;
    vmc_remote_source_last = remote_control_source;
  }

  if (remote_control.valid == 0U || remote_control.rx_count == 0U ||
      (now - remote_control.last_update_ms) > RC_LOST_TIMEOUT_MS) {
    vmc_control.enabled = 0U;
    vmc_balance.wheel_output_enable = 0U;
    vmc_balance.speed_target = 0.0f;
    vmc_balance.yaw_hold_active = 0U;
    vmc_balance.gimbal_follow_active = 0U;
    vmc_wheel_control.gimbal_follow_current = 0;
    vmc_fall_monitor_armed = 0U;
    return;
  }

  if ((vmc_fall_rearm_required != 0U && remote_control.s[1] != RC_SW_DOWN) ||
      (vmc_fall_rearm_armed != 0U && remote_control.s[1] != RC_SW_DOWN &&
       remote_control.s[1] != RC_SW_MID)) {
    vmc_control.enabled = 0U;
    vmc_balance.wheel_output_enable = 0U;
    vmc_balance.speed_target = 0.0f;
    vmc_balance.yaw_hold_active = 0U;
    vmc_balance.gimbal_follow_active = 0U;
    vmc_wheel_control.gimbal_follow_current = 0;
    return;
  }

  if (remote_control_is_enabled() != 0U) {
    vmc_control.enabled = 1U;
    /* Channel 4 DOWN is the only position that drives.  Jump, stairs and spin
     * have their own channels and are additionally gated by this master
     * enable. */
    vmc_balance.wheel_output_enable =
        (vmc_control.manual_prepose_active != 0U ||
         (vmc_control.manual_prepose_exit_length_check_active == 0U &&
          vmc_getup_fully_fallen == 0U && vmc_getup_is_active() == 0U))
            ? 1U
            : 0U;
  } else {
    vmc_control.enabled = 0U;
    vmc_balance.wheel_output_enable = 0U;
    vmc_balance.speed_target = 0.0f;
    vmc_balance.yaw_hold_active = 0U;
    vmc_balance.gimbal_follow_active = 0U;
    vmc_wheel_control.gimbal_follow_current = 0;
    return;
  }

  if (vmc_jump_blocks_wheel_output() != 0U) {
    vmc_balance.wheel_output_enable = 0U;
    vmc_balance.speed_target = 0.0f;
    vmc_balance.speed_target_ramped = 0.0f;
    vmc_balance.speed_stop_hold_active = 0U;
    vmc_balance.yaw_hold_active = 0U;
    vmc_balance.gimbal_follow_active = 0U;
    vmc_wheel_control.yaw_hold_current = 0;
    vmc_wheel_control.gimbal_follow_current = 0;
    vmc_wheel_control.left_current = 0;
    vmc_wheel_control.right_current = 0;
    return;
  }

  if (spin_enabled != 0U) {
    if (vmc_balance.spin_line_heading_valid != 0U && imu_ready != 0U) {
      const double phase_error = (double)vmc_rad_format(
          (float)((double)vmc_probe_state.body.yaw -
                  (double)vmc_balance.spin_line_yaw));
      const double phase_lead =
          vmc_spin_phase_lead_rad((double)vmc_balance.yaw_spin_rate);

      vmc_balance.spin_line_phase_error = (float)phase_error;
      vmc_balance.spin_line_phase_lead_rad = (float)phase_lead;
      vmc_balance.spin_line_gate = (float)vmc_spin_projection_gain(
          phase_error, phase_lead,
          (double)vmc_balance.spin_line_projection_power);
    } else {
      vmc_balance.spin_line_phase_lead_rad = 0.0f;
      vmc_balance.spin_line_gate = 0.0f;
    }
  } else {
    vmc_balance.spin_line_phase_lead_rad = 0.0f;
    vmc_balance.spin_line_gate = 1.0f;
  }

  if (vmc_control.manual_prepose_active != 0U) {
    /* Manual pre-positioning uses the left vertical stick as a direct
     * forward/reverse wheel command. There is no yaw, spin-line projection or
     * balance-loop contribution in this path. */
    forward_current = rc_channel_to_current(
        (int16_t)(-remote_control_channel(RC_CHASSIS_Y) *
                  RC_CHANNAL_ERROR_VALUE),
        VMC_RC_WHEEL_FWD_CURRENT_MAX);
    /* Channel 0 positive is stick-right. Differential wheel current with
     * positive turn commanding the left wheel forward and right wheel reverse
     * rotates the chassis clockwise from the driver's view. */
    manual_turn_current = rc_channel_to_current(
        (int16_t)(remote_control_channel(RC_CHASSIS_X) *
                  RC_CHANNAL_ERROR_VALUE),
        VMC_RC_WHEEL_TURN_CURRENT_MAX);
  } else {
    forward_current =
        (spin_enabled != 0U)
            ? rc_channel_to_current(
                  (int16_t)(spin_input_magnitude * RC_CHANNAL_ERROR_VALUE),
                  VMC_RC_WHEEL_FWD_CURRENT_MAX)
            : rc_channel_to_current(
                  (int16_t)(remote_control_channel(RC_CHASSIS_Y) *
                            RC_CHANNAL_ERROR_VALUE),
                  VMC_RC_WHEEL_FWD_CURRENT_MAX);
    forward_current = (int16_t)((double)forward_current * forward_speed_scale *
                                (double)vmc_balance.spin_line_gate);
  }
  turn_input = rc_channel_to_unit(
      (int16_t)(-remote_control_channel(RC_CHASSIS_X) *
                RC_CHANNAL_ERROR_VALUE));
  vmc_wheel_control.manual_forward_current = forward_current;
  vmc_wheel_control.manual_turn_current = manual_turn_current;

  if (remote_control_source == RC_SOURCE_CBOARD &&
      vmc_balance.gimbal_follow_enable != 0U &&
      vmc_balance.gimbal_follow_front_valid != 0U &&
      vmc_balance.wheel_output_enable != 0U &&
      board_comm_remote_is_valid() != 0U &&
      (board_comm_state.remote_flags & BOARD_COMM_FLAG_YAW_ACTIVE) != 0U &&
      (board_comm_state.remote_flags & BOARD_COMM_FLAG_YAW_MOTOR_ONLINE) != 0U &&
      vmc_getup_fully_fallen == 0U &&
      vmc_getup_is_active() == 0U) {
    follow_requested = 1U;
  }

  if (follow_requested != 0U) {
    vmc_balance.gimbal_follow_active = 1U;
    vmc_balance.yaw_hold_active = 0U;
  } else {
    if (vmc_balance.gimbal_follow_active != 0U) {
      vmc_balance.yaw_target = vmc_probe_state.body.yaw;
      vmc_balance.yaw_hold_active = 0U;
      vmc_wheel_control.gimbal_follow_current = 0;
    }
    vmc_balance.gimbal_follow_active = 0U;
  }

  if (vmc_balance.gimbal_follow_active == 0U &&
      vmc_control.manual_prepose_active == 0U &&
      vmc_balance.yaw_hold_enable != 0U &&
      vmc_balance.wheel_output_enable != 0U &&
      vmc_probe_state.body.imu_valid != 0U &&
      vmc_probe_state.body.imu_connected != 0U) {
    if (vmc_balance.yaw_hold_active == 0U) {
      vmc_balance.yaw_target = vmc_probe_state.body.yaw;
      vmc_balance.yaw_hold_active = 1U;
    }
    if (spin_enabled != 0U) {
      const double spin_rate = clamp_double(
          vmc_balance.yaw_spin_rate,
          (double)VMC_RC_YAW_SPIN_RATE_MIN_DEFAULT,
          (double)VMC_RC_YAW_SPIN_RATE_MAX_DEFAULT);
      vmc_balance.yaw_target =
          (float)((double)vmc_balance.yaw_target + spin_rate * dt);
    } else {
      yaw_rate_max = clamp_double(vmc_balance.yaw_target_rate_max, 0.0, 10.0);
      vmc_balance.yaw_target =
          (float)((double)vmc_balance.yaw_target +
                  turn_input * yaw_rate_max * dt);
    }
  }

  if (vmc_control.manual_prepose_active != 0U) {
    vmc_balance.speed_target = 0.0f;
    vmc_balance.speed_target_ramped = 0.0f;
    vmc_balance.speed_stop_hold_active = 0U;
  } else if (vmc_balance.speed_control_enable != 0U) {
    speed_target_max = clamp_double(vmc_balance.speed_target_max, 0.0, 3.0);
    vmc_balance.speed_target =
        (float)((-(double)forward_current * speed_target_max) /
                (double)VMC_RC_WHEEL_FWD_CURRENT_MAX);

  } else {
    vmc_balance.speed_target = 0.0f;
    vmc_balance.speed_target_ramped = 0.0f;
    vmc_balance.speed_stop_hold_active = 0U;

    vmc_wheel_control.manual_left_current =
        clamp_i16(forward_current,
                  -VMC_WHEEL_CURRENT_MAX_DEFAULT,
                  VMC_WHEEL_CURRENT_MAX_DEFAULT);
    vmc_wheel_control.manual_right_current =
        clamp_i16(forward_current,
                  -VMC_WHEEL_CURRENT_MAX_DEFAULT,
                  VMC_WHEEL_CURRENT_MAX_DEFAULT);
  }

}

static void vmc_update_body_state(void)
{
  float pitch;
  float pitch_rate;

  vmc_probe_state.body.roll = hi12_imu.angle_rad[0];
  pitch = VMC_BODY_PITCH_DIRECTION * hi12_imu.angle_rad[1];
  pitch_rate = VMC_BODY_PITCH_RATE_DIRECTION * hi12_imu.gyro_radps[0];
  vmc_probe_state.body.pitch = pitch;
  {
    const float yaw_wrapped = hi12_imu.angle_rad[2];
    if (hi12_imu.connected == 0U || hi12_imu.valid == 0U) {
      vmc_yaw_unwrap_initialized = 0U;
      vmc_yaw_unwrapped = yaw_wrapped;
    } else if (vmc_yaw_unwrap_initialized == 0U) {
      vmc_yaw_last_wrapped = yaw_wrapped;
      vmc_yaw_unwrapped = yaw_wrapped;
      vmc_yaw_unwrap_initialized = 1U;
    } else {
      vmc_yaw_unwrapped += vmc_rad_format(yaw_wrapped - vmc_yaw_last_wrapped);
      vmc_yaw_last_wrapped = yaw_wrapped;
    }
    vmc_probe_state.body.yaw = vmc_yaw_unwrapped;
  }
  vmc_probe_state.body.pitch_rate = pitch_rate;
  vmc_probe_state.body.wx = pitch_rate;
  vmc_probe_state.body.wy = hi12_imu.gyro_radps[1];
  vmc_probe_state.body.wz = hi12_imu.gyro_radps[2];
  vmc_probe_state.body.ax = hi12_imu.acc_g[0];
  vmc_probe_state.body.ay = hi12_imu.acc_g[1];
  vmc_probe_state.body.az = hi12_imu.acc_g[2];
  vmc_probe_state.body.imu_connected = hi12_imu.connected;
  vmc_probe_state.body.imu_valid = hi12_imu.valid;
}

/* Remove the gravity reaction from the body-frame accelerometer reading.
 *
 * The HI12 publishes acc_b in g with an undocumented sign convention, so the
 * direction of gravity is taken from the attitude quaternion and the sign is
 * measured instead of assumed: the raw reading's projection onto that
 * direction averages to +1 g or -1 g, and the gravity term keeps that sign
 * even while the vehicle moves.  Until the average resolves, no compensation
 * is applied and gravity_sign stays 0 so consumers can withhold the sample. */
static void vmc_update_gravity_compensation(void)
{
  const float w = hi12_imu.quaternion[0];
  const float qx = hi12_imu.quaternion[1];
  const float qy = hi12_imu.quaternion[2];
  const float qz = hi12_imu.quaternion[3];
  const float norm_squared = w * w + qx * qx + qy * qy + qz * qz;
  float g_unit_x = 0.0f;
  float g_unit_y = 0.0f;
  float g_unit_z = 0.0f;

  if (hi12_imu.valid == 0U || norm_squared <= 1.0e-6f) {
    /* No trustworthy attitude: restart calibration and do not compensate. */
    vmc_gravity_dot_lpf = 0.0f;
    vmc_gravity_sign = 0;
  } else {
    /* Normalise first so the rotation-matrix rows stay unit length even if the
     * module reports a slightly scaled quaternion. */
    const float inv_norm = 1.0f / sqrtf(norm_squared);
    const float nw = w * inv_norm;
    const float nx = qx * inv_norm;
    const float ny = qy * inv_norm;
    const float nz = qz * inv_norm;

    /* World +Z axis expressed in the body frame (third row of R). */
    g_unit_x = 2.0f * (nx * nz - nw * ny);
    g_unit_y = 2.0f * (ny * nz + nw * nx);
    g_unit_z = 1.0f - 2.0f * (nx * nx + ny * ny);

    if (vmc_gravity_sign == 0) {
      const float dot = vmc_probe_state.body.ax * g_unit_x +
                        vmc_probe_state.body.ay * g_unit_y +
                        vmc_probe_state.body.az * g_unit_z;

      vmc_gravity_dot_lpf +=
          VMC_GRAVITY_SIGN_ALPHA * (dot - vmc_gravity_dot_lpf);
      if (fabsf(vmc_gravity_dot_lpf) >= VMC_GRAVITY_SIGN_MARGIN) {
        vmc_gravity_sign = (vmc_gravity_dot_lpf >= 0.0f) ? 1 : -1;
      } else {
        /* Unresolved: leave g_unit at zero so nothing is subtracted. */
        g_unit_x = 0.0f;
        g_unit_y = 0.0f;
        g_unit_z = 0.0f;
      }
    }
  }

  if (vmc_gravity_sign < 0) {
    g_unit_x = -g_unit_x;
    g_unit_y = -g_unit_y;
    g_unit_z = -g_unit_z;
  }

  vmc_probe_state.body.ax_lin = vmc_probe_state.body.ax - g_unit_x;
  vmc_probe_state.body.ay_lin = vmc_probe_state.body.ay - g_unit_y;
  vmc_probe_state.body.az_lin = vmc_probe_state.body.az - g_unit_z;
  if (vmc_gravity_sign != 0) {
    vmc_probe_state.body.world_az_lin =
        (float)vmc_gravity_sign *
        (vmc_probe_state.body.ax_lin * g_unit_x +
         vmc_probe_state.body.ay_lin * g_unit_y +
         vmc_probe_state.body.az_lin * g_unit_z) *
        VMC_GRAVITY_ACCEL;
  } else {
    vmc_probe_state.body.world_az_lin = 0.0f;
  }
  vmc_probe_state.body.gravity_sign = vmc_gravity_sign;
}

static void vmc_update_motion_state(void)
{
  const double left_theta = vmc_rad_format((float)(vmc_probe_state.left_leg.phi0 -
                                                  VMC_PHI0_VERTICAL_RAD -
                                                  vmc_probe_state.body.pitch));
  const double right_theta = vmc_rad_format((float)(vmc_probe_state.right_leg.phi0 -
                                                   VMC_PHI0_VERTICAL_RAD -
                                                   vmc_probe_state.body.pitch));
  const double d_theta = (vmc_probe_state.left_leg.d_phi0 +
                         vmc_probe_state.right_leg.d_phi0) *
                             0.5 -
                         vmc_probe_state.body.pitch_rate;
  const double left_wheel_w = vmc_probe_state.wheel_left.speed_rads -
                              vmc_probe_state.body.pitch_rate -
                              vmc_probe_state.left_leg.d_phi0;
  const double right_wheel_w = -vmc_probe_state.wheel_right.speed_rads -
                               vmc_probe_state.body.pitch_rate -
                               vmc_probe_state.right_leg.d_phi0;

  vmc_probe_state.motion.left_theta = left_theta;
  vmc_probe_state.motion.left_d_theta = vmc_probe_state.left_leg.d_phi0 -
                                        vmc_probe_state.body.pitch_rate;
  vmc_probe_state.motion.right_theta = right_theta;
  vmc_probe_state.motion.right_d_theta = vmc_probe_state.right_leg.d_phi0 -
                                         vmc_probe_state.body.pitch_rate;
  vmc_probe_state.motion.theta = vmc_rad_format((float)(
      ((vmc_probe_state.left_leg.phi0 - VMC_PHI0_VERTICAL_RAD) +
       (vmc_probe_state.right_leg.phi0 - VMC_PHI0_VERTICAL_RAD)) * 0.5 -
      vmc_probe_state.body.pitch));
  vmc_probe_state.motion.d_theta = d_theta;
  vmc_probe_state.motion.phi = vmc_probe_state.body.pitch;
  vmc_probe_state.motion.d_phi = vmc_probe_state.body.pitch_rate;
  vmc_probe_state.motion.left_wheel_w = left_wheel_w;
  vmc_probe_state.motion.right_wheel_w = right_wheel_w;
  vmc_probe_state.motion.wheel_dx = (-left_wheel_w - right_wheel_w) * VMC_WHEEL_RADIUS;
  vmc_probe_state.motion.wheel_dx_gap = (left_wheel_w - right_wheel_w) * VMC_WHEEL_RADIUS;
  vmc_probe_state.motion.left_body_dx =
      -left_wheel_w * VMC_WHEEL_RADIUS +
      vmc_probe_state.left_leg.length * d_theta * cos(left_theta) +
      vmc_probe_state.left_leg.d_length * sin(left_theta);
  vmc_probe_state.motion.right_body_dx =
      -right_wheel_w * VMC_WHEEL_RADIUS +
      vmc_probe_state.right_leg.length * d_theta * cos(right_theta) +
      vmc_probe_state.right_leg.d_length * sin(right_theta);
  vmc_probe_state.motion.d_x_raw =
      (vmc_probe_state.motion.left_body_dx + vmc_probe_state.motion.right_body_dx) * 0.5;
  /* Gravity-compensated fore-aft linear acceleration in m/s2.
   *
   * MEASURED, not assumed - the HI12's accelerometer channels are not the
   * body frame's.  At rest gravity sits on channel 2, so that one is vertical
   * as expected, but it moves onto channel 1 when the body PITCHES and onto
   * channel 0 when it ROLLS.  The fore-aft axis is therefore channel 1;
   * body.ax/ay/az above keep the IMU's own channel order, which is what the
   * published telemetry reports.
   *
   * Pushing the machine by hand fixes the sign: over sixty 1-second windows of
   * a walk-push recording,
   *
   *     corr(delta wheel velocity, integral of ay_lin) = -0.999
   *     corr(delta wheel velocity, integral of ax_lin) = +0.019
   *
   * with the fitted scale at -0.87 against an ideal -1 (hand push, some wheel
   * skid, and a 50 Hz integral).  So the fore-aft acceleration is -ay_lin, and
   * the wheel kinematics agree with it to within that scale error.
   *
   * This used to read ax_lin, the LATERAL axis.  That is why an earlier
   * recording showed ax_lin integrating to 0.04 m/s across a run that moved
   * the machine 2.2 m/s: it was watching the wrong axis, and the only thing it
   * correlated with was roll.  dd_x feeds both the prediction of the
   * body-motion Kalman filter and condition C of the slip detector
   * (|wheel acceleration| - |body acceleration|), so both were being fed a
   * sideways signal - and the slip detector's one action, rejecting the wheel
   * measurement while latched, coasts that filter on this same input. */
  vmc_probe_state.motion.dd_x =
      -(double)vmc_probe_state.body.ay_lin * (double)VMC_GRAVITY_ACCEL;

  if (vmc_control.enabled == 0U || vmc_probe_state.feedback_timeout_active != 0U) {
    vmc_body_motion_kf_reset();
    vmc_body_motion_kf_slip_reset(0.0, 0.0, 0.0);
    body_motion_kf_slip_initialized = 0U;
    vmc_last_imu_rx_count = hi12_imu.rx_count;
  } else {
    const uint8_t acceleration_valid =
        (hi12_imu.rx_count != vmc_last_imu_rx_count &&
         vmc_probe_state.body.imu_valid != 0U &&
         vmc_probe_state.body.gravity_sign != 0)
            ? 1U
            : 0U;
    /* Raw per-wheel body speed in m/s.  Condition C must differentiate these
     * rather than left_body_dx/right_body_dx, which additionally carry the
     * leg swing and leg-length servo terms; differentiating those turns
     * commanded leg motion into apparent wheel acceleration. */
    const double wheel_left_raw =
        (double)vmc_probe_state.wheel_left.speed_rads * (double)VMC_WHEEL_RADIUS;
    const double wheel_right_raw =
        (double)vmc_probe_state.wheel_right.speed_rads * (double)VMC_WHEEL_RADIUS;
    double slip_metric = 0.0;
    uint8_t slip_latched = 0U;
    uint8_t slip_filter_active = 0U;

    vmc_last_imu_rx_count = hi12_imu.rx_count;
    if (body_motion_kf_slip_initialized == 0U) {
      vmc_body_motion_kf_slip_reset(wheel_left_raw, wheel_right_raw,
                                    vmc_probe_state.motion.dd_x);
    }
    /* Keep the evidence channel independent of the filter action.  The gate
     * is always evaluated so a capture can compare raw features, but the
     * estimator only rejects wheel measurements when slip_enable is set. */
    slip_latched = vmc_body_motion_kf_slip_gate(
        vmc_probe_state.motion.left_body_dx,
        vmc_probe_state.motion.right_body_dx,
        wheel_left_raw,
        wheel_right_raw,
        vmc_probe_state.motion.dd_x,
        /* The previous fused velocity is the body-speed reference used by
         * condition B, matching the USTB d_x. */
        body_motion_kf_x[0],
        (double)vmc_probe_state.body.wz,
        acceleration_valid,
        vmc_loop_dt(),
        &slip_metric);
    slip_filter_active =
        (vmc_balance.kalman_slip_enable != 0U && slip_latched != 0U) ? 1U : 0U;
    vmc_probe_state.motion.kf_slip =
        (slip_filter_active != 0U) ? 1.0 : 0.0;
    vmc_probe_state.motion.kf_slip_metric = slip_metric;
    vmc_probe_state.motion.kf_slip_left =
        (slip_filter_active != 0U && body_motion_kf_slip_left != 0U) ? 1.0 : 0.0;
    vmc_probe_state.motion.kf_slip_right =
        (slip_filter_active != 0U && body_motion_kf_slip_right != 0U) ? 1.0 : 0.0;
    vmc_probe_state.motion.slip_raw_latched =
        (slip_latched != 0U) ? 1.0 : 0.0;
    vmc_probe_state.motion.slip_raw_left =
        (body_motion_kf_slip_left != 0U) ? 1.0 : 0.0;
    vmc_probe_state.motion.slip_raw_right =
        (body_motion_kf_slip_right != 0U) ? 1.0 : 0.0;
    vmc_probe_state.motion.kf_wheel_left_raw = wheel_left_raw;
    vmc_probe_state.motion.kf_wheel_right_raw = wheel_right_raw;

    vmc_body_motion_kf_update(vmc_probe_state.motion.d_x_raw,
                              vmc_probe_state.motion.dd_x,
                              acceleration_valid,
                              slip_filter_active);
  }
}

static void vmc_update_lqr_gain(void)
{
  const double leg_length_avg =
      (vmc_probe_state.left_leg.length + vmc_probe_state.right_leg.length) * 0.5;
  uint8_t i;

  vmc_probe_state.motion.leg_length_avg = leg_length_avg;
  if (vmc_probe_state.lqr.rtt_override_active != 0U) {
    for (i = 0U; i < 12U; i++) {
      vmc_probe_state.lqr.left_k[i] = vmc_probe_state.lqr.rtt_k[i];
      vmc_probe_state.lqr.right_k[i] = vmc_probe_state.lqr.rtt_k[i];
    }
    return;
  }

  vmc_lqr_update_gain(leg_length_avg, (double *)vmc_probe_state.lqr.left_k);
  vmc_lqr_update_gain(leg_length_avg, (double *)vmc_probe_state.lqr.right_k);
}

static uint8_t vmc_safety_allows_output(void)
{
  return (vmc_fall_rearm_required == 0U &&
          vmc_probe_state.feedback_timeout_active == 0U &&
          vmc_probe_state.airborne_active == 0U &&
          vmc_jump_blocks_wheel_output() == 0U &&
          remote_control_allows_joint_enable() != 0U)
             ? 1U
             : 0U;
}

static uint8_t vmc_manual_prepose_allows_wheel_output(void)
{
  /* Manual pre-positioning intentionally bypasses airborne detection: the
   * operator may be lying on one side with both legs unloaded while moving the
   * chassis away from an obstacle. Remote, feedback and fall-rearm protection
   * still apply. */
  return (vmc_control.manual_prepose_active != 0U &&
          vmc_fall_rearm_required == 0U &&
          vmc_probe_state.feedback_timeout_active == 0U &&
          remote_control_allows_joint_enable() != 0U)
             ? 1U
             : 0U;
}

/* Reconstruct Shanhai's virtual-leg force from the measured joint torques.
 * Their input is motor torque after removing the gas-spring feedforward.  This
 * linkage has no gas-spring term in vmc_leg_force(), so the inverse of that
 * same command Jacobian supplies the equivalent F/Tp directly. */
static uint8_t vmc_leg_force_from_measurement(
    volatile vmc_leg_state_t *leg, double ahead_direction,
    double back_direction, double *force, double *torque_phi0)
{
  double joint_torque[2];

  if (ahead_direction == 0.0 || back_direction == 0.0) {
    return 0U;
  }
  joint_torque[0] =
      (double)((leg == &vmc_probe_state.left_leg)
                   ? vmc_probe_state.joint_left_ahead.torque
                   : vmc_probe_state.joint_right_ahead.torque) /
      ahead_direction;
  joint_torque[1] =
      (double)((leg == &vmc_probe_state.left_leg)
                   ? vmc_probe_state.joint_left_back.torque
                   : vmc_probe_state.joint_right_back.torque) /
      back_direction;
  return vmc_leg_virtual_force(joint_torque, leg->phi1, leg->phi4, force,
                               torque_phi0);
}

/* Shanhai Observer_GetFN().  theta is body-relative and has the opposite sign
 * to motion.*_theta, matching the Shanhai convention used in the formula. */
static uint8_t vmc_side_support_force(uint8_t side, float *normal_force)
{
  const uint8_t left = (side == VMC_AIRBORNE_SIDE_LEFT) ? 1U : 0U;
  volatile vmc_leg_state_t *leg =
      left != 0U ? &vmc_probe_state.left_leg : &vmc_probe_state.right_leg;
  const double ahead_direction =
      left != 0U ? (double)VMC_DIR_LEFT_AHEAD : (double)VMC_DIR_RIGHT_AHEAD;
  const double back_direction =
      left != 0U ? (double)VMC_DIR_LEFT_BACK : (double)VMC_DIR_RIGHT_BACK;
  const double theta =
      (double)vmc_rad_format((float)(
          VMC_PHI0_VERTICAL_RAD - leg->phi0 - vmc_probe_state.body.pitch));
  const double d_theta =
      -(leg->d_phi0 + (double)vmc_probe_state.body.pitch_rate);
  double force;
  double torque_phi0;
  double p;
  double ddz_w;

  if (leg->length <= 0.01 ||
      vmc_leg_force_from_measurement(leg, ahead_direction, back_direction,
                                     &force, &torque_phi0) == 0U) {
    return 0U;
  }

  /* This controller's support-force command points through
   * VMC_LEG_SUPPORT_FORCE_DIRECTION = -1; Shanhai's FN observer uses the
   * opposite positive-compression convention.  Tp has the same sign in both
   * paths, while theta above already uses Shanhai's body-relative sign. */
  force = -force;
  p = force * cos(theta) + torque_phi0 * sin(theta) / leg->length;
  ddz_w = (double)vmc_probe_state.body.world_az_lin -
          leg->dd_length * cos(theta) +
          2.0 * leg->d_length * d_theta * sin(theta) +
          leg->length * leg->dd_theta * sin(theta) +
          leg->length * d_theta * d_theta * cos(theta);
  *normal_force = (float)(
      p + (double)VMC_WHEEL_MASS_KG * (double)VMC_GRAVITY_ACCEL +
      (double)VMC_WHEEL_MASS_KG * ddz_w);
  return 1U;
}

/* Per-side Shanhai ground latch.  FN below the cutoff means the wheel is
 * unloaded, so only that wheel is disabled.  The leg controller, its gains and
 * the selected channel-9 length target remain untouched. */
static void vmc_update_airborne_state(void)
{
  const uint32_t now = HAL_GetTick();
  uint32_t dt_ms = 0U;
  const float left_wheel_speed = fabsf(vmc_probe_state.wheel_left.speed_rads);
  const float right_wheel_speed = fabsf(vmc_probe_state.wheel_right.speed_rads);
  const float wheel_speed = (left_wheel_speed > right_wheel_speed)
                                ? left_wheel_speed
                                : right_wheel_speed;
  const float joint_torque = vmc_average_joint_torque_abs();
  float joint_torque_min;
  float joint_torque_max;
  const float force_threshold =
      clamp_float(vmc_balance.airborne_joint_torque_threshold, 1.0f, 500.0f);
  const float release_threshold =
      force_threshold * VMC_AIRBORNE_FN_RELEASE_RATIO_DEFAULT;
  uint32_t confirm_ms = vmc_balance.airborne_confirm_ms;
  float side_fn[VMC_AIRBORNE_SIDE_COUNT];
  uint8_t side_fn_valid[VMC_AIRBORNE_SIDE_COUNT];
  const uint8_t detector_ready =
      (vmc_balance.airborne_detect_enable != 0U &&
       vmc_control.enabled != 0U &&
       vmc_probe_state.feedback_timeout_active == 0U &&
       vmc_getup_is_active() == 0U &&
       vmc_jump_is_active_or_aborted() == 0U &&
       remote_control_allows_joint_enable() != 0U)
          ? 1U
          : 0U;

  vmc_joint_torque_abs_extremes(&joint_torque_min, &joint_torque_max);

  if (vmc_airborne_last_update_ms != 0U) {
    dt_ms = now - vmc_airborne_last_update_ms;
    if (dt_ms > 100U) {
      dt_ms = 100U;
    }
  }
  vmc_airborne_last_update_ms = now;
  if (confirm_ms > 10000U) {
    confirm_ms = 10000U;
  }
  vmc_probe_state.airborne_wheel_speed = wheel_speed;
  vmc_probe_state.airborne_joint_torque = joint_torque;
  vmc_probe_state.airborne_joint_torque_min = joint_torque_min;
  vmc_probe_state.airborne_joint_torque_max = joint_torque_max;
  vmc_probe_state.airborne_pitch_reference = VMC_AIRBORNE_PITCH_REFERENCE_NONE;
  vmc_probe_state.airborne_pitch_reference_valid = 0U;
  vmc_probe_state.airborne_pitch_reference_hold_ms = 0U;

  {
    const uint32_t side_confirm_ms =
        (vmc_balance.airborne_side_confirm_ms > 10000U)
            ? 10000U
            : vmc_balance.airborne_side_confirm_ms;
    const uint8_t side_ready =
        (detector_ready != 0U &&
         vmc_balance.airborne_side_enable != 0U &&
         vmc_jump_joint_lqr_active() == 0U)
            ? 1U
            : 0U;
    uint8_t side;

    for (side = 0U; side < VMC_AIRBORNE_SIDE_COUNT; side++) {
      side_fn_valid[side] = vmc_side_support_force(side, &side_fn[side]);
      if (side_fn_valid[side] != 0U) {
        if (vmc_probe_state.airborne_side_fn_filter_initialized[side] == 0U) {
          vmc_probe_state.airborne_side_fn_filtered[side] = side_fn[side];
          vmc_probe_state.airborne_side_fn_filter_initialized[side] = 1U;
        } else {
          vmc_probe_state.airborne_side_fn_filtered[side] +=
              VMC_AIRBORNE_FN_FILTER_ALPHA_DEFAULT *
              (side_fn[side] -
               vmc_probe_state.airborne_side_fn_filtered[side]);
        }
        side_fn[side] = vmc_probe_state.airborne_side_fn_filtered[side];
      }
      vmc_probe_state.airborne_side_torque[side] =
          (side_fn_valid[side] != 0U) ? side_fn[side] : 0.0f;

      if (side_ready == 0U || side_fn_valid[side] == 0U) {
        vmc_probe_state.airborne_side_active[side] = 0U;
        vmc_probe_state.airborne_side_candidate[side] = 0U;
        vmc_probe_state.airborne_side_candidate_ms[side] = 0U;
        vmc_probe_state.airborne_side_release_candidate_ms[side] = 0U;
        vmc_probe_state.airborne_side_fn_filter_initialized[side] = 0U;
        vmc_probe_state.airborne_side_posture_active[side] = 0U;
        vmc_probe_state.airborne_side_posture_ms[side] = 0U;
        vmc_probe_state.airborne_side_length_returning[side] = 0U;
        continue;
      }

      if (vmc_probe_state.airborne_side_active[side] != 0U) {
        if (side_fn[side] >= release_threshold) {
          vmc_probe_state.airborne_side_active[side] = 0U;
          vmc_probe_state.airborne_side_candidate[side] = 0U;
          vmc_probe_state.airborne_side_candidate_ms[side] = 0U;
        }
      } else if (side_fn[side] < force_threshold) {
        vmc_probe_state.airborne_side_candidate[side] = 1U;
        if (vmc_probe_state.airborne_side_candidate_ms[side] <
            side_confirm_ms) {
          vmc_probe_state.airborne_side_candidate_ms[side] += dt_ms;
        }
        if (vmc_probe_state.airborne_side_candidate_ms[side] >=
            side_confirm_ms) {
          vmc_probe_state.airborne_side_active[side] = 1U;
          vmc_probe_state.airborne_side_release_candidate_ms[side] = 0U;
          vmc_probe_state.airborne_side_posture_active[side] = 1U;
          vmc_probe_state.airborne_side_posture_ms[side] = 0U;
          vmc_probe_state.airborne_side_length_returning[side] = 0U;
        }
      } else {
        vmc_probe_state.airborne_side_candidate[side] = 0U;
        vmc_probe_state.airborne_side_candidate_ms[side] = 0U;
      }

      vmc_probe_state.airborne_side_release_candidate_ms[side] = 0U;
      if (vmc_probe_state.airborne_side_posture_active[side] != 0U) {
        if (vmc_probe_state.airborne_side_posture_ms[side] <
            VMC_AIRBORNE_SIDE_HOLD_MS_DEFAULT) {
          vmc_probe_state.airborne_side_posture_ms[side] += dt_ms;
          if (vmc_probe_state.airborne_side_posture_ms[side] >
              VMC_AIRBORNE_SIDE_HOLD_MS_DEFAULT) {
            vmc_probe_state.airborne_side_posture_ms[side] =
                VMC_AIRBORNE_SIDE_HOLD_MS_DEFAULT;
          }
        }
        if (vmc_probe_state.airborne_side_active[side] == 0U &&
            vmc_probe_state.airborne_side_posture_ms[side] >=
                VMC_AIRBORNE_SIDE_HOLD_MS_DEFAULT) {
          vmc_probe_state.airborne_side_posture_active[side] = 0U;
          vmc_probe_state.airborne_side_posture_ms[side] = 0U;
          vmc_probe_state.airborne_side_length_returning[side] = 1U;
        }
      }
    }
  }

  if (vmc_jump_joint_lqr_active() != 0U) {
    vmc_probe_state.airborne_active = 1U;
    vmc_probe_state.airborne_candidate = 0U;
    vmc_probe_state.airborne_candidate_ms = 0U;
    vmc_probe_state.airborne_release_candidate_ms = 0U;
    return;
  }

  if (detector_ready == 0U) {
    vmc_probe_state.airborne_active = 0U;
    vmc_probe_state.airborne_candidate = 0U;
    vmc_probe_state.airborne_candidate_ms = 0U;
    vmc_probe_state.airborne_release_candidate_ms = 0U;
    for (uint8_t side = 0U; side < VMC_AIRBORNE_SIDE_COUNT; side++) {
      vmc_probe_state.airborne_side_fn_filter_initialized[side] = 0U;
      vmc_probe_state.airborne_side_posture_active[side] = 0U;
      vmc_probe_state.airborne_side_posture_ms[side] = 0U;
      vmc_probe_state.airborne_side_length_returning[side] = 0U;
    }
    return;
  }

  /* Preserve the whole-machine compatibility latch as the AND of the two
   * side latches.  Wheel gating itself is always per side. */
  {
    uint8_t side;
    uint8_t all_candidate = 1U;
    uint32_t candidate_min = confirm_ms;

    vmc_probe_state.airborne_active = 1U;
    for (side = 0U; side < VMC_AIRBORNE_SIDE_COUNT; side++) {
      if (vmc_probe_state.airborne_side_active[side] == 0U) {
        vmc_probe_state.airborne_active = 0U;
      }
      if (vmc_probe_state.airborne_side_candidate[side] == 0U) {
        all_candidate = 0U;
      }
      if (vmc_probe_state.airborne_side_candidate_ms[side] < candidate_min) {
        candidate_min = vmc_probe_state.airborne_side_candidate_ms[side];
      }
      vmc_probe_state.airborne_side_release_candidate_ms[side] = 0U;
    }
    vmc_probe_state.airborne_candidate = all_candidate;
    vmc_probe_state.airborne_candidate_ms =
        (all_candidate != 0U) ? candidate_min : 0U;
    vmc_probe_state.airborne_release_candidate_ms = 0U;
  }
}

void vmc_balance_init(void)
{
  vmc_balance.enabled = 1U;
  vmc_balance.wheel_output_enable = 0U;
  vmc_balance.wheel_output_user_enable = VMC_BALANCE_WHEEL_OUTPUT_USER_ENABLE_DEFAULT;
  vmc_balance.tp_output_enable = VMC_BALANCE_TP_OUTPUT_ENABLE_DEFAULT;
  vmc_balance.dx_feedback_enable = VMC_BALANCE_DX_FEEDBACK_ENABLE_DEFAULT;
  vmc_balance.x_feedback_enable = VMC_BALANCE_X_FEEDBACK_ENABLE_DEFAULT;
  vmc_balance.safety_allows_output = 0U;
  vmc_balance.roll_remote_active = 0U;
  vmc_balance.roll_remote_input = 0.0f;
  vmc_balance.pitch_offset = VMC_BALANCE_PITCH_OFFSET_DEFAULT;
  vmc_balance.phi_offset = VMC_BALANCE_PHI_OFFSET_DEFAULT;
  vmc_balance.theta_deadband = VMC_BALANCE_THETA_DEADBAND_DEFAULT;
  vmc_balance.phi_deadband = VMC_BALANCE_PHI_DEADBAND_DEFAULT;
  vmc_balance.x_offset = 0.0f;
  vmc_balance.x_limit = VMC_BODY_MOTION_X_LIMIT_DEFAULT;
  vmc_balance.x_feedback_gain = VMC_BALANCE_X_FEEDBACK_GAIN_DEFAULT;
  vmc_balance.dx_feedback_gain = VMC_BALANCE_DX_FEEDBACK_GAIN_DEFAULT;
  vmc_balance.dx_feedback_gain_static =
      VMC_BALANCE_DX_GAIN_STATIC_DEFAULT;
  vmc_balance.dx_static_delay_ms = VMC_BALANCE_DX_STATIC_DELAY_MS;
  vmc_balance.dx_static_speed_limit =
      VMC_BALANCE_DX_STATIC_SPEED_LIMIT;
  vmc_dx_static_hold_ms = 0U;
  vmc_dx_gain_applied = VMC_BALANCE_DX_FEEDBACK_GAIN_DEFAULT;
  vmc_balance.x_feedback_limit = VMC_BALANCE_X_FEEDBACK_LIMIT_DEFAULT;
  vmc_balance.dx_feedback_limit = VMC_BALANCE_DX_FEEDBACK_LIMIT_DEFAULT;
  vmc_balance.speed_control_enable = VMC_BALANCE_SPEED_CONTROL_ENABLE_DEFAULT;
  vmc_balance.speed_target = 0.0f;
  vmc_balance.speed_target_ramped = 0.0f;
  vmc_balance.speed_target_max = VMC_BALANCE_SPEED_TARGET_MAX_DEFAULT;
  vmc_balance.speed_ramp_rate = VMC_BALANCE_SPEED_RAMP_RATE_DEFAULT;
  vmc_balance.spin_speed_ramp_rate = VMC_BALANCE_SPIN_SPEED_RAMP_RATE_DEFAULT;
  vmc_balance.speed_deadband = VMC_BALANCE_SPEED_DEADBAND_DEFAULT;
  vmc_balance.stop_speed_threshold = VMC_BALANCE_STOP_SPEED_THRESHOLD_DEFAULT;
  vmc_balance.speed_pitch_gain = VMC_BALANCE_SPEED_PITCH_GAIN_DEFAULT;
  vmc_balance.speed_pitch_limit = VMC_BALANCE_SPEED_PITCH_LIMIT_DEFAULT;
  vmc_balance.speed_pitch_ramp_rate = VMC_BALANCE_SPEED_PITCH_RAMP_RATE_DEFAULT;
  vmc_balance.speed_pitch_offset = 0.0f;
  vmc_balance.speed_stop_hold_active = 0U;
  vmc_balance.x_hold_max_stretch = VMC_BALANCE_X_HOLD_MAX_STRETCH_DEFAULT;
  vmc_balance.dtheta_lpf_alpha = VMC_BALANCE_DTHETA_LPF_ALPHA_DEFAULT;
  vmc_balance.lqr_dtheta_gain = VMC_BALANCE_LQR_DTHETA_GAIN_DEFAULT;
  vmc_balance.lqr_dtheta_deadband = VMC_BALANCE_LQR_DTHETA_DEADBAND_DEFAULT;
  vmc_balance.lqr_dtheta_limit = VMC_BALANCE_LQR_DTHETA_LIMIT_DEFAULT;
  vmc_balance.dpitch_lpf_alpha = VMC_BALANCE_DPITCH_LPF_ALPHA_DEFAULT;
  vmc_balance.disturbance_observer_enable =
      VMC_BALANCE_DISTURBANCE_OBSERVER_ENABLE_DEFAULT;
  vmc_balance.disturbance_observer_gain =
      VMC_BALANCE_DISTURBANCE_OBSERVER_GAIN_DEFAULT;
  vmc_balance.disturbance_observer_leak =
      VMC_BALANCE_DISTURBANCE_OBSERVER_LEAK_DEFAULT;
  vmc_balance.disturbance_observer_limit =
      VMC_BALANCE_DISTURBANCE_OBSERVER_LIMIT_DEFAULT;
  vmc_balance.disturbance_observer_speed_limit =
      VMC_BALANCE_DISTURBANCE_OBSERVER_SPEED_LIMIT_DEFAULT;
  vmc_balance.disturbance_observer_delay =
      VMC_BALANCE_DISTURBANCE_OBSERVER_DELAY_DEFAULT;
  vmc_balance.disturbance_observer_pitch_gain =
      VMC_BALANCE_DISTURBANCE_OBSERVER_PITCH_GAIN_DEFAULT;
  vmc_balance.disturbance_observer_theta_gain =
      VMC_BALANCE_DISTURBANCE_OBSERVER_THETA_GAIN_DEFAULT;
  vmc_balance.disturbance_observer_output = 0.0f;
  vmc_balance.disturbance_observer_residual = 0.0f;
  vmc_balance.disturbance_observer_neutral_time = 0.0f;
  vmc_balance.disturbance_observer_active = 0U;
  vmc_dpitch_lpf = 0.0;
  vmc_speed_dx_prev = 0.0;
  vmc_speed_accel_lpf = 0.0;
  vmc_speed_accel_initialized = 0U;
  vmc_balance.yaw_hold_enable = VMC_YAW_HOLD_ENABLE_DEFAULT;
  vmc_balance.yaw_hold_active = 0U;
  vmc_balance.gimbal_follow_enable = VMC_GIMBAL_FOLLOW_ENABLE_DEFAULT;
  vmc_balance.gimbal_follow_active = 0U;
  vmc_balance.gimbal_follow_front_valid = VMC_GIMBAL_FOLLOW_FRONT_VALID_DEFAULT;
  vmc_balance.yaw_target = 0.0f;
  vmc_balance.yaw_hold_kp = VMC_YAW_HOLD_KP_DEFAULT;
  vmc_balance.yaw_hold_kd = VMC_YAW_HOLD_KD_DEFAULT;
  vmc_balance.yaw_hold_deadband = VMC_YAW_HOLD_DEADBAND_DEFAULT;
  vmc_balance.yaw_target_rate_max = VMC_RC_YAW_TARGET_RATE_MAX_DEFAULT;
  vmc_balance.yaw_spin_rate = VMC_RC_YAW_SPIN_RATE_MIN_DEFAULT;
  vmc_balance.yaw_spin_rate_max = VMC_RC_YAW_SPIN_RATE_MAX_DEFAULT;
  vmc_balance.spin_line_origin_yaw = 0.0f;
  vmc_balance.spin_line_yaw = 0.0f;
  vmc_balance.spin_line_path_offset_deg =
      VMC_SPIN_LINE_PATH_OFFSET_DEG_DEFAULT;
  vmc_balance.spin_line_input_forward = 0.0f;
  vmc_balance.spin_line_input_lateral = 0.0f;
  vmc_balance.spin_line_input_magnitude = 0.0f;
  vmc_balance.spin_line_phase_error = 0.0f;
  vmc_balance.spin_line_phase_lead_rad = 0.0f;
  vmc_balance.spin_line_projection_power = VMC_SPIN_PROJECTION_POWER_DEFAULT;
  vmc_balance.spin_line_gate = 1.0f;
  vmc_balance.spin_line_heading_valid = 0U;
  vmc_balance.spin_mode_enabled = 0U;
  vmc_balance.yaw_hold_current_limit = VMC_YAW_HOLD_CURRENT_LIMIT_DEFAULT;
  vmc_balance.gimbal_follow_front_ecd = VMC_GIMBAL_FOLLOW_FRONT_ECD_DEFAULT;
  vmc_balance.gimbal_follow_kp = VMC_GIMBAL_FOLLOW_KP_DEFAULT;
  vmc_balance.gimbal_follow_kd = VMC_GIMBAL_FOLLOW_KD_DEFAULT;
  vmc_balance.gimbal_follow_deadband = VMC_GIMBAL_FOLLOW_DEADBAND_DEFAULT;
  vmc_balance.gimbal_follow_direction = VMC_GIMBAL_FOLLOW_DIRECTION_DEFAULT;
  vmc_balance.gimbal_follow_current_limit =
      VMC_GIMBAL_FOLLOW_CURRENT_LIMIT_DEFAULT;
  vmc_balance.turn_current_max = VMC_RC_WHEEL_TURN_CURRENT_MAX;
  vmc_lqr_d_theta_lpf = 0.0;
  vmc_lqr_left_d_theta_lpf = 0.0;
  vmc_lqr_right_d_theta_lpf = 0.0;
  vmc_balance.wheel_current_limit = VMC_BALANCE_WHEEL_CURRENT_LIMIT_DEFAULT;
  vmc_balance.wheel_torque_limit = VMC_BALANCE_WHEEL_TORQUE_LIMIT_DEFAULT;
  vmc_balance.wheel_torque_to_current = VMC_BALANCE_WHEEL_TORQUE_TO_CURRENT;
  vmc_balance.wheel_current_gain = VMC_BALANCE_WHEEL_CURRENT_GAIN_DEFAULT;
  vmc_balance.airborne_detect_enable = VMC_AIRBORNE_DETECT_ENABLE_DEFAULT;
  vmc_balance.airborne_wheel_speed_threshold =
      VMC_AIRBORNE_WHEEL_SPEED_THRESHOLD_DEFAULT;
  vmc_balance.airborne_joint_torque_threshold =
      VMC_AIRBORNE_JOINT_TORQUE_THRESHOLD_DEFAULT;
  vmc_balance.airborne_confirm_ms = VMC_AIRBORNE_CONFIRM_MS_DEFAULT;
  vmc_balance.airborne_release_ms = VMC_AIRBORNE_RELEASE_MS_DEFAULT;
  vmc_balance.airborne_pitch_dev_max = VMC_AIRBORNE_PITCH_DEV_MAX_DEFAULT;
  vmc_balance.airborne_pitch_rate_max = VMC_AIRBORNE_PITCH_RATE_MAX_DEFAULT;
  vmc_balance.airborne_side_enable = VMC_AIRBORNE_SIDE_ENABLE_DEFAULT;
  vmc_balance.airborne_side_confirm_ms = VMC_AIRBORNE_SIDE_CONFIRM_MS_DEFAULT;
  vmc_balance.airborne_side_release_ms = VMC_AIRBORNE_SIDE_RELEASE_MS_DEFAULT;
  vmc_balance.kalman_q_v = VMC_BODY_MOTION_KF_Q_V_DEFAULT;
  vmc_balance.kalman_q_a = VMC_BODY_MOTION_KF_Q_A_DEFAULT;
  vmc_balance.kalman_r_wheel = VMC_BODY_MOTION_KF_R_WHEEL_DEFAULT;
  vmc_balance.kalman_r_imu = VMC_BODY_MOTION_KF_R_IMU_DEFAULT;
  vmc_balance.kalman_slip_enable = VMC_BODY_MOTION_KF_SLIP_ENABLE_DEFAULT;
  vmc_balance.kalman_slip_accel = VMC_BODY_MOTION_KF_SLIP_ACCEL_DEFAULT;
  vmc_balance.kalman_slip_vel_diff = VMC_BODY_MOTION_KF_SLIP_VEL_DIFF_DEFAULT;
  vmc_balance.kalman_slip_yaw_track = VMC_BODY_MOTION_KF_SLIP_YAW_TRACK_DEFAULT;
  vmc_balance.kalman_slip_yaw = VMC_BODY_MOTION_KF_SLIP_YAW_DEFAULT;
  vmc_balance.kalman_slip_hold_ms = VMC_BODY_MOTION_KF_SLIP_HOLD_MS_DEFAULT;
  vmc_balance.slip_lqr_enable = VMC_SLIP_LQR_ENABLE_DEFAULT;
  vmc_balance.slip_lqr_theta_scale = VMC_SLIP_LQR_THETA_SCALE_DEFAULT;
  vmc_balance.slip_lqr_dtheta_scale = VMC_SLIP_LQR_DTHETA_SCALE_DEFAULT;
  vmc_balance.slip_wheel_current_scale =
      VMC_SLIP_WHEEL_CURRENT_SCALE_DEFAULT;
  vmc_balance.slip_curr_applied_left = 1.0f;
  vmc_balance.slip_curr_applied_right = 1.0f;
  vmc_balance.kalman_slip_confirm_ms =
      VMC_BODY_MOTION_KF_SLIP_CONFIRM_MS_DEFAULT;
  body_motion_kf_slip_initialized = 0U;
  vmc_balance.tp_limit = VMC_BALANCE_TP_LIMIT_DEFAULT;
  vmc_balance.tp_phi_gain = VMC_BALANCE_TP_PHI_GAIN_DEFAULT;
  vmc_balance.tp_x_gain = VMC_BALANCE_TP_X_GAIN_DEFAULT;
  vmc_balance.leg_sync_enable = VMC_LEG_SYNC_ENABLE_DEFAULT;
  vmc_balance.leg_sync_kp = VMC_LEG_SYNC_KP_DEFAULT;
  vmc_balance.leg_sync_kd = VMC_LEG_SYNC_KD_DEFAULT;
  vmc_balance.leg_sync_torque_limit = VMC_LEG_SYNC_TORQUE_LIMIT_DEFAULT;
  vmc_balance.leg_sync_error = 0.0;
  vmc_balance.leg_sync_rate_error = 0.0;
  vmc_balance.leg_sync_torque = 0.0;
  vmc_probe_state.balance = vmc_balance;
  vmc_airborne_last_update_ms = 0U;
}

void vmc_balance_update(void)
{
  const volatile vmc_motion_state_t *motion = &vmc_probe_state.motion;
  const double lqr_theta =
      ((vmc_probe_state.left_leg.phi0 - VMC_PHI0_VERTICAL_RAD) +
       (vmc_probe_state.right_leg.phi0 - VMC_PHI0_VERTICAL_RAD)) *
          0.5 -
      vmc_probe_state.body.pitch;
  const double lqr_d_theta_raw =
      (vmc_probe_state.left_leg.d_phi0 + vmc_probe_state.right_leg.d_phi0) * 0.5 -
      vmc_probe_state.body.pitch_rate;
  const double dtheta_alpha =
      clamp_double(vmc_balance.dtheta_lpf_alpha, 0.001, 1.0);
  const double lqr_d_theta =
      vmc_lqr_d_theta_lpf + dtheta_alpha * (lqr_d_theta_raw - vmc_lqr_d_theta_lpf);
  vmc_lqr_d_theta_lpf = lqr_d_theta;
  const double speed_deadband =
      clamp_double(vmc_balance.speed_deadband, 0.0, 0.5);
  const double speed_target =
      (vmc_balance.speed_control_enable != 0U)
          ? clamp_double(vmc_balance.speed_target,
                         -clamp_double(vmc_balance.speed_target_max, 0.0, 3.0),
                         clamp_double(vmc_balance.speed_target_max, 0.0, 3.0))
          : 0.0;
  const uint8_t stick_speed_active =
      (vmc_balance.speed_control_enable != 0U && fabs(speed_target) > speed_deadband) ? 1U : 0U;
  const uint8_t released_from_speed =
      (stick_speed_active == 0U && vmc_balance.speed_stop_hold_active != 0U) ? 1U : 0U;
  double speed_target_ramped = vmc_balance.speed_target_ramped;
  if (released_from_speed != 0U) {
    /* On release, brake from the measured velocity instead of carrying a
       stale ramped command into the parking controller. */
    speed_target_ramped = 0.0;
  }
  const double active_speed_ramp_rate =
      (vmc_balance.spin_mode_enabled != 0U)
          ? clamp_double(vmc_balance.spin_speed_ramp_rate, 0.0, 10.0)
          : clamp_double(vmc_balance.speed_ramp_rate, 0.0, 10.0);
  const double speed_ramp_step = active_speed_ramp_rate * vmc_loop_dt();
  const double dx_target =
      vmc_step_towards(speed_target_ramped, speed_target, speed_ramp_step);
  const double stop_speed_threshold =
      clamp_double(vmc_balance.stop_speed_threshold, 0.0, 1.0);

  const double speed_pitch_limit =
      clamp_double(vmc_balance.speed_pitch_limit, 0.0, 0.5);
  /* Estimate actual body acceleration for braking-only Pitch feedforward.
   * The previous implementation used dx_target directly, which also tilted
   * the body during normal cruising.  Only the component opposing the
   * current measured velocity is retained, so this term is active during
   * braking and fades out when the chassis settles. */
  double speed_accel = 0.0;
  double braking_accel = 0.0;
  if (vmc_control.enabled != 0U &&
      vmc_probe_state.feedback_timeout_active == 0U) {
    if (vmc_speed_accel_initialized == 0U) {
      vmc_speed_dx_prev = motion->kf_dx;
      vmc_speed_accel_initialized = 1U;
    }
    speed_accel = (motion->kf_dx - vmc_speed_dx_prev) / vmc_loop_dt();
    vmc_speed_dx_prev = motion->kf_dx;
    speed_accel = clamp_double(speed_accel, -20.0, 20.0);
    /* A fixed light LPF suppresses encoder/KF quantization without adding a
     * user-facing parameter to the existing tuning protocol. */
    vmc_speed_accel_lpf +=
        0.20 * (speed_accel - vmc_speed_accel_lpf);
    if (fabs(motion->kf_dx) > speed_deadband &&
        motion->kf_dx * vmc_speed_accel_lpf < 0.0) {
      braking_accel = vmc_speed_accel_lpf;
    }
  } else {
    vmc_speed_dx_prev = motion->kf_dx;
    vmc_speed_accel_lpf = 0.0;
    vmc_speed_accel_initialized = 0U;
  }
  const double speed_pitch_target =
      clamp_double(braking_accel * clamp_double(vmc_balance.speed_pitch_gain, -1.0, 1.0),
                   -speed_pitch_limit,
                   speed_pitch_limit);
  const double speed_pitch_step =
      clamp_double(vmc_balance.speed_pitch_ramp_rate, 0.0, 5.0) * vmc_loop_dt();
  const double speed_pitch_offset =
      vmc_step_towards(vmc_balance.speed_pitch_offset,
                       speed_pitch_target,
                       speed_pitch_step);
  const double pitch_target_base = vmc_balance.phi_offset + speed_pitch_offset;
  double pitch_target = pitch_target_base;
  double phi_error_raw;
  if (vmc_balance.disturbance_observer_enable != 0U &&
      vmc_balance.speed_control_enable != 0U &&
      vmc_control.enabled != 0U &&
      vmc_control.manual_prepose_active == 0U &&
      vmc_balance.wheel_output_enable != 0U &&
      vmc_probe_state.feedback_timeout_active == 0U &&
      vmc_getup_is_active() == 0U) {
    const double observer_gain =
        clamp_double(vmc_balance.disturbance_observer_gain, -20.0, 20.0);
    const double observer_leak =
        clamp_double(vmc_balance.disturbance_observer_leak, 0.0, 2.0);
    const double observer_limit =
        clamp_double(vmc_balance.disturbance_observer_limit, 0.0, 0.5);
    const double observer_speed_limit =
        clamp_double(vmc_balance.disturbance_observer_speed_limit, 0.01, 2.0);
    const double observer_delay =
        clamp_double(vmc_balance.disturbance_observer_delay, 0.0, 5.0);
    double observer_output = clamp_double(
        vmc_balance.disturbance_observer_output, -observer_limit, observer_limit);

    /* Learn the static balance trim only while the forward stick is released.
     * The velocity sign follows the existing speed loop: a positive gain
     * integrates (target velocity - measured velocity) into the trim state.
     * During commanded travel, keep applying the learned trim but freeze the
     * estimator so acceleration and braking do not wind it up. */
    if (stick_speed_active == 0U) {
      const double speed_error = -motion->kf_dx;
      vmc_balance.disturbance_observer_neutral_time = (float)clamp_double(
          vmc_balance.disturbance_observer_neutral_time + vmc_loop_dt(),
          0.0,
          observer_delay + 1.0);
      if ((double)vmc_balance.disturbance_observer_neutral_time >= observer_delay &&
          fabs(speed_error) <= observer_speed_limit) {
        vmc_balance.disturbance_observer_active = 1U;
        vmc_balance.disturbance_observer_residual = (float)speed_error;
        observer_output +=
            (observer_gain * speed_error - observer_leak * observer_output) *
            vmc_loop_dt();
        observer_output =
            clamp_double(observer_output, -observer_limit, observer_limit);
      } else {
        vmc_balance.disturbance_observer_active = 0U;
        vmc_balance.disturbance_observer_residual = 0.0f;
      }
    } else {
      vmc_balance.disturbance_observer_neutral_time = 0.0f;
      vmc_balance.disturbance_observer_active = 0U;
      vmc_balance.disturbance_observer_residual = 0.0f;
    }
    vmc_balance.disturbance_observer_output = (float)observer_output;
  } else {
    vmc_balance.disturbance_observer_output = 0.0f;
    vmc_balance.disturbance_observer_residual = 0.0f;
    vmc_balance.disturbance_observer_neutral_time = 0.0f;
    vmc_balance.disturbance_observer_active = 0U;
  }
  pitch_target +=
      (double)vmc_balance.disturbance_observer_output *
      clamp_double(vmc_balance.disturbance_observer_pitch_gain, -2.0, 2.0);
  phi_error_raw = motion->phi - pitch_target;
  const double phi_error =
      apply_soft_deadband(phi_error_raw, vmc_balance.phi_deadband);
  const double dpitch_raw = motion->d_phi;
  const double dpitch_alpha = clamp_double(vmc_balance.dpitch_lpf_alpha, 0.001, 1.0);
  const double dpitch_lpf =
      vmc_dpitch_lpf + dpitch_alpha * (dpitch_raw - vmc_dpitch_lpf);
  vmc_dpitch_lpf = dpitch_lpf;

  vmc_balance.speed_target_ramped = (float)dx_target;
  vmc_balance.speed_pitch_offset = (float)speed_pitch_offset;
  double x_error = 0.0;
  double dx_feedback_target = dx_target;
  if (stick_speed_active != 0U) {
    /* Cruise mode: keep the parking anchor at the current position and only
       feed velocity error into LQR, matching the MuJoCo controller. */
    vmc_balance.speed_stop_hold_active = 1U;
    vmc_balance.x_offset = (float)motion->kf_x;
    x_error = 0.0;
  } else {
    if (released_from_speed != 0U) {
      /* Capture the release position exactly once.  The next command will
         recapture it again, so reverse motion never inherits an old anchor. */
      vmc_balance.x_offset = (float)motion->kf_x;
      vmc_balance.speed_target_ramped = 0.0f;
      vmc_balance.speed_pitch_offset = 0.0f;
      vmc_balance.speed_stop_hold_active = 0U;
    }
    /* Parking mode: hold the last anchor, but drag it if stretch exceeds the
       allowed spring travel so the robot does not reverse hard to an old point. */
    const double x_hold_max_stretch =
        clamp_double(vmc_balance.x_hold_max_stretch, 0.0, VMC_BODY_MOTION_X_LIMIT_DEFAULT);
    dx_feedback_target = 0.0;
    x_error = motion->kf_x - vmc_balance.x_offset;
    if (x_error > x_hold_max_stretch) {
      vmc_balance.x_offset = (float)(motion->kf_x - x_hold_max_stretch);
      x_error = x_hold_max_stretch;
    } else if (x_error < -x_hold_max_stretch) {
      vmc_balance.x_offset = (float)(motion->kf_x + x_hold_max_stretch);
      x_error = -x_hold_max_stretch;
    }
    if (fabs(motion->kf_dx) <= stop_speed_threshold && fabs(dx_target) <= speed_deadband) {
      vmc_balance.speed_stop_hold_active = 0U;
    }
  }

  if (vmc_balance.dx_feedback_enable == 0U || vmc_balance.x_feedback_enable == 0U) {
    x_error = 0.0;
  }
  const double x_feedback_raw =
      x_error * clamp_double(vmc_balance.x_feedback_gain, -200.0, 200.0);
  const double x_feedback_limit =
      clamp_double(vmc_balance.x_feedback_limit, 0.0, VMC_BODY_MOTION_X_LIMIT_DEFAULT);
  const double x_feedback_error =
      clamp_double(x_feedback_raw, -x_feedback_limit, x_feedback_limit);
  /* Static-hold damping.  Holding still is when the stiction limit cycle
   * shows up, and it shrinks monotonically with more velocity feedback: the
   * 2026-11-12 sweep measured 56.5 mm peak-to-peak at gain -1.0 and 37.4 mm at
   * -2.0.  The stronger gain is only wanted while parked, though, because it
   * makes the machine leaden to drive.  So it engages after the stick has been
   * released for dx_static_delay_ms and releases as soon as a command appears,
   * both through a slew-limited gain so the leg target cannot step.
   *
   * It must not engage during a scripted sequence.  The stick is released for
   * the whole of a get-up, so without this gate the strong gain would land in
   * the middle of the get-up on top of the state machine's own leg target. */
  {
    const double dt = vmc_loop_dt();
    const uint32_t hold_dt_ms = clamp_u32(
        (uint32_t)(dt * 1000.0), 1U, 100U);
    const uint8_t static_hold_allowed =
        (vmc_getup_is_active() == 0U &&
         vmc_jump_is_active_or_aborted() == 0U &&
         vmc_stair_is_active() == 0U &&
         vmc_control.manual_prepose_active == 0U &&
         vmc_probe_state.airborne_active == 0U &&
         vmc_probe_state.feedback_timeout_active == 0U)
            ? 1U
            : 0U;
    /* Speed gate: a kick leaves the stick released, so without this the
     * strong gain would fight a disturbance at 0.3 m/s and ring.  Above the
     * limit the velocity loop drops straight back to the normal gain. */
    const uint8_t static_speed_ok =
        (fabs(motion->kf_dx) <=
         clamp_double(vmc_balance.dx_static_speed_limit, 0.0, 1.0))
            ? 1U
            : 0U;
    if (stick_speed_active != 0U || static_hold_allowed == 0U ||
        static_speed_ok == 0U) {
      vmc_dx_static_hold_ms = 0U;
    } else {
      vmc_dx_static_hold_ms =
          clamp_u32(vmc_dx_static_hold_ms + hold_dt_ms, 0U, 60000U);
    }
    {
      const uint32_t delay_ms = (uint32_t)clamp_double(
          vmc_balance.dx_static_delay_ms, 0.0, 10000.0);
      const double target_gain =
          (vmc_dx_static_hold_ms >= delay_ms)
              ? clamp_double(vmc_balance.dx_feedback_gain_static, -200.0, 200.0)
              : clamp_double(vmc_balance.dx_feedback_gain, -200.0, 200.0);
      const double ramp_rate =
          (fabs(target_gain) < fabs((double)vmc_dx_gain_applied))
              ? VMC_BALANCE_DX_GAIN_RELEASE_RATE
              : VMC_BALANCE_DX_GAIN_RAMP_RATE;
      vmc_dx_gain_applied = (float)vmc_step_towards(
          (double)vmc_dx_gain_applied, target_gain, ramp_rate * dt);
    }
  }
  const double dx_raw =
      (vmc_balance.dx_feedback_enable != 0U)
          ? (motion->kf_dx - dx_feedback_target) *
                (double)vmc_dx_gain_applied
          : 0.0;
  const double dx_feedback_limit =
      clamp_double(vmc_balance.dx_feedback_limit, 0.0, 30.0);
  const double dx_error =
      clamp_double(dx_raw, -dx_feedback_limit, dx_feedback_limit);
  /* Shifting the virtual-leg theta target moves the wheel contact point under
   * the combined chassis/gimbal centre of mass.  This provides substantially
   * more static balancing authority than body-pitch trim alone. */
  double theta_target =
      vmc_balance.pitch_offset +
      (double)vmc_balance.disturbance_observer_output *
          clamp_double(vmc_balance.disturbance_observer_theta_gain, -2.0, 2.0);
  if (vmc_stair_is_active() != 0U) {
    /* The stair action now owns the legs through the absolute phi0 PD.  Keep
     * the wheel-side theta target at the measured average so the balance loop
     * does not fight the commanded 2.5 rad leg pose.  The pitch/x/dx terms
     * remain active and still provide wheel balance. */
    theta_target = lqr_theta;
  }
  vmc_control.stair_theta_ramped = (float)theta_target;
  const double theta_error_raw = lqr_theta - theta_target;
  const double theta_error =
      apply_soft_deadband(theta_error_raw, vmc_balance.theta_deadband);
  const double left_lqr_theta_error_raw = motion->left_theta - theta_target;
  const double right_lqr_theta_error_raw = motion->right_theta - theta_target;
  const double left_lqr_theta_error =
      apply_soft_deadband(left_lqr_theta_error_raw, vmc_balance.theta_deadband);
  const double right_lqr_theta_error =
      apply_soft_deadband(right_lqr_theta_error_raw, vmc_balance.theta_deadband);
  const double left_lqr_d_theta_raw = motion->left_d_theta;
  const double right_lqr_d_theta_raw = motion->right_d_theta;
  const double left_lqr_d_theta =
      vmc_lqr_left_d_theta_lpf +
      dtheta_alpha * (left_lqr_d_theta_raw - vmc_lqr_left_d_theta_lpf);
  const double right_lqr_d_theta =
      vmc_lqr_right_d_theta_lpf +
      dtheta_alpha * (right_lqr_d_theta_raw - vmc_lqr_right_d_theta_lpf);
  vmc_lqr_left_d_theta_lpf = left_lqr_d_theta;
  vmc_lqr_right_d_theta_lpf = right_lqr_d_theta;
  const double lqr_dtheta_gain =
      clamp_double(vmc_balance.lqr_dtheta_gain, -10.0, 10.0);
  const double lqr_dtheta_deadband =
      clamp_double(vmc_balance.lqr_dtheta_deadband, 0.0, 5.0);
  const double lqr_dtheta_limit =
      clamp_double(vmc_balance.lqr_dtheta_limit, 0.0, 50.0);
  const double left_lqr_d_theta_limited =
      clamp_double(apply_soft_deadband(left_lqr_d_theta, lqr_dtheta_deadband) *
                       lqr_dtheta_gain,
                   -lqr_dtheta_limit,
                   lqr_dtheta_limit);
  const double right_lqr_d_theta_limited =
      clamp_double(apply_soft_deadband(right_lqr_d_theta, lqr_dtheta_deadband) *
                       lqr_dtheta_gain,
                   -lqr_dtheta_limit,
                   lqr_dtheta_limit);
  /* Slip weight switching: scale the leg-ANGLE feedback of whichever side is
   * slipping so the posture term dominates its control output.  This is the
   * posture half of the Nanhang sentry response; see
   * VMC_SLIP_LQR_THETA_SCALE_DEFAULT for what does and does not transfer.
   *
   * The scale multiplies the ERROR rather than the gain, so the K arrays are
   * left untouched - they are rewritten every cycle by vmc_update_lqr_gain()
   * and must not accumulate. */
  const uint8_t slip_weight_enabled =
      (vmc_balance.slip_lqr_enable != 0U &&
       vmc_control.enabled != 0U &&
       vmc_jump_joint_lqr_active() == 0U) != 0U;
  const double slip_theta_scale =
      clamp_double(vmc_balance.slip_lqr_theta_scale, 0.0, 10.0);
  const double slip_dtheta_scale =
      clamp_double(vmc_balance.slip_lqr_dtheta_scale, 0.0, 10.0);
  const uint8_t left_slipping =
      (slip_weight_enabled != 0U && body_motion_kf_slip_left != 0U) ? 1U : 0U;
  const uint8_t right_slipping =
      (slip_weight_enabled != 0U && body_motion_kf_slip_right != 0U) ? 1U : 0U;
  const double left_theta_scale =
      (left_slipping != 0U) ? slip_theta_scale : 1.0;
  const double left_dtheta_scale =
      (left_slipping != 0U) ? slip_dtheta_scale : 1.0;
  const double right_theta_scale =
      (right_slipping != 0U) ? slip_theta_scale : 1.0;
  const double right_dtheta_scale =
      (right_slipping != 0U) ? slip_dtheta_scale : 1.0;
  vmc_balance.slip_lqr_applied_left = (float)left_theta_scale;
  vmc_balance.slip_lqr_applied_right = (float)right_theta_scale;
  const double left_wheel_theta_term =
      vmc_probe_state.lqr.left_k[0] * (left_lqr_theta_error * left_theta_scale);
  const double left_wheel_dtheta_term =
      vmc_probe_state.lqr.left_k[2] *
      (left_lqr_d_theta_limited * left_dtheta_scale);
  const double left_wheel_x_term = vmc_probe_state.lqr.left_k[4] * x_feedback_error;
  const double left_wheel_dx_term = vmc_probe_state.lqr.left_k[6] * dx_error;
  const double left_wheel_phi_term = vmc_probe_state.lqr.left_k[8] * phi_error;
  const double left_wheel_dphi_term = vmc_probe_state.lqr.left_k[10] * dpitch_lpf;
  const double right_wheel_theta_term =
      vmc_probe_state.lqr.right_k[0] *
      (right_lqr_theta_error * right_theta_scale);
  const double right_wheel_dtheta_term =
      vmc_probe_state.lqr.right_k[2] *
      (right_lqr_d_theta_limited * right_dtheta_scale);
  const double right_wheel_x_term = vmc_probe_state.lqr.right_k[4] * x_feedback_error;
  const double right_wheel_dx_term = vmc_probe_state.lqr.right_k[6] * dx_error;
  const double right_wheel_phi_term = vmc_probe_state.lqr.right_k[8] * phi_error;
  const double right_wheel_dphi_term = vmc_probe_state.lqr.right_k[10] * dpitch_lpf;
  const double left_wheel_raw =
      left_wheel_theta_term +
      left_wheel_dtheta_term +
      left_wheel_x_term +
      left_wheel_dx_term +
      left_wheel_phi_term +
      left_wheel_dphi_term;
  const double right_wheel_raw =
      right_wheel_theta_term +
      right_wheel_dtheta_term +
      right_wheel_x_term +
      right_wheel_dx_term +
      right_wheel_phi_term +
      right_wheel_dphi_term;
  const double left_tp_theta_term =
      vmc_probe_state.lqr.left_k[1] * (left_lqr_theta_error * left_theta_scale);
  const double left_tp_dtheta_term =
      vmc_probe_state.lqr.left_k[3] *
      (left_lqr_d_theta_limited * left_dtheta_scale);
  const double right_tp_theta_term =
      vmc_probe_state.lqr.right_k[1] *
      (right_lqr_theta_error * right_theta_scale);
  const double right_tp_dtheta_term =
      vmc_probe_state.lqr.right_k[3] *
      (right_lqr_d_theta_limited * right_dtheta_scale);
  const double tp_x_gain = clamp_double(vmc_balance.tp_x_gain, -2.0, 2.0);
  const double left_tp_x_term =
      tp_x_gain * vmc_probe_state.lqr.left_k[5] * x_feedback_error;
  const double left_tp_dx_term =
      tp_x_gain * vmc_probe_state.lqr.left_k[7] * dx_error;
  const double right_tp_x_term =
      tp_x_gain * vmc_probe_state.lqr.right_k[5] * x_feedback_error;
  const double right_tp_dx_term =
      tp_x_gain * vmc_probe_state.lqr.right_k[7] * dx_error;
  const double tp_phi_gain = clamp_double(vmc_balance.tp_phi_gain, -2.0, 2.0);
  const double left_tp_phi_term =
      tp_phi_gain * vmc_probe_state.lqr.left_k[9] * phi_error;
  const double left_tp_dphi_term =
      tp_phi_gain * vmc_probe_state.lqr.left_k[11] * dpitch_lpf;
  const double right_tp_phi_term =
      tp_phi_gain * vmc_probe_state.lqr.right_k[9] * phi_error;
  const double right_tp_dphi_term =
      tp_phi_gain * vmc_probe_state.lqr.right_k[11] * dpitch_lpf;
  /* Anti-splay (leg synchronisation).
   *
   * Body roll reaches both legs as a common-mode theta error, so every other
   * feedback term is blind to a pure left/right difference.  This term acts
   * only on that difference: it is added to the right leg's joint torque and
   * subtracted from the left leg's, leaving their average - and therefore the
   * balance loop - untouched.
   *
   * Sign: theta_error > 0 means that leg is rotated "forward".  The LQR theta
   * row drives a positive error with negative Tp, so to close
   * e = left_error - right_error the LEFT leg takes -leg_sync_torque and the
   * RIGHT leg +leg_sync_torque.
   *
   * Raw (pre-deadband) errors are used because the deadband would erase the
   * small differences this loop exists to correct, and the shared theta
   * target cancels out of the difference anyway.
   *
   * Only active when the channel-9 knob is at/above the old high-leg target.
   * At shorter knob positions the legs have more stroke headroom and the plain
   * balance loop is the established tune, so this loop must not silently
   * change it.  The latched stair short-leg override is excluded separately.
   *
   * Also deliberately inert while the get-up or jump state machines own the
   * legs: those sequences command both legs from their own profiles and are
   * tuned without this term. */
  const double leg_sync_limit =
      clamp_double(vmc_balance.leg_sync_torque_limit, 0.0, VMC_LEG_TP_MAX);
  /* The channel-9 knob selects the gear band.  The stair length override can
   * deliberately hold the legs near 0.16 m after a trigger, so exclude that
   * latched state as well. */
  const uint8_t leg_sync_high_gear =
      (vmc_remote_leg_length_gear() == VMC_LEG_GEAR_HIGH)
          ? 1U
          : 0U;
  const uint8_t leg_sync_active =
      (vmc_balance.leg_sync_enable != 0U &&
       leg_sync_high_gear != 0U &&
       vmc_control.stair_length_override_active == 0U &&
       vmc_getup_is_active() == 0U &&
       vmc_stair_is_active() == 0U &&
       vmc_jump_is_active_or_aborted() == 0U &&
       vmc_probe_state.airborne_side_active[VMC_AIRBORNE_SIDE_LEFT] == 0U &&
       vmc_probe_state.airborne_side_active[VMC_AIRBORNE_SIDE_RIGHT] == 0U &&
       vmc_probe_state.airborne_side_posture_active[VMC_AIRBORNE_SIDE_LEFT] ==
           0U &&
       vmc_probe_state.airborne_side_posture_active[VMC_AIRBORNE_SIDE_RIGHT] ==
           0U)
          ? 1U
          : 0U;
  const double leg_sync_error =
      left_lqr_theta_error_raw - right_lqr_theta_error_raw;
  const double leg_sync_rate_error = left_lqr_d_theta - right_lqr_d_theta;
  const double leg_sync_torque =
      (leg_sync_active != 0U)
          ? clamp_double(clamp_double(vmc_balance.leg_sync_kp, 0.0, 500.0) *
                             leg_sync_error +
                         clamp_double(vmc_balance.leg_sync_kd, 0.0, 50.0) *
                             leg_sync_rate_error,
                         -leg_sync_limit,
                         leg_sync_limit)
          : 0.0;
  const double left_tp_raw =
      -(left_tp_theta_term +
        left_tp_dtheta_term +
        left_tp_x_term +
        left_tp_dx_term +
        left_tp_phi_term +
        left_tp_dphi_term);
  const double right_tp_raw =
      -(right_tp_theta_term +
        right_tp_dtheta_term +
        right_tp_x_term +
        right_tp_dx_term +
        right_tp_phi_term +
        right_tp_dphi_term);
  int16_t wheel_limit = vmc_balance.wheel_current_limit;
  /* LQR T and its limit are motor-shaft torque.  The model already reflects
   * the drivetrain relationship, so do not apply another ratio here. */
  const double motor_torque_limit =
      (vmc_balance.wheel_torque_limit < 0.0f)
          ? -(double)vmc_balance.wheel_torque_limit
          : (double)vmc_balance.wheel_torque_limit;
  const double wheel_torque_to_current =
      vmc_balance.wheel_torque_to_current * vmc_balance.wheel_current_gain;
  const double left_wheel_limited =
      clamp_double(left_wheel_raw, -motor_torque_limit, motor_torque_limit);
  const double right_wheel_limited =
      clamp_double(right_wheel_raw, -motor_torque_limit, motor_torque_limit);
  float tp_limit = vmc_balance.tp_limit;
  int16_t left_current;
  int16_t right_current;

  if (wheel_limit < 0) {
    wheel_limit = (int16_t)-wheel_limit;
  }
  if (wheel_limit > VMC_WHEEL_CURRENT_MAX_DEFAULT) {
    wheel_limit = VMC_WHEEL_CURRENT_MAX_DEFAULT;
  }

  vmc_balance.safety_allows_output = vmc_safety_allows_output();
  vmc_balance.lqr_theta = lqr_theta;
  vmc_balance.lqr_d_theta = lqr_d_theta;
  vmc_balance.lqr_d_theta_raw = lqr_d_theta_raw;
  vmc_balance.theta_error_raw = theta_error_raw;
  vmc_balance.theta_error = theta_error;
  vmc_balance.left_lqr_theta_error_raw = left_lqr_theta_error_raw;
  vmc_balance.right_lqr_theta_error_raw = right_lqr_theta_error_raw;
  vmc_balance.left_lqr_theta_error = left_lqr_theta_error;
  vmc_balance.right_lqr_theta_error = right_lqr_theta_error;
  vmc_balance.left_lqr_d_theta_raw = left_lqr_d_theta_raw;
  vmc_balance.right_lqr_d_theta_raw = right_lqr_d_theta_raw;
  vmc_balance.left_lqr_d_theta = left_lqr_d_theta;
  vmc_balance.right_lqr_d_theta = right_lqr_d_theta;
  vmc_balance.left_lqr_d_theta_limited = left_lqr_d_theta_limited;
  vmc_balance.right_lqr_d_theta_limited = right_lqr_d_theta_limited;
  vmc_balance.leg_sync_error = leg_sync_error;
  vmc_balance.leg_sync_rate_error = leg_sync_rate_error;
  vmc_balance.leg_sync_torque = leg_sync_torque;
  vmc_balance.phi_error_raw = phi_error_raw;
  vmc_balance.phi_error = phi_error;
  vmc_balance.dpitch_raw = dpitch_raw;
  vmc_balance.dpitch_lpf = dpitch_lpf;
  vmc_balance.x_error = x_error;
  vmc_balance.dx_error = dx_error;
  vmc_balance.dx_target = dx_target;
  vmc_balance.dx_raw = dx_raw;
  vmc_balance.left_wheel_theta_term = left_wheel_theta_term;
  vmc_balance.left_wheel_dtheta_term = left_wheel_dtheta_term;
  vmc_balance.left_wheel_x_term = left_wheel_x_term;
  vmc_balance.left_wheel_dx_term = left_wheel_dx_term;
  vmc_balance.left_wheel_phi_term = left_wheel_phi_term;
  vmc_balance.left_wheel_dphi_term = left_wheel_dphi_term;
  vmc_balance.left_tp_theta_term = left_tp_theta_term;
  vmc_balance.left_tp_dtheta_term = left_tp_dtheta_term;
  vmc_balance.left_tp_x_term = left_tp_x_term;
  vmc_balance.left_tp_dx_term = left_tp_dx_term;
  vmc_balance.right_wheel_theta_term = right_wheel_theta_term;
  vmc_balance.right_wheel_dtheta_term = right_wheel_dtheta_term;
  vmc_balance.right_wheel_x_term = right_wheel_x_term;
  vmc_balance.right_wheel_dx_term = right_wheel_dx_term;
  vmc_balance.right_wheel_phi_term = right_wheel_phi_term;
  vmc_balance.right_wheel_dphi_term = right_wheel_dphi_term;
  vmc_balance.right_tp_theta_term = right_tp_theta_term;
  vmc_balance.right_tp_dtheta_term = right_tp_dtheta_term;
  vmc_balance.right_tp_x_term = right_tp_x_term;
  vmc_balance.right_tp_dx_term = right_tp_dx_term;
  vmc_balance.wheel_torque_raw = (left_wheel_raw + right_wheel_raw) * 0.5;
  vmc_balance.left_wheel_torque_raw = left_wheel_raw;
  vmc_balance.right_wheel_torque_raw = right_wheel_raw;
  vmc_balance.left_wheel_torque_limited = left_wheel_limited;
  vmc_balance.right_wheel_torque_limited = right_wheel_limited;
  vmc_balance.left_tp_raw = left_tp_raw;
  vmc_balance.right_tp_raw = right_tp_raw;

  left_current = clamp_i16((int16_t)(left_wheel_limited * wheel_torque_to_current),
                           (int16_t)-wheel_limit,
                           wheel_limit);
  right_current = clamp_i16((int16_t)(right_wheel_limited * wheel_torque_to_current),
                            (int16_t)-wheel_limit,
                            wheel_limit);
  vmc_balance.left_wheel_current_cmd = left_current;
  vmc_balance.right_wheel_current_cmd = right_current;
  vmc_balance.left_wheel_current_send =
      (int16_t)(VMC_WHEEL_LEFT_CMD_DIRECTION * left_current);
  vmc_balance.right_wheel_current_send =
      (int16_t)(VMC_WHEEL_RIGHT_CMD_DIRECTION * right_current);
  vmc_balance.left_tp_cmd = clamp_float_abs((float)left_tp_raw, tp_limit);
  vmc_balance.right_tp_cmd = clamp_float_abs((float)right_tp_raw, tp_limit);
  vmc_probe_state.balance = vmc_balance;
}

static int16_t vmc_update_yaw_hold_current(void)
{
  double yaw_error_raw;
  double yaw_error;
  double yaw_rate_error;
  double yaw_current;
  int16_t yaw_limit = vmc_balance.yaw_hold_current_limit;

  if (yaw_limit < 0) {
    yaw_limit = (int16_t)-yaw_limit;
  }
  if (yaw_limit > vmc_wheel_control.max_current) {
    yaw_limit = vmc_wheel_control.max_current;
  }

  if (vmc_balance.yaw_hold_enable == 0U ||
      vmc_balance.wheel_output_enable == 0U ||
      vmc_probe_state.body.imu_valid == 0U ||
      vmc_probe_state.body.imu_connected == 0U) {
    vmc_balance.yaw_hold_active = 0U;
    vmc_balance.yaw_target = vmc_probe_state.body.yaw;
    vmc_balance.yaw_error_raw = 0.0;
    vmc_balance.yaw_error = 0.0;
    vmc_balance.yaw_rate_error = 0.0;
    vmc_wheel_control.yaw_hold_current = 0;
    return 0;
  }

  if (vmc_balance.yaw_hold_active == 0U) {
    vmc_balance.yaw_target = vmc_probe_state.body.yaw;
    vmc_balance.yaw_hold_active = 1U;
  }

  /* Both target and feedback are unwrapped, so this error remains valid over
   * multiple complete rotations instead of jumping at +/-pi. */
  yaw_error_raw = (double)vmc_balance.yaw_target -
                  (double)vmc_probe_state.body.yaw;
  yaw_error = apply_soft_deadband(yaw_error_raw, vmc_balance.yaw_hold_deadband);
  yaw_rate_error = -vmc_probe_state.body.wz;
  yaw_current =
      (double)VMC_YAW_HOLD_DIRECTION *
      ((double)vmc_balance.yaw_hold_kp * yaw_error +
       (double)vmc_balance.yaw_hold_kd * yaw_rate_error);

  yaw_current = clamp_double(yaw_current, (double)-yaw_limit, (double)yaw_limit);
  vmc_balance.yaw_error_raw = yaw_error_raw;
  vmc_balance.yaw_error = yaw_error;
  vmc_balance.yaw_rate_error = yaw_rate_error;
  vmc_wheel_control.yaw_hold_current = (int16_t)yaw_current;
  return vmc_wheel_control.yaw_hold_current;
}

static int16_t vmc_update_gimbal_follow_current(void)
{
  double error_raw;
  double error;
  double p_output;
  double d_output;
  double output;
  int16_t current_limit = vmc_balance.gimbal_follow_current_limit;

  if (current_limit < 0) {
    current_limit = (int16_t)-current_limit;
  }
  if (current_limit > vmc_wheel_control.max_current) {
    current_limit = vmc_wheel_control.max_current;
  }

  if (vmc_balance.gimbal_follow_enable == 0U ||
      vmc_balance.gimbal_follow_active == 0U ||
      vmc_balance.gimbal_follow_front_valid == 0U ||
      vmc_balance.wheel_output_enable == 0U ||
      remote_control_source != RC_SOURCE_CBOARD ||
      board_comm_remote_is_valid() == 0U ||
      (board_comm_state.remote_flags & BOARD_COMM_FLAG_YAW_MOTOR_ONLINE) == 0U) {
    vmc_balance.gimbal_follow_error_raw = 0.0;
    vmc_balance.gimbal_follow_error = 0.0;
    vmc_balance.gimbal_follow_p_output = 0.0;
    vmc_balance.gimbal_follow_d_output = 0.0;
    vmc_balance.gimbal_follow_output = 0.0;
    vmc_wheel_control.gimbal_follow_current = 0;
    return 0;
  }

  error_raw = (double)vmc_gimbal_encoder_error(
      board_comm_state.yaw_ecd, vmc_balance.gimbal_follow_front_ecd);
  error = apply_encoder_deadband(error_raw,
                                 (double)vmc_balance.gimbal_follow_deadband);
  p_output = (double)vmc_balance.gimbal_follow_kp * error;
  d_output = -(double)vmc_balance.gimbal_follow_kd *
             (double)board_comm_state.yaw_speed_rpm;
  output = (double)vmc_balance.gimbal_follow_direction *
           (p_output + d_output);
  output = clamp_double(output, (double)-current_limit, (double)current_limit);

  vmc_balance.gimbal_follow_error_raw = error_raw;
  vmc_balance.gimbal_follow_error = error;
  vmc_balance.gimbal_follow_p_output = p_output;
  vmc_balance.gimbal_follow_d_output = d_output;
  vmc_balance.gimbal_follow_output = output;
  vmc_wheel_control.gimbal_follow_current = (int16_t)output;
  return vmc_wheel_control.gimbal_follow_current;
}

static void vmc_balance_apply_wheel_output(void)
{
  int16_t yaw_current;

  vmc_wheel_control.max_current = vmc_balance.wheel_current_limit;

  if (vmc_control.manual_prepose_active != 0U &&
      vmc_balance.enabled != 0U &&
      vmc_balance.wheel_output_user_enable != 0U &&
      vmc_manual_prepose_allows_wheel_output() != 0U) {
    vmc_wheel_control.enabled = 1U;
    vmc_wheel_control.yaw_hold_current = 0;
    vmc_wheel_control.gimbal_follow_current = 0;
    vmc_balance.yaw_hold_active = 0U;
    vmc_balance.gimbal_follow_active = 0U;
    vmc_wheel_control.left_current =
        clamp_i16((int16_t)(vmc_wheel_control.manual_forward_current +
                            vmc_wheel_control.manual_turn_current),
                  -vmc_wheel_control.max_current,
                  vmc_wheel_control.max_current);
    vmc_wheel_control.right_current =
        clamp_i16((int16_t)(vmc_wheel_control.manual_forward_current -
                            vmc_wheel_control.manual_turn_current),
                  -vmc_wheel_control.max_current,
                  vmc_wheel_control.max_current);
  } else if (vmc_balance.enabled != 0U &&
      vmc_balance.wheel_output_enable != 0U &&
      vmc_balance.wheel_output_user_enable != 0U &&
      vmc_getup_is_active() == 0U &&
      vmc_jump_blocks_wheel_output() == 0U &&
      vmc_safety_allows_output() != 0U) {
    if (vmc_balance.gimbal_follow_active != 0U) {
      yaw_current = vmc_update_gimbal_follow_current();
      vmc_wheel_control.yaw_hold_current = 0;
    } else {
      yaw_current = vmc_update_yaw_hold_current();
      vmc_wheel_control.gimbal_follow_current = 0;
    }
    vmc_wheel_control.enabled = 1U;
    vmc_wheel_control.left_current =
        clamp_i16((int16_t)(vmc_balance.left_wheel_current_cmd +
                            vmc_wheel_control.manual_left_current -
                            yaw_current),
                  -vmc_wheel_control.max_current,
                  vmc_wheel_control.max_current);
    vmc_wheel_control.right_current =
        clamp_i16((int16_t)(vmc_balance.right_wheel_current_cmd +
                            vmc_wheel_control.manual_right_current +
                            yaw_current),
                  -vmc_wheel_control.max_current,
                  vmc_wheel_control.max_current);
    /* Slip current scaling: reduce the FINAL current of whichever side is
     * slipping, matching the reference (which scales joint_m[i]->A after
     * limiting and after yaw is folded in).  Applied here rather than in the
     * balance term so it covers the whole wheel output, as the reference
     * does.  See VMC_SLIP_WHEEL_CURRENT_SCALE_DEFAULT for what this costs. */
    {
      const uint8_t slip_curr_enabled =
          (vmc_balance.slip_lqr_enable != 0U) ? 1U : 0U;
      const double slip_curr_scale =
          clamp_double(vmc_balance.slip_wheel_current_scale, 0.0, 1.0);
      const double left_current_scale =
          (slip_curr_enabled != 0U && body_motion_kf_slip_left != 0U)
              ? slip_curr_scale
              : 1.0;
      const double right_current_scale =
          (slip_curr_enabled != 0U && body_motion_kf_slip_right != 0U)
              ? slip_curr_scale
              : 1.0;
      vmc_balance.slip_curr_applied_left = (float)left_current_scale;
      vmc_balance.slip_curr_applied_right = (float)right_current_scale;
      vmc_wheel_control.left_current =
          (int16_t)((double)vmc_wheel_control.left_current * left_current_scale);
      vmc_wheel_control.right_current =
          (int16_t)((double)vmc_wheel_control.right_current * right_current_scale);
    }
  } else {
    vmc_balance.slip_curr_applied_left = 1.0f;
    vmc_balance.slip_curr_applied_right = 1.0f;
    vmc_balance.yaw_hold_active = 0U;
    vmc_balance.yaw_target = vmc_probe_state.body.yaw;
    vmc_balance.yaw_error_raw = 0.0;
    vmc_balance.yaw_error = 0.0;
    vmc_balance.yaw_rate_error = 0.0;
    vmc_wheel_control.yaw_hold_current = 0;
    vmc_balance.gimbal_follow_active = 0U;
    vmc_balance.gimbal_follow_error_raw = 0.0;
    vmc_balance.gimbal_follow_error = 0.0;
    vmc_balance.gimbal_follow_p_output = 0.0;
    vmc_balance.gimbal_follow_d_output = 0.0;
    vmc_balance.gimbal_follow_output = 0.0;
    vmc_wheel_control.gimbal_follow_current = 0;
    vmc_wheel_control.enabled = 0U;
    vmc_wheel_control.left_current = 0;
    vmc_wheel_control.right_current = 0;
  }
  vmc_probe_state.balance = vmc_balance;
}

static void vmc_dm_send_init_phase(uint8_t phase)
{
  HAL_StatusTypeDef right_status;
  HAL_StatusTypeDef left_status;

  if (phase == 0U) {
    right_status = can_send_joint_init(VMC_JOINT_RIGHT_AHEAD_CAN,
                                       VMC_JOINT_RIGHT_AHEAD_CMD_ID);
    left_status = can_send_joint_init(VMC_JOINT_LEFT_AHEAD_CAN,
                                      VMC_JOINT_LEFT_AHEAD_CMD_ID);
  } else {
    right_status = can_send_joint_init(VMC_JOINT_RIGHT_BACK_CAN,
                                       VMC_JOINT_RIGHT_BACK_CMD_ID);
    left_status = can_send_joint_init(VMC_JOINT_LEFT_BACK_CAN,
                                      VMC_JOINT_LEFT_BACK_CMD_ID);
  }

  update_init_tx_counters(right_status);
  update_init_tx_counters(left_status);
}

static void vmc_dm_send_recover_phase(uint8_t phase)
{
  HAL_StatusTypeDef right_status;
  HAL_StatusTypeDef left_status;

  if (phase == 0U) {
    right_status = can_send_joint_enable(VMC_JOINT_RIGHT_AHEAD_CAN,
                                         VMC_JOINT_RIGHT_AHEAD_CMD_ID);
    left_status = can_send_joint_enable(VMC_JOINT_LEFT_AHEAD_CAN,
                                        VMC_JOINT_LEFT_AHEAD_CMD_ID);
  } else {
    right_status = can_send_joint_enable(VMC_JOINT_RIGHT_BACK_CAN,
                                         VMC_JOINT_RIGHT_BACK_CMD_ID);
    left_status = can_send_joint_enable(VMC_JOINT_LEFT_BACK_CAN,
                                        VMC_JOINT_LEFT_BACK_CMD_ID);
  }

  update_recover_tx_counters(right_status);
  update_recover_tx_counters(left_status);
}

static void vmc_dm_send_disable_phase(uint8_t phase)
{
  HAL_StatusTypeDef right_status;
  HAL_StatusTypeDef left_status;

  if (phase == 0U) {
    right_status = can_send_joint_disable(VMC_JOINT_RIGHT_AHEAD_CAN,
                                          VMC_JOINT_RIGHT_AHEAD_CMD_ID);
    left_status = can_send_joint_disable(VMC_JOINT_LEFT_AHEAD_CAN,
                                         VMC_JOINT_LEFT_AHEAD_CMD_ID);
  } else {
    right_status = can_send_joint_disable(VMC_JOINT_RIGHT_BACK_CAN,
                                          VMC_JOINT_RIGHT_BACK_CMD_ID);
    left_status = can_send_joint_disable(VMC_JOINT_LEFT_BACK_CAN,
                                         VMC_JOINT_LEFT_BACK_CMD_ID);
  }

  update_disable_tx_counters(right_status);
  update_disable_tx_counters(left_status);
}

static void vmc_dm_send_control_phase(uint8_t phase)
{
  HAL_StatusTypeDef right_status;
  HAL_StatusTypeDef left_status;

  if (phase == 0U) {
    right_status = can_send_joint_torque(VMC_JOINT_RIGHT_AHEAD_CAN,
                                         VMC_JOINT_RIGHT_AHEAD_CMD_ID,
                                         vmc_probe_state.right_leg.joint_torque_ahead);
    left_status = can_send_joint_torque(VMC_JOINT_LEFT_AHEAD_CAN,
                                        VMC_JOINT_LEFT_AHEAD_CMD_ID,
                                        vmc_probe_state.left_leg.joint_torque_ahead);
  } else {
    right_status = can_send_joint_torque(VMC_JOINT_RIGHT_BACK_CAN,
                                         VMC_JOINT_RIGHT_BACK_CMD_ID,
                                         vmc_probe_state.right_leg.joint_torque_back);
    left_status = can_send_joint_torque(VMC_JOINT_LEFT_BACK_CAN,
                                        VMC_JOINT_LEFT_BACK_CMD_ID,
                                        vmc_probe_state.left_leg.joint_torque_back);
  }

  update_tx_counters(right_status);
  update_tx_counters(left_status);
}

void vmc_dm_state_machine_update(void)
{
  const uint8_t phase = vmc_probe_state.dm_tx_phase;
  const uint8_t feedback_source = vmc_feedback_timeout_source();

  vmc_probe_state.feedback_timeout_source = feedback_source;
  vmc_probe_state.feedback_timeout_active =
      (feedback_source == VMC_FEEDBACK_TIMEOUT_NONE) ? 0U : 1U;

  vmc_probe_state.phi0_protect_latched = 0U;
  vmc_probe_state.phi0_protect_source = VMC_PHI0_PROTECT_NONE;

  if (vmc_fall_rearm_required != 0U ||
      vmc_probe_state.feedback_timeout_active != 0U ||
      remote_control_allows_joint_enable() == 0U) {
    clear_leg_command(&vmc_probe_state.left_leg);
    clear_leg_command(&vmc_probe_state.right_leg);
    vmc_probe_state.dm_state = VMC_DM_STATE_DISABLE;
    vmc_dm_send_disable_phase(phase);
  } else if (vmc_joint_state_is_any(VMC_DM_STATUS_NOT_ENABLED) != 0U) {
    vmc_probe_state.dm_state = VMC_DM_STATE_INIT;
    vmc_dm_send_init_phase(phase);
  } else if (vmc_joint_state_is_any(VMC_DM_STATUS_CAN_TIMEOUT) != 0U) {
    vmc_probe_state.dm_state = VMC_DM_STATE_RECOVER;
    vmc_dm_send_recover_phase(phase);
  } else {
    vmc_probe_state.dm_state = VMC_DM_STATE_CONTROL;
    vmc_dm_send_control_phase(phase);
  }

  vmc_probe_state.dm_tx_phase = (phase == 0U) ? 1U : 0U;
}

void vmc_can_poll_joints_zero(void)
{
  vmc_dm_state_machine_update();
}

void vmc_update_leg_state(void)
{
  double length;
  double phi0;

  vmc_probe_state.left_leg.phi1 =
      joint_model_angle(vmc_probe_state.joint_left_back.position,
                        VMC_REF_RAW_LEFT_BACK,
                        VMC_REF_MODEL_PHI1,
                        VMC_DIR_LEFT_BACK);
  vmc_probe_state.left_leg.phi4 =
      joint_model_angle(vmc_probe_state.joint_left_ahead.position,
                        VMC_REF_RAW_LEFT_AHEAD,
                        VMC_REF_MODEL_PHI4,
                        VMC_DIR_LEFT_AHEAD);
  vmc_leg_position(vmc_probe_state.left_leg.phi1, vmc_probe_state.left_leg.phi4,
                   &length, &phi0);
  vmc_probe_state.left_leg.length = length;
  vmc_probe_state.left_leg.phi0 = phi0;

  vmc_probe_state.right_leg.phi4 =
      joint_model_angle(vmc_probe_state.joint_right_ahead.position,
                        VMC_REF_RAW_RIGHT_AHEAD,
                        VMC_REF_MODEL_PHI4,
                        VMC_DIR_RIGHT_AHEAD);
  vmc_probe_state.right_leg.phi1 =
      joint_model_angle(vmc_probe_state.joint_right_back.position,
                        VMC_REF_RAW_RIGHT_BACK,
                        VMC_REF_MODEL_PHI1,
                        VMC_DIR_RIGHT_BACK);
  vmc_leg_position(vmc_probe_state.right_leg.phi1, vmc_probe_state.right_leg.phi4,
                   &length, &phi0);
  vmc_probe_state.right_leg.length = length;
  vmc_probe_state.right_leg.phi0 = phi0;

  vmc_probe_state.left_leg.d_phi1 =
      joint_model_speed(vmc_probe_state.joint_left_back.speed, VMC_DIR_LEFT_BACK);
  vmc_probe_state.left_leg.d_phi4 =
      joint_model_speed(vmc_probe_state.joint_left_ahead.speed, VMC_DIR_LEFT_AHEAD);
  vmc_leg_speed(vmc_probe_state.left_leg.d_phi1, vmc_probe_state.left_leg.d_phi4,
                vmc_probe_state.left_leg.phi1, vmc_probe_state.left_leg.phi4,
                &length, &phi0);
  vmc_probe_state.left_leg.d_length = length;
  vmc_probe_state.left_leg.d_phi0 = phi0;

  vmc_probe_state.right_leg.d_phi4 =
      joint_model_speed(vmc_probe_state.joint_right_ahead.speed, VMC_DIR_RIGHT_AHEAD);
  vmc_probe_state.right_leg.d_phi1 =
      joint_model_speed(vmc_probe_state.joint_right_back.speed, VMC_DIR_RIGHT_BACK);
  vmc_leg_speed(vmc_probe_state.right_leg.d_phi1, vmc_probe_state.right_leg.d_phi4,
                vmc_probe_state.right_leg.phi1, vmc_probe_state.right_leg.phi4,
                &length, &phi0);
  vmc_probe_state.right_leg.d_length = length;
  vmc_probe_state.right_leg.d_phi0 = phi0;

  /* Shanhai's FN observer needs ddL0 and ddtheta.  Differentiate the measured
   * virtual-leg rates and low-pass the result; no commanded leg motion is
   * substituted here, so free-fall and landing are visible to the observer. */
  {
    if (vmc_control.enabled == 0U ||
        vmc_probe_state.feedback_timeout_active != 0U ||
        remote_control_allows_joint_enable() == 0U) {
      vmc_probe_state.left_leg.acceleration_valid = 0U;
      vmc_probe_state.right_leg.acceleration_valid = 0U;
      vmc_probe_state.left_leg.dd_length = 0.0;
      vmc_probe_state.left_leg.dd_theta = 0.0;
      vmc_probe_state.right_leg.dd_length = 0.0;
      vmc_probe_state.right_leg.dd_theta = 0.0;
      return;
    }

    const double dt = vmc_loop_dt();
    const double alpha = VMC_LEG_ACCEL_FILTER_ALPHA_DEFAULT;
    const double left_theta_rate =
        -(vmc_probe_state.left_leg.d_phi0 +
          (double)vmc_probe_state.body.pitch_rate);
    const double right_theta_rate =
        -(vmc_probe_state.right_leg.d_phi0 +
          (double)vmc_probe_state.body.pitch_rate);

    if (vmc_probe_state.left_leg.acceleration_valid == 0U) {
      vmc_probe_state.left_leg.previous_d_length =
          vmc_probe_state.left_leg.d_length;
      vmc_probe_state.left_leg.previous_d_theta = left_theta_rate;
      vmc_probe_state.left_leg.dd_length = 0.0;
      vmc_probe_state.left_leg.dd_theta = 0.0;
      vmc_probe_state.left_leg.acceleration_valid = 1U;
    } else {
      const double dd_length_raw =
          (vmc_probe_state.left_leg.d_length -
           vmc_probe_state.left_leg.previous_d_length) /
          dt;
      const double dd_theta_raw =
          (left_theta_rate - vmc_probe_state.left_leg.previous_d_theta) / dt;
      vmc_probe_state.left_leg.dd_length +=
          alpha * (dd_length_raw - vmc_probe_state.left_leg.dd_length);
      vmc_probe_state.left_leg.dd_theta +=
          alpha * (dd_theta_raw - vmc_probe_state.left_leg.dd_theta);
      vmc_probe_state.left_leg.previous_d_length =
          vmc_probe_state.left_leg.d_length;
      vmc_probe_state.left_leg.previous_d_theta = left_theta_rate;
    }

    if (vmc_probe_state.right_leg.acceleration_valid == 0U) {
      vmc_probe_state.right_leg.previous_d_length =
          vmc_probe_state.right_leg.d_length;
      vmc_probe_state.right_leg.previous_d_theta = right_theta_rate;
      vmc_probe_state.right_leg.dd_length = 0.0;
      vmc_probe_state.right_leg.dd_theta = 0.0;
      vmc_probe_state.right_leg.acceleration_valid = 1U;
    } else {
      const double dd_length_raw =
          (vmc_probe_state.right_leg.d_length -
           vmc_probe_state.right_leg.previous_d_length) /
          dt;
      const double dd_theta_raw =
          (right_theta_rate - vmc_probe_state.right_leg.previous_d_theta) / dt;
      vmc_probe_state.right_leg.dd_length +=
          alpha * (dd_length_raw - vmc_probe_state.right_leg.dd_length);
      vmc_probe_state.right_leg.dd_theta +=
          alpha * (dd_theta_raw - vmc_probe_state.right_leg.dd_theta);
      vmc_probe_state.right_leg.previous_d_length =
          vmc_probe_state.right_leg.d_length;
      vmc_probe_state.right_leg.previous_d_theta = right_theta_rate;
    }
  }
}

void vmc_control_init(void)
{
  vmc_control.enabled = 0U;
  vmc_control.target_length = VMC_LEG_LENGTH_DEFAULT;
  vmc_control.jump_enable = VMC_JUMP_ENABLE_DEFAULT;
  vmc_control.jump_moving_enable = VMC_JUMP_MOVING_ENABLE_DEFAULT;
  vmc_control.jump_stage = VMC_JUMP_STAGE_IDLE;
  vmc_control.jump_armed = 0U;
  vmc_control.jump_abort_reason = VMC_JUMP_ABORT_NONE;
  vmc_control.jump_stage_start_ms = 0U;
  vmc_control.jump_start_ms = 0U;
  vmc_control.jump_last_finish_ms = 0U;
  vmc_control.jump_stage_ms = 0U;
  vmc_control.jump_land_contact_ms = 0U;
  vmc_control.jump_trigger_count = 0U;
  vmc_control.jump_abort_count = 0U;
  vmc_control.jump_target_length = VMC_LEG_LENGTH_DEFAULT;
  vmc_control.jump_compress_length = VMC_JUMP_COMPRESS_LENGTH_DEFAULT;
  vmc_control.jump_extend_length = VMC_JUMP_EXTEND_LENGTH_DEFAULT;
  vmc_control.jump_retract_length = VMC_JUMP_RETRACT_LENGTH_DEFAULT;
  vmc_control.jump_land_length = VMC_JUMP_LAND_LENGTH_DEFAULT;
  vmc_control.jump_kp_length = VMC_JUMP_LENGTH_KP_DEFAULT;
  vmc_control.jump_kd_length = VMC_JUMP_LENGTH_KD_DEFAULT;
  vmc_control.jump_max_force = VMC_JUMP_FORCE_MAX_DEFAULT;
  vmc_control.jump_support_force = VMC_JUMP_SUPPORT_FORCE_DEFAULT;
  vmc_control.jump_compress_timeout_ms = VMC_JUMP_COMPRESS_TIMEOUT_MS_DEFAULT;
  vmc_control.jump_extend_timeout_ms = VMC_JUMP_EXTEND_TIMEOUT_MS_DEFAULT;
  vmc_control.jump_retract_timeout_ms = VMC_JUMP_RETRACT_TIMEOUT_MS_DEFAULT;
  vmc_control.jump_land_timeout_ms = VMC_JUMP_LAND_TIMEOUT_MS_DEFAULT;
  vmc_control.jump_cooldown_ms = VMC_JUMP_COOLDOWN_MS_DEFAULT;
  vmc_control.jump_total_timeout_ms = VMC_JUMP_TOTAL_TIMEOUT_MS_DEFAULT;
  vmc_control.jump_attitude_limit = VMC_JUMP_ATTITUDE_LIMIT_DEFAULT;
  vmc_control.jump_land_torque_threshold = VMC_JUMP_LAND_TORQUE_THRESHOLD_DEFAULT;
  vmc_control.jump_land_confirm_ms = VMC_JUMP_LAND_CONFIRM_MS_DEFAULT;
  vmc_control.jump_auto_speed_min = VMC_JUMP_AUTO_SPEED_MIN;
  vmc_control.jump_auto_dist_min_cm = VMC_JUMP_AUTO_DIST_MIN_CM;
  vmc_control.jump_auto_dist_max_cm = VMC_JUMP_AUTO_DIST_MAX_CM;
  vmc_control.jump_auto_confirm_ms = VMC_JUMP_AUTO_CONFIRM_MS;
  vmc_control.stair_enable = VMC_STAIR_ENABLE_DEFAULT;
  vmc_control.stair_stage = VMC_STAIR_STAGE_IDLE;
  vmc_control.stair_abort_reason = VMC_STAIR_ABORT_NONE;
  vmc_control.stair_length_override_active = 0U;
  vmc_control.stair_fast_detected = 0U;
  vmc_control.stair_impact_detected = 0U;
  vmc_control.stair_ready_detected = 0U;
  vmc_control.stair_preimpact_armed = 0U;
  vmc_control.stair_left_impact = 0U;
  vmc_control.stair_right_impact = 0U;
  vmc_control.stair_stage_start_ms = 0U;
  vmc_control.stair_start_ms = 0U;
  vmc_control.stair_last_finish_ms = 0U;
  vmc_control.stair_stage_ms = 0U;
  vmc_control.stair_trigger_count = 0U;
  vmc_control.stair_abort_count = 0U;
  vmc_control.stair_ready_ms = 0U;
  vmc_control.stair_speed_ms = 0U;
  vmc_control.stair_preimpact_ms = 0U;
  vmc_control.stair_impact_ms = 0U;
  vmc_control.stair_ready_confirm_ms = VMC_STAIR_READY_CONFIRM_MS_DEFAULT;
  vmc_control.stair_speed_confirm_ms = VMC_STAIR_SPEED_CONFIRM_MS_DEFAULT;
  vmc_control.stair_preimpact_confirm_ms =
      VMC_STAIR_PREIMPACT_CONFIRM_MS_DEFAULT;
  vmc_control.stair_impact_confirm_ms =
      VMC_STAIR_IMPACT_CONFIRM_MS_DEFAULT;
  vmc_control.stair_retract_timeout_ms =
      VMC_STAIR_RETRACT_TIMEOUT_MS_DEFAULT;
  vmc_control.stair_pose_confirm_ms = VMC_STAIR_POSE_CONFIRM_MS_DEFAULT;
  vmc_control.stair_pose_confirm_elapsed_ms = 0U;
  vmc_control.stair_hold_ms = VMC_STAIR_HOLD_MS_DEFAULT;
  vmc_control.stair_recover_ms = VMC_STAIR_RECOVER_MS_DEFAULT;
  vmc_control.stair_timeout_ms = VMC_STAIR_TIMEOUT_MS_DEFAULT;
  vmc_control.stair_cooldown_ms = VMC_STAIR_COOLDOWN_MS_DEFAULT;
  vmc_control.stair_lift_length = VMC_STAIR_LIFT_LENGTH_DEFAULT;
  vmc_control.stair_retract_length = VMC_STAIR_RETRACT_LENGTH_DEFAULT;
  vmc_control.stair_lift_theta = VMC_STAIR_LIFT_THETA_DEFAULT;
  vmc_control.stair_retract_theta = VMC_STAIR_RETRACT_THETA_DEFAULT;
  vmc_control.stair_pose_length_tol =
      VMC_STAIR_POSE_LENGTH_TOL_DEFAULT;
  vmc_control.stair_pose_theta_tol =
      VMC_STAIR_POSE_THETA_TOL_DEFAULT;
  vmc_control.stair_theta_ramped = VMC_BALANCE_PITCH_OFFSET_DEFAULT;
  vmc_control.stair_phi0_target = VMC_STAIR_PHI0_TARGET_DEFAULT;
  vmc_control.stair_phi0_ramped = VMC_LEG_PHI0_TARGET_DEFAULT;
  vmc_control.stair_lift_phi0_ramp_rate =
      VMC_STAIR_LIFT_PHI0_RAMP_RATE_DEFAULT;
  vmc_control.stair_phi0_ramp_rate = VMC_STAIR_PHI0_RAMP_RATE_DEFAULT;
  vmc_control.stair_phi0_kp = VMC_STAIR_PHI0_KP_DEFAULT;
  vmc_control.stair_phi0_kd = VMC_STAIR_PHI0_KD_DEFAULT;
  vmc_control.stair_speed_threshold = VMC_STAIR_SPEED_THRESHOLD_DEFAULT;
  vmc_control.stair_command_speed_threshold =
      VMC_STAIR_COMMAND_SPEED_THRESHOLD_DEFAULT;
  vmc_control.stair_ready_length = VMC_STAIR_READY_LENGTH_DEFAULT;
  vmc_control.stair_preimpact_diff_max =
      VMC_STAIR_PREIMPACT_DIFF_MAX_DEFAULT;
  vmc_control.stair_impact_diff_threshold =
      VMC_STAIR_IMPACT_DIFF_THRESHOLD_DEFAULT;
  vmc_control.stair_impact_torque_threshold =
      VMC_STAIR_IMPACT_TORQUE_THRESHOLD_DEFAULT;
  vmc_control.stair_impact_theta_threshold =
      VMC_STAIR_IMPACT_THETA_THRESHOLD_DEFAULT;
  vmc_control.stair_speed_abs = 0.0f;
  vmc_control.stair_speed_diff = 0.0f;
  vmc_control.stair_accel = 0.0f;
  vmc_control.stair_kp_length = VMC_STAIR_LENGTH_KP_DEFAULT;
  vmc_control.stair_kd_length = VMC_STAIR_LENGTH_KD_DEFAULT;
  vmc_control.stair_length_ramp_rate = VMC_STAIR_LENGTH_RAMP_RATE_DEFAULT;
  vmc_control.stair_max_force = VMC_STAIR_FORCE_MAX_DEFAULT;
  vmc_control.stair_lift_support_force =
      VMC_STAIR_LIFT_SUPPORT_FORCE_DEFAULT;
  vmc_control.stair_retract_support_force =
      VMC_STAIR_RETRACT_SUPPORT_FORCE_DEFAULT;
  vmc_control.stair_support_force = VMC_STAIR_SUPPORT_FORCE_DEFAULT;
  vmc_stair_reset_detector();
  vmc_control.ramped_target_length = VMC_LEG_LENGTH_DEFAULT;
  vmc_control.length_ramp_rate = VMC_LEG_LENGTH_RAMP_RATE_DEFAULT;
  vmc_control.kp_length = VMC_LEG_LENGTH_KP_DEFAULT;
  vmc_control.kd_length = VMC_LEG_LENGTH_KD_DEFAULT;
  vmc_control.target_phi0 = VMC_LEG_PHI0_TARGET_DEFAULT;
  vmc_control.kp_phi0 = VMC_LEG_PHI0_KP_DEFAULT;
  vmc_control.kd_phi0 = VMC_LEG_PHI0_KD_DEFAULT;
  vmc_control.max_phi0_torque = VMC_LEG_PHI0_TORQUE_MAX_DEFAULT;
  vmc_control.max_force = VMC_LEG_FORCE_MAX_DEFAULT;
  vmc_control.support_force = VMC_LEG_SUPPORT_FORCE_DEFAULT;
  vmc_control.roll_comp_enable = VMC_ROLL_COMP_ENABLE_DEFAULT;
  vmc_control.roll_target = VMC_ROLL_TARGET_DEFAULT;
  vmc_control.roll_remote_target_max = VMC_ROLL_REMOTE_TARGET_MAX_DEFAULT;
  vmc_control.roll_remote_deadband = VMC_ROLL_REMOTE_DEADBAND_DEFAULT;
  vmc_control.roll_comp_gain = VMC_ROLL_LENGTH_COMP_GAIN_DEFAULT;
  vmc_control.roll_comp_limit = VMC_ROLL_LENGTH_COMP_LIMIT_DEFAULT;
  vmc_control.roll_comp_ramp_rate = VMC_ROLL_LENGTH_COMP_RAMP_RATE_DEFAULT;
  vmc_control.roll_comp_offset = 0.0f;
  vmc_control.left_target_length = VMC_LEG_LENGTH_DEFAULT;
  vmc_control.right_target_length = VMC_LEG_LENGTH_DEFAULT;
  vmc_control.getup_state = VMC_GETUP_STATE_IDLE;
  vmc_control.getup_turn_direction = 0;
  vmc_control.getup_body_flip_direction = 0;
  vmc_control.getup_left_direction = 0;
  vmc_control.getup_right_direction = 0;
  vmc_control.getup_left_done = 0U;
  vmc_control.getup_right_done = 0U;
  vmc_control.getup_lqr_release_pending = 0U;
  vmc_control.getup_leg_align_complete = 0U;
  vmc_control.getup_partial_armed = 1U;
  vmc_control.getup_partial_active = 0U;
  vmc_control.manual_prepose_armed = 0U;
  vmc_control.manual_prepose_active = 0U;
  vmc_control.manual_prepose_exit_length_check_active = 0U;
  vmc_control.manual_prepose_phi0_left = VMC_LEG_PHI0_TARGET_DEFAULT;
  vmc_control.manual_prepose_phi0_right = VMC_LEG_PHI0_TARGET_DEFAULT;
  vmc_control.manual_prepose_knob_position = 0.0f;
  vmc_control.manual_prepose_base_length_left = VMC_LEG_LENGTH_DEFAULT;
  vmc_control.manual_prepose_base_length_right = VMC_LEG_LENGTH_DEFAULT;
  vmc_control.manual_prepose_length_left = VMC_LEG_LENGTH_DEFAULT;
  vmc_control.manual_prepose_length_right = VMC_LEG_LENGTH_DEFAULT;
  vmc_control.getup_left_confirm_ms = 0U;
  vmc_control.getup_right_confirm_ms = 0U;
  vmc_control.getup_left_hold_length = VMC_LEG_LENGTH_DEFAULT;
  vmc_control.getup_right_hold_length = VMC_LEG_LENGTH_DEFAULT;
  vmc_control.getup_left_hold_theta = 0.0f;
  vmc_control.getup_right_hold_theta = 0.0f;
  vmc_control.getup_body_fan_sync_theta_left = 0.0f;
  vmc_control.getup_body_fan_sync_theta_right = 0.0f;
  vmc_control.getup_body_fan_sync_complete = 0U;
  vmc_control.getup_body_fan_sync_phi0_target = 0.0f;
  vmc_control.getup_body_fan_sync_fold_side = 0U;
  vmc_control.getup_body_fan_sync_prelift_active = 0U;
  vmc_control.getup_body_fan_sync_targets_ready = 0U;
  vmc_control.getup_body_fan_sync_initial_phase_close = 0U;
  vmc_control.getup_body_fan_sync_leader_side = 0U;
  vmc_control.getup_body_fan_sync_prelift_confirm_ms = 0U;
  vmc_control.getup_body_fan_escape_active = 0U;
  vmc_control.getup_body_fan_retry_used = 0U;
  vmc_control.getup_body_fan_stall_ms = 0U;
  vmc_control.getup_shrink_target_theta_left = 0.0f;
  vmc_control.getup_shrink_target_theta_right = 0.0f;
  vmc_control.getup_target_phi0_left = VMC_LEG_PHI0_TARGET_DEFAULT;
  vmc_control.getup_target_phi0_right = VMC_LEG_PHI0_TARGET_DEFAULT;
  vmc_control.getup_upright_phi0_left = VMC_LEG_PHI0_TARGET_DEFAULT;
  vmc_control.getup_upright_phi0_right = VMC_LEG_PHI0_TARGET_DEFAULT;
  vmc_control.getup_phi0_feedforward_left = 0.0f;
  vmc_control.getup_phi0_feedforward_right = 0.0f;
  vmc_control.getup_state_ms = 0U;
  vmc_control.getup_confirm_ms = 0U;
  vmc_getup_fully_fallen = 0U;
  vmc_fall_rearm_required = 0U;
  vmc_fall_rearm_armed = 0U;
  vmc_fall_monitor_armed = 0U;
  vmc_fall_prev_fully_fallen = 0U;
  vmc_jump_last_update_ms = 0U;
  vmc_jump_auto_hold_ms = 0U;
  vmc_stair_auto_armed = 0U;
  vmc_control.ramp_active = 0U;
}

void vmc_wheel_control_init(void)
{
  vmc_wheel_control.enabled = 0U;
  vmc_wheel_control.left_current = 0;
  vmc_wheel_control.right_current = 0;
  vmc_wheel_control.max_current = VMC_WHEEL_CURRENT_MAX_DEFAULT;
  vmc_wheel_control.manual_left_current = 0;
  vmc_wheel_control.manual_right_current = 0;
  vmc_wheel_control.left_current_output = 0;
  vmc_wheel_control.right_current_output = 0;
}

static void clear_leg_command(volatile vmc_leg_state_t *leg)
{
  leg->length_error = 0.0;
  leg->length_p_output = 0.0;
  leg->length_d_output = 0.0;
  leg->length_pid_output = 0.0;
  leg->phi0_error = 0.0;
  leg->phi0_p_output = 0.0;
  leg->phi0_d_output = 0.0;
  leg->phi0_pid_output = 0.0;
  leg->support_force_output = 0.0;
  leg->force_command = 0.0;
  leg->torque_phi0_command = 0.0;
  leg->joint_torque_ahead = 0.0;
  leg->joint_torque_back = 0.0;
}

static void update_leg_length_control(volatile vmc_leg_state_t *leg,
                                      double target_length,
                                      double kp_length,
                                      double kd_length,
                                      double max_force,
                                      double target_phi0,
                                      double kp_phi0,
                                      double kd_phi0,
                                      double max_phi0_torque,
                                      double support_force,
                                      double leg_sync_torque,
                                      double ahead_direction,
                                      double back_direction,
                                      double phi0_torque_feedforward,
                                      uint8_t airborne_posture,
                                      double airborne_posture_tp)
{
  double joint_torque[2];
  const uint8_t manual_pd_hold =
      (vmc_control.manual_prepose_active != 0U ||
       vmc_control.manual_prepose_exit_length_check_active != 0U)
          ? 1U
          : 0U;
  const double length_error = target_length - leg->length;
  const double p_output = kp_length * length_error;
  const double d_output = -kd_length * leg->d_length;
  const double pid_output = VMC_LEG_LENGTH_PID_DIRECTION * (p_output + d_output);
  const double support_output = VMC_LEG_SUPPORT_FORCE_DIRECTION * support_force;
  const double force = clamp_double(pid_output + support_output, -max_force, max_force);
  const double phi0_error =
      (vmc_stair_is_active() != 0U ||
       manual_pd_hold != 0U)
          ? (double)vmc_rad_format((float)(target_phi0 - leg->phi0))
          : target_phi0 - leg->phi0;
  const double phi0_p_output = kp_phi0 * phi0_error;
  const double phi0_d_output = -kd_phi0 * leg->d_phi0;
  const double phi0_pid_output =
      VMC_LEG_PHI0_PID_DIRECTION * (phi0_p_output + phi0_d_output);
  const double phi0_torque_raw =
      clamp_double(phi0_pid_output + phi0_torque_feedforward,
                   -max_phi0_torque,
                   max_phi0_torque);
  const uint8_t use_lqr_tp =
      (vmc_balance.tp_output_enable != 0U &&
       manual_pd_hold == 0U &&
       vmc_getup_is_active() == 0U &&
       vmc_stair_is_active() == 0U &&
       (vmc_jump_is_active_or_aborted() == 0U ||
        vmc_jump_joint_lqr_active() != 0U))
          ? 1U
          : 0U;
  const double phi0_torque =
      (use_lqr_tp != 0U) ? 0.0 : phi0_torque_raw;
  const double lqr_tp_torque =
      (use_lqr_tp != 0U)
          ? ((leg == &vmc_probe_state.left_leg)
                 ? (double)vmc_balance.left_tp_cmd
                 : (double)vmc_balance.right_tp_cmd)
                * vmc_stair_release_scale()
          : 0.0;
  if (manual_pd_hold != 0U) {
    leg_sync_torque = 0.0;
  }
  const double phi0_torque_total =
      (airborne_posture != 0U &&
       manual_pd_hold == 0U)
          ? clamp_double(airborne_posture_tp,
                         -max_phi0_torque,
                         max_phi0_torque)
          : clamp_double(phi0_torque + lqr_tp_torque + leg_sync_torque,
                         -max_phi0_torque,
                         max_phi0_torque);

  leg->length_error = length_error;
  leg->length_p_output = p_output;
  leg->length_d_output = d_output;
  leg->length_pid_output = pid_output;
  leg->phi0_error = phi0_error;
  leg->phi0_p_output = phi0_p_output;
  leg->phi0_d_output = phi0_d_output;
  leg->phi0_pid_output = phi0_pid_output;
  leg->support_force_output = support_output;
  leg->force_command = force;
  leg->torque_phi0_command = phi0_torque_total;

  vmc_leg_force(force, leg->torque_phi0_command, leg->phi1, leg->phi4, joint_torque);

  /* Final per-joint bound.  vmc_leg_force() has no limit of its own, and the
   * force path alone (max_force is 500 N, about 0.147 N.m per N at the
   * shortest stance) can already ask for more than the motor rating.  Scale
   * both joints by the same factor so the commanded (force, Tp) direction is
   * preserved; clipping one joint on its own would silently rotate the leg. */
  {
    const double joint_peak =
        (fabs(joint_torque[0]) > fabs(joint_torque[1])) ? fabs(joint_torque[0])
                                                        : fabs(joint_torque[1]);
    if (joint_peak > (double)VMC_DM_J8009_TMAX) {
      const double scale = (double)VMC_DM_J8009_TMAX / joint_peak;
      joint_torque[0] *= scale;
      joint_torque[1] *= scale;
    }
  }

  leg->joint_torque_ahead = ahead_direction * joint_torque[0];
  leg->joint_torque_back = back_direction * joint_torque[1];
}

void vmc_control_update(void)
{
  double target_length;
  double ramped_target_length;
  double max_force;
  double left_max_force;
  double right_max_force;
  double left_target_phi0;
  double right_target_phi0;
  double left_phi0_speed_target;
  double right_phi0_speed_target;
  double left_phi0_feedforward;
  double right_phi0_feedforward;
  double max_phi0_torque;
  double left_max_phi0_torque;
  double right_max_phi0_torque;
  double support_force;
  double left_support_force;
  double right_support_force;
  double roll_error;
  double roll_comp_desired;
  double roll_comp_offset;
  double roll_comp_limit;
  double left_target_length;
  double right_target_length;
  double length_ramp_rate;
  double kp_length;
  double kd_length;
  double kp_phi0;
  double kd_phi0;
  uint8_t jump_override;
  uint8_t stair_override;
  uint8_t jump_force_profile;
  uint8_t stair_force_profile;
  uint8_t getup_force_profile;
  uint8_t getup_fan_profile;
  uint8_t getup_sync_torque_profile;
  uint8_t left_airborne_posture;
  uint8_t right_airborne_posture;
  double left_airborne_tp;
  double right_airborne_tp;

  vmc_update_remote_target_length();
  vmc_update_getup_state();
  getup_force_profile = vmc_getup_is_active();
  getup_fan_profile =
      (vmc_control.getup_state == VMC_GETUP_STATE_BODY_ALIGN ||
       vmc_control.getup_state == VMC_GETUP_STATE_BODY_FLIP ||
       vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN ||
       vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN_SYNC ||
       vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER)
          ? 1U
          : 0U;
  getup_sync_torque_profile =
      (vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN_SYNC &&
       vmc_control.getup_body_fan_sync_prelift_active == 0U &&
       vmc_control.getup_body_fan_sync_targets_ready != 0U)
          ? 1U
          : 0U;
  left_phi0_speed_target = 0.0;
  right_phi0_speed_target = 0.0;
  if (vmc_jump_is_active() != 0U && vmc_getup_is_active() != 0U) {
    vmc_jump_finish(VMC_JUMP_STAGE_ABORT,
                    VMC_JUMP_ABORT_GETUP,
                    HAL_GetTick());
  }
  jump_override = vmc_jump_overrides_target();
  stair_override = vmc_stair_overrides_target();

  if (vmc_control.enabled == 0U ||
      vmc_fall_rearm_required != 0U ||
      vmc_probe_state.feedback_timeout_active != 0U ||
      remote_control_allows_joint_enable() == 0U) {
    vmc_control.ramp_active = 0U;
    vmc_control.roll_comp_offset = 0.0f;
    vmc_control.left_target_length = vmc_control.target_length;
    vmc_control.right_target_length = vmc_control.target_length;
    vmc_control.getup_target_phi0_left = VMC_LEG_PHI0_TARGET_DEFAULT;
    vmc_control.getup_target_phi0_right = VMC_LEG_PHI0_TARGET_DEFAULT;
    vmc_control.getup_phi0_feedforward_left = 0.0f;
    vmc_control.getup_phi0_feedforward_right = 0.0f;
    vmc_control.getup_state_ms = 0U;
    vmc_control.getup_confirm_ms = 0U;
    clear_leg_command(&vmc_probe_state.left_leg);
    clear_leg_command(&vmc_probe_state.right_leg);
    return;
  }

  if (vmc_control.ramp_active == 0U) {
    vmc_control.ramped_target_length = (float)clamp_double(
        (vmc_probe_state.left_leg.length + vmc_probe_state.right_leg.length) * 0.5,
        VMC_LEG_LENGTH_MIN,
        VMC_LEG_LENGTH_MAX);
    vmc_control.ramp_active = 1U;
  }

  target_length = clamp_double(vmc_control.target_length,
                               VMC_LEG_LENGTH_MIN,
                               VMC_LEG_LENGTH_MAX);
  if (vmc_control.manual_prepose_active != 0U) {
    target_length =
        ((double)vmc_control.manual_prepose_length_left +
         (double)vmc_control.manual_prepose_length_right) *
        0.5;
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_BRACE ||
      vmc_control.getup_state == VMC_GETUP_STATE_BODY_ALIGN ||
      vmc_control.getup_state == VMC_GETUP_STATE_BODY_FLIP ||
      vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER) {
    target_length =
        (vmc_probe_state.left_leg.length + vmc_probe_state.right_leg.length) *
        0.5;
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_TURN) {
    target_length = VMC_GETUP_TURN_LENGTH;
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN ||
             vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN_SYNC) {
    target_length = VMC_GETUP_BODY_FAN_SYNC_LENGTH;
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_FOLDED_LOWER) {
    target_length = VMC_GETUP_PARTIAL_LOWER_LENGTH;
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_SHRINK ||
             vmc_control.getup_state == VMC_GETUP_STATE_UPRIGHT) {
    target_length = VMC_GETUP_STAND_LENGTH;
  } else if (jump_override != 0U) {
    target_length = vmc_jump_stage_target_length();
  } else if (stair_override != 0U ||
             vmc_control.stair_length_override_active != 0U) {
    target_length = vmc_stair_target_length();
  }
  vmc_control.jump_target_length = (float)target_length;
  if (vmc_control.manual_prepose_active != 0U) {
    ramped_target_length = target_length;
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_BRACE ||
      vmc_control.getup_state == VMC_GETUP_STATE_BODY_ALIGN ||
      vmc_control.getup_state == VMC_GETUP_STATE_BODY_FLIP ||
      vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER) {
    ramped_target_length = target_length;
  } else if (jump_override != 0U) {
    ramped_target_length = target_length;
  } else {
    length_ramp_rate =
        (stair_override != 0U ||
         vmc_control.stair_length_override_active != 0U)
            ? clamp_double(vmc_control.stair_length_ramp_rate, 0.0, 1.0)
            : ((vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN_SYNC)
                   ? clamp_double(VMC_GETUP_BODY_FAN_SYNC_LENGTH_RATE, 0.0, 1.0)
                   : ((getup_force_profile != 0U)
                          ? clamp_double(VMC_GETUP_LENGTH_RAMP_RATE, 0.0, 1.0)
                          : clamp_double(vmc_control.length_ramp_rate,
                                         0.0,
                                         1.0)));
    ramped_target_length =
        vmc_step_towards(vmc_control.ramped_target_length,
                         target_length,
                         length_ramp_rate * vmc_loop_dt());
  }
  vmc_control.ramped_target_length = (float)ramped_target_length;

  roll_comp_limit =
      clamp_double(vmc_control.roll_comp_limit, 0.0,
                   (double)VMC_ROLL_LENGTH_COMP_LIMIT_MAX);
  roll_error = (double)vmc_probe_state.body.roll - (double)vmc_control.roll_target;
  roll_comp_desired =
      (vmc_control.roll_comp_enable != 0U &&
       vmc_control.manual_prepose_active == 0U &&
       vmc_control.manual_prepose_exit_length_check_active == 0U &&
       vmc_getup_is_active() == 0U &&
       jump_override == 0U &&
       stair_override == 0U &&
       vmc_control.stair_length_override_active == 0U &&
       vmc_probe_state.body.imu_valid != 0U &&
       vmc_probe_state.body.imu_connected != 0U)
          ? clamp_double(roll_error * (double)vmc_control.roll_comp_gain,
                         -roll_comp_limit,
                         roll_comp_limit)
          : 0.0;
  roll_comp_offset = vmc_step_towards(
      vmc_control.roll_comp_offset,
      roll_comp_desired,
      clamp_double(vmc_control.roll_comp_ramp_rate, 0.0, 1.0) * vmc_loop_dt());
  vmc_control.roll_comp_offset = (float)roll_comp_offset;

  left_target_length =
      clamp_double(ramped_target_length + roll_comp_offset,
                   VMC_LEG_LENGTH_MIN,
                   VMC_LEG_LENGTH_MAX);
  right_target_length =
      clamp_double(ramped_target_length - roll_comp_offset,
                   VMC_LEG_LENGTH_MIN,
                   VMC_LEG_LENGTH_MAX);

  /* Shanhai substitutes a 0.30 m air target on an unloaded side.  Keep this
   * inside normal drive only; jump, stair and get-up own their targets. */
  {
    const double normal_left_target_length = left_target_length;
    const double normal_right_target_length = right_target_length;
    const uint8_t airborne_control_allowed =
        (vmc_control.manual_prepose_active == 0U &&
         vmc_control.manual_prepose_exit_length_check_active == 0U &&
         vmc_getup_is_active() == 0U &&
         vmc_jump_is_active_or_aborted() == 0U &&
         vmc_stair_is_active() == 0U &&
         stair_override == 0U &&
         vmc_control.stair_length_override_active == 0U)
            ? 1U
            : 0U;
    left_airborne_posture =
        (airborne_control_allowed != 0U &&
         vmc_probe_state.airborne_side_posture_active[VMC_AIRBORNE_SIDE_LEFT] !=
             0U)
            ? 1U
            : 0U;
    right_airborne_posture =
        (airborne_control_allowed != 0U &&
         vmc_probe_state.airborne_side_posture_active
                 [VMC_AIRBORNE_SIDE_RIGHT] != 0U)
            ? 1U
            : 0U;
    left_airborne_tp =
        -(vmc_balance.left_tp_theta_term +
          vmc_balance.left_tp_dtheta_term);
    right_airborne_tp =
        -(vmc_balance.right_tp_theta_term +
          vmc_balance.right_tp_dtheta_term);
    const double airborne_length_step =
        clamp_double(VMC_AIRBORNE_LENGTH_RAMP_RATE_DEFAULT, 0.0, 2.0) *
        vmc_loop_dt();
    if (left_airborne_posture != 0U) {
      left_target_length =
          vmc_step_towards(vmc_control.left_target_length,
                           VMC_AIRBORNE_LEG_LENGTH_TARGET_DEFAULT,
                           airborne_length_step);
    } else if (vmc_probe_state.airborne_side_length_returning
                   [VMC_AIRBORNE_SIDE_LEFT] != 0U) {
      left_target_length =
          vmc_step_towards(vmc_control.left_target_length,
                           left_target_length,
                           airborne_length_step);
      if (fabs(left_target_length - normal_left_target_length) <= 1.0e-4) {
        vmc_probe_state.airborne_side_length_returning
            [VMC_AIRBORNE_SIDE_LEFT] = 0U;
      }
    }
    if (right_airborne_posture != 0U) {
      right_target_length =
          vmc_step_towards(vmc_control.right_target_length,
                           VMC_AIRBORNE_LEG_LENGTH_TARGET_DEFAULT,
                           airborne_length_step);
    } else if (vmc_probe_state.airborne_side_length_returning
                   [VMC_AIRBORNE_SIDE_RIGHT] != 0U) {
      right_target_length =
          vmc_step_towards(vmc_control.right_target_length,
                           right_target_length,
                           airborne_length_step);
      if (fabs(right_target_length - normal_right_target_length) <= 1.0e-4) {
        vmc_probe_state.airborne_side_length_returning
            [VMC_AIRBORNE_SIDE_RIGHT] = 0U;
      }
    }
  }

  if (vmc_control.manual_prepose_active != 0U) {
    left_target_length =
        (double)vmc_control.manual_prepose_length_left;
    right_target_length =
        (double)vmc_control.manual_prepose_length_right;
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER) {
    /* Keep each leg at its own captured length.  A completed leg must not be
     * passive: with zero force it can extend under the spring/gravity while
     * the other leg is still being turned through its dead point. */
    left_target_length = (double)vmc_control.getup_left_hold_length;
    right_target_length = (double)vmc_control.getup_right_hold_length;
  }

  left_target_length = clamp_double(left_target_length,
                                    VMC_LEG_LENGTH_MIN,
                                    VMC_LEG_LENGTH_MAX);
  right_target_length = clamp_double(right_target_length,
                                     VMC_LEG_LENGTH_MIN,
                                     VMC_LEG_LENGTH_MAX);
  vmc_control.left_target_length = (float)left_target_length;
  vmc_control.right_target_length = (float)right_target_length;

  jump_force_profile =
      (vmc_control.jump_stage == VMC_JUMP_STAGE_EXTEND ||
       vmc_control.jump_stage == VMC_JUMP_STAGE_RETRACT)
          ? 1U
          : 0U;
  stair_force_profile =
      (stair_override != 0U)
          ? 1U
          : 0U;
  if (getup_force_profile != 0U) {
    kp_length = clamp_double(VMC_GETUP_LENGTH_KP, 0.0, 5000.0);
    kd_length = clamp_double(VMC_GETUP_LENGTH_KD, 0.0, 1000.0);
    max_force =
        clamp_double(VMC_GETUP_FORCE_MAX, 0.0, VMC_LEG_FORCE_MAX_DEFAULT);
  } else if (jump_force_profile != 0U) {
    kp_length = clamp_double(vmc_control.jump_kp_length, 0.0, 5000.0);
    kd_length = clamp_double(vmc_control.jump_kd_length, 0.0, 1000.0);
    max_force = clamp_double(vmc_control.jump_max_force,
                             0.0,
                             VMC_LEG_FORCE_MAX_DEFAULT);
  } else if (stair_force_profile != 0U) {
    kp_length = clamp_double(vmc_control.stair_kp_length, 0.0, 5000.0);
    kd_length = clamp_double(vmc_control.stair_kd_length, 0.0, 1000.0);
    max_force = clamp_double(vmc_control.stair_max_force,
                             0.0,
                             VMC_LEG_FORCE_MAX_DEFAULT);
  } else {
    kp_length = clamp_double(vmc_control.kp_length, 0.0, 5000.0);
    kd_length = clamp_double(vmc_control.kd_length, 0.0, 1000.0);
    max_force = clamp_double(vmc_control.max_force,
                             0.0,
                             VMC_LEG_FORCE_MAX_DEFAULT);
  }
  left_target_phi0 = vmc_control.target_phi0;
  right_target_phi0 = vmc_control.target_phi0;
  if (vmc_control.manual_prepose_active != 0U ||
      vmc_control.manual_prepose_exit_length_check_active != 0U) {
    left_target_phi0 = vmc_control.manual_prepose_phi0_left;
    right_target_phi0 = vmc_control.manual_prepose_phi0_right;
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_BRACE) {
    left_target_phi0 = vmc_probe_state.left_leg.phi0;
    right_target_phi0 = vmc_probe_state.right_leg.phi0;
  } else if (vmc_control.getup_state ==
             VMC_GETUP_STATE_BODY_FAN_SYNC) {
    if (vmc_control.getup_body_fan_sync_prelift_active != 0U) {
      /* Hold both joints while the length loop extends the folded leg and
       * unloads its dead point.  Theta synchronization starts afterwards. */
      left_target_phi0 = vmc_probe_state.left_leg.phi0;
      right_target_phi0 = vmc_probe_state.right_leg.phi0;
    } else {
      const double speed_limit = clamp_double(
          VMC_GETUP_BODY_FAN_SYNC_RATE_RADPS, 0.0, 5.0);
      const double phase_kp =
          clamp_double(VMC_GETUP_BODY_FAN_SYNC_PHASE_KP, 0.0, 20.0);

      /* Torque-level synchronization: hold one leg's absolute phi0 and
       * derive the other leg's velocity from the remaining phase error. */
      left_target_phi0 = vmc_probe_state.left_leg.phi0;
      right_target_phi0 = vmc_probe_state.right_leg.phi0;
      if (vmc_control.getup_body_fan_sync_leader_side == 1U) {
        const double follower_delta = vmc_angle_forward_to(
            vmc_probe_state.left_leg.phi0,
            vmc_probe_state.right_leg.phi0,
            vmc_control.getup_turn_direction) -
            vmc_probe_state.right_leg.phi0;

        right_phi0_speed_target =
            clamp_double(phase_kp * follower_delta,
                         -speed_limit,
                         speed_limit);
      } else if (vmc_control.getup_body_fan_sync_leader_side == 2U) {
        const double follower_delta = vmc_angle_forward_to(
            vmc_probe_state.right_leg.phi0,
            vmc_probe_state.left_leg.phi0,
            vmc_control.getup_turn_direction) -
            vmc_probe_state.left_leg.phi0;

        left_phi0_speed_target =
            clamp_double(phase_kp * follower_delta,
                         -speed_limit,
                         speed_limit);
      } else {
        const double left_target =
            (double)VMC_PHI0_VERTICAL_RAD +
            (double)vmc_probe_state.body.pitch +
            (double)vmc_control.getup_body_fan_sync_theta_left;
        const double right_target =
            (double)VMC_PHI0_VERTICAL_RAD +
            (double)vmc_probe_state.body.pitch +
            (double)vmc_control.getup_body_fan_sync_theta_right;
        const double left_delta =
            vmc_angle_forward_to(
                left_target, vmc_probe_state.left_leg.phi0,
                vmc_control.getup_turn_direction) -
            vmc_probe_state.left_leg.phi0;
        const double right_delta =
            vmc_angle_forward_to(
                right_target, vmc_probe_state.right_leg.phi0,
                vmc_control.getup_turn_direction) -
            vmc_probe_state.right_leg.phi0;

        left_phi0_speed_target =
            clamp_double(phase_kp * left_delta,
                         -speed_limit,
                         speed_limit);
        right_phi0_speed_target =
            clamp_double(phase_kp * right_delta,
                         -speed_limit,
                         speed_limit);
      }
    }
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN) {
    /* BODY_FAN owns Tp directly.  Keep the position target at the measured
     * angle and disable the position PD below so the speed loop does not get
     * re-shaped by kp/kd. */
    left_target_phi0 = vmc_probe_state.left_leg.phi0;
    right_target_phi0 = vmc_probe_state.right_leg.phi0;
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_BODY_ALIGN ||
             vmc_control.getup_state == VMC_GETUP_STATE_BODY_FLIP ||
             vmc_control.getup_state == VMC_GETUP_STATE_TURN) {
    const double phi0_lead =
        (vmc_control.getup_state == VMC_GETUP_STATE_TURN)
            ? VMC_GETUP_PHI0_LEAD_RAD
            : VMC_GETUP_FAN_PHI0_LEAD_RAD;
    /* TURN and BODY_FLIP intentionally set both directions to the same value.
     * BODY_ALIGN sets them independently. */
    left_target_phi0 = vmc_probe_state.left_leg.phi0 +
                       (double)vmc_control.getup_left_direction *
                           phi0_lead;
    right_target_phi0 = vmc_probe_state.right_leg.phi0 +
                        (double)vmc_control.getup_right_direction *
                            phi0_lead;
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER) {
    if (vmc_control.getup_left_done != 0U) {
      const double left_theta_error = vmc_rad_format((float)(
          vmc_control.getup_left_hold_theta -
          vmc_probe_state.motion.left_theta));
      left_target_phi0 = vmc_probe_state.left_leg.phi0 + left_theta_error;
    } else {
      left_target_phi0 = vmc_probe_state.left_leg.phi0 +
                         (double)vmc_control.getup_left_direction *
                             VMC_GETUP_FAN_PHI0_LEAD_RAD;
    }
    if (vmc_control.getup_right_done != 0U) {
      const double right_theta_error = vmc_rad_format((float)(
          vmc_control.getup_right_hold_theta -
          vmc_probe_state.motion.right_theta));
      right_target_phi0 = vmc_probe_state.right_leg.phi0 + right_theta_error;
    } else {
      right_target_phi0 = vmc_probe_state.right_leg.phi0 +
                          (double)vmc_control.getup_right_direction *
                              VMC_GETUP_FAN_PHI0_LEAD_RAD;
    }
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_SHRINK) {
    const double left_theta_error = vmc_rad_format((float)(
        vmc_control.getup_shrink_target_theta_left -
        vmc_probe_state.motion.left_theta));
    const double right_theta_error = vmc_rad_format((float)(
        vmc_control.getup_shrink_target_theta_right -
        vmc_probe_state.motion.right_theta));
    left_target_phi0 = vmc_probe_state.left_leg.phi0 + left_theta_error;
    right_target_phi0 = vmc_probe_state.right_leg.phi0 + right_theta_error;
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_FOLDED_LOWER) {
    const double target_absolute =
        (double)VMC_PHI0_VERTICAL_RAD +
        (double)vmc_probe_state.body.pitch;

    left_target_phi0 = vmc_angle_nearest_to(
        target_absolute, vmc_probe_state.left_leg.phi0);
    right_target_phi0 = vmc_angle_nearest_to(
        target_absolute, vmc_probe_state.right_leg.phi0);
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_UPRIGHT) {
    const double left_raw_target = vmc_angle_nearest_to(
        (double)VMC_PHI0_VERTICAL_RAD + (double)vmc_probe_state.body.pitch,
        vmc_probe_state.left_leg.phi0);
    const double right_raw_target = vmc_angle_nearest_to(
        (double)VMC_PHI0_VERTICAL_RAD + (double)vmc_probe_state.body.pitch,
        vmc_probe_state.right_leg.phi0);
    const double upright_step =
        clamp_double(VMC_GETUP_UPRIGHT_PHI0_RATE_RADPS, 0.0, 5.0) *
        vmc_loop_dt();

    vmc_control.getup_upright_phi0_left = (float)vmc_step_towards(
        vmc_control.getup_upright_phi0_left,
        left_raw_target,
        upright_step);
    vmc_control.getup_upright_phi0_right = (float)vmc_step_towards(
        vmc_control.getup_upright_phi0_right,
        right_raw_target,
        upright_step);
    left_target_phi0 = vmc_control.getup_upright_phi0_left;
    right_target_phi0 = vmc_control.getup_upright_phi0_right;
  }
  if (vmc_control.manual_prepose_active == 0U &&
      vmc_stair_is_active() != 0U) {
    const double stair_phi0_ramp_rate =
        (vmc_control.stair_stage == VMC_STAIR_STAGE_LIFT)
            ? clamp_double(vmc_control.stair_lift_phi0_ramp_rate, 0.0, 20.0)
            : clamp_double(vmc_control.stair_phi0_ramp_rate, 0.0, 20.0);
    const double stair_phi0_step =
        stair_phi0_ramp_rate * vmc_loop_dt();
    /* Match Shanhai's body-relative theta poses: first lift to 90 degrees,
     * then retract to the 60 degree climbing pose.  Convert the desired
     * world-frame theta back to the absolute phi0 command every cycle. */
    const double stair_phi0_command = vmc_angle_nearest_to(
        (double)VMC_PHI0_VERTICAL_RAD +
            (double)vmc_probe_state.body.pitch +
            ((vmc_control.stair_stage == VMC_STAIR_STAGE_LIFT)
                 ? (double)vmc_control.stair_lift_theta
                 : (double)vmc_control.stair_retract_theta),
        (vmc_control.stair_phi0_ramped));
    vmc_control.stair_phi0_ramped = (float)vmc_step_towards(
        vmc_control.stair_phi0_ramped,
        stair_phi0_command,
        stair_phi0_step);
    left_target_phi0 = vmc_control.stair_phi0_ramped;
    right_target_phi0 = vmc_control.stair_phi0_ramped;
  }
  left_phi0_feedforward = 0.0;
  right_phi0_feedforward = 0.0;
  if (vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN) {
    const double fan_speed_target =
        (double)vmc_control.getup_turn_direction *
        (double)VMC_GETUP_BODY_FAN_SPEED_RADPS;

    left_phi0_feedforward = vmc_body_fan_speed_torque(
        fan_speed_target,
        vmc_probe_state.left_leg.d_phi0,
        vmc_probe_state.left_leg.length,
        vmc_probe_state.left_leg.phi0,
        VMC_GETUP_BODY_FAN_SPEED_KP,
        VMC_GETUP_BODY_FAN_SPEED_TORQUE_MAX);
    right_phi0_feedforward = vmc_body_fan_speed_torque(
        fan_speed_target,
        vmc_probe_state.right_leg.d_phi0,
        vmc_probe_state.right_leg.length,
        vmc_probe_state.right_leg.phi0,
        VMC_GETUP_BODY_FAN_SPEED_KP,
        VMC_GETUP_BODY_FAN_SPEED_TORQUE_MAX);
  } else if (getup_sync_torque_profile != 0U) {
    const double sync_speed_limit = clamp_double(
        VMC_GETUP_BODY_FAN_SYNC_RATE_RADPS, 0.0, 5.0);

    left_phi0_feedforward = vmc_body_fan_speed_torque(
        left_phi0_speed_target,
        vmc_probe_state.left_leg.d_phi0,
        vmc_probe_state.left_leg.length,
        vmc_probe_state.left_leg.phi0,
        VMC_GETUP_BODY_FAN_SPEED_KP,
        VMC_GETUP_BODY_FAN_SPEED_TORQUE_MAX) +
        vmc_getup_body_fan_sync_torque_feedforward(
            left_phi0_speed_target,
            vmc_probe_state.left_leg.d_phi0,
            sync_speed_limit);
    right_phi0_feedforward = vmc_body_fan_speed_torque(
        right_phi0_speed_target,
        vmc_probe_state.right_leg.d_phi0,
        vmc_probe_state.right_leg.length,
        vmc_probe_state.right_leg.phi0,
        VMC_GETUP_BODY_FAN_SPEED_KP,
        VMC_GETUP_BODY_FAN_SPEED_TORQUE_MAX) +
        vmc_getup_body_fan_sync_torque_feedforward(
            right_phi0_speed_target,
            vmc_probe_state.right_leg.d_phi0,
            sync_speed_limit);
  }
  if (vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN ||
      getup_sync_torque_profile != 0U) {
    const uint8_t body_fan =
        (vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN) ? 1U : 0U;
    const double rise_rate =
        (body_fan != 0U)
            ? (double)VMC_GETUP_BODY_FAN_TORQUE_FF_RISE_RATE
            : (double)VMC_GETUP_BODY_FAN_SYNC_TORQUE_FF_RISE_RATE;
    const double fall_rate =
        (body_fan != 0U)
            ? (double)VMC_GETUP_BODY_FAN_TORQUE_FF_FALL_RATE
            : (double)VMC_GETUP_BODY_FAN_SYNC_TORQUE_FF_FALL_RATE;

    left_phi0_feedforward = vmc_getup_phi0_feedforward_slew(
        (double)vmc_control.getup_phi0_feedforward_left,
        left_phi0_feedforward,
        rise_rate,
        fall_rate);
    right_phi0_feedforward = vmc_getup_phi0_feedforward_slew(
        (double)vmc_control.getup_phi0_feedforward_right,
        right_phi0_feedforward,
        rise_rate,
        fall_rate);
    vmc_control.getup_phi0_feedforward_left =
        (float)left_phi0_feedforward;
    vmc_control.getup_phi0_feedforward_right =
        (float)right_phi0_feedforward;
  } else {
    vmc_control.getup_phi0_feedforward_left = 0.0f;
    vmc_control.getup_phi0_feedforward_right = 0.0f;
  }
  vmc_control.getup_target_phi0_left = (float)left_target_phi0;
  vmc_control.getup_target_phi0_right = (float)right_target_phi0;
  max_phi0_torque = clamp_double(vmc_control.max_phi0_torque,
                                 0.0,
                                 VMC_LEG_TP_MAX);
  kp_phi0 = clamp_double(vmc_control.kp_phi0, 0.0, 500.0);
  kd_phi0 = clamp_double(vmc_control.kd_phi0, 0.0, 50.0);
  if (getup_force_profile != 0U) {
    if (vmc_control.getup_partial_active != 0U &&
        (vmc_control.getup_state == VMC_GETUP_STATE_FOLDED_LOWER ||
         vmc_control.getup_state == VMC_GETUP_STATE_UPRIGHT)) {
      max_phi0_torque =
          clamp_double(VMC_GETUP_PHI0_TORQUE_MAX, 0.0, VMC_LEG_TP_MAX);
      kp_phi0 = clamp_double(VMC_GETUP_PARTIAL_PHI0_KP, 0.0, 500.0);
      kd_phi0 = clamp_double(VMC_GETUP_PHI0_KD, 0.0, 50.0);
    } else if (vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN) {
      max_phi0_torque = clamp_double(
          VMC_GETUP_BODY_FAN_TORQUE_MAX, 0.0, VMC_LEG_TP_MAX);
      kp_phi0 = clamp_double(VMC_GETUP_BODY_FAN_PHI0_KP, 0.0, 500.0);
      kd_phi0 = clamp_double(VMC_GETUP_BODY_FAN_PHI0_KD, 0.0, 50.0);
    } else if (getup_sync_torque_profile != 0U) {
      max_phi0_torque = clamp_double(
          VMC_GETUP_BODY_FAN_SYNC_TORQUE_MAX, 0.0, VMC_LEG_TP_MAX);
      kp_phi0 = 0.0;
      kd_phi0 = 0.0;
    } else {
      max_phi0_torque =
          clamp_double(VMC_GETUP_PHI0_TORQUE_MAX, 0.0, VMC_LEG_TP_MAX);
      kp_phi0 = clamp_double(
          (getup_fan_profile != 0U) ? VMC_GETUP_FAN_PHI0_KP
                                    : VMC_GETUP_PHI0_KP,
          0.0,
          500.0);
      kd_phi0 = clamp_double(
          (getup_fan_profile != 0U) ? VMC_GETUP_FAN_PHI0_KD
                                    : VMC_GETUP_PHI0_KD,
          0.0,
          50.0);
    }
  } else if (vmc_control.manual_prepose_active == 0U &&
             vmc_stair_is_active() != 0U) {
    kp_phi0 = clamp_double(vmc_control.stair_phi0_kp, 0.0, 500.0);
    kd_phi0 = clamp_double(vmc_control.stair_phi0_kd, 0.0, 50.0);
  }
  left_max_phi0_torque = max_phi0_torque;
  right_max_phi0_torque = max_phi0_torque;
  if (getup_force_profile != 0U) {
    support_force = clamp_double(VMC_GETUP_SUPPORT_FORCE,
                                 -VMC_LEG_SUPPORT_FORCE_MAX,
                                 VMC_LEG_SUPPORT_FORCE_MAX);
  } else if (jump_force_profile != 0U) {
    support_force = clamp_double(vmc_control.jump_support_force,
                                 -VMC_LEG_SUPPORT_FORCE_MAX,
                                 VMC_LEG_SUPPORT_FORCE_MAX);
  } else if (stair_force_profile != 0U) {
    const double stair_support =
        (vmc_control.stair_stage == VMC_STAIR_STAGE_LIFT)
            ? (double)vmc_control.stair_lift_support_force
            : ((vmc_control.stair_stage == VMC_STAIR_STAGE_RETRACT ||
                vmc_control.stair_stage == VMC_STAIR_STAGE_SETTLE)
                   ? (double)vmc_control.stair_retract_support_force
                   : 0.0);
    support_force = clamp_double(stair_support,
                                 -VMC_LEG_SUPPORT_FORCE_MAX,
                                 VMC_LEG_SUPPORT_FORCE_MAX);
  } else {
    support_force = clamp_double(vmc_control.support_force,
                                 -VMC_LEG_SUPPORT_FORCE_MAX,
                                 VMC_LEG_SUPPORT_FORCE_MAX);
  }
  left_max_force = max_force;
  right_max_force = max_force;
  left_support_force = support_force;
  right_support_force = support_force;
  if (vmc_control.getup_state == VMC_GETUP_STATE_BODY_ALIGN ||
      vmc_control.getup_state == VMC_GETUP_STATE_BODY_FLIP ||
      vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN) {
    left_support_force = VMC_GETUP_FAN_SUPPORT_FORCE;
    right_support_force = VMC_GETUP_FAN_SUPPORT_FORCE;
  } else if (vmc_control.getup_state ==
             VMC_GETUP_STATE_BODY_FAN_SYNC) {
    left_support_force = VMC_GETUP_BODY_FAN_SYNC_SUPPORT_FORCE;
    right_support_force = VMC_GETUP_BODY_FAN_SYNC_SUPPORT_FORCE;
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_SHRINK) {
    /* The brace-level support feedforward points toward extension.  With the
     * 300 N/m get-up length gain, 30 N cancels retraction for every remaining
     * error below 0.1 m, leaving direct-shrink starts short of the target. */
    left_support_force = 0.0;
    right_support_force = 0.0;
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER) {
    left_support_force =
        (vmc_control.getup_left_done == 0U) ? VMC_GETUP_FAN_SUPPORT_FORCE : 0.0;
    right_support_force =
        (vmc_control.getup_right_done == 0U) ? VMC_GETUP_FAN_SUPPORT_FORCE : 0.0;
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_FOLDED_LOWER) {
    left_support_force = 0.0;
    right_support_force = 0.0;
  }
  if (vmc_control.getup_state == VMC_GETUP_STATE_BODY_ALIGN) {
    if (vmc_control.getup_left_direction == 0) {
      left_max_phi0_torque = 0.0;
    }
    if (vmc_control.getup_right_direction == 0) {
      right_max_phi0_torque = 0.0;
    }
  }

  update_leg_length_control(&vmc_probe_state.left_leg,
                            left_target_length,
                            kp_length,
                            kd_length,
                            left_max_force,
                            left_target_phi0,
                            kp_phi0,
                            kd_phi0,
                            left_max_phi0_torque,
                            left_support_force,
                            -vmc_balance.leg_sync_torque,
                            VMC_DIR_LEFT_AHEAD,
                            VMC_DIR_LEFT_BACK,
                            left_phi0_feedforward,
                            left_airborne_posture,
                            left_airborne_tp);
  update_leg_length_control(&vmc_probe_state.right_leg,
                            right_target_length,
                            kp_length,
                            kd_length,
                            right_max_force,
                            right_target_phi0,
                            kp_phi0,
                            kd_phi0,
                            right_max_phi0_torque,
                            right_support_force,
                            vmc_balance.leg_sync_torque,
                            VMC_DIR_RIGHT_AHEAD,
                            VMC_DIR_RIGHT_BACK,
                            right_phi0_feedforward,
                            right_airborne_posture,
                            right_airborne_tp);
}

void vmc_wheel_control_update(void)
{
  HAL_StatusTypeDef status;
  int16_t max_current = vmc_wheel_control.max_current;
  int16_t left_current = 0;
  int16_t right_current = 0;
  int16_t left_send_current = 0;
  int16_t right_send_current = 0;
  double release_scale = vmc_stair_release_scale();

  if (vmc_control.manual_prepose_active != 0U) {
    release_scale = 1.0;
  }

  if (max_current < 0) {
    max_current = (int16_t)-max_current;
  }
  if (max_current > VMC_WHEEL_CURRENT_MAX_DEFAULT) {
    max_current = VMC_WHEEL_CURRENT_MAX_DEFAULT;
  }

  /* Gating is per side.  airborne_active is the whole-machine latch and kills
   * both wheels; the per-side latches each kill only their own wheel, so a
   * single lifted leg stops spinning while the other keeps the machine
   * balanced. */
  {
    const uint8_t manual_output_ready =
        (vmc_wheel_control.enabled != 0U &&
         vmc_balance.wheel_output_enable != 0U &&
         vmc_balance.wheel_output_user_enable != 0U &&
         vmc_manual_prepose_allows_wheel_output() != 0U)
            ? 1U
            : 0U;
    const uint8_t automatic_output_ready =
        (vmc_wheel_control.enabled != 0U &&
         vmc_control.manual_prepose_active == 0U &&
         vmc_balance.wheel_output_enable != 0U &&
         vmc_balance.wheel_output_user_enable != 0U &&
         vmc_probe_state.airborne_active == 0U &&
         vmc_probe_state.feedback_timeout_active == 0U &&
         vmc_jump_blocks_wheel_output() == 0U &&
         vmc_stair_blocks_wheel_output() == 0U &&
         remote_control_allows_joint_enable() != 0U)
            ? 1U
            : 0U;
    const uint8_t output_ready =
        (manual_output_ready != 0U || automatic_output_ready != 0U) ? 1U : 0U;

    if (output_ready != 0U &&
        (manual_output_ready != 0U ||
         vmc_probe_state.airborne_side_active[VMC_AIRBORNE_SIDE_LEFT] == 0U)) {
      left_current = clamp_i16(vmc_wheel_control.left_current,
                               (int16_t)-max_current,
                               max_current);
    }
    if (output_ready != 0U &&
        (manual_output_ready != 0U ||
         vmc_probe_state.airborne_side_active[VMC_AIRBORNE_SIDE_RIGHT] == 0U)) {
      right_current = clamp_i16(vmc_wheel_control.right_current,
                                (int16_t)-max_current,
                                max_current);
    }
  }

  left_send_current = (int16_t)(
      (double)VMC_WHEEL_LEFT_CMD_DIRECTION * (double)left_current *
      release_scale);
  right_send_current = (int16_t)(
      (double)VMC_WHEEL_RIGHT_CMD_DIRECTION * (double)right_current *
      release_scale);

  vmc_wheel_control.left_current_output = left_send_current;
  vmc_wheel_control.right_current_output = right_send_current;

  status = can_send_wheel_current(left_send_current, right_send_current);
  if (status == HAL_OK) {
    vmc_probe_state.wheel_tx_count++;
  } else {
    vmc_probe_state.wheel_tx_fail_count++;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK &&
        header.IDE == CAN_ID_STD && header.DLC == 8U) {
      parse_can_frame(hcan, header.StdId, data);
    }
  }
}

/* Microsecond timebase built from the HAL tick plus TIM1's sub-millisecond
 * counter.  The tick is only re-read to confirm the millisecond did not roll
 * over between the two reads; if it did, the pair is inconsistent and is
 * retried.  Without that check the function could pair a stale tick with a
 * fresh counter and under-report by a whole millisecond. */
static uint32_t vmc_loop_now_us(void)
{
  uint32_t tick;
  uint32_t counter;
  uint32_t tick_after;

  do {
    tick = HAL_GetTick();
    counter = TIM1->CNT;
    tick_after = HAL_GetTick();
  } while (tick != tick_after);

  return tick * 1000U + counter;
}

static void vmc_loop_timing_init(void)
{
  vmc_loop_us_last = vmc_loop_now_us();
  vmc_loop_dt_s = VMC_CONTROL_DT;
  vmc_probe_state.loop_dt_s = vmc_loop_dt_s;
  vmc_probe_state.loop_dt_us = (uint32_t)(VMC_CONTROL_DT * 1.0e6f);
  vmc_probe_state.loop_count = 0U;
}

/* Measured control period in seconds.  Every time-based term must read this
 * rather than the VMC_CONTROL_DT fallback. */
static double vmc_loop_dt(void)
{
  return (double)vmc_loop_dt_s;
}

/* Call once per control cycle, before anything that needs dt. */
static void vmc_loop_timing_update(void)
{
  const uint32_t now_us = vmc_loop_now_us();
  const uint32_t delta_us = now_us - vmc_loop_us_last;
  float dt = (float)delta_us * 1.0e-6f;

  vmc_loop_us_last = now_us;
  if (dt < VMC_LOOP_DT_MIN) {
    dt = VMC_LOOP_DT_MIN;
  } else if (dt > VMC_LOOP_DT_MAX) {
    dt = VMC_LOOP_DT_MAX;
  }
  vmc_loop_dt_s = dt;
  vmc_probe_state.loop_dt_s = vmc_loop_dt_s;
  vmc_probe_state.loop_dt_us = (uint32_t)(vmc_loop_dt_s * 1.0e6f + 0.5f);
  vmc_probe_state.loop_count++;
}

/* Wait out the rest of the fixed control period.
 *
 * Uses an absolute deadline so the period does not accumulate the RTOS tick's
 * quantisation error.  A whole tick is slept only when it comfortably fits in
 * the remaining time; the sub-millisecond remainder is spent polling the
 * 1 MHz TIM1 counter, which resolves to about a microsecond. */
static void vmc_loop_wait_for_period(void)
{
  for (;;) {
    const uint32_t now_us = vmc_loop_now_us();
    const int32_t remaining_us = (int32_t)(vmc_loop_deadline_us - now_us);

    if (remaining_us <= 0) {
      break;
    }
    if (remaining_us >= VMC_CONTROL_SLEEP_MIN_US) {
      osDelay(1);
    }
    /* Otherwise the remainder is shorter than one tick: keep polling. */
  }

  vmc_loop_deadline_us += VMC_CONTROL_PERIOD_US;
  /* If the body overran by a whole period, resynchronise instead of trying to
   * catch up.  A burst of back-to-back cycles would disturb the machine far
   * more than one long period, and dt is measured, so the controller already
   * sees the truth. */
  if ((int32_t)(vmc_loop_now_us() - vmc_loop_deadline_us) > 0) {
    vmc_loop_deadline_us = vmc_loop_now_us() + VMC_CONTROL_PERIOD_US;
  }
}

void vmc_probe_task(void const *argument)
{
  (void)argument;

  memset((void *)&vmc_probe_state, 0, sizeof(vmc_probe_state));
  sbus_remote_init();
  remote_control_init();
  hi12_imu_init();
  vmc_lqr_rtt_init();
  vmc_can_init();
  board_comm_init();
  vmc_control_init();
  vmc_wheel_control_init();
  vmc_balance_init();
  vmc_loop_timing_init();
  vmc_loop_deadline_us = vmc_loop_now_us() + VMC_CONTROL_PERIOD_US;

  for (;;) {
    vmc_loop_timing_update();
    hi12_imu_update();
    tf02_pro_update();
    sbus_remote_update();
    remote_control_update_source();
    vmc_update_remote_mode();
    vmc_update_body_state();
    vmc_update_gravity_compensation();
    vmc_update_leg_state();
    vmc_update_motion_state();
    vmc_jump_update();
    vmc_update_airborne_state();
    vmc_stair_update();
    vmc_lqr_rtt_update();
    vmc_update_lqr_gain();
    vmc_balance_update();
    vmc_lqr_rtt_telemetry_update();
    vmc_control_update();
    vmc_dm_state_machine_update();
    vmc_balance_apply_wheel_output();
    vmc_wheel_control_update();
    board_comm_update();
    vmc_loop_wait_for_period();
  }
}
