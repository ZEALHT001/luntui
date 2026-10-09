#include "vmc_probe.h"

#include "board_comm.h"
#include "hi12_imu.h"
#include "remote_control.h"
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
/* A fall detected while the robot was enabled requires an explicit emergency
 * stop followed by S1=3 before control can be enabled again. */
static uint8_t vmc_fall_rearm_required;
static uint8_t vmc_fall_rearm_armed;
static uint8_t vmc_fall_monitor_armed;
static uint8_t vmc_fall_prev_fully_fallen;

static void clear_leg_command(volatile vmc_leg_state_t *leg);
static uint8_t vmc_feedback_timeout_source(void);
static void vmc_update_remote_target_length(void);
static uint8_t vmc_jump_is_active(void);
static uint8_t vmc_jump_is_active_or_aborted(void);
static uint8_t vmc_jump_blocks_wheel_output(void);
static uint8_t vmc_jump_joint_lqr_active(void);
static uint8_t vmc_jump_overrides_target(void);
static double vmc_jump_stage_target_length(void);
static void vmc_jump_update(void);
static uint8_t vmc_getup_is_active(void);
static uint8_t vmc_getup_pose_ready(void);
static void vmc_update_getup_state(void);
static void vmc_update_remote_mode(void);
static void vmc_update_body_state(void);
static void vmc_update_gravity_compensation(void);
static void vmc_update_motion_state(void);
static void vmc_update_airborne_state(void);
static double vmc_loop_dt(void);
static void vmc_update_lqr_gain(void);
static uint8_t vmc_safety_allows_output(void);
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
  if (remote_control.valid == 0U || remote_control.rx_count == 0U ||
      (HAL_GetTick() - remote_control.last_update_ms) > RC_LOST_TIMEOUT_MS) {
    return;
  }

  if (remote_control.s[0] == RC_SW_UP) {
    vmc_control.target_length = VMC_LEG_TARGET_LONG;
  } else if (remote_control.s[0] == RC_SW_MID) {
    vmc_control.target_length = VMC_LEG_TARGET_MID;
  } else if (remote_control.s[0] == RC_SW_DOWN) {
    vmc_control.target_length = VMC_LEG_TARGET_SHORT;
  }
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

/* Load carried by one leg, averaged over its two joints.  Both joints of a
 * lifted leg lose the body weight, so the pair is a much stronger signal than
 * either joint alone. */
static float vmc_side_joint_torque_abs(uint8_t side)
{
  if (side == VMC_AIRBORNE_SIDE_LEFT) {
    return (fabsf(vmc_probe_state.joint_left_ahead.torque) +
            fabsf(vmc_probe_state.joint_left_back.torque)) *
           0.5f;
  }
  return (fabsf(vmc_probe_state.joint_right_ahead.torque) +
          fabsf(vmc_probe_state.joint_right_back.torque)) *
         0.5f;
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

/* True for every stage of a jump, plus the aborted-with-S1-up case.
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
          remote_control.s[1] == RC_SW_UP)
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
 * NOTE the aborted-with-S1-up latch deliberately does NOT appear here, in
 * either mode.  It used to, and the effect was that an aborted jump left the
 * wheels dead until the pilot cycled S1 from up to middle: the balance loop
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
          remote_control.s[1] == RC_SW_UP)
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
      remote_control.s[1] == RC_SW_DOWN ||
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
 * S1 is the third condition and is checked by the caller, because it is also
 * the switch the normal drive logic reads.
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
  const uint8_t s1_mid =
      (remote_control.valid != 0U && remote_control.s[1] == RC_SW_MID) ? 1U : 0U;
  const uint8_t s1_is_up =
      (remote_control.valid != 0U && remote_control.s[1] == RC_SW_UP) ? 1U : 0U;
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
        (cooled_down != 0U && s1_is_up != 0U && vmc_jump_auto_ready() != 0U)
            ? 1U
            : 0U;

    vmc_control.jump_abort_reason = VMC_JUMP_ABORT_NONE;

    /* Require the conditions to hold continuously for a short window: the
     * laser is one echo per frame and the speed estimate can spike on a single
     * sample, so no single frame may launch a jump.
     *
     * jump_armed is kept as the telemetry-visible "about to fire" indication:
     * 1 means S1 is up, the robot is at speed, the laser is in the window and
     * the confirmation has elapsed. */
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

    if (vmc_control.jump_armed != 0U && vmc_jump_start_is_allowed() != 0U) {
      vmc_control.jump_trigger_count++;
      vmc_control.jump_start_ms = now;
      vmc_jump_auto_hold_ms = 0U;
      vmc_jump_enter_stage(VMC_JUMP_STAGE_COMPRESS, now);
    }
    return;
  }

  if (vmc_control.jump_stage == VMC_JUMP_STAGE_FINISHED ||
      vmc_control.jump_stage == VMC_JUMP_STAGE_ABORT) {
    const uint8_t cooled_down =
        ((now - vmc_control.jump_last_finish_ms) >= cooldown_ms) ? 1U : 0U;

    vmc_control.jump_target_length = (float)vmc_jump_stage_target_length();
    if (s1_mid != 0U && cooled_down != 0U) {
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

static uint8_t vmc_getup_is_active(void)
{
  return (vmc_control.getup_state == VMC_GETUP_STATE_TURN ||
          vmc_control.getup_state == VMC_GETUP_STATE_SHRINK ||
          vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER ||
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

static uint8_t vmc_getup_leg_recover_is_required(void)
{
  return (vmc_getup_theta_is_in_bad_zone(vmc_probe_state.motion.theta) != 0U ||
          vmc_getup_theta_is_in_bad_zone(vmc_probe_state.motion.left_theta) != 0U ||
          vmc_getup_theta_is_in_bad_zone(vmc_probe_state.motion.right_theta) != 0U)
             ? 1U
             : 0U;
}

static uint8_t vmc_getup_leg_recover_pose_ready(void)
{
  const double left_error = vmc_rad_format((float)(
      vmc_probe_state.motion.left_theta - VMC_GETUP_LEG_RECOVER_TARGET_RAD));
  const double right_error = vmc_rad_format((float)(
      vmc_probe_state.motion.right_theta - VMC_GETUP_LEG_RECOVER_TARGET_RAD));

  return (fabs(left_error) < VMC_GETUP_LEG_RECOVER_TOL_RAD &&
          fabs(right_error) < VMC_GETUP_LEG_RECOVER_TOL_RAD)
             ? 1U
             : 0U;
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

  vmc_getup_fully_fallen = fully_fallen;

  if (remote_control.s[1] == RC_SW_DOWN) {
    vmc_fall_rearm_required = 0U;
    vmc_fall_rearm_armed = 1U;
    vmc_fall_monitor_armed = 0U;
    vmc_fall_prev_fully_fallen = fully_fallen;
  }

  if (vmc_control.enabled == 0U || joint_allowed == 0U || imu_ready == 0U ||
      vmc_probe_state.feedback_timeout_active != 0U) {
    vmc_control.getup_state = VMC_GETUP_STATE_IDLE;
    vmc_control.getup_turn_direction = 0;
    return;
  }

  /* Arm fall detection only after observing an upright running state.  This
   * deliberately ignores the case where S1=3 is selected while the chassis
   * was already lying down, allowing that explicit restart to perform one
   * get-up sequence. */
  if (fully_fallen == 0U &&
      (remote_control.s[1] == RC_SW_MID || remote_control.s[1] == RC_SW_UP)) {
    vmc_fall_monitor_armed = 1U;
  }

  /* Once re-armed by S1=2, only S1=3 may start the controller again.  The
   * one-shot arm is consumed when starting recovery from a fallen pose. */
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
    vmc_fall_prev_fully_fallen = fully_fallen;
    return;
  }

  vmc_fall_prev_fully_fallen = fully_fallen;

  if ((vmc_control.getup_state == VMC_GETUP_STATE_IDLE ||
       vmc_control.getup_state == VMC_GETUP_STATE_READY) &&
      fully_fallen != 0U) {
    vmc_control.getup_state = VMC_GETUP_STATE_TURN;
    vmc_control.getup_turn_direction = (pitch < 0.0) ? 1 : -1;
    vmc_control.getup_lqr_release_pending = 1U;
  } else if ((vmc_control.getup_state == VMC_GETUP_STATE_IDLE ||
              vmc_control.getup_state == VMC_GETUP_STATE_READY) &&
             fully_fallen == 0U &&
             vmc_getup_leg_recover_is_required() != 0U) {
    /* The chassis is not fully fallen, but the links are behind/above the
     * body and can be pinned against the ground.  Rotate toward the forward
     * recovery angle using the dedicated negative direction. */
    vmc_control.getup_state = VMC_GETUP_STATE_LEG_RECOVER;
    vmc_control.getup_turn_direction = -1;
    vmc_control.getup_lqr_release_pending = 1U;
  } else if ((vmc_control.getup_state == VMC_GETUP_STATE_IDLE ||
              vmc_control.getup_state == VMC_GETUP_STATE_READY) &&
             vmc_control.getup_lqr_release_pending != 0U) {
    /* A recovery was interrupted after the legs had been extended.  Resume
     * the safe length-settle stage before any LQR joint output is released. */
    vmc_control.getup_state = VMC_GETUP_STATE_SHRINK;
    vmc_control.getup_turn_direction = 0;
    vmc_control.getup_shrink_target_theta_left =
        (float)vmc_probe_state.motion.left_theta;
    vmc_control.getup_shrink_target_theta_right =
        (float)vmc_probe_state.motion.right_theta;
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER &&
      fully_fallen != 0U) {
    vmc_control.getup_state = VMC_GETUP_STATE_TURN;
    vmc_control.getup_turn_direction = (pitch < 0.0) ? 1 : -1;
    vmc_control.getup_lqr_release_pending = 1U;
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_TURN &&
      vmc_getup_pose_ready() != 0U) {
    vmc_control.getup_state = VMC_GETUP_STATE_SHRINK;
    vmc_control.getup_shrink_target_theta_left =
        (float)vmc_probe_state.motion.left_theta;
    vmc_control.getup_shrink_target_theta_right =
        (float)vmc_probe_state.motion.right_theta;
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_SHRINK &&
      fabs(vmc_probe_state.left_leg.length - VMC_GETUP_STAND_LENGTH) <
          VMC_GETUP_LENGTH_READY_TOL &&
      fabs(vmc_probe_state.right_leg.length - VMC_GETUP_STAND_LENGTH) <
          VMC_GETUP_LENGTH_READY_TOL) {
    vmc_control.getup_state = VMC_GETUP_STATE_READY;
    vmc_control.getup_lqr_release_pending = 0U;
  }

  if (vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER &&
      vmc_getup_leg_recover_pose_ready() != 0U) {
    /* Never hand the joints directly from the 0.30 m turn pose to LQR.
     * First reuse the normal SHRINK stage and wait for both measured leg
     * lengths to reach the safe stand target. */
    vmc_control.getup_state = VMC_GETUP_STATE_SHRINK;
    vmc_control.getup_turn_direction = 0;
    vmc_control.getup_shrink_target_theta_left =
        VMC_GETUP_LEG_RECOVER_TARGET_RAD;
    vmc_control.getup_shrink_target_theta_right =
        VMC_GETUP_LEG_RECOVER_TARGET_RAD;
  }
}

static void vmc_update_remote_mode(void)
{
  int16_t forward_current;
  double turn_input;
  double speed_target_max;
  double yaw_rate_max;
  const double dt = vmc_loop_dt();
  const uint32_t now = HAL_GetTick();
  uint8_t follow_requested = 0U;

  vmc_wheel_control.manual_left_current = 0;
  vmc_wheel_control.manual_right_current = 0;
  vmc_wheel_control.manual_forward_current = 0;
  vmc_wheel_control.manual_turn_current = 0;

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

  if (remote_control.s[1] == RC_SW_UP ||
      remote_control.s[1] == RC_SW_MID) {
    vmc_control.enabled = 1U;
    /* Both positions drive.  S1 up additionally enables the automatic jump
     * gate (see vmc_jump_auto_ready); S1 mid is normal driving, so moving the
     * switch back to mid is what disarms the next jump. */
    vmc_balance.wheel_output_enable =
        (vmc_getup_fully_fallen == 0U && vmc_getup_is_active() == 0U) ? 1U : 0U;
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

  forward_current = rc_channel_to_current(remote_control.ch[3],
                                          VMC_RC_WHEEL_FWD_CURRENT_MAX);
  turn_input = rc_channel_to_unit((int16_t)-remote_control.ch[2]);
  vmc_wheel_control.manual_forward_current = forward_current;

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
      vmc_balance.yaw_hold_enable != 0U &&
      vmc_balance.wheel_output_enable != 0U &&
      vmc_probe_state.body.imu_valid != 0U &&
      vmc_probe_state.body.imu_connected != 0U) {
    if (vmc_balance.yaw_hold_active == 0U) {
      vmc_balance.yaw_target = vmc_probe_state.body.yaw;
      vmc_balance.yaw_hold_active = 1U;
    }
    yaw_rate_max = clamp_double(vmc_balance.yaw_target_rate_max, 0.0, 10.0);
    vmc_balance.yaw_target =
        (float)((double)vmc_balance.yaw_target +
                turn_input * yaw_rate_max * dt);
  }

  if (vmc_balance.speed_control_enable != 0U) {
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
  /* Gravity-compensated body-X linear acceleration in m/s2. */
  vmc_probe_state.motion.dd_x =
      (double)vmc_probe_state.body.ax_lin * (double)VMC_GRAVITY_ACCEL;

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

/* Detect "the whole machine is off the ground": the wheels are spinning, the
 * legs are carrying almost nothing, and the body is not being swung around.
 * Once confirmed, latch the condition until leg load recovers so the wheels
 * cannot repeatedly restart while still in the air.
 *
 * The test is the AVERAGE leg load, not the strongest leg.  Recording
 * 20260927_173327 settles this: held in the air, the weakest leg reads
 * p50 0.49 N.m (it hangs free) but the strongest reads p50 3.89 N.m, because
 * the leg controller keeps driving the joints while airborne.  Requiring all
 * four legs under 2 N.m is therefore satisfied on only 7 of 275 in-air frames,
 * and the latch chattered - 24 transitions, wheels reaching 4.19 m/s with the
 * wheel output gated for just 7.6% of the air time.  The old mean-based test
 * satisfies 126 of those frames and gates 57% of the air time with no
 * ground-side false trigger, since the average stays above 10 N.m whenever the
 * machine is resting on its legs.
 *
 * The reachable-but-supported case - a wheel spinning on a low-friction
 * surface while the body still rests on its legs - is handled by the slip
 * gate in the body-motion estimator, not here.  The attitude guard is on top:
 * while the body is being carried or thrown by the pilot its pitch leaves the
 * reference the guard captured when the wheel-speed condition first held. */
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
  const float body_pitch = vmc_probe_state.body.pitch;
  const float pitch_rate = fabsf(vmc_probe_state.body.pitch_rate);
  const float wheel_speed_threshold =
      clamp_float(vmc_balance.airborne_wheel_speed_threshold, 0.1f, 30.0f);
  const float joint_torque_threshold =
      clamp_float(vmc_balance.airborne_joint_torque_threshold, 0.1f, 54.0f);
  const float pitch_dev_max =
      clamp_float(vmc_balance.airborne_pitch_dev_max, 0.01f, 3.0f);
  const float pitch_rate_max =
      clamp_float(vmc_balance.airborne_pitch_rate_max, 0.1f, 30.0f);
  uint32_t confirm_ms = vmc_balance.airborne_confirm_ms;
  uint32_t release_ms = vmc_balance.airborne_release_ms;
  uint8_t wheels_fast;
  uint8_t joints_unloaded;
  uint8_t attitude_quiet;
  uint8_t entry_ready;
  const uint8_t detector_ready =
      (vmc_balance.airborne_detect_enable != 0U &&
       vmc_control.enabled != 0U &&
       vmc_balance.wheel_output_enable != 0U &&
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
  if (release_ms > 10000U) {
    release_ms = 10000U;
  }

  vmc_probe_state.airborne_wheel_speed = wheel_speed;
  vmc_probe_state.airborne_joint_torque = joint_torque;
  vmc_probe_state.airborne_joint_torque_min = joint_torque_min;
  vmc_probe_state.airborne_joint_torque_max = joint_torque_max;

  /* Arm the attitude reference the moment the wheels start spinning on
   * unloaded legs and keep it briefly after that stops holding, so the latch
   * still has to happen while the body is where it just was rather than
   * re-arming the reference on the cycle that trips the latch. */
  wheels_fast = (wheel_speed >= wheel_speed_threshold) ? 1U : 0U;
  joints_unloaded = (joint_torque <= joint_torque_threshold) ? 1U : 0U;
  if (wheels_fast != 0U && joints_unloaded != 0U && detector_ready != 0U) {
    if (vmc_probe_state.airborne_pitch_reference_valid == 0U) {
      vmc_probe_state.airborne_pitch_reference = body_pitch;
    }
    vmc_probe_state.airborne_pitch_reference_valid = 1U;
    vmc_probe_state.airborne_pitch_reference_hold_ms =
        VMC_AIRBORNE_PITCH_REF_HOLD_MS_DEFAULT;
  } else if (vmc_probe_state.airborne_pitch_reference_valid != 0U) {
    if (vmc_probe_state.airborne_pitch_reference_hold_ms > dt_ms) {
      vmc_probe_state.airborne_pitch_reference_hold_ms -= dt_ms;
    } else {
      vmc_probe_state.airborne_pitch_reference_valid = 0U;
      vmc_probe_state.airborne_pitch_reference_hold_ms = 0U;
    }
  } else {
    vmc_probe_state.airborne_pitch_reference_hold_ms = 0U;
  }
  attitude_quiet =
      (pitch_rate <= pitch_rate_max &&
       (vmc_probe_state.airborne_pitch_reference_valid == 0U ||
        fabsf(body_pitch - vmc_probe_state.airborne_pitch_reference) <=
            pitch_dev_max))
          ? 1U
          : 0U;
  entry_ready =
      (wheels_fast != 0U && joints_unloaded != 0U && attitude_quiet != 0U)
          ? 1U
          : 0U;

  /* Per-side latch: one leg lifted while the other still carries the body.
   * The whole-machine test above cannot see that case - the average of all
   * four joints stays high - so each side is tested on its own.  Only the
   * lifted side's wheel gets gated; the grounded side must keep balancing. */
  {
    const float side_torque[VMC_AIRBORNE_SIDE_COUNT] = {
        vmc_side_joint_torque_abs(VMC_AIRBORNE_SIDE_LEFT),
        vmc_side_joint_torque_abs(VMC_AIRBORNE_SIDE_RIGHT)};
    const float side_wheel_speed[VMC_AIRBORNE_SIDE_COUNT] = {left_wheel_speed,
                                                             right_wheel_speed};
    const uint32_t side_confirm_ms =
        (vmc_balance.airborne_side_confirm_ms > 10000U)
            ? 10000U
            : vmc_balance.airborne_side_confirm_ms;
    const uint32_t side_release_ms =
        (vmc_balance.airborne_side_release_ms > 10000U)
            ? 10000U
            : vmc_balance.airborne_side_release_ms;
    const float side_release_torque = joint_torque_threshold * 1.5f;
    const uint8_t side_ready =
        (detector_ready != 0U &&
         vmc_balance.airborne_side_enable != 0U &&
         vmc_jump_joint_lqr_active() == 0U)
            ? 1U
            : 0U;
    uint8_t side;

    for (side = 0U; side < VMC_AIRBORNE_SIDE_COUNT; side++) {
      vmc_probe_state.airborne_side_torque[side] = side_torque[side];

      if (side_ready == 0U) {
        vmc_probe_state.airborne_side_active[side] = 0U;
        vmc_probe_state.airborne_side_candidate[side] = 0U;
        vmc_probe_state.airborne_side_candidate_ms[side] = 0U;
        vmc_probe_state.airborne_side_release_candidate_ms[side] = 0U;
        continue;
      }

      if (vmc_probe_state.airborne_side_active[side] != 0U) {
        if (side_torque[side] >= side_release_torque) {
          if (vmc_probe_state.airborne_side_release_candidate_ms[side] <
              side_release_ms) {
            vmc_probe_state.airborne_side_release_candidate_ms[side] += dt_ms;
          }
          if (vmc_probe_state.airborne_side_release_candidate_ms[side] >=
              side_release_ms) {
            vmc_probe_state.airborne_side_active[side] = 0U;
            vmc_probe_state.airborne_side_candidate[side] = 0U;
            vmc_probe_state.airborne_side_candidate_ms[side] = 0U;
            vmc_probe_state.airborne_side_release_candidate_ms[side] = 0U;
          }
        } else {
          vmc_probe_state.airborne_side_release_candidate_ms[side] = 0U;
        }
        continue;
      }

      if (side_wheel_speed[side] >= wheel_speed_threshold &&
          side_torque[side] <= joint_torque_threshold) {
        vmc_probe_state.airborne_side_candidate[side] = 1U;
        if (vmc_probe_state.airborne_side_candidate_ms[side] <
            side_confirm_ms) {
          vmc_probe_state.airborne_side_candidate_ms[side] += dt_ms;
        }
        if (vmc_probe_state.airborne_side_candidate_ms[side] >=
            side_confirm_ms) {
          vmc_probe_state.airborne_side_active[side] = 1U;
          vmc_probe_state.airborne_side_release_candidate_ms[side] = 0U;
        }
      } else {
        vmc_probe_state.airborne_side_candidate[side] = 0U;
        vmc_probe_state.airborne_side_candidate_ms[side] = 0U;
      }
    }
  }

  if (vmc_jump_joint_lqr_active() != 0U) {
    vmc_probe_state.airborne_active = 1U;
    vmc_probe_state.airborne_candidate = 0U;
    vmc_probe_state.airborne_candidate_ms = 0U;
    vmc_probe_state.airborne_release_candidate_ms = 0U;
    vmc_probe_state.airborne_pitch_reference_valid = 0U;
    return;
  }

  if (detector_ready == 0U) {
    vmc_probe_state.airborne_active = 0U;
    vmc_probe_state.airborne_candidate = 0U;
    vmc_probe_state.airborne_candidate_ms = 0U;
    vmc_probe_state.airborne_release_candidate_ms = 0U;
    vmc_probe_state.airborne_pitch_reference_valid = 0U;
    return;
  }

  if (vmc_probe_state.airborne_active != 0U) {
    const float release_torque_threshold = joint_torque_threshold * 1.5f;
    if (joint_torque >= release_torque_threshold) {
      if (vmc_probe_state.airborne_release_candidate_ms < release_ms) {
        vmc_probe_state.airborne_release_candidate_ms += dt_ms;
      }
      if (vmc_probe_state.airborne_release_candidate_ms >= release_ms) {
        vmc_probe_state.airborne_active = 0U;
        vmc_probe_state.airborne_candidate = 0U;
        vmc_probe_state.airborne_candidate_ms = 0U;
        vmc_probe_state.airborne_release_candidate_ms = 0U;
        vmc_probe_state.airborne_pitch_reference_valid = 0U;
      }
    } else {
      vmc_probe_state.airborne_release_candidate_ms = 0U;
    }
    return;
  }

  if (entry_ready != 0U) {
    vmc_probe_state.airborne_candidate = 1U;
    if (vmc_probe_state.airborne_candidate_ms < confirm_ms) {
      vmc_probe_state.airborne_candidate_ms += dt_ms;
    }
    if (vmc_probe_state.airborne_candidate_ms >= confirm_ms) {
      vmc_probe_state.airborne_active = 1U;
      vmc_probe_state.airborne_release_candidate_ms = 0U;
    }
  } else {
    vmc_probe_state.airborne_candidate = 0U;
    vmc_probe_state.airborne_candidate_ms = 0U;
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
  vmc_balance.pitch_offset = VMC_BALANCE_PITCH_OFFSET_DEFAULT;
  vmc_balance.phi_offset = VMC_BALANCE_PHI_OFFSET_DEFAULT;
  vmc_balance.theta_deadband = VMC_BALANCE_THETA_DEADBAND_DEFAULT;
  vmc_balance.phi_deadband = VMC_BALANCE_PHI_DEADBAND_DEFAULT;
  vmc_balance.x_offset = 0.0f;
  vmc_balance.x_limit = VMC_BODY_MOTION_X_LIMIT_DEFAULT;
  vmc_balance.x_feedback_gain = VMC_BALANCE_X_FEEDBACK_GAIN_DEFAULT;
  vmc_balance.dx_feedback_gain = VMC_BALANCE_DX_FEEDBACK_GAIN_DEFAULT;
  vmc_balance.x_feedback_limit = VMC_BALANCE_X_FEEDBACK_LIMIT_DEFAULT;
  vmc_balance.dx_feedback_limit = VMC_BALANCE_DX_FEEDBACK_LIMIT_DEFAULT;
  vmc_balance.speed_control_enable = VMC_BALANCE_SPEED_CONTROL_ENABLE_DEFAULT;
  vmc_balance.speed_target = 0.0f;
  vmc_balance.speed_target_ramped = 0.0f;
  vmc_balance.speed_target_max = VMC_BALANCE_SPEED_TARGET_MAX_DEFAULT;
  vmc_balance.speed_ramp_rate = VMC_BALANCE_SPEED_RAMP_RATE_DEFAULT;
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
  vmc_balance.kalman_slip_confirm_ms =
      VMC_BODY_MOTION_KF_SLIP_CONFIRM_MS_DEFAULT;
  body_motion_kf_slip_initialized = 0U;
  vmc_balance.tp_limit = VMC_BALANCE_TP_LIMIT_DEFAULT;
  vmc_balance.tp_phi_gain = VMC_BALANCE_TP_PHI_GAIN_DEFAULT;
  vmc_balance.tp_x_gain = VMC_BALANCE_TP_X_GAIN_DEFAULT;
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
  const double speed_ramp_step =
      clamp_double(vmc_balance.speed_ramp_rate, 0.0, 10.0) * vmc_loop_dt();
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
  const double dx_raw = (vmc_balance.dx_feedback_enable != 0U)
                            ? (motion->kf_dx - dx_feedback_target) *
                                  clamp_double(vmc_balance.dx_feedback_gain, -200.0, 200.0)
                            : 0.0;
  const double dx_feedback_limit =
      clamp_double(vmc_balance.dx_feedback_limit, 0.0, 30.0);
  const double dx_error =
      clamp_double(dx_raw, -dx_feedback_limit, dx_feedback_limit);
  /* Shifting the virtual-leg theta target moves the wheel contact point under
   * the combined chassis/gimbal centre of mass.  This provides substantially
   * more static balancing authority than body-pitch trim alone. */
  const double theta_target =
      vmc_balance.pitch_offset +
      (double)vmc_balance.disturbance_observer_output *
          clamp_double(vmc_balance.disturbance_observer_theta_gain, -2.0, 2.0);
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
  const double left_wheel_theta_term =
      vmc_probe_state.lqr.left_k[0] * left_lqr_theta_error;
  const double left_wheel_dtheta_term =
      vmc_probe_state.lqr.left_k[2] * left_lqr_d_theta_limited;
  const double left_wheel_x_term = vmc_probe_state.lqr.left_k[4] * x_feedback_error;
  const double left_wheel_dx_term = vmc_probe_state.lqr.left_k[6] * dx_error;
  const double left_wheel_phi_term = vmc_probe_state.lqr.left_k[8] * phi_error;
  const double left_wheel_dphi_term = vmc_probe_state.lqr.left_k[10] * dpitch_lpf;
  const double right_wheel_theta_term =
      vmc_probe_state.lqr.right_k[0] * right_lqr_theta_error;
  const double right_wheel_dtheta_term =
      vmc_probe_state.lqr.right_k[2] * right_lqr_d_theta_limited;
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
      vmc_probe_state.lqr.left_k[1] * left_lqr_theta_error;
  const double left_tp_dtheta_term =
      vmc_probe_state.lqr.left_k[3] * left_lqr_d_theta_limited;
  const double right_tp_theta_term =
      vmc_probe_state.lqr.right_k[1] * right_lqr_theta_error;
  const double right_tp_dtheta_term =
      vmc_probe_state.lqr.right_k[3] * right_lqr_d_theta_limited;
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

  if (vmc_balance.enabled != 0U &&
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
  } else {
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
  vmc_control.roll_comp_gain = VMC_ROLL_LENGTH_COMP_GAIN_DEFAULT;
  vmc_control.roll_comp_limit = VMC_ROLL_LENGTH_COMP_LIMIT_DEFAULT;
  vmc_control.roll_comp_ramp_rate = VMC_ROLL_LENGTH_COMP_RAMP_RATE_DEFAULT;
  vmc_control.roll_comp_offset = 0.0f;
  vmc_control.left_target_length = VMC_LEG_LENGTH_DEFAULT;
  vmc_control.right_target_length = VMC_LEG_LENGTH_DEFAULT;
  vmc_control.leg_follow_enable = VMC_LEG_FOLLOW_ENABLE_DEFAULT;
  vmc_control.leg_follow_machine_wide = VMC_LEG_FOLLOW_MACHINE_WIDE_DEFAULT;
  vmc_control.leg_follow_auto_scope = VMC_LEG_FOLLOW_AUTO_SCOPE_DEFAULT;
  vmc_control.leg_follow_roll_deadband =
      VMC_LEG_FOLLOW_ROLL_DEADBAND_DEFAULT;
  vmc_control.leg_follow_episode_timeout_ms =
      VMC_LEG_FOLLOW_EPISODE_TIMEOUT_MS_DEFAULT;
  vmc_control.leg_follow_scope_machine_wide =
      VMC_LEG_FOLLOW_MACHINE_WIDE_DEFAULT;
  vmc_control.leg_follow_episode_active = 0U;
  vmc_control.leg_follow_scope_resolved = 0U;
  vmc_control.leg_follow_episode_side = VMC_LEG_FOLLOW_SIDE_NONE;
  vmc_control.leg_follow_episode_ms = 0U;
  vmc_control.leg_follow_rate = VMC_LEG_FOLLOW_RATE_DEFAULT;
  vmc_control.leg_follow_return_rate = VMC_LEG_FOLLOW_RETURN_RATE_DEFAULT;
  vmc_control.leg_follow_offset_left = 0.0f;
  vmc_control.leg_follow_offset_right = 0.0f;
  vmc_control.getup_state = VMC_GETUP_STATE_IDLE;
  vmc_control.getup_turn_direction = 0;
  vmc_control.getup_lqr_release_pending = 0U;
  vmc_control.getup_shrink_target_theta_left = 0.0f;
  vmc_control.getup_shrink_target_theta_right = 0.0f;
  vmc_control.getup_target_phi0_left = VMC_LEG_PHI0_TARGET_DEFAULT;
  vmc_control.getup_target_phi0_right = VMC_LEG_PHI0_TARGET_DEFAULT;
  vmc_getup_fully_fallen = 0U;
  vmc_fall_rearm_required = 0U;
  vmc_fall_rearm_armed = 0U;
  vmc_fall_monitor_armed = 0U;
  vmc_fall_prev_fully_fallen = 0U;
  vmc_jump_last_update_ms = 0U;
  vmc_jump_auto_hold_ms = 0U;
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

/* Scope implied by the roll sign, if the sign is decisive yet.
 *
 *   0  per-leg      the unloaded side is the LOW one: it went over an edge
 *   1  whole machine the unloaded side is the HIGH one: the machine was lifted
 *   2  undecided    |roll| is inside the deadband
 *
 * body.roll is positive when the RIGHT side is high, so the unloaded side's
 * signed roll is +roll when the right side unloaded and -roll when the left
 * did; positive therefore means "the unloaded side is the high one".
 *
 * This is split out from the episode state machine so the sign convention can
 * be tested on its own - getting it backwards inverts both cases.
 */
static uint8_t vmc_leg_follow_scope_from_roll(uint8_t left_unloaded,
                                              uint8_t right_unloaded,
                                              double roll,
                                              double deadband)
{
  double signed_roll;

  if (left_unloaded != 0U && right_unloaded != 0U) {
    /* Both sides unloaded: the whole machine is off the ground, so there is
     * no grounded leg to lever against either way. */
    return 1U;
  }
  if (left_unloaded == 0U && right_unloaded == 0U) {
    return 2U;
  }

  signed_roll = (right_unloaded != 0U) ? roll : -roll;
  if (signed_roll > deadband) {
    return 1U;
  }
  if (signed_roll < -deadband) {
    return 0U;
  }
  return 2U;
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
                                      double ahead_direction,
                                      double back_direction)
{
  double joint_torque[2];
  const double length_error = target_length - leg->length;
  const double p_output = kp_length * length_error;
  const double d_output = -kd_length * leg->d_length;
  const double pid_output = VMC_LEG_LENGTH_PID_DIRECTION * (p_output + d_output);
  const double support_output = VMC_LEG_SUPPORT_FORCE_DIRECTION * support_force;
  const double force = clamp_double(pid_output + support_output, -max_force, max_force);
  const double phi0_error = target_phi0 - leg->phi0;
  const double phi0_p_output = kp_phi0 * phi0_error;
  const double phi0_d_output = -kd_phi0 * leg->d_phi0;
  const double phi0_pid_output =
      VMC_LEG_PHI0_PID_DIRECTION * (phi0_p_output + phi0_d_output);
  const double phi0_torque_raw =
      clamp_double(phi0_pid_output, -max_phi0_torque, max_phi0_torque);
  const uint8_t use_lqr_tp =
      (vmc_balance.tp_output_enable != 0U &&
       vmc_getup_is_active() == 0U &&
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
          : 0.0;
  const double phi0_torque_total =
      clamp_double(phi0_torque + lqr_tp_torque, -max_phi0_torque, max_phi0_torque);

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
  double left_target_phi0;
  double right_target_phi0;
  double max_phi0_torque;
  double support_force;
  double roll_error;
  double roll_comp_desired;
  double roll_comp_offset;
  double roll_comp_limit;
  double left_target_length;
  double right_target_length;
  double length_ramp_rate;
  double kp_length;
  double kd_length;
  uint8_t jump_override;
  uint8_t jump_force_profile;

  vmc_update_remote_target_length();
  vmc_update_getup_state();
  if (vmc_jump_is_active() != 0U && vmc_getup_is_active() != 0U) {
    vmc_jump_finish(VMC_JUMP_STAGE_ABORT,
                    VMC_JUMP_ABORT_GETUP,
                    HAL_GetTick());
  }
  jump_override = vmc_jump_overrides_target();

  if (vmc_control.enabled == 0U ||
      vmc_fall_rearm_required != 0U ||
      vmc_probe_state.feedback_timeout_active != 0U ||
      remote_control_allows_joint_enable() == 0U) {
    vmc_control.ramp_active = 0U;
    vmc_control.roll_comp_offset = 0.0f;
    vmc_control.leg_follow_offset_left = 0.0f;
    vmc_control.leg_follow_offset_right = 0.0f;
    vmc_control.leg_follow_episode_active = 0U;
    vmc_control.leg_follow_scope_resolved = 0U;
    vmc_control.leg_follow_episode_ms = 0U;
    vmc_control.leg_follow_episode_side = VMC_LEG_FOLLOW_SIDE_NONE;
    vmc_control.left_target_length = vmc_control.target_length;
    vmc_control.right_target_length = vmc_control.target_length;
    vmc_control.getup_target_phi0_left = VMC_LEG_PHI0_TARGET_DEFAULT;
    vmc_control.getup_target_phi0_right = VMC_LEG_PHI0_TARGET_DEFAULT;
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
  if (vmc_control.getup_state == VMC_GETUP_STATE_TURN ||
      vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER) {
    target_length = VMC_GETUP_TURN_LENGTH;
  } else if (vmc_control.getup_state == VMC_GETUP_STATE_SHRINK) {
    target_length = VMC_GETUP_STAND_LENGTH;
  } else if (jump_override != 0U) {
    target_length = vmc_jump_stage_target_length();
  }
  vmc_control.jump_target_length = (float)target_length;
  if (jump_override != 0U) {
    ramped_target_length = target_length;
  } else {
    length_ramp_rate = clamp_double(vmc_control.length_ramp_rate, 0.0, 1.0);
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
       vmc_getup_is_active() == 0U &&
       jump_override == 0U &&
       vmc_probe_state.body.imu_valid != 0U &&
       vmc_probe_state.body.imu_connected != 0U)
          ? clamp_double(roll_error * (double)vmc_control.roll_comp_gain,
                         -roll_comp_limit,
                         roll_comp_limit)
          : 0.0;
  roll_comp_offset =
      vmc_step_towards(vmc_control.roll_comp_offset,
                       roll_comp_desired,
                       clamp_double(vmc_control.roll_comp_ramp_rate, 0.0, 1.0) *
                           vmc_loop_dt());
  vmc_control.roll_comp_offset = (float)roll_comp_offset;

  left_target_length =
      clamp_double(ramped_target_length + roll_comp_offset,
                   VMC_LEG_LENGTH_MIN,
                   VMC_LEG_LENGTH_MAX);
  right_target_length =
      clamp_double(ramped_target_length - roll_comp_offset,
                   VMC_LEG_LENGTH_MIN,
                   VMC_LEG_LENGTH_MAX);

  /* Ground-following extension.  A leg that has stopped carrying the machine
   * lets its target chase outward so the support feedforward can push that
   * wheel back down to the ground; a loaded leg returns to the preset.
   *
   * This is what makes a slowly lifted body behave: without it the loop holds
   * a fixed length, treats the lift as a position error, and pulls the wheel
   * UP toward the body - so the leg stops following long before it reaches
   * the stroke limit and the wheel leaves the ground early.
   *
   * The offset is capped at the remaining stroke, so the target can never be
   * driven past VMC_LEG_LENGTH_MAX; once there the leg is fully extended and
   * further lifting is genuine flight, which the airborne detector handles.
   *
   * Scope comes from the roll direction and is decided ONCE per episode; see
   * VMC_LEG_FOLLOW_AUTO_SCOPE_DEFAULT for why the sign, and why it stays
   * pending until the sign is decisive.
   *
   * The episode outlives the load latch on purpose.  Extending the leg raises
   * the joint torque, which trips the very latch the follow is waiting on, so
   * a naive "follow while latched" loop extended ~14 mm, released itself,
   * unwound 102 mm and started over - it never accumulated enough reach to
   * touch anything.  An episode therefore persists while the body is still
   * tilted and only unwinds once it is level again (or the timeout expires,
   * so a machine parked on an edge cannot hold its extension forever). */
  if (vmc_control.leg_follow_enable != 0U &&
      vmc_getup_is_active() == 0U &&
      jump_override == 0U) {
    const double dt = vmc_loop_dt();
    const double deadband =
        clamp_double(vmc_control.leg_follow_roll_deadband, 0.0, 0.5);
    const double roll_now = (double)vmc_probe_state.body.roll;
    const uint8_t body_level = (fabs(roll_now) <= deadband) ? 1U : 0U;
    const double follow_step =
        clamp_double(vmc_control.leg_follow_rate, 0.0, 1.0) * dt;
    const double return_step =
        clamp_double(vmc_control.leg_follow_return_rate, 0.0, 1.0) * dt;
    const uint8_t left_unloaded =
        vmc_probe_state.airborne_side_active[VMC_AIRBORNE_SIDE_LEFT];
    const uint8_t right_unloaded =
        vmc_probe_state.airborne_side_active[VMC_AIRBORNE_SIDE_RIGHT];
    const uint8_t any_unloaded =
        (left_unloaded != 0U || right_unloaded != 0U) ? 1U : 0U;
    /* Side-low flags, from the same sign convention as the scope: roll is
     * positive when the RIGHT side is high, so the LEFT side is the low one
     * when roll is positive. */
    const uint8_t left_low = (roll_now > deadband) ? 1U : 0U;
    const uint8_t right_low = (roll_now < -deadband) ? 1U : 0U;
    const uint32_t episode_timeout_ms =
        (vmc_control.leg_follow_episode_timeout_ms > 60000U)
            ? 60000U
            : vmc_control.leg_follow_episode_timeout_ms;
    uint8_t machine_wide;
    uint8_t follow_left;
    uint8_t follow_right;

    /* --- episode lifetime --------------------------------------------- */
    if (any_unloaded != 0U) {
      if (vmc_control.leg_follow_episode_active == 0U) {
        /* Fresh episode: clear everything inherited from the last one, and
         * default to per-leg until the sign is decisive.  Remember which side
         * triggered it - the latch cannot be asked again once it lets go, but
         * the scope decision and the per-leg reach both still need to know. */
        vmc_control.leg_follow_episode_active = 1U;
        vmc_control.leg_follow_scope_resolved = 0U;
        vmc_control.leg_follow_scope_machine_wide = 0U;
        if (left_unloaded != 0U && right_unloaded != 0U) {
          vmc_control.leg_follow_episode_side = VMC_LEG_FOLLOW_SIDE_NONE;
        } else {
          vmc_control.leg_follow_episode_side = (right_unloaded != 0U)
                                                    ? VMC_AIRBORNE_SIDE_RIGHT
                                                    : VMC_AIRBORNE_SIDE_LEFT;
        }
      }
      vmc_control.leg_follow_episode_ms = 0U;
    } else if (vmc_control.leg_follow_episode_active != 0U) {
      if (vmc_control.leg_follow_episode_ms < episode_timeout_ms) {
        vmc_control.leg_follow_episode_ms += (uint32_t)(dt * 1000.0);
      }
      /* Hold the episode while the body is still tilted, so the extension
       * this feature just paid for is not given straight back. */
      if (body_level != 0U ||
          vmc_control.leg_follow_episode_ms >= episode_timeout_ms) {
        vmc_control.leg_follow_episode_active = 0U;
        vmc_control.leg_follow_scope_resolved = 0U;
        vmc_control.leg_follow_episode_ms = 0U;
      }
    }

    /* --- scope, settled once the sign is decisive --------------------- */
    {
      /* Both sides latched at any point in the episode means real flight, so
       * there is no grounded leg to lever against. */
      const uint8_t both_latched =
          (left_unloaded != 0U && right_unloaded != 0U) ? 1U : 0U;

      if (vmc_control.leg_follow_auto_scope == 0U) {
        vmc_control.leg_follow_scope_machine_wide =
            (vmc_control.leg_follow_machine_wide != 0U) ? 1U : 0U;
        vmc_control.leg_follow_scope_resolved = 1U;
      } else if (both_latched != 0U ||
                 vmc_control.leg_follow_episode_side ==
                     VMC_LEG_FOLLOW_SIDE_NONE) {
        vmc_control.leg_follow_scope_machine_wide = 1U;
        vmc_control.leg_follow_scope_resolved = 1U;
      } else if (vmc_control.leg_follow_episode_active != 0U &&
                 vmc_control.leg_follow_scope_resolved == 0U) {
        /* Evaluate the sign against the side that actually unloaded, using
         * the remembered side - not the instantaneous latch, which is
         * already gone by the time the body has tilted enough to be
         * decisive. */
        const uint8_t decided = vmc_leg_follow_scope_from_roll(
            (vmc_control.leg_follow_episode_side == VMC_AIRBORNE_SIDE_LEFT)
                ? 1U
                : 0U,
            (vmc_control.leg_follow_episode_side == VMC_AIRBORNE_SIDE_RIGHT)
                ? 1U
                : 0U,
            roll_now, deadband);
        if (decided != 2U) {
          vmc_control.leg_follow_scope_machine_wide = decided;
          vmc_control.leg_follow_scope_resolved = 1U;
        }
      }
    }

    /* --- which legs extend -------------------------------------------- */
    machine_wide = vmc_control.leg_follow_scope_machine_wide;
    if (machine_wide != 0U) {
      /* A lift has to take the whole body with it, so both legs follow while
       * the episode is live rather than only while a latch happens to be on -
       * the latch releases as soon as the legs start carrying load. */
      follow_left = vmc_control.leg_follow_episode_active;
      follow_right = vmc_control.leg_follow_episode_active;
    } else if (vmc_control.leg_follow_episode_side == VMC_AIRBORNE_SIDE_LEFT) {
      /* Per-leg: the side that unloaded keeps reaching while it is the LOW
       * one, even after its latch let go - reaching for ground is the point,
       * and the latch drops as soon as the leg starts pushing. */
      follow_left = (left_unloaded != 0U || left_low != 0U) ? 1U : 0U;
      follow_right = 0U;
    } else if (vmc_control.leg_follow_episode_side == VMC_AIRBORNE_SIDE_RIGHT) {
      follow_left = 0U;
      follow_right = (right_unloaded != 0U || right_low != 0U) ? 1U : 0U;
    } else {
      follow_left = left_unloaded;
      follow_right = right_unloaded;
    }

    double offset_left = (double)vmc_control.leg_follow_offset_left;
    double offset_right = (double)vmc_control.leg_follow_offset_right;

    /* Each side is still capped by its OWN remaining stroke, so a leg that is
     * already near full extension stops there while the other keeps
     * following. */
    if (follow_left != 0U) {
      offset_left = clamp_double(offset_left + follow_step,
                                 0.0,
                                 VMC_LEG_LENGTH_MAX - left_target_length);
    } else {
      offset_left = vmc_step_towards(offset_left, 0.0, return_step);
    }
    if (follow_right != 0U) {
      offset_right = clamp_double(offset_right + follow_step,
                                  0.0,
                                  VMC_LEG_LENGTH_MAX - right_target_length);
    } else {
      offset_right = vmc_step_towards(offset_right, 0.0, return_step);
    }

    vmc_control.leg_follow_offset_left = (float)offset_left;
    vmc_control.leg_follow_offset_right = (float)offset_right;
    left_target_length += offset_left;
    right_target_length += offset_right;
  } else {
    /* Disabled, or a jump / recovery owns the leg: drop the extra extension
     * so the preset is restored the moment this stops applying. */
    vmc_control.leg_follow_offset_left = 0.0f;
    vmc_control.leg_follow_offset_right = 0.0f;
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
  kp_length = (jump_force_profile != 0U)
                  ? clamp_double(vmc_control.jump_kp_length, 0.0, 5000.0)
                  : clamp_double(vmc_control.kp_length, 0.0, 5000.0);
  kd_length = (jump_force_profile != 0U)
                  ? clamp_double(vmc_control.jump_kd_length, 0.0, 1000.0)
                  : clamp_double(vmc_control.kd_length, 0.0, 1000.0);
  max_force = (jump_force_profile != 0U)
                  ? clamp_double(vmc_control.jump_max_force,
                                 0.0,
                                 VMC_LEG_FORCE_MAX_DEFAULT)
                  : clamp_double(vmc_control.max_force,
                                 0.0,
                                 VMC_LEG_FORCE_MAX_DEFAULT);
  left_target_phi0 = vmc_control.target_phi0;
  right_target_phi0 = vmc_control.target_phi0;
  if (vmc_control.getup_state == VMC_GETUP_STATE_TURN ||
      vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER) {
    if (vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER) {
      /* For an abnormal (not fully fallen) leg pose, deliberately use the
       * configured reverse direction instead of the shortest wrapped path.
       * This keeps the link moving away from the ground obstruction; both
       * legs follow the same safe reverse direction until their independent
       * theta checks report the 1.1 rad recovery target. */
      left_target_phi0 = vmc_probe_state.left_leg.phi0 +
                         (double)vmc_control.getup_turn_direction *
                             VMC_GETUP_PHI0_STEP;
      right_target_phi0 = vmc_probe_state.right_leg.phi0 +
                          (double)vmc_control.getup_turn_direction *
                              VMC_GETUP_PHI0_STEP;
    } else {
      /* Restore the original fully-fallen TURN behavior from
       * d1186266c51e70b405d2a31326b389d67704fa00: both legs follow the same
       * chassis-level recovery direction.  Independent leg handling is
       * reserved for the non-fully-fallen LEG_RECOVER path above. */
      left_target_phi0 = vmc_probe_state.left_leg.phi0 +
                         (double)vmc_control.getup_turn_direction *
                             VMC_GETUP_PHI0_STEP;
      right_target_phi0 = vmc_probe_state.right_leg.phi0 +
                          (double)vmc_control.getup_turn_direction *
                              VMC_GETUP_PHI0_STEP;
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
  }
  vmc_control.getup_target_phi0_left = (float)left_target_phi0;
  vmc_control.getup_target_phi0_right = (float)right_target_phi0;
  max_phi0_torque = clamp_double(vmc_control.max_phi0_torque,
                                 0.0,
                                 VMC_LEG_TP_MAX);
  support_force = (jump_force_profile != 0U)
                      ? clamp_double(vmc_control.jump_support_force,
                                     -VMC_LEG_SUPPORT_FORCE_MAX,
                                     VMC_LEG_SUPPORT_FORCE_MAX)
                      : clamp_double(vmc_control.support_force,
                                     -VMC_LEG_SUPPORT_FORCE_MAX,
                                     VMC_LEG_SUPPORT_FORCE_MAX);

  update_leg_length_control(&vmc_probe_state.left_leg,
                            left_target_length,
                            kp_length,
                            kd_length,
                            max_force,
                            left_target_phi0,
                            vmc_control.kp_phi0,
                            vmc_control.kd_phi0,
                            max_phi0_torque,
                            support_force,
                            VMC_DIR_LEFT_AHEAD,
                            VMC_DIR_LEFT_BACK);
  update_leg_length_control(&vmc_probe_state.right_leg,
                            right_target_length,
                            kp_length,
                            kd_length,
                            max_force,
                            right_target_phi0,
                            vmc_control.kp_phi0,
                            vmc_control.kd_phi0,
                            max_phi0_torque,
                            support_force,
                            VMC_DIR_RIGHT_AHEAD,
                            VMC_DIR_RIGHT_BACK);
}

void vmc_wheel_control_update(void)
{
  HAL_StatusTypeDef status;
  int16_t max_current = vmc_wheel_control.max_current;
  int16_t left_current = 0;
  int16_t right_current = 0;
  int16_t left_send_current = 0;
  int16_t right_send_current = 0;

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
    const uint8_t output_ready =
        (vmc_wheel_control.enabled != 0U &&
         vmc_balance.wheel_output_enable != 0U &&
         vmc_balance.wheel_output_user_enable != 0U &&
         vmc_probe_state.airborne_active == 0U &&
         vmc_probe_state.feedback_timeout_active == 0U &&
         vmc_jump_blocks_wheel_output() == 0U &&
         remote_control_allows_joint_enable() != 0U)
            ? 1U
            : 0U;

    if (output_ready != 0U &&
        vmc_probe_state.airborne_side_active[VMC_AIRBORNE_SIDE_LEFT] == 0U) {
      left_current = clamp_i16(vmc_wheel_control.left_current,
                               (int16_t)-max_current,
                               max_current);
    }
    if (output_ready != 0U &&
        vmc_probe_state.airborne_side_active[VMC_AIRBORNE_SIDE_RIGHT] == 0U) {
      right_current = clamp_i16(vmc_wheel_control.right_current,
                                (int16_t)-max_current,
                                max_current);
    }
  }

  left_send_current = (int16_t)(VMC_WHEEL_LEFT_CMD_DIRECTION * left_current);
  right_send_current = (int16_t)(VMC_WHEEL_RIGHT_CMD_DIRECTION * right_current);

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
    remote_control_update_source();
    vmc_update_remote_mode();
    vmc_update_body_state();
    vmc_update_gravity_compensation();
    vmc_update_leg_state();
    vmc_update_motion_state();
    vmc_jump_update();
    vmc_update_airborne_state();
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
