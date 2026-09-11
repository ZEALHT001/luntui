#include "vmc_probe.h"

#include "board_comm.h"
#include "hi12_imu.h"
#include "remote_control.h"
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
static uint32_t vmc_remote_mode_last_ms;
static uint8_t vmc_remote_source_last;
static float vmc_yaw_last_wrapped;
static float vmc_yaw_unwrapped;
static uint8_t vmc_yaw_unwrap_initialized;
/* A fall detected while the robot was enabled requires an explicit emergency
 * stop followed by S1=3 before control can be enabled again. */
static uint8_t vmc_fall_rearm_required;
static uint8_t vmc_fall_rearm_armed;
static uint8_t vmc_fall_monitor_armed;
static uint8_t vmc_fall_prev_fully_fallen;

static void clear_leg_command(volatile vmc_leg_state_t *leg);
static uint8_t vmc_feedback_timeout_source(void);
static void vmc_update_remote_target_length(void);
static uint8_t vmc_getup_is_active(void);
static uint8_t vmc_getup_pose_ready(void);
static void vmc_update_getup_state(void);
static void vmc_update_remote_mode(void);
static void vmc_update_body_state(void);
static void vmc_update_motion_state(void);
static void vmc_update_lqr_gain(void);
static uint8_t vmc_safety_allows_output(void);
static int16_t vmc_update_yaw_hold_current(void);
static int16_t vmc_update_gimbal_follow_current(void);
static void vmc_balance_apply_wheel_output(void);
static void vmc_body_motion_kf_reset(void);
static void vmc_body_motion_kf_update(double measured_dx, double measured_acc);
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

static double body_motion_kf_x[2];
static double body_motion_kf_p[4];
static uint8_t body_motion_kf_initialized;

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
}

static void vmc_body_motion_kf_update(double measured_dx, double measured_acc)
{
  const double dt = VMC_CONTROL_DT;
  const double q_v = clamp_double((double)vmc_balance.kalman_q_v, 0.0, 1.0e12);
  const double q_a = clamp_double((double)vmc_balance.kalman_q_a, 0.0, 1.0e12);
  const double r_wheel = clamp_double((double)vmc_balance.kalman_r_wheel,
                                      1.0e-9, 1.0e12);
  const double r_imu = clamp_double((double)vmc_balance.kalman_r_imu,
                                    1.0e-9, 1.0e12);
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

  if (body_motion_kf_initialized == 0U) {
    vmc_body_motion_kf_reset();
  }

  x0 = body_motion_kf_x[0] + dt * body_motion_kf_x[1];
  x1 = body_motion_kf_x[1];
  p00 = body_motion_kf_p[0] + dt * (body_motion_kf_p[2] + body_motion_kf_p[1]) +
        dt * dt * body_motion_kf_p[3] + q_v;
  p01 = body_motion_kf_p[1] + dt * body_motion_kf_p[3];
  p10 = body_motion_kf_p[2] + dt * body_motion_kf_p[3];
  p11 = body_motion_kf_p[3] + q_a;

  y = measured_dx - x0;
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

  y = measured_acc - x1;
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

  body_motion_kf_x[0] = x0;
  body_motion_kf_x[1] = x1;
  body_motion_kf_p[0] = p00;
  body_motion_kf_p[1] = p01;
  body_motion_kf_p[2] = p10;
  body_motion_kf_p[3] = p11;

  vmc_probe_state.motion.kf_dx = x0;
  vmc_probe_state.motion.kf_acc = x1;
  vmc_probe_state.motion.kf_x = clamp_double(
      vmc_probe_state.motion.kf_x + x0 * VMC_CONTROL_DT,
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
  static uint32_t s1_up_oscillation_start_ms;
  const uint32_t now = HAL_GetTick();

  if (remote_control.valid == 0U || remote_control.rx_count == 0U ||
      (HAL_GetTick() - remote_control.last_update_ms) > RC_LOST_TIMEOUT_MS) {
    s1_up_oscillation_start_ms = 0U;
    return;
  }

  if (remote_control.s[0] == RC_SW_UP) {
    vmc_control.target_length = VMC_LEG_TARGET_LONG;
  } else if (remote_control.s[0] == RC_SW_MID) {
    vmc_control.target_length = VMC_LEG_TARGET_MID;
  } else if (remote_control.s[0] == RC_SW_DOWN) {
    vmc_control.target_length = VMC_LEG_TARGET_SHORT;
  }

  /* S1 up is a continuous chassis test mode.  Keep the legs moving between
   * the established short/long targets with a triangular waveform. */
  if (remote_control.s[1] == RC_SW_UP) {
    float low = clamp_float(vmc_control.s1_up_leg_low,
                            VMC_LEG_LENGTH_MIN, VMC_LEG_LENGTH_MAX);
    float high = clamp_float(vmc_control.s1_up_leg_high,
                             VMC_LEG_LENGTH_MIN, VMC_LEG_LENGTH_MAX);
    const float speed = clamp_float(vmc_control.s1_up_leg_speed, 0.001f, 1.0f);
    uint32_t period;
    uint32_t half_period;
    float span;
    uint32_t elapsed;
    uint32_t phase;

    if (high < low) {
      const float tmp = low;
      low = high;
      high = tmp;
    }
    span = high - low;
    period = (uint32_t)((2.0f * span / speed) * 1000.0f);
    if (period < 2U) period = 2U;
    half_period = period / 2U;

    if (s1_up_oscillation_start_ms == 0U) {
      s1_up_oscillation_start_ms = now;
    }
    elapsed = now - s1_up_oscillation_start_ms;
    phase = (period == 0U) ? 0U : (elapsed % period);
    if (phase < half_period) {
      vmc_control.target_length =
          low + span * ((float)phase / (float)half_period);
    } else {
      vmc_control.target_length =
          high - span * ((float)(phase - half_period) / (float)half_period);
    }
  } else {
    s1_up_oscillation_start_ms = 0U;
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
  double dt = VMC_CONTROL_DT;
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
    vmc_remote_mode_last_ms = now;
    return;
  }

  if (vmc_remote_mode_last_ms != 0U) {
    dt = (double)(now - vmc_remote_mode_last_ms) * 0.001;
    dt = clamp_double(dt, VMC_CONTROL_DT, 0.10);
  }
  vmc_remote_mode_last_ms = now;

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

  if (remote_control.s[1] == RC_SW_UP) {
    vmc_control.enabled = 1U;
    vmc_balance.wheel_output_enable = 1U;
  } else if (remote_control.s[1] == RC_SW_MID) {
    vmc_control.enabled = 1U;
    /* In the middle position, allow wheels only when the chassis is upright
     * and the automatic get-up sequence is no longer active.  Joints remain
     * enabled during get-up so the state machine can recover the chassis. */
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

  forward_current = rc_channel_to_current(remote_control.ch[3],
                                          VMC_RC_WHEEL_FWD_CURRENT_MAX);
  /* S1 up is an in-place rotation test.  Do not convert the neutral forward
   * stick into a forward speed command; the configured current is applied as
   * an equal/opposite differential to the two wheels below. */
  if (remote_control.s[1] == RC_SW_UP) {
    forward_current = 0;
  }
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
    yaw_rate_max = clamp_double(
        (remote_control.s[1] == RC_SW_UP)
            ? vmc_balance.s1_up_yaw_target_rate_max
            : vmc_balance.yaw_target_rate_max,
        0.0, 10.0);
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
  vmc_probe_state.body.ax = hi12_imu.acc_mss[0];
  vmc_probe_state.body.ay = hi12_imu.acc_mss[1];
  vmc_probe_state.body.az = hi12_imu.acc_mss[2];
  vmc_probe_state.body.imu_connected = hi12_imu.connected;
  vmc_probe_state.body.imu_valid = hi12_imu.valid;
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
  vmc_probe_state.motion.dd_x = vmc_probe_state.body.ax;

  if (vmc_control.enabled == 0U || vmc_probe_state.feedback_timeout_active != 0U) {
    vmc_body_motion_kf_reset();
  } else {
    vmc_body_motion_kf_update(vmc_probe_state.motion.d_x_raw,
                              vmc_probe_state.motion.dd_x);
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
          remote_control_allows_joint_enable() != 0U)
             ? 1U
             : 0U;
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
  vmc_balance.s1_up_yaw_target_rate_max =
      VMC_S1_UP_YAW_TARGET_RATE_MAX_DEFAULT;
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
  vmc_balance.kalman_q_v = VMC_BODY_MOTION_KF_Q_V_DEFAULT;
  vmc_balance.kalman_q_a = VMC_BODY_MOTION_KF_Q_A_DEFAULT;
  vmc_balance.kalman_r_wheel = VMC_BODY_MOTION_KF_R_WHEEL_DEFAULT;
  vmc_balance.kalman_r_imu = VMC_BODY_MOTION_KF_R_IMU_DEFAULT;
  vmc_balance.tp_limit = VMC_BALANCE_TP_LIMIT_DEFAULT;
  vmc_balance.tp_phi_gain = VMC_BALANCE_TP_PHI_GAIN_DEFAULT;
  vmc_balance.tp_x_gain = VMC_BALANCE_TP_X_GAIN_DEFAULT;
  vmc_probe_state.balance = vmc_balance;
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
      clamp_double(vmc_balance.speed_ramp_rate, 0.0, 10.0) * VMC_CONTROL_DT;
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
    speed_accel = (motion->kf_dx - vmc_speed_dx_prev) / VMC_CONTROL_DT;
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
      clamp_double(vmc_balance.speed_pitch_ramp_rate, 0.0, 5.0) * VMC_CONTROL_DT;
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
          vmc_balance.disturbance_observer_neutral_time + VMC_CONTROL_DT,
          0.0,
          observer_delay + 1.0);
      if ((double)vmc_balance.disturbance_observer_neutral_time >= observer_delay &&
          fabs(speed_error) <= observer_speed_limit) {
        vmc_balance.disturbance_observer_active = 1U;
        vmc_balance.disturbance_observer_residual = (float)speed_error;
        observer_output +=
            (observer_gain * speed_error - observer_leak * observer_output) *
            VMC_CONTROL_DT;
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
  vmc_control.s1_up_leg_low = VMC_S1_UP_LEG_LOW_DEFAULT;
  vmc_control.s1_up_leg_high = VMC_S1_UP_LEG_HIGH_DEFAULT;
  vmc_control.s1_up_leg_speed = VMC_S1_UP_LEG_SPEED_DEFAULT;
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
      (vmc_balance.tp_output_enable != 0U && vmc_getup_is_active() == 0U) ? 1U : 0U;
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

  vmc_update_remote_target_length();
  vmc_update_getup_state();

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
  }
  length_ramp_rate = (remote_control.valid != 0U &&
                      remote_control.s[1] == RC_SW_UP)
                         ? clamp_double(vmc_control.s1_up_leg_speed, 0.001, 1.0)
                         : clamp_double(vmc_control.length_ramp_rate, 0.0, 1.0);
  ramped_target_length =
      vmc_step_towards(vmc_control.ramped_target_length,
                       target_length,
                       length_ramp_rate * VMC_CONTROL_DT);
  vmc_control.ramped_target_length = (float)ramped_target_length;

  roll_comp_limit = clamp_double(vmc_control.roll_comp_limit, 0.0, 0.10);
  roll_error = (double)vmc_probe_state.body.roll - (double)vmc_control.roll_target;
  roll_comp_desired =
      (vmc_control.roll_comp_enable != 0U &&
       vmc_getup_is_active() == 0U &&
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
                           VMC_CONTROL_DT);
  vmc_control.roll_comp_offset = (float)roll_comp_offset;

  left_target_length =
      clamp_double(ramped_target_length + roll_comp_offset,
                   VMC_LEG_LENGTH_MIN,
                   VMC_LEG_LENGTH_MAX);
  right_target_length =
      clamp_double(ramped_target_length - roll_comp_offset,
                   VMC_LEG_LENGTH_MIN,
                   VMC_LEG_LENGTH_MAX);
  vmc_control.left_target_length = (float)left_target_length;
  vmc_control.right_target_length = (float)right_target_length;

  max_force = clamp_double(vmc_control.max_force, 0.0, VMC_LEG_FORCE_MAX_DEFAULT);
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
                                 VMC_DM_J8009_TMAX);
  support_force = clamp_double(vmc_control.support_force,
                               -VMC_LEG_SUPPORT_FORCE_MAX,
                               VMC_LEG_SUPPORT_FORCE_MAX);

  update_leg_length_control(&vmc_probe_state.left_leg,
                            left_target_length,
                            vmc_control.kp_length,
                            vmc_control.kd_length,
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
                            vmc_control.kp_length,
                            vmc_control.kd_length,
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

  if (vmc_wheel_control.enabled != 0U &&
      vmc_balance.wheel_output_enable != 0U &&
      vmc_balance.wheel_output_user_enable != 0U &&
      vmc_probe_state.feedback_timeout_active == 0U &&
      remote_control_allows_joint_enable() != 0U) {
    left_current = clamp_i16(vmc_wheel_control.left_current,
                             (int16_t)-max_current,
                             max_current);
    right_current = clamp_i16(vmc_wheel_control.right_current,
                              (int16_t)-max_current,
                              max_current);
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

  for (;;) {
    hi12_imu_update();
    remote_control_update_source();
    vmc_update_remote_mode();
    vmc_update_body_state();
    vmc_update_leg_state();
    vmc_update_motion_state();
    vmc_lqr_rtt_update();
    vmc_update_lqr_gain();
    vmc_balance_update();
    vmc_lqr_rtt_telemetry_update();
    vmc_control_update();
    vmc_dm_state_machine_update();
    vmc_balance_apply_wheel_output();
    vmc_wheel_control_update();
    board_comm_update();
    osDelay(1);
  }
}
