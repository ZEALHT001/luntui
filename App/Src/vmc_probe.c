#include "vmc_probe.h"

#include "hi12_imu.h"
#include "remote_control.h"
#include "vmc_math.h"

#include <string.h>

volatile vmc_probe_state_t vmc_probe_state;
volatile vmc_control_t vmc_control;

static void clear_leg_command(volatile vmc_leg_state_t *leg);
static uint8_t vmc_phi0_protect_source(void);

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
  motor->torque = (float)motor->given_current / 16384.0f * 20.0f * 0.3f /
                  3591.0f * 187.0f * 268.0f / 17.0f;
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

static uint8_t vmc_phi0_protect_source(void)
{
  uint8_t source = VMC_PHI0_PROTECT_NONE;

  if (vmc_probe_state.left_leg.phi0 < VMC_PHI0_PROTECT_MIN_RAD ||
      vmc_probe_state.left_leg.phi0 > VMC_PHI0_PROTECT_MAX_RAD) {
    source |= VMC_PHI0_PROTECT_LEFT;
  }

  if (vmc_probe_state.right_leg.phi0 < VMC_PHI0_PROTECT_MIN_RAD ||
      vmc_probe_state.right_leg.phi0 > VMC_PHI0_PROTECT_MAX_RAD) {
    source |= VMC_PHI0_PROTECT_RIGHT;
  }

  return source;
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
  const uint8_t phi0_source = vmc_phi0_protect_source();

  if (phi0_source != VMC_PHI0_PROTECT_NONE) {
    vmc_probe_state.phi0_protect_latched = 1U;
    vmc_probe_state.phi0_protect_source = phi0_source;
  } else if (remote_control_is_s1_down() != 0U) {
    vmc_probe_state.phi0_protect_latched = 0U;
    vmc_probe_state.phi0_protect_source = VMC_PHI0_PROTECT_NONE;
  }

  if (vmc_probe_state.phi0_protect_latched != 0U ||
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
  vmc_control.kp_length = VMC_LEG_LENGTH_KP_DEFAULT;
  vmc_control.kd_length = VMC_LEG_LENGTH_KD_DEFAULT;
  vmc_control.max_force = VMC_LEG_FORCE_MAX_DEFAULT;
}

static void clear_leg_command(volatile vmc_leg_state_t *leg)
{
  leg->length_error = 0.0;
  leg->length_p_output = 0.0;
  leg->length_d_output = 0.0;
  leg->length_pid_output = 0.0;
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
                                      double ahead_direction,
                                      double back_direction)
{
  double joint_torque[2];
  const double length_error = target_length - leg->length;
  const double p_output = kp_length * length_error;
  const double d_output = -kd_length * leg->d_length;
  const double pid_output = VMC_LEG_LENGTH_PID_DIRECTION * (p_output + d_output);
  const double force = clamp_double(pid_output, -max_force, max_force);

  leg->length_error = length_error;
  leg->length_p_output = p_output;
  leg->length_d_output = d_output;
  leg->length_pid_output = pid_output;
  leg->force_command = force;
  leg->torque_phi0_command = 0.0;

  vmc_leg_force(force, leg->torque_phi0_command, leg->phi1, leg->phi4, joint_torque);

  leg->joint_torque_ahead = ahead_direction * joint_torque[0];
  leg->joint_torque_back = back_direction * joint_torque[1];
}

void vmc_control_update(void)
{
  double target_length;
  double max_force;

  if (vmc_control.enabled == 0U) {
    clear_leg_command(&vmc_probe_state.left_leg);
    clear_leg_command(&vmc_probe_state.right_leg);
    return;
  }

  target_length = clamp_double(vmc_control.target_length,
                               VMC_LEG_LENGTH_MIN,
                               VMC_LEG_LENGTH_MAX);
  max_force = clamp_double(vmc_control.max_force, 0.0, VMC_LEG_FORCE_MAX_DEFAULT);

  update_leg_length_control(&vmc_probe_state.left_leg,
                            target_length,
                            vmc_control.kp_length,
                            vmc_control.kd_length,
                            max_force,
                            VMC_DIR_LEFT_AHEAD,
                            VMC_DIR_LEFT_BACK);
  update_leg_length_control(&vmc_probe_state.right_leg,
                            target_length,
                            vmc_control.kp_length,
                            vmc_control.kd_length,
                            max_force,
                            VMC_DIR_RIGHT_AHEAD,
                            VMC_DIR_RIGHT_BACK);
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
  vmc_can_init();
  vmc_control_init();

  for (;;) {
    hi12_imu_update();
    vmc_update_leg_state();
    vmc_control_update();
    vmc_dm_state_machine_update();
    osDelay(1);
  }
}
