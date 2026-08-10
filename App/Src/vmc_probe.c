#include "vmc_probe.h"

#include "vmc_math.h"

#include <string.h>

volatile vmc_probe_state_t vmc_probe_state;

static float dm_uint_to_float(uint16_t x, float x_min, float x_max, uint8_t bits)
{
  const float span = x_max - x_min;
  const float max_int = (float)((1UL << bits) - 1UL);
  return ((float)x) * span / max_int + x_min;
}

static uint16_t dm_float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
  const float span = x_max - x_min;
  const float max_int = (float)((1UL << bits) - 1UL);

  if (x > x_max) {
    x = x_max;
  } else if (x < x_min) {
    x = x_min;
  }

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

static HAL_StatusTypeDef can_send_joint_mit_zero(CAN_HandleTypeDef *hcan, uint32_t std_id)
{
  CAN_TxHeaderTypeDef header;
  uint8_t data[8] = {0};
  uint32_t mailbox;
  const uint16_t position_zero = dm_float_to_uint(0.0f, VMC_DM_J8009P_PMIN,
                                                  VMC_DM_J8009P_PMAX, 16);
  const uint16_t speed_zero = dm_float_to_uint(0.0f, VMC_DM_J8009_VMIN,
                                               VMC_DM_J8009_VMAX, 12);
  const uint16_t kp_zero = 0U;
  const uint16_t kd_zero = 0U;
  const uint16_t torque_zero = dm_float_to_uint(0.0f, VMC_DM_J8009_TMIN,
                                                VMC_DM_J8009_TMAX, 12);

  header.StdId = std_id;
  header.ExtId = 0;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;
  header.TransmitGlobalTime = DISABLE;

  data[0] = (uint8_t)(position_zero >> 8);
  data[1] = (uint8_t)(position_zero & 0xFFU);
  data[2] = (uint8_t)(speed_zero >> 4);
  data[3] = (uint8_t)(((speed_zero & 0x0FU) << 4) | ((kp_zero >> 8) & 0x0FU));
  data[4] = (uint8_t)(kp_zero & 0xFFU);
  data[5] = (uint8_t)(kd_zero >> 4);
  data[6] = (uint8_t)(((kd_zero & 0x0FU) << 4) | ((torque_zero >> 8) & 0x0FU));
  data[7] = (uint8_t)(torque_zero & 0xFFU);

  return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
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

void vmc_can_poll_joints_zero(void)
{
  static uint8_t poll_phase;
  HAL_StatusTypeDef can1_status;
  HAL_StatusTypeDef can2_status;

  if (poll_phase == 0U) {
    can1_status = can_send_joint_mit_zero(VMC_JOINT_RIGHT_AHEAD_CAN, VMC_JOINT_RIGHT_AHEAD_CMD_ID);
    can2_status = can_send_joint_mit_zero(VMC_JOINT_RIGHT_BACK_CAN, VMC_JOINT_RIGHT_BACK_CMD_ID);
    poll_phase = 1U;
  } else {
    can1_status = can_send_joint_mit_zero(VMC_JOINT_LEFT_AHEAD_CAN, VMC_JOINT_LEFT_AHEAD_CMD_ID);
    can2_status = can_send_joint_mit_zero(VMC_JOINT_LEFT_BACK_CAN, VMC_JOINT_LEFT_BACK_CMD_ID);
    poll_phase = 0U;
  }

  if (can1_status == HAL_OK) {
    vmc_probe_state.mit_zero_tx_count++;
  } else {
    vmc_probe_state.mit_zero_tx_fail_count++;
  }

  if (can2_status == HAL_OK) {
    vmc_probe_state.mit_zero_tx_count++;
  } else {
    vmc_probe_state.mit_zero_tx_fail_count++;
  }
}

void vmc_update_leg_state(void)
{
  double length;
  double phi0;

  vmc_probe_state.left_leg.phi1 =
      vmc_rad_format(vmc_probe_state.joint_left_back.position - VMC_ZERO_LEFT_BACK);
  vmc_probe_state.left_leg.phi4 =
      vmc_rad_format(vmc_probe_state.joint_left_ahead.position - VMC_ZERO_LEFT_AHEAD);
  vmc_leg_position(vmc_probe_state.left_leg.phi1, vmc_probe_state.left_leg.phi4,
                   &length, &phi0);
  vmc_probe_state.left_leg.length = length;
  vmc_probe_state.left_leg.phi0 = phi0;

  vmc_probe_state.right_leg.phi4 =
      -vmc_rad_format(vmc_probe_state.joint_right_ahead.position - VMC_ZERO_RIGHT_AHEAD);
  vmc_probe_state.right_leg.phi1 =
      -vmc_rad_format(vmc_probe_state.joint_right_back.position - VMC_ZERO_RIGHT_BACK);
  vmc_leg_position(vmc_probe_state.right_leg.phi1, vmc_probe_state.right_leg.phi4,
                   &length, &phi0);
  vmc_probe_state.right_leg.length = length;
  vmc_probe_state.right_leg.phi0 = phi0;
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
  vmc_can_init();

  for (;;) {
    vmc_can_poll_joints_zero();
    vmc_update_leg_state();
    osDelay(1);
  }
}
