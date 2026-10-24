#include "remote_control.h"

#include "board_comm.h"
#include "hi12_imu.h"
#include "sbus_remote.h"
#include "tf02_pro.h"
#include "usart.h"

#include <string.h>

volatile remote_control_t remote_control;
volatile uint8_t remote_control_source;
static volatile remote_control_t remote_control_local;
static uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];

static int16_t rc_abs(int16_t value)
{
  return (value < 0) ? (int16_t)-value : value;
}

static uint8_t rc_switch_is_valid(uint8_t value)
{
  return (value == RC_SW_UP || value == RC_SW_MID || value == RC_SW_DOWN) ? 1U : 0U;
}

static uint8_t remote_frame_is_valid(const remote_control_t *rc)
{
  if (rc_switch_is_valid(rc->s[0]) == 0U || rc_switch_is_valid(rc->s[1]) == 0U) {
    return 0U;
  }

  for (uint8_t i = 0U; i < 5U; i++) {
    if (rc_abs(rc->ch[i]) > RC_CHANNAL_ERROR_VALUE) {
      return 0U;
    }
  }

  return 1U;
}

static void dbus_to_remote(const uint8_t *dbus_buf)
{
  remote_control_t rc;

  memset(&rc, 0, sizeof(rc));
  rc.ch[0] = (int16_t)((dbus_buf[0] | (dbus_buf[1] << 8)) & 0x07FFU);
  rc.ch[1] = (int16_t)(((dbus_buf[1] >> 3) | (dbus_buf[2] << 5)) & 0x07FFU);
  rc.ch[2] = (int16_t)(((dbus_buf[2] >> 6) | (dbus_buf[3] << 2) |
                        (dbus_buf[4] << 10)) &
                       0x07FFU);
  rc.ch[3] = (int16_t)(((dbus_buf[4] >> 1) | (dbus_buf[5] << 7)) & 0x07FFU);
  rc.s[0] = (uint8_t)((dbus_buf[5] >> 4) & 0x03U);
  rc.s[1] = (uint8_t)(((dbus_buf[5] >> 4) & 0x0CU) >> 2);
  rc.mouse_x = (int16_t)(dbus_buf[6] | (dbus_buf[7] << 8));
  rc.mouse_y = (int16_t)(dbus_buf[8] | (dbus_buf[9] << 8));
  rc.mouse_z = (int16_t)(dbus_buf[10] | (dbus_buf[11] << 8));
  rc.mouse_l = dbus_buf[12];
  rc.mouse_r = dbus_buf[13];
  rc.key = (uint16_t)(dbus_buf[14] | (dbus_buf[15] << 8));
  rc.ch[4] = (int16_t)(dbus_buf[16] | (dbus_buf[17] << 8));

  rc.ch[0] -= RC_CH_VALUE_OFFSET;
  rc.ch[1] -= RC_CH_VALUE_OFFSET;
  rc.ch[2] -= RC_CH_VALUE_OFFSET;
  rc.ch[3] -= RC_CH_VALUE_OFFSET;
  rc.ch[4] -= RC_CH_VALUE_OFFSET;

  if (remote_frame_is_valid(&rc) == 0U) {
    remote_control_local.valid = 0U;
    remote_control_local.rx_error_count++;
    return;
  }

  rc.rx_count = remote_control_local.rx_count + 1U;
  rc.rx_error_count = remote_control_local.rx_error_count;
  rc.valid = 1U;
  rc.last_update_ms = HAL_GetTick();
  remote_control_local = rc;
}

static void remote_control_restart_rx(void)
{
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, sbus_rx_buf, SBUS_RX_BUF_NUM);
  if (huart1.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

void remote_control_init(void)
{
  memset((void *)&remote_control, 0, sizeof(remote_control));
  memset((void *)&remote_control_local, 0, sizeof(remote_control_local));
  memset(sbus_rx_buf, 0, sizeof(sbus_rx_buf));
  remote_control_source = RC_SOURCE_NONE;
  remote_control_restart_rx();
}

void remote_control_update_source(void)
{
  const uint32_t now = HAL_GetTick();
  remote_control_t selected;

  memset(&selected, 0, sizeof(selected));
  if (sbus_remote_data.valid != 0U && sbus_remote_data.connected != 0U &&
      (now - sbus_remote_data.last_frame_ms) <= RC_LOST_TIMEOUT_MS) {
    for (uint8_t i = 0U; i < RC_NORMALIZED_CHANNEL_COUNT; i++) {
      selected.normalized[i] = sbus_remote_data.normalized[i];
    }

    /* Keep the legacy fields populated for RTT and older consumers. */
    selected.ch[0] = (int16_t)(selected.normalized[RC_GIMBAL_X] *
                               RC_CHANNAL_ERROR_VALUE);
    selected.ch[1] = (int16_t)(selected.normalized[RC_GIMBAL_Y] *
                               RC_CHANNAL_ERROR_VALUE);
    selected.ch[2] = (int16_t)(-selected.normalized[RC_CHASSIS_X] *
                               RC_CHANNAL_ERROR_VALUE);
    selected.ch[3] = (int16_t)(selected.normalized[RC_CHASSIS_Y] *
                               RC_CHANNAL_ERROR_VALUE);
    selected.s[1] = (selected.normalized[RC_START_STOP] >
                     RC_SWITCH_ON_THRESHOLD)
                        ? RC_SW_MID
                        : RC_SW_DOWN;
    selected.s[0] = (selected.normalized[RC_LEG_LENGTH] > 0.35f)
                        ? RC_SW_UP
                        : ((selected.normalized[RC_LEG_LENGTH] < -0.35f)
                               ? RC_SW_DOWN
                               : RC_SW_MID);
    selected.rx_count = sbus_remote_data.rx_count;
    selected.rx_error_count = sbus_remote_data.frame_error_count;
    selected.valid = 1U;
    selected.last_update_ms = sbus_remote_data.last_frame_ms;
    remote_control_source = RC_SOURCE_SBUS;
  } else {
    remote_control_source = RC_SOURCE_NONE;
  }
  remote_control = selected;
}

uint8_t remote_control_is_s1_down(void)
{
  return (remote_control_is_enabled() == 0U) ? 1U : 0U;
}

uint8_t remote_control_allows_joint_enable(void)
{
  if (remote_control.valid == 0U || remote_control.rx_count == 0U ||
      remote_control_source != RC_SOURCE_SBUS) {
    return 0U;
  }

  if ((HAL_GetTick() - remote_control.last_update_ms) > RC_LOST_TIMEOUT_MS) {
    return 0U;
  }

  return remote_control_is_enabled();
}

float remote_control_channel(uint8_t channel)
{
  if (remote_control.valid == 0U || channel >= RC_NORMALIZED_CHANNEL_COUNT) {
    return 0.0f;
  }
  return remote_control.normalized[channel];
}

uint8_t remote_control_is_enabled(void)
{
  if (remote_control.valid == 0U || remote_control.rx_count == 0U ||
      remote_control_source != RC_SOURCE_SBUS ||
      (HAL_GetTick() - remote_control.last_update_ms) > RC_LOST_TIMEOUT_MS) {
    return 0U;
  }
  return (remote_control.normalized[RC_START_STOP] > RC_SWITCH_ON_THRESHOLD)
             ? 1U
             : 0U;
}

uint8_t remote_control_spin_mode_enabled(void)
{
  return (remote_control_is_enabled() != 0U &&
          remote_control.normalized[RC_SPIN_MODE] > RC_SWITCH_ON_THRESHOLD)
             ? 1U
             : 0U;
}

uint8_t remote_control_jump_mode(void)
{
  const float value = remote_control.normalized[RC_JUMP_MODE];

  if (remote_control_is_enabled() == 0U) {
    return RC_JUMP_DISABLED;
  }
  if (value >= -RC_SWITCH_CENTER_HALF_RANGE &&
      value <= RC_SWITCH_CENTER_HALF_RANGE) {
    return RC_JUMP_AUTO;
  }
  return (value >= RC_SWITCH_ON_THRESHOLD) ? RC_JUMP_MANUAL : RC_JUMP_DISABLED;
}

uint8_t remote_control_stair_enabled(void)
{
  return (remote_control_is_enabled() != 0U &&
          remote_control.normalized[RC_STAIR_ENABLE] > RC_SWITCH_ON_THRESHOLD)
             ? 1U
             : 0U;
}

float remote_control_max_speed_scale(void)
{
  float scale;

  if (remote_control_is_enabled() == 0U) {
    return 0.0f;
  }
  scale = (remote_control.normalized[RC_SPEED_LIMIT] + 1.0f) * 0.5f;
  if (scale < 0.0f) {
    return 0.0f;
  }
  if (scale > 1.0f) {
    return 1.0f;
  }
  return scale;
}

float remote_control_leg_length_position(void)
{
  if (remote_control.valid == 0U || remote_control_source != RC_SOURCE_SBUS) {
    return -1.0f;
  }
  return remote_control.normalized[RC_LEG_LENGTH];
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart8) {
    board_comm_on_uart_rx_event(huart, Size);
  } else if (huart == &huart1) {
    if (sbus_remote_handle_frame(sbus_rx_buf, Size) != 0U) {
      /* SBUS is copied into remote_control by remote_control_update_source(). */
    } else if (Size >= RC_FRAME_LENGTH) {
      dbus_to_remote(sbus_rx_buf);
    } else {
      remote_control_local.rx_error_count++;
    }
    remote_control_restart_rx();
  } else if (huart == &huart6) {
    hi12_imu_on_uart_rx_event(huart, Size);
  } else if (huart == &huart3) {
    tf02_pro_on_uart_rx_event(huart, Size);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart8) {
    board_comm_on_uart_error(huart);
  } else if (huart == &huart1) {
    remote_control_restart_rx();
  } else if (huart == &huart6) {
    hi12_imu_init();
  } else if (huart == &huart3) {
    tf02_pro_on_uart_error(huart);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  board_comm_on_uart_tx_complete(huart);
}
