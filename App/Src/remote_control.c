#include "remote_control.h"

#include "board_comm.h"
#include "hi12_imu.h"
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

static void sbus_to_remote(const uint8_t *sbus_buf)
{
  remote_control_t rc;

  memset(&rc, 0, sizeof(rc));

  rc.ch[0] = (int16_t)((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07FFU);
  rc.ch[1] = (int16_t)(((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07FFU);
  rc.ch[2] = (int16_t)(((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |
                        (sbus_buf[4] << 10)) &
                       0x07FFU);
  rc.ch[3] = (int16_t)(((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07FFU);
  rc.s[0] = (uint8_t)((sbus_buf[5] >> 4) & 0x03U);
  rc.s[1] = (uint8_t)(((sbus_buf[5] >> 4) & 0x0CU) >> 2);
  rc.mouse_x = (int16_t)(sbus_buf[6] | (sbus_buf[7] << 8));
  rc.mouse_y = (int16_t)(sbus_buf[8] | (sbus_buf[9] << 8));
  rc.mouse_z = (int16_t)(sbus_buf[10] | (sbus_buf[11] << 8));
  rc.mouse_l = sbus_buf[12];
  rc.mouse_r = sbus_buf[13];
  rc.key = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
  rc.ch[4] = (int16_t)(sbus_buf[16] | (sbus_buf[17] << 8));

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
  if (board_comm_remote_is_valid() != 0U) {
    selected.ch[0] = board_comm_state.remote_ch[0];
    selected.ch[1] = board_comm_state.remote_ch[1];
    selected.ch[2] = board_comm_state.remote_ch[2];
    selected.ch[3] = board_comm_state.remote_ch[3];
    /* Keep the native DBUS layout used throughout A-board firmware:
     * s[0] is physical S2 and s[1] is physical S1. */
    selected.s[0] = board_comm_state.remote_s2;
    selected.s[1] = board_comm_state.remote_s1;
    selected.rx_count = board_comm_state.rx_success_count;
    selected.valid = 1U;
    selected.last_update_ms = board_comm_state.state_last_rx_ms;
    remote_control_source = RC_SOURCE_CBOARD;
  } else if (remote_control_local.valid != 0U &&
             remote_control_local.rx_count != 0U &&
             (now - remote_control_local.last_update_ms) <= RC_LOST_TIMEOUT_MS) {
    selected = remote_control_local;
    remote_control_source = RC_SOURCE_LOCAL;
  } else {
    remote_control_source = RC_SOURCE_NONE;
  }

  remote_control = selected;
}

uint8_t remote_control_is_s1_down(void)
{
  return (remote_control.valid != 0U && remote_control.s[1] == RC_SW_DOWN) ? 1U : 0U;
}

uint8_t remote_control_allows_joint_enable(void)
{
  if (remote_control.valid == 0U || remote_control.rx_count == 0U) {
    return 0U;
  }

  if ((HAL_GetTick() - remote_control.last_update_ms) > RC_LOST_TIMEOUT_MS) {
    remote_control.valid = 0U;
    return 0U;
  }

  return (remote_control.s[1] != RC_SW_DOWN) ? 1U : 0U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart1) {
    if (Size >= RC_FRAME_LENGTH) {
      sbus_to_remote(sbus_rx_buf);
    } else {
      remote_control_local.rx_error_count++;
    }
    remote_control_restart_rx();
  } else if (huart == &huart6) {
    hi12_imu_on_uart_rx_event(huart, Size);
  }
}
