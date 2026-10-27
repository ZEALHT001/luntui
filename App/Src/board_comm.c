#include "board_comm.h"

#include "remote_control.h"
#include "vmc_probe.h"
#include "usart.h"

#include <math.h>
#include <string.h>

#define BOARD_COMM_UART          (&huart8)
#define BOARD_COMM_SRC           'A'
#define BOARD_COMM_DST           'C'
#define BOARD_COMM_EXPECTED_SRC  'C'
#define BOARD_COMM_EXPECTED_DST  'A'
#define BOARD_COMM_TX_PERIOD_MS  10U
#define BOARD_COMM_RX_DMA_SIZE   64U
#define BOARD_COMM_RC_LIMIT      700

volatile board_comm_state_t board_comm_state;

static uint8_t board_comm_tx_frame[BOARD_COMM_FRAME_SIZE];
static uint8_t board_comm_rx_dma[BOARD_COMM_RX_DMA_SIZE];
static uint8_t board_comm_rx_frame[BOARD_COMM_FRAME_SIZE];
static uint8_t board_comm_rx_index;

static uint8_t board_comm_checksum(const uint8_t *data, uint8_t length)
{
  uint8_t checksum = 0U;
  for (uint8_t i = 0U; i < length; i++) checksum ^= data[i];
  return checksum;
}

static int16_t board_comm_read_i16_le(const uint8_t *data, uint8_t offset)
{
  return (int16_t)((uint16_t)data[offset] | ((uint16_t)data[offset + 1U] << 8));
}

static uint8_t board_comm_switch_is_valid(uint8_t value)
{
  return (value == RC_SW_UP || value == RC_SW_MID || value == RC_SW_DOWN) ? 1U : 0U;
}

static uint8_t board_comm_payload_is_valid(uint8_t type, const uint8_t payload[8])
{
  if (type == BOARD_COMM_TYPE_RC) {
    for (uint8_t i = 0U; i < 4U; i++) {
      int16_t value = board_comm_read_i16_le(payload, (uint8_t)(i * 2U));
      if (value > BOARD_COMM_RC_LIMIT || value < -BOARD_COMM_RC_LIMIT) return 0U;
    }
  } else if (type == BOARD_COMM_TYPE_STATE) {
    if (board_comm_read_i16_le(payload, 0U) < 0 ||
        board_comm_read_i16_le(payload, 0U) > 8191) return 0U;
    if (board_comm_switch_is_valid(payload[4]) == 0U ||
        board_comm_switch_is_valid(payload[5]) == 0U) return 0U;
  } else if (type != BOARD_COMM_TYPE_HEARTBEAT) {
    return 0U;
  }
  return 1U;
}

static void board_comm_parse_frame(const uint8_t frame[BOARD_COMM_FRAME_SIZE])
{
  const uint8_t type = frame[4];
  const uint8_t *payload = &frame[7];

  if (frame[0] != BOARD_COMM_MAGIC0 || frame[1] != BOARD_COMM_MAGIC1 ||
      frame[2] != (uint8_t)BOARD_COMM_EXPECTED_SRC ||
      frame[3] != (uint8_t)BOARD_COMM_EXPECTED_DST ||
      frame[15] != board_comm_checksum(frame, 15U) ||
      board_comm_payload_is_valid(type, payload) == 0U) {
    board_comm_state.rx_invalid_count++;
    return;
  }

  memcpy((void *)board_comm_state.last_rx_data, frame, BOARD_COMM_FRAME_SIZE);
  board_comm_state.last_rx_type = type;
  board_comm_state.last_rx_seq = (uint16_t)frame[5] | ((uint16_t)frame[6] << 8);
  board_comm_state.last_rx_ms = HAL_GetTick();
  board_comm_state.rx_success_count++;
  board_comm_state.online = 1U;

  if (type == BOARD_COMM_TYPE_RC) {
    for (uint8_t i = 0U; i < 4U; i++) {
      board_comm_state.remote_ch[i] = board_comm_read_i16_le(payload, (uint8_t)(i * 2U));
    }
    board_comm_state.rc_last_rx_ms = board_comm_state.last_rx_ms;
  } else if (type == BOARD_COMM_TYPE_STATE) {
    board_comm_state.yaw_ecd = (uint16_t)board_comm_read_i16_le(payload, 0U);
    board_comm_state.yaw_speed_rpm = board_comm_read_i16_le(payload, 2U);
    board_comm_state.remote_s1 = payload[4];
    board_comm_state.remote_s2 = payload[5];
    board_comm_state.remote_flags = payload[6];
    board_comm_state.state_last_rx_ms = board_comm_state.last_rx_ms;
  }
  board_comm_state.remote_online = board_comm_remote_is_valid();
}

static void board_comm_feed_byte(uint8_t byte)
{
  if (board_comm_rx_index == 0U && byte != BOARD_COMM_MAGIC0) return;
  if (board_comm_rx_index == 1U && byte != BOARD_COMM_MAGIC1) {
    board_comm_rx_index = (byte == BOARD_COMM_MAGIC0) ? 1U : 0U;
    return;
  }
  board_comm_rx_frame[board_comm_rx_index++] = byte;
  if (board_comm_rx_index >= BOARD_COMM_FRAME_SIZE) {
    board_comm_parse_frame(board_comm_rx_frame);
    board_comm_rx_index = 0U;
  }
}

static HAL_StatusTypeDef board_comm_send_heartbeat(void)
{
  uint8_t payload[8] = {0};
  const uint32_t now = HAL_GetTick();
  uint8_t flags = 0U;

  /* Keep the gimbal inhibited for the complete self-righting sequence.  A
   * normal upright state is immediately allowed; after a fall, READY plus a
   * level body pitch is required before re-enabling the gimbal. */
  const uint8_t getup_active =
      (vmc_control.getup_state == VMC_GETUP_STATE_BRACE ||
       vmc_control.getup_state == VMC_GETUP_STATE_TURN ||
       vmc_control.getup_state == VMC_GETUP_STATE_SHRINK ||
       vmc_control.getup_state == VMC_GETUP_STATE_LEG_RECOVER ||
       vmc_control.getup_state == VMC_GETUP_STATE_UPRIGHT ||
       vmc_control.getup_state == VMC_GETUP_STATE_BODY_ALIGN ||
       vmc_control.getup_state == VMC_GETUP_STATE_BODY_FLIP ||
       vmc_control.getup_state == VMC_GETUP_STATE_BODY_FAN ||
       vmc_control.getup_lqr_release_pending != 0U) ? 1U : 0U;
  const uint8_t pitch_ready =
      (fabsf(vmc_probe_state.body.pitch) < VMC_GETUP_PITCH_READY_RAD) ? 1U : 0U;
  const uint8_t getup_required =
      (vmc_getup_fully_fallen != 0U || getup_active != 0U ||
       (vmc_control.getup_state == VMC_GETUP_STATE_READY && pitch_ready == 0U))
          ? 1U
          : 0U;

  if (getup_required != 0U) {
    flags |= BOARD_COMM_FLAG_CHASSIS_FALLEN;
  }
  if (getup_required == 0U ||
      (vmc_control.getup_state == VMC_GETUP_STATE_READY && pitch_ready != 0U)) {
    flags |= BOARD_COMM_FLAG_GIMBAL_ALLOWED;
  }
  memcpy(payload, &now, sizeof(now));
  payload[4] = flags;

  if (board_comm_state.tx_busy != 0U) return HAL_BUSY;
  board_comm_tx_frame[0] = BOARD_COMM_MAGIC0;
  board_comm_tx_frame[1] = BOARD_COMM_MAGIC1;
  board_comm_tx_frame[2] = (uint8_t)BOARD_COMM_SRC;
  board_comm_tx_frame[3] = (uint8_t)BOARD_COMM_DST;
  board_comm_tx_frame[4] = BOARD_COMM_TYPE_HEARTBEAT;
  board_comm_tx_frame[5] = (uint8_t)(board_comm_state.tx_seq & 0xFFU);
  board_comm_tx_frame[6] = (uint8_t)(board_comm_state.tx_seq >> 8);
  memcpy(&board_comm_tx_frame[7], payload, 8U);
  board_comm_tx_frame[15] = board_comm_checksum(board_comm_tx_frame, 15U);
  if (HAL_UART_Transmit_DMA(BOARD_COMM_UART, board_comm_tx_frame,
                            BOARD_COMM_FRAME_SIZE) != HAL_OK) return HAL_BUSY;
  board_comm_state.tx_busy = 1U;
  board_comm_state.tx_seq++;
  return HAL_OK;
}

void board_comm_init(void)
{
  memset((void *)&board_comm_state, 0, sizeof(board_comm_state));
  board_comm_rx_index = 0U;
  (void)HAL_UARTEx_ReceiveToIdle_DMA(BOARD_COMM_UART, board_comm_rx_dma,
                                     sizeof(board_comm_rx_dma));
  if (BOARD_COMM_UART->hdmarx != NULL) __HAL_DMA_DISABLE_IT(BOARD_COMM_UART->hdmarx, DMA_IT_HT);
}

void board_comm_update(void)
{
  const uint32_t now = HAL_GetTick();
  HAL_StatusTypeDef status;

  if (board_comm_state.last_rx_ms != 0U &&
      (now - board_comm_state.last_rx_ms) > BOARD_COMM_TIMEOUT_MS) {
    board_comm_state.online = 0U;
  }
  if (board_comm_state.rc_last_rx_ms != 0U &&
      (now - board_comm_state.rc_last_rx_ms) > BOARD_COMM_TIMEOUT_MS) {
    board_comm_state.remote_online = 0U;
  }
  if ((now - board_comm_state.last_tx_ms) < BOARD_COMM_TX_PERIOD_MS) return;

  status = board_comm_send_heartbeat();
  if (status == HAL_OK) {
    board_comm_state.tx_count++;
    board_comm_state.last_tx_ms = now;
  } else {
    board_comm_state.tx_fail_count++;
  }
}

uint8_t board_comm_remote_is_valid(void)
{
  const uint32_t now = HAL_GetTick();
  return (board_comm_state.rc_last_rx_ms != 0U &&
          board_comm_state.state_last_rx_ms != 0U &&
          (now - board_comm_state.rc_last_rx_ms) <= BOARD_COMM_TIMEOUT_MS &&
          (now - board_comm_state.state_last_rx_ms) <= BOARD_COMM_TIMEOUT_MS &&
          board_comm_switch_is_valid(board_comm_state.remote_s1) != 0U &&
          board_comm_switch_is_valid(board_comm_state.remote_s2) != 0U &&
          (board_comm_state.remote_flags & BOARD_COMM_FLAG_RC_ONLINE) != 0U) ? 1U : 0U;
}

uint8_t board_comm_gimbal_state_is_valid(void)
{
  const uint32_t now = HAL_GetTick();
  return (board_comm_state.state_last_rx_ms != 0U &&
          (now - board_comm_state.state_last_rx_ms) <= BOARD_COMM_TIMEOUT_MS &&
          (board_comm_state.remote_flags & BOARD_COMM_FLAG_YAW_MOTOR_ONLINE) != 0U) ? 1U : 0U;
}

void board_comm_on_uart_rx_event(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart != BOARD_COMM_UART) return;
  for (uint16_t i = 0U; i < size; i++) board_comm_feed_byte(board_comm_rx_dma[i]);
  (void)HAL_UARTEx_ReceiveToIdle_DMA(BOARD_COMM_UART, board_comm_rx_dma,
                                     sizeof(board_comm_rx_dma));
  if (BOARD_COMM_UART->hdmarx != NULL) __HAL_DMA_DISABLE_IT(BOARD_COMM_UART->hdmarx, DMA_IT_HT);
}

void board_comm_on_uart_error(UART_HandleTypeDef *huart)
{
  if (huart != BOARD_COMM_UART) return;
  board_comm_state.rx_invalid_count++;
  (void)HAL_UART_DMAStop(BOARD_COMM_UART);
  __HAL_UART_CLEAR_OREFLAG(BOARD_COMM_UART);
  __HAL_UART_CLEAR_NEFLAG(BOARD_COMM_UART);
  __HAL_UART_CLEAR_FEFLAG(BOARD_COMM_UART);
  __HAL_UART_CLEAR_PEFLAG(BOARD_COMM_UART);
  BOARD_COMM_UART->ErrorCode = HAL_UART_ERROR_NONE;
  BOARD_COMM_UART->RxState = HAL_UART_STATE_READY;
  (void)HAL_UARTEx_ReceiveToIdle_DMA(BOARD_COMM_UART, board_comm_rx_dma,
                                     sizeof(board_comm_rx_dma));
  if (BOARD_COMM_UART->hdmarx != NULL) __HAL_DMA_DISABLE_IT(BOARD_COMM_UART->hdmarx, DMA_IT_HT);
}

void board_comm_on_uart_tx_complete(UART_HandleTypeDef *huart)
{
  if (huart == BOARD_COMM_UART) board_comm_state.tx_busy = 0U;
}
