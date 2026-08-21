#include "board_comm.h"

#include <string.h>

#define BOARD_COMM_CAN             (&hcan2)
#define BOARD_COMM_TX_STD_ID       BOARD_COMM_A_TO_C_STD_ID
#define BOARD_COMM_RX_STD_ID       BOARD_COMM_C_TO_A_STD_ID
#define BOARD_COMM_SRC             'A'
#define BOARD_COMM_DST             'C'
#define BOARD_COMM_EXPECTED_SRC    'C'
#define BOARD_COMM_EXPECTED_DST    'A'
#define BOARD_COMM_MAGIC0          0xA5U
#define BOARD_COMM_MAGIC1          0x5AU
#define BOARD_COMM_TX_PERIOD_MS    10U
#define BOARD_COMM_TIMEOUT_MS      100U

volatile board_comm_state_t board_comm_state;

static uint8_t board_comm_checksum(const uint8_t data[8])
{
  uint8_t checksum = 0U;

  for (uint8_t i = 0U; i < 7U; i++) {
    checksum ^= data[i];
  }

  return checksum;
}

static uint8_t board_comm_frame_is_valid(const uint8_t data[8])
{
  return (data[0] == BOARD_COMM_MAGIC0 &&
          data[1] == BOARD_COMM_MAGIC1 &&
          data[2] == (uint8_t)BOARD_COMM_EXPECTED_SRC &&
          data[3] == (uint8_t)BOARD_COMM_EXPECTED_DST &&
          data[7] == board_comm_checksum(data))
             ? 1U
             : 0U;
}

void board_comm_init(void)
{
  memset((void *)&board_comm_state, 0, sizeof(board_comm_state));
}

void board_comm_update(void)
{
  const uint32_t now = HAL_GetTick();
  CAN_TxHeaderTypeDef header;
  uint8_t data[8];
  uint32_t mailbox;
  HAL_StatusTypeDef status;
  const uint16_t seq = board_comm_state.tx_seq;

  if (board_comm_state.last_rx_ms != 0U &&
      (now - board_comm_state.last_rx_ms) > BOARD_COMM_TIMEOUT_MS) {
    board_comm_state.online = 0U;
  }

  if ((now - board_comm_state.last_tx_ms) < BOARD_COMM_TX_PERIOD_MS) {
    return;
  }

  data[0] = BOARD_COMM_MAGIC0;
  data[1] = BOARD_COMM_MAGIC1;
  data[2] = (uint8_t)BOARD_COMM_SRC;
  data[3] = (uint8_t)BOARD_COMM_DST;
  data[4] = (uint8_t)(seq & 0xFFU);
  data[5] = (uint8_t)(seq >> 8);
  data[6] = (uint8_t)(now & 0xFFU);
  data[7] = board_comm_checksum(data);

  header.StdId = BOARD_COMM_TX_STD_ID;
  header.ExtId = 0U;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8U;
  header.TransmitGlobalTime = DISABLE;

  status = HAL_CAN_AddTxMessage(BOARD_COMM_CAN, &header, data, &mailbox);
  if (status == HAL_OK) {
    board_comm_state.tx_count++;
    board_comm_state.tx_seq++;
    board_comm_state.last_tx_ms = now;
  } else {
    board_comm_state.tx_fail_count++;
  }
}

uint8_t board_comm_on_can_rx(CAN_HandleTypeDef *hcan,
                             uint32_t std_id,
                             const uint8_t data[8])
{
  if (hcan->Instance != BOARD_COMM_CAN->Instance || std_id != BOARD_COMM_RX_STD_ID) {
    return 0U;
  }

  board_comm_state.last_rx_std_id = std_id;
  memcpy((void *)board_comm_state.last_rx_data, data, sizeof(board_comm_state.last_rx_data));

  if (board_comm_frame_is_valid(data) != 0U) {
    board_comm_state.rx_success_count++;
    board_comm_state.last_rx_seq = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    board_comm_state.last_rx_ms = HAL_GetTick();
    board_comm_state.online = 1U;
  } else {
    board_comm_state.rx_invalid_count++;
  }

  return 1U;
}
