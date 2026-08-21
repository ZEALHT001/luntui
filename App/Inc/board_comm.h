#ifndef BOARD_COMM_H
#define BOARD_COMM_H

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_COMM_A_TO_C_STD_ID 0x310U
#define BOARD_COMM_C_TO_A_STD_ID 0x311U

typedef struct {
  uint32_t rx_success_count;
  uint32_t rx_invalid_count;
  uint32_t tx_count;
  uint32_t tx_fail_count;
  uint32_t last_rx_ms;
  uint32_t last_tx_ms;
  uint32_t last_rx_std_id;
  uint16_t last_rx_seq;
  uint16_t tx_seq;
  uint8_t last_rx_data[8];
  uint8_t online;
} board_comm_state_t;

extern volatile board_comm_state_t board_comm_state;

void board_comm_init(void);
void board_comm_update(void);
uint8_t board_comm_on_can_rx(CAN_HandleTypeDef *hcan,
                             uint32_t std_id,
                             const uint8_t data[8]);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_COMM_H */
