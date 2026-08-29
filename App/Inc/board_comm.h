#ifndef BOARD_COMM_H
#define BOARD_COMM_H

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_COMM_A_TO_C_STD_ID 0x310U
#define BOARD_COMM_C_TO_A_RC_STD_ID 0x311U
#define BOARD_COMM_C_TO_A_STATE_STD_ID 0x312U
#define BOARD_COMM_C_TO_A_STD_ID BOARD_COMM_C_TO_A_RC_STD_ID

#define BOARD_COMM_FLAG_RC_ONLINE        (1U << 0)
#define BOARD_COMM_FLAG_YAW_MOTOR_ONLINE (1U << 1)
#define BOARD_COMM_FLAG_GIMBAL_ENABLED   (1U << 2)
#define BOARD_COMM_FLAG_YAW_ACTIVE       (1U << 3)
#define BOARD_COMM_TIMEOUT_MS 100U

typedef struct {
  uint32_t rx_success_count;
  uint32_t rx_invalid_count;
  uint32_t tx_count;
  uint32_t tx_fail_count;
  uint32_t last_rx_ms;
  uint32_t last_tx_ms;
  uint32_t rc_last_rx_ms;
  uint32_t state_last_rx_ms;
  uint32_t last_rx_std_id;
  uint16_t last_rx_seq;
  uint16_t tx_seq;
  int16_t remote_ch[4];
  uint16_t yaw_ecd;
  int16_t yaw_speed_rpm;
  uint8_t remote_s1;
  uint8_t remote_s2;
  uint8_t remote_flags;
  uint8_t remote_online;
  uint8_t last_rx_data[8];
  uint8_t online;
} board_comm_state_t;

extern volatile board_comm_state_t board_comm_state;

void board_comm_init(void);
void board_comm_update(void);
uint8_t board_comm_remote_is_valid(void);
uint8_t board_comm_gimbal_state_is_valid(void);
uint8_t board_comm_on_can_rx(CAN_HandleTypeDef *hcan,
                             uint32_t std_id,
                             const uint8_t data[8]);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_COMM_H */
