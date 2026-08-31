#ifndef BOARD_COMM_H
#define BOARD_COMM_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_COMM_FRAME_SIZE       16U
#define BOARD_COMM_MAGIC0           0xA5U
#define BOARD_COMM_MAGIC1           0x5AU
#define BOARD_COMM_TYPE_HEARTBEAT  1U
#define BOARD_COMM_TYPE_RC         2U
#define BOARD_COMM_TYPE_STATE      3U
#define BOARD_COMM_TIMEOUT_MS      100U

#define BOARD_COMM_FLAG_RC_ONLINE        (1U << 0)
#define BOARD_COMM_FLAG_YAW_MOTOR_ONLINE (1U << 1)
#define BOARD_COMM_FLAG_GIMBAL_ENABLED   (1U << 2)
#define BOARD_COMM_FLAG_YAW_ACTIVE       (1U << 3)
#define BOARD_COMM_FLAG_CHASSIS_FALLEN   (1U << 4)
#define BOARD_COMM_FLAG_GIMBAL_ALLOWED   (1U << 5)

typedef struct {
  uint32_t rx_success_count;
  uint32_t rx_invalid_count;
  uint32_t tx_count;
  uint32_t tx_fail_count;
  uint32_t last_rx_ms;
  uint32_t last_tx_ms;
  uint32_t rc_last_rx_ms;
  uint32_t state_last_rx_ms;
  uint16_t last_rx_seq;
  uint16_t tx_seq;
  int16_t remote_ch[4];
  uint16_t yaw_ecd;
  int16_t yaw_speed_rpm;
  uint8_t remote_s1;
  uint8_t remote_s2;
  uint8_t remote_flags;
  uint8_t remote_online;
  uint8_t last_rx_type;
  uint8_t last_rx_data[BOARD_COMM_FRAME_SIZE];
  uint8_t online;
  uint8_t tx_busy;
} board_comm_state_t;

extern volatile board_comm_state_t board_comm_state;

void board_comm_init(void);
void board_comm_update(void);
uint8_t board_comm_remote_is_valid(void);
uint8_t board_comm_gimbal_state_is_valid(void);
void board_comm_on_uart_rx_event(UART_HandleTypeDef *huart, uint16_t size);
void board_comm_on_uart_error(UART_HandleTypeDef *huart);
void board_comm_on_uart_tx_complete(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_COMM_H */
