#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC_FRAME_LENGTH 18U
#define SBUS_RX_BUF_NUM 36U
#define RC_CH_VALUE_OFFSET 1024
#define RC_SW_UP 1U
#define RC_SW_MID 3U
#define RC_SW_DOWN 2U
#define RC_CHANNAL_ERROR_VALUE 700
#define RC_LOST_TIMEOUT_MS 100U

typedef struct {
  int16_t ch[5];
  uint8_t s[2];
  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  uint8_t mouse_l;
  uint8_t mouse_r;
  uint16_t key;
  uint32_t rx_count;
  uint32_t rx_error_count;
  uint8_t valid;
  uint32_t last_update_ms;
} remote_control_t;

extern volatile remote_control_t remote_control;

void remote_control_init(void);
uint8_t remote_control_is_s1_down(void);
uint8_t remote_control_allows_joint_enable(void);

#ifdef __cplusplus
}
#endif

#endif
