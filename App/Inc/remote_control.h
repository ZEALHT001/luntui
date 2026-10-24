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
#define RC_SOURCE_NONE  0U
#define RC_SOURCE_LOCAL 1U
#define RC_SOURCE_CBOARD 2U
#define RC_SOURCE_SBUS 3U
#define RC_NORMALIZED_CHANNEL_COUNT 10U

/* New FS-i6S SBUS layout.  Indices match sbus_remote_data.normalized[].
 * 0/1 are chassis X/Y, 2/3 are gimbal X/Y, 4 is master start-stop, 5 is spin,
 * 6 is jump mode, 7 is stair enable, 8 is speed scale and 9 is leg length. */
#define RC_CHASSIS_X 0U
#define RC_CHASSIS_Y 1U
#define RC_GIMBAL_X 2U
#define RC_GIMBAL_Y 3U
#define RC_START_STOP 4U
#define RC_SPIN_MODE 5U
#define RC_JUMP_MODE 6U
#define RC_STAIR_ENABLE 7U
#define RC_SPEED_LIMIT 8U
#define RC_LEG_LENGTH 9U

#define RC_JUMP_DISABLED 0U
#define RC_JUMP_AUTO 1U
#define RC_JUMP_MANUAL 2U
#define RC_SWITCH_ON_THRESHOLD 0.5f
#define RC_SWITCH_CENTER_HALF_RANGE 0.35f

typedef struct {
  int16_t ch[5];
  uint8_t s[2];
  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  uint8_t mouse_l;
  uint8_t mouse_r;
  uint16_t key;
  float normalized[RC_NORMALIZED_CHANNEL_COUNT];
  uint32_t rx_count;
  uint32_t rx_error_count;
  uint8_t valid;
  uint32_t last_update_ms;
} remote_control_t;

extern volatile remote_control_t remote_control;
extern volatile uint8_t remote_control_source;

void remote_control_init(void);
void remote_control_update_source(void);
uint8_t remote_control_is_s1_down(void);
uint8_t remote_control_allows_joint_enable(void);
float remote_control_channel(uint8_t channel);
uint8_t remote_control_is_enabled(void);
uint8_t remote_control_spin_mode_enabled(void);
uint8_t remote_control_jump_mode(void);
uint8_t remote_control_stair_enabled(void);
float remote_control_max_speed_scale(void);
float remote_control_leg_length_position(void);

#ifdef __cplusplus
}
#endif

#endif
