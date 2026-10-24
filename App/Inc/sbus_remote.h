#ifndef SBUS_REMOTE_H
#define SBUS_REMOTE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SBUS_REMOTE_FRAME_SIZE      25U
#define SBUS_REMOTE_CHANNEL_COUNT   16U
#define SBUS_REMOTE_TIMEOUT_MS      100U
#define SBUS_REMOTE_STARTUP_HOLD_MS 200U
#define SBUS_REMOTE_RECOVERY_HOLD_MS 5000U
#define SBUS_REMOTE_STARTUP_TOLERANCE 0.03f
#define SBUS_REMOTE_SPEED_CHANNEL 8U

#define SBUS_REMOTE_STARTUP_BAD_STICK    (1U << 0)
#define SBUS_REMOTE_STARTUP_BAD_CONTROL  (1U << 1)
#define SBUS_REMOTE_STARTUP_BAD_FRAME    (1U << 2)

/* Raw SBUS state plus the power-on safety gate.  remote_control copies these
 * channels only after startup_checked is set and valid remains true. */
typedef struct {
  uint16_t channels[SBUS_REMOTE_CHANNEL_COUNT];
  float normalized[SBUS_REMOTE_CHANNEL_COUNT];
  uint32_t rx_count;
  uint32_t frame_error_count;
  uint32_t timeout_count;
  uint32_t failsafe_count;
  uint32_t frame_lost_count;
  uint32_t last_frame_ms;
  uint32_t startup_since_ms;
  uint32_t startup_samples;
  uint32_t recovery_since_ms;
  uint32_t recovery_samples;
  uint32_t recovery_hold_ms;
  uint16_t last_frame_size;
  uint8_t last_header;
  uint8_t last_flags;
  uint8_t last_footer;
  uint8_t connected;
  uint8_t valid;
  uint8_t startup_checked;
  uint8_t startup_rejected;
  uint8_t startup_reject_reason;
  uint8_t failsafe;
  uint8_t frame_lost;
} sbus_remote_data_t;

extern volatile sbus_remote_data_t sbus_remote_data;

void sbus_remote_init(void);
void sbus_remote_update(void);

/* Returns 1 when the received block is a complete 25-byte SBUS frame and is
 * therefore consumed by the SBUS parser.  Returns 0 for the existing
 * 18-byte DBUS path, which is left untouched. */
uint8_t sbus_remote_handle_frame(const uint8_t *frame, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* SBUS_REMOTE_H */
