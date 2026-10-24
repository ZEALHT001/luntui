#include "sbus_remote.h"

#ifdef SBUS_REMOTE_HOST_TEST
uint32_t sbus_remote_test_tick_ms(void);
#define SBUS_REMOTE_TICK_MS() sbus_remote_test_tick_ms()
#else
#include "main.h"
#define SBUS_REMOTE_TICK_MS() HAL_GetTick()
#endif

#include <string.h>

#define SBUS_REMOTE_HEADER          0x0FU
#define SBUS_REMOTE_FOOTER          0x00U
#define SBUS_REMOTE_FLAG_FRAME_LOST 0x04U
#define SBUS_REMOTE_FLAG_FAILSAFE   0x08U
#define SBUS_REMOTE_VALUE_MIN       240U
#define SBUS_REMOTE_VALUE_MID       1023U
#define SBUS_REMOTE_VALUE_MAX       1807U
#define SBUS_REMOTE_CHANNEL_BITS    11U

volatile sbus_remote_data_t sbus_remote_data;

static uint16_t sbus_remote_extract_channel(const uint8_t *frame, uint8_t channel)
{
  const uint16_t bit_offset = (uint16_t)channel * SBUS_REMOTE_CHANNEL_BITS;
  const uint8_t byte_offset = (uint8_t)(1U + (bit_offset / 8U));
  const uint8_t bit_shift = (uint8_t)(bit_offset % 8U);
  const uint32_t word = (uint32_t)frame[byte_offset] |
                        ((uint32_t)frame[byte_offset + 1U] << 8) |
                        ((uint32_t)frame[byte_offset + 2U] << 16);

  return (uint16_t)((word >> bit_shift) & 0x07FFU);
}

static float sbus_remote_normalize(uint16_t value)
{
  float normalized;

  if (value <= SBUS_REMOTE_VALUE_MID) {
    normalized = -(float)(SBUS_REMOTE_VALUE_MID - value) /
                 (float)(SBUS_REMOTE_VALUE_MID - SBUS_REMOTE_VALUE_MIN);
  } else {
    normalized = (float)(value - SBUS_REMOTE_VALUE_MID) /
                 (float)(SBUS_REMOTE_VALUE_MAX - SBUS_REMOTE_VALUE_MID);
  }

  if (normalized < -1.0f) {
    return -1.0f;
  }
  if (normalized > 1.0f) {
    return 1.0f;
  }
  return normalized;
}

void sbus_remote_init(void)
{
  memset((void *)&sbus_remote_data, 0, sizeof(sbus_remote_data));
}

void sbus_remote_update(void)
{
  if (sbus_remote_data.connected != 0U &&
      (SBUS_REMOTE_TICK_MS() - sbus_remote_data.last_frame_ms) >
          SBUS_REMOTE_TIMEOUT_MS) {
    sbus_remote_data.connected = 0U;
    sbus_remote_data.valid = 0U;
    sbus_remote_data.startup_samples = 0U;
    sbus_remote_data.startup_since_ms = 0U;
    sbus_remote_data.recovery_samples = 0U;
    sbus_remote_data.recovery_since_ms = 0U;
    sbus_remote_data.recovery_hold_ms = 0U;
    sbus_remote_data.timeout_count++;
  }
}

static uint8_t sbus_remote_startup_control_reason(void)
{
  uint8_t reason = 0U;

  for (uint8_t i = 0U; i <= 3U; i++) {
    const float value = sbus_remote_data.normalized[i];
    if (value < -SBUS_REMOTE_STARTUP_TOLERANCE ||
        value > SBUS_REMOTE_STARTUP_TOLERANCE) {
      reason |= SBUS_REMOTE_STARTUP_BAD_STICK;
    }
  }

  for (uint8_t i = 4U; i <= 9U; i++) {
    if (sbus_remote_data.normalized[i] >
        (-1.0f + SBUS_REMOTE_STARTUP_TOLERANCE)) {
      reason |= SBUS_REMOTE_STARTUP_BAD_CONTROL;
    }
  }

  return reason;
}

static void sbus_remote_update_startup_gate(uint8_t frame_healthy, uint32_t now)
{
  if (sbus_remote_data.startup_checked != 0U &&
      sbus_remote_data.startup_rejected == 0U) {
    sbus_remote_data.valid =
        (frame_healthy != 0U) ? 1U : 0U;
    return;
  }

  if (sbus_remote_data.startup_rejected != 0U) {
    const uint8_t reason =
        (frame_healthy != 0U) ? sbus_remote_startup_control_reason() : 0xFFU;

    if (reason != 0U) {
      sbus_remote_data.recovery_since_ms = 0U;
      sbus_remote_data.recovery_samples = 0U;
      sbus_remote_data.recovery_hold_ms = 0U;
      sbus_remote_data.valid = 0U;
      return;
    }

    if (sbus_remote_data.recovery_samples == 0U) {
      sbus_remote_data.recovery_since_ms = now;
    }
    if (sbus_remote_data.recovery_samples < UINT32_MAX) {
      sbus_remote_data.recovery_samples++;
    }
    sbus_remote_data.recovery_hold_ms =
        now - sbus_remote_data.recovery_since_ms;
    if (sbus_remote_data.recovery_hold_ms >
            SBUS_REMOTE_RECOVERY_HOLD_MS &&
        sbus_remote_data.recovery_samples >= 5U) {
      sbus_remote_data.startup_rejected = 0U;
      sbus_remote_data.startup_checked = 1U;
      sbus_remote_data.startup_reject_reason = 0U;
      sbus_remote_data.valid = 1U;
    } else {
      sbus_remote_data.valid = 0U;
    }
    return;
  }

  if (frame_healthy == 0U) {
    sbus_remote_data.startup_samples = 0U;
    sbus_remote_data.startup_since_ms = 0U;
    sbus_remote_data.valid = 0U;
    return;
  }

  {
    const uint8_t reason = sbus_remote_startup_control_reason();
    if (reason != 0U) {
      sbus_remote_data.startup_rejected = 1U;
      sbus_remote_data.startup_reject_reason |= reason;
      sbus_remote_data.recovery_since_ms = 0U;
      sbus_remote_data.recovery_samples = 0U;
      sbus_remote_data.recovery_hold_ms = 0U;
      sbus_remote_data.valid = 0U;
      return;
    }
  }

  if (sbus_remote_data.startup_samples == 0U) {
    sbus_remote_data.startup_since_ms = now;
  }
  if (sbus_remote_data.startup_samples < UINT32_MAX) {
    sbus_remote_data.startup_samples++;
  }

  if ((now - sbus_remote_data.startup_since_ms) >=
          SBUS_REMOTE_STARTUP_HOLD_MS &&
      sbus_remote_data.startup_samples >= 5U) {
    sbus_remote_data.startup_checked = 1U;
    sbus_remote_data.valid = 1U;
  } else {
    sbus_remote_data.valid = 0U;
  }
}

uint8_t sbus_remote_handle_frame(const uint8_t *frame, uint16_t size)
{
  const uint32_t now = SBUS_REMOTE_TICK_MS();
  uint8_t flags;
  uint8_t frame_healthy;

  if (size != SBUS_REMOTE_FRAME_SIZE) {
    return 0U;
  }

  sbus_remote_data.last_frame_size = size;
  sbus_remote_data.last_header = frame[0];
  sbus_remote_data.last_footer = frame[SBUS_REMOTE_FRAME_SIZE - 1U];
  if (frame[0] != SBUS_REMOTE_HEADER ||
      frame[SBUS_REMOTE_FRAME_SIZE - 1U] != SBUS_REMOTE_FOOTER) {
    sbus_remote_data.frame_error_count++;
    return 1U;
  }

  flags = frame[23];
  for (uint8_t i = 0U; i < SBUS_REMOTE_CHANNEL_COUNT; i++) {
    const uint16_t value = sbus_remote_extract_channel(frame, i);
    sbus_remote_data.channels[i] = value;
    sbus_remote_data.normalized[i] = sbus_remote_normalize(value);
  }

  sbus_remote_data.last_flags = flags;
  sbus_remote_data.last_frame_ms = now;
  sbus_remote_data.rx_count++;

  if ((flags & SBUS_REMOTE_FLAG_FAILSAFE) != 0U) {
    sbus_remote_data.failsafe = 1U;
    sbus_remote_data.frame_lost = 0U;
    sbus_remote_data.failsafe_count++;
    sbus_remote_data.connected = 0U;
    sbus_remote_data.valid = 0U;
  } else {
    sbus_remote_data.failsafe = 0U;
    sbus_remote_data.frame_lost =
        ((flags & SBUS_REMOTE_FLAG_FRAME_LOST) != 0U) ? 1U : 0U;
    if (sbus_remote_data.frame_lost != 0U) {
      sbus_remote_data.frame_lost_count++;
    }
    sbus_remote_data.connected = 1U;
  }

  frame_healthy = (sbus_remote_data.failsafe == 0U &&
                   sbus_remote_data.frame_lost == 0U)
                      ? 1U
                      : 0U;
  if (sbus_remote_data.failsafe != 0U &&
      sbus_remote_data.startup_checked == 0U) {
    sbus_remote_data.startup_rejected = 1U;
    sbus_remote_data.startup_reject_reason |= SBUS_REMOTE_STARTUP_BAD_FRAME;
  }
  sbus_remote_update_startup_gate(frame_healthy, now);

  return 1U;
}
