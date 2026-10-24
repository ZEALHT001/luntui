/*
 * Host regression test for the SBUS startup safety gate.
 *
 * The STM32 build gets time from HAL_GetTick(); this test compiles
 * sbus_remote.c with SBUS_REMOTE_HOST_TEST and supplies a controllable clock.
 *
 * Build and run through:
 *   python3 tools/test_sbus_remote.py
 */

#include "sbus_remote.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t test_tick_ms;
static int failures;

uint32_t sbus_remote_test_tick_ms(void)
{
  return test_tick_ms;
}

static void check(int condition, const char *message)
{
  if (condition == 0) {
    printf("FAIL: %s\n", message);
    failures++;
  }
}

static void pack_frame(uint8_t frame[SBUS_REMOTE_FRAME_SIZE],
                       const uint16_t channels[SBUS_REMOTE_CHANNEL_COUNT],
                       uint8_t flags)
{
  memset(frame, 0, SBUS_REMOTE_FRAME_SIZE);
  frame[0] = 0x0FU;
  for (uint8_t channel = 0U; channel < SBUS_REMOTE_CHANNEL_COUNT; channel++) {
    const uint16_t value = channels[channel] & 0x07FFU;
    for (uint8_t bit = 0U; bit < 11U; bit++) {
      if ((value & (uint16_t)(1U << bit)) != 0U) {
        const uint8_t position = (uint8_t)(channel * 11U + bit);
        frame[1U + position / 8U] |= (uint8_t)(1U << (position % 8U));
      }
    }
  }
  frame[23] = flags;
  frame[24] = 0x00U;
}

static void setup_safe_channels(
    uint16_t channels[SBUS_REMOTE_CHANNEL_COUNT])
{
  channels[0] = 1023U;
  channels[1] = 1023U;
  channels[2] = 1023U;
  channels[3] = 1023U;
  for (uint8_t i = 4U; i <= 9U; i++) {
    channels[i] = 240U;
  }
  for (uint8_t i = 10U; i < SBUS_REMOTE_CHANNEL_COUNT; i++) {
    channels[i] = 1023U;
  }
}

static void feed_safe_frame(
    uint8_t frame[SBUS_REMOTE_FRAME_SIZE],
    const uint16_t channels[SBUS_REMOTE_CHANNEL_COUNT], uint8_t flags)
{
  pack_frame(frame, channels, flags);
  sbus_remote_handle_frame(frame, SBUS_REMOTE_FRAME_SIZE);
}

static void test_good_startup_and_recoverable_runtime_fault(void)
{
  uint8_t frame[SBUS_REMOTE_FRAME_SIZE];
  uint16_t channels[SBUS_REMOTE_CHANNEL_COUNT];

  sbus_remote_init();
  setup_safe_channels(channels);
  test_tick_ms = 0U;
  for (uint8_t i = 0U; i < 5U; i++) {
    feed_safe_frame(frame, channels, 0U);
    test_tick_ms += 50U;
  }
  check(sbus_remote_data.startup_checked == 1U,
        "safe posture should pass startup");
  check(sbus_remote_data.valid == 1U, "startup should validate control data");

  feed_safe_frame(frame, channels, 0x08U);
  check(sbus_remote_data.valid == 0U, "failsafe frame must invalidate data");
  check(sbus_remote_data.startup_rejected == 0U,
        "runtime failsafe must not latch the startup gate");

  test_tick_ms += 20U;
  feed_safe_frame(frame, channels, 0U);
  check(sbus_remote_data.valid == 1U, "valid frame should recover after failsafe");

  test_tick_ms += SBUS_REMOTE_TIMEOUT_MS + 1U;
  sbus_remote_update();
  check(sbus_remote_data.connected == 0U, "timeout should mark receiver offline");
  check(sbus_remote_data.valid == 0U, "timeout must invalidate control data");
}

static void test_bad_startup_recovers_after_five_seconds(void)
{
  uint8_t frame[SBUS_REMOTE_FRAME_SIZE];
  uint16_t channels[SBUS_REMOTE_CHANNEL_COUNT];

  sbus_remote_init();
  setup_safe_channels(channels);
  channels[1] = 1100U;
  test_tick_ms = 0U;
  feed_safe_frame(frame, channels, 0U);
  check(sbus_remote_data.startup_rejected == 1U,
        "off-centre stick should reject startup");
  check((sbus_remote_data.startup_reject_reason &
         SBUS_REMOTE_STARTUP_BAD_STICK) != 0U,
        "reject reason should record a bad stick");

  setup_safe_channels(channels);
  test_tick_ms = 50U;
  for (uint16_t i = 0U; i <= 50U; i++) {
    feed_safe_frame(frame, channels, 0U);
    test_tick_ms += 50U;
  }
  check(sbus_remote_data.valid == 0U,
        "recovery must remain invalid through the first five seconds");

  setup_safe_channels(channels);
  channels[0] = 1100U;
  feed_safe_frame(frame, channels, 0U);
  check(sbus_remote_data.recovery_hold_ms == 0U,
        "an off-centre frame must restart the recovery timer");

  setup_safe_channels(channels);
  test_tick_ms += 50U;
  feed_safe_frame(frame, channels, 0U);
  test_tick_ms += SBUS_REMOTE_TIMEOUT_MS + 1U;
  sbus_remote_update();
  check(sbus_remote_data.recovery_hold_ms == 0U,
        "a telemetry gap must restart the recovery timer");

  test_tick_ms += 50U;
  for (uint16_t i = 0U; i < 100U; i++) {
    feed_safe_frame(frame, channels, 0U);
    test_tick_ms += 50U;
  }
  feed_safe_frame(frame, channels, 0U);
  check(sbus_remote_data.valid == 0U,
        "recovery must use strictly more than five seconds");

  test_tick_ms += 50U;
  feed_safe_frame(frame, channels, 0U);
  check(sbus_remote_data.startup_checked == 1U,
        "safe posture should restore startup after five seconds");
  check(sbus_remote_data.startup_rejected == 0U,
        "startup rejection should clear after recovery");
  check(sbus_remote_data.startup_reject_reason == 0U,
        "recovery should clear the startup reject reason");
  check(sbus_remote_data.valid == 1U,
        "recovered controls should become valid");
}

static void test_bad_startup_control_latches(void)
{
  uint8_t frame[SBUS_REMOTE_FRAME_SIZE];
  uint16_t channels[SBUS_REMOTE_CHANNEL_COUNT];

  sbus_remote_init();
  setup_safe_channels(channels);
  channels[8] = 1023U;
  test_tick_ms = 0U;
  feed_safe_frame(frame, channels, 0U);
  check(sbus_remote_data.startup_rejected == 1U,
        "centred speed knob should reject startup");
  check((sbus_remote_data.startup_reject_reason &
         SBUS_REMOTE_STARTUP_BAD_CONTROL) != 0U,
        "reject reason should record a bad switch or knob");
  check(sbus_remote_data.valid == 0U,
        "bad switch startup must not expose control data");
}

int main(void)
{
  test_good_startup_and_recoverable_runtime_fault();
  test_bad_startup_recovers_after_five_seconds();
  test_bad_startup_control_latches();
  printf("%d failure(s)\n", failures);
  return (failures == 0) ? 0 : 1;
}
