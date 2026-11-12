#include "vmc_leso.h"

#include <math.h>
#include <string.h>

static float vmc_leso_state[VMC_LESO_STATE_COUNT];
static float vmc_leso_a[VMC_LESO_STATE_COUNT][VMC_LESO_STATE_COUNT];
static float vmc_leso_b[VMC_LESO_STATE_COUNT][VMC_LESO_INPUT_COUNT];
static float vmc_leso_l[VMC_LESO_STATE_COUNT][VMC_LESO_PLANT_STATES];
static uint8_t vmc_leso_enable;
static float vmc_leso_compensation;
static float vmc_leso_wheel_limit;
static float vmc_leso_leg_limit;
/* The observer still carries measurement noise; only the steady component is
 * wanted, so the applied correction is low-passed at about 5 Hz. */
static float vmc_leso_filtered[VMC_LESO_INPUT_COUNT];

static float clamp_float(float value, float minimum, float maximum)
{
  if (value > maximum) {
    return maximum;
  }
  if (value < minimum) {
    return minimum;
  }
  return value;
}

static void interpolate_model(float leg_length)
{
  uint32_t upper = 1U;
  uint32_t row;
  uint32_t column;
  float fraction;

  while (upper < VMC_LESO_MODEL_POINTS &&
         vmc_leso_model_points[upper].leg_length < leg_length) {
    upper++;
  }
  if (upper >= VMC_LESO_MODEL_POINTS) {
    upper = VMC_LESO_MODEL_POINTS - 1U;
  }
  {
    const uint32_t lower = (upper == 0U) ? 0U : upper - 1U;
    const float low_length =
        vmc_leso_model_points[lower].leg_length;
    const float high_length =
        vmc_leso_model_points[upper].leg_length;
    const float span = high_length - low_length;
    fraction = (span > 1.0e-6f)
                   ? clamp_float((leg_length - low_length) / span, 0.0f, 1.0f)
                   : 0.0f;

    for (row = 0U; row < VMC_LESO_STATE_COUNT; row++) {
      for (column = 0U; column < VMC_LESO_STATE_COUNT; column++) {
        vmc_leso_a[row][column] =
            vmc_leso_model_points[lower].a[row][column] +
            fraction * (vmc_leso_model_points[upper].a[row][column] -
                        vmc_leso_model_points[lower].a[row][column]);
      }
      for (column = 0U; column < VMC_LESO_INPUT_COUNT; column++) {
        vmc_leso_b[row][column] =
            vmc_leso_model_points[lower].b[row][column] +
            fraction * (vmc_leso_model_points[upper].b[row][column] -
                        vmc_leso_model_points[lower].b[row][column]);
      }
      for (column = 0U; column < VMC_LESO_PLANT_STATES; column++) {
        vmc_leso_l[row][column] =
            vmc_leso_model_points[lower].l[row][column] +
            fraction * (vmc_leso_model_points[upper].l[row][column] -
                        vmc_leso_model_points[lower].l[row][column]);
      }
    }
  }
}

void vmc_leso_reset(void)
{
  memset(vmc_leso_state, 0, sizeof(vmc_leso_state));
  memset(vmc_leso_filtered, 0, sizeof(vmc_leso_filtered));
}

void vmc_leso_set_enable(uint8_t enable)
{
  if (enable == 0U && vmc_leso_enable != 0U) {
    vmc_leso_reset();
  }
  vmc_leso_enable = (enable != 0U) ? 1U : 0U;
}

uint8_t vmc_leso_is_enabled(void)
{
  return vmc_leso_enable;
}

void vmc_leso_set_compensation(float compensation)
{
  vmc_leso_compensation = clamp_float(compensation, 0.0f, 1.0f);
}

void vmc_leso_set_limit(float wheel_limit, float leg_limit)
{
  vmc_leso_wheel_limit = fabsf(wheel_limit);
  vmc_leso_leg_limit = fabsf(leg_limit);
}

void vmc_leso_update(float leg_length,
                     const float measurement[VMC_LESO_PLANT_STATES],
                     const float input[VMC_LESO_INPUT_COUNT],
                     float disturbance[VMC_LESO_INPUT_COUNT])
{
  float next[VMC_LESO_STATE_COUNT];
  float innovation[VMC_LESO_PLANT_STATES];
  uint32_t row;
  uint32_t column;

  disturbance[0] = 0.0f;
  disturbance[1] = 0.0f;
  if (vmc_leso_enable == 0U) {
    return;
  }

  interpolate_model(leg_length);

  for (row = 0U; row < VMC_LESO_PLANT_STATES; row++) {
    innovation[row] = measurement[row] - vmc_leso_state[row];
  }

  for (row = 0U; row < VMC_LESO_STATE_COUNT; row++) {
    float value = 0.0f;
    for (column = 0U; column < VMC_LESO_STATE_COUNT; column++) {
      value += vmc_leso_a[row][column] * vmc_leso_state[column];
    }
    for (column = 0U; column < VMC_LESO_INPUT_COUNT; column++) {
      value += vmc_leso_b[row][column] * input[column];
    }
    for (column = 0U; column < VMC_LESO_PLANT_STATES; column++) {
      value += vmc_leso_l[row][column] * innovation[column];
    }
    next[row] = value;
  }

  memcpy(vmc_leso_state, next, sizeof(vmc_leso_state));

  {
    const float alpha = 0.086f; /* ~5 Hz */
    uint32_t channel;
    for (channel = 0U; channel < VMC_LESO_INPUT_COUNT; channel++) {
      const float limit = (channel == 0U) ? vmc_leso_wheel_limit
                                          : vmc_leso_leg_limit;
      const float raw = clamp_float(
          vmc_leso_state[VMC_LESO_PLANT_STATES + channel], -limit, limit);
      vmc_leso_filtered[channel] +=
          alpha * (raw - vmc_leso_filtered[channel]);
      disturbance[channel] = vmc_leso_compensation * vmc_leso_filtered[channel];
    }
  }
}
