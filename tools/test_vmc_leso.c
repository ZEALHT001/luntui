#include "vmc_leso.h"

#include <math.h>
#include <stdio.h>

static int failures;

static void check(int condition, const char *message)
{
  if (condition == 0) {
    printf("FAIL: %s\n", message);
    failures++;
  }
}

/* Stabilising state feedback for the test plant (leg length 0.2825, 3 ms).
 * Without it the inverted pendulum runs away long before the observer could
 * possibly track it, and the test would only measure that. */
static const float test_gain[VMC_LESO_INPUT_COUNT][VMC_LESO_PLANT_STATES] = {
    {-22.0769772f, -3.14634951f, -2.97559694f, -3.32568398f, 8.11248441f,
     2.18651368f},
    {23.2904792f, 5.005874f, 7.82188717f, 7.05811016f, 31.3625541f,
     3.51433328f},
};

int main(void)
{
  const float leg_length = 0.28f;
  const uint32_t point = 4U;
  const float disturbance = 0.5f;
  float state[VMC_LESO_PLANT_STATES] = {0.0f};
  float measurement[VMC_LESO_PLANT_STATES];
  const float input[VMC_LESO_INPUT_COUNT] = {0.0f, 0.0f};
  float estimate[VMC_LESO_INPUT_COUNT] = {0.0f};
  float command[VMC_LESO_INPUT_COUNT];
  uint32_t step;
  uint32_t row;
  uint32_t column;

  /* Disabled: the observer must report nothing and stay quiet. */
  vmc_leso_set_compensation(1.0f);
  vmc_leso_set_limit(3.0f, 20.0f);
  check(vmc_leso_is_enabled() == 0U, "observer started enabled");
  for (row = 0U; row < VMC_LESO_PLANT_STATES; row++) {
    measurement[row] = 0.0f;
  }
  vmc_leso_update(leg_length, measurement, input, estimate);
  check(estimate[0] == 0.0f && estimate[1] == 0.0f,
        "disabled observer produced a correction");

  /* Run a plant driven by a constant input disturbance and let the observer
   * estimate it from the states alone.  This is the property that removes a
   * slope's steady-state error. */
  vmc_leso_set_enable(1U);
  for (step = 0U; step < 4000U; step++) {
    float next[VMC_LESO_PLANT_STATES] = {0.0f};
    command[0] = -estimate[0];
    command[1] = -estimate[1];
    for (row = 0U; row < VMC_LESO_PLANT_STATES; row++) {
      for (column = 0U; column < VMC_LESO_PLANT_STATES; column++) {
        command[0] -= test_gain[0][column] * state[column];
        command[1] -= test_gain[1][column] * state[column];
      }
      next[row] += vmc_leso_model_points[point].b[row][0] *
                       (command[0] + disturbance) +
                   vmc_leso_model_points[point].b[row][1] * command[1];
      for (column = 0U; column < VMC_LESO_PLANT_STATES; column++) {
        next[row] += vmc_leso_model_points[point].a[row][column] *
                     state[column];
      }
    }
    for (row = 0U; row < VMC_LESO_PLANT_STATES; row++) {
      state[row] = next[row];
      measurement[row] = state[row];
    }
    vmc_leso_update(leg_length, measurement, command, estimate);
  }

  check(isfinite(estimate[0]) && isfinite(estimate[1]),
        "disturbance estimate was not finite");
  check(fabsf(estimate[0] - disturbance) < 0.05f,
        "observer did not converge to the wheel disturbance");

  /* Disabling must clear the state so a later enable starts clean. */
  vmc_leso_set_enable(0U);
  for (row = 0U; row < VMC_LESO_PLANT_STATES; row++) {
    measurement[row] = 0.0f;
  }
  vmc_leso_update(leg_length, measurement, input, estimate);
  check(estimate[0] == 0.0f, "disable did not clear the estimate");
  vmc_leso_set_enable(1U);
  for (row = 0U; row < VMC_LESO_PLANT_STATES; row++) {
    measurement[row] = 0.0f;
  }
  vmc_leso_update(leg_length, measurement, input, estimate);
  check(fabsf(estimate[0]) < 0.01f, "re-enable did not restart from zero");

  if (failures != 0) {
    return 1;
  }
  puts("vmc_leso observer OK");
  return 0;
}
