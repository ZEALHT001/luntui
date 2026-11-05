#include "vmc_mpc.h"

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

int main(void)
{
  const vmc_mpc_limits_t limits = {
      -5.5f, 5.5f, -108.0f, 108.0f, 0.2f, 2.0f};
  const float zero_state[VMC_MPC_STATE_COUNT] = {0.0f};
  const float forward_state[VMC_MPC_STATE_COUNT] = {
      0.0f, 0.0f, 0.02f, 0.0f, 0.0f, 0.0f};
  vmc_mpc_output_t output;

  vmc_mpc_reset(0U);
  vmc_mpc_prime(0U, 0.0f, 0.0f);
  check(vmc_mpc_solve(0U, 0.28f, zero_state, &limits, &output) != 0U,
        "zero-state solve failed");
  check(output.valid != 0U, "zero-state output was not marked valid");
  check(fabsf(output.wheel_torque) < 1.0e-4f,
        "zero state produced nonzero wheel torque");
  check(fabsf(output.previous_wheel_torque) < 1.0e-4f &&
            fabsf(output.previous_virtual_leg_torque) < 1.0e-4f,
        "zero-state previous inputs were not reported");

  vmc_mpc_reset(0U);
  vmc_mpc_prime(0U, 0.0f, 0.0f);
  check(vmc_mpc_solve(0U, 0.28f, forward_state, &limits, &output) != 0U,
        "forward-error solve failed");
  check(isfinite(output.wheel_torque) &&
            isfinite(output.virtual_leg_torque),
        "forward-error solve produced a non-finite command");
  check(fabsf(output.wheel_delta) <= limits.wheel_delta_max + 1.0e-5f,
        "wheel delta exceeded its configured limit");
  check(fabsf(output.virtual_leg_delta) <= limits.leg_delta_max + 1.0e-5f,
        "leg delta exceeded its configured limit");
  check(fabsf(output.previous_wheel_torque) < 1.0e-4f &&
            fabsf(output.previous_virtual_leg_torque) < 1.0e-4f,
        "forward-state previous inputs were not reported");

  vmc_mpc_reset(0U);
  vmc_mpc_prime(0U, 2.25f, 7.5f);
  check(vmc_mpc_solve(0U, 0.28f, zero_state, &limits, &output) != 0U,
        "primed-state solve failed");
  check(fabsf(output.previous_wheel_torque - 2.25f) < 1.0e-5f &&
            fabsf(output.previous_virtual_leg_torque - 7.5f) < 1.0e-5f,
        "previous inputs did not preserve public T/Tp signs");
  check(fabsf(output.initial_plan[0] - 2.25f) < 1.0e-5f &&
            fabsf(output.initial_plan[1] - 7.5f) < 1.0e-5f &&
            fabsf(output.initial_plan[6] - 2.25f) < 1.0e-5f &&
            fabsf(output.initial_plan[7] - 7.5f) < 1.0e-5f,
        "initial QP plan did not preserve public T/Tp signs");

  vmc_mpc_reset(1U);
  vmc_mpc_prime(1U, 0.0f, 0.0f);
  {
    const float large_state[VMC_MPC_STATE_COUNT] = {
        0.4f, 2.0f, 0.5f, 1.0f, 0.3f, 1.5f};
    check(vmc_mpc_solve(1U, 0.18f, large_state, &limits, &output) != 0U,
          "large-state solve failed");
    check(output.wheel_torque >= limits.wheel_torque_min &&
              output.wheel_torque <= limits.wheel_torque_max,
          "wheel torque escaped its box constraint");
    check(output.virtual_leg_torque >= limits.leg_torque_min &&
              output.virtual_leg_torque <= limits.leg_torque_max,
          "virtual-leg torque escaped its box constraint");
  }

  {
    const vmc_mpc_model_point_t *model = &vmc_mpc_model_points[4];
    float state[VMC_MPC_STATE_COUNT] = {0.0f, 0.0f, 0.02f, 0.0f, 0.0f, 0.0f};
    float tail_min = state[2];
    float tail_max = state[2];
    uint32_t step;

    vmc_mpc_reset(0U);
    vmc_mpc_prime(0U, 0.0f, 0.0f);
    for (step = 0U; step < 3000U; step++) {
      float input[VMC_MPC_INPUT_COUNT];
      float next[VMC_MPC_STATE_COUNT] = {0.0f};
      uint32_t row;
      uint32_t column;

      check(vmc_mpc_solve(0U, model->leg_length, state, &limits, &output) != 0U,
            "closed-loop solve failed");
      input[0] = output.wheel_torque;
      input[1] = -output.virtual_leg_torque;
      if (fabsf(state[3]) < 0.01f && fabsf(input[0]) < 1.2f) {
        input[0] = 0.0f;
      }
      for (row = 0U; row < VMC_MPC_STATE_COUNT; row++) {
        for (column = 0U; column < VMC_MPC_STATE_COUNT; column++) {
          next[row] += model->a[row][column] * state[column];
        }
        for (column = 0U; column < VMC_MPC_INPUT_COUNT; column++) {
          next[row] += model->b[row][column] * input[column];
        }
      }
      for (row = 0U; row < VMC_MPC_STATE_COUNT; row++) {
        state[row] = next[row];
      }
      check(fabsf(state[2]) < 0.1f, "closed-loop position diverged");
      if (step == 2500U) {
        tail_min = state[2];
        tail_max = state[2];
      } else if (step > 2500U) {
        if (state[2] < tail_min) {
          tail_min = state[2];
        }
        if (state[2] > tail_max) {
          tail_max = state[2];
        }
      }
    }
    /* With the MPC stage cost aligned to the 10-step LQR model, this model
     * inherits LQR's Coulomb-friction limit cycle.  The independent LQR
     * baseline is about 25.7 mm at this leg length; fail on a material
     * regression rather than the old four-step controller's 20 mm result. */
    check(tail_max - tail_min < 0.030f,
          "closed-loop stiction limit cycle exceeded the LQR baseline");
  }
  check(vmc_mpc_model_sample_time_s > 0.0022f &&
            vmc_mpc_model_sample_time_s < 0.0023f,
        "MPC model was not generated for the 450 Hz control period");

  if (failures != 0) {
    return 1;
  }
  puts("vmc_mpc fixed-size solver OK");
  return 0;
}
