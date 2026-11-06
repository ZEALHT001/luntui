#ifndef VMC_MPC_H
#define VMC_MPC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VMC_MPC_STATE_COUNT 6U
#define VMC_MPC_INPUT_COUNT 2U
#define VMC_MPC_HORIZON      10U
#define VMC_MPC_CONTROL_HORIZON VMC_MPC_HORIZON
#define VMC_MPC_MODEL_POINTS 9U
#define VMC_MPC_DECISION_COUNT \
  (VMC_MPC_CONTROL_HORIZON * VMC_MPC_INPUT_COUNT)
#define VMC_MPC_PREDICTION_STATES \
  (VMC_MPC_HORIZON * VMC_MPC_STATE_COUNT)

typedef struct {
  float leg_length;
  float a[VMC_MPC_STATE_COUNT][VMC_MPC_STATE_COUNT];
  float b[VMC_MPC_STATE_COUNT][VMC_MPC_INPUT_COUNT];
  float terminal_cost[VMC_MPC_STATE_COUNT][VMC_MPC_STATE_COUNT];
  float g[VMC_MPC_PREDICTION_STATES][VMC_MPC_DECISION_COUNT];
  float hessian[VMC_MPC_DECISION_COUNT][VMC_MPC_DECISION_COUNT];
} vmc_mpc_model_point_t;

extern const vmc_mpc_model_point_t
    vmc_mpc_model_points[VMC_MPC_MODEL_POINTS];
extern const float vmc_mpc_model_sample_time_s;

typedef struct {
  float wheel_torque_min;
  float wheel_torque_max;
  float leg_torque_min;
  float leg_torque_max;
  float wheel_delta_max;
  float leg_delta_max;
} vmc_mpc_limits_t;

typedef struct {
  float wheel_torque;
  float virtual_leg_torque;
  float wheel_delta;
  float virtual_leg_delta;
  float raw_wheel_torque;
  float raw_virtual_leg_torque;
  float previous_wheel_torque;
  float previous_virtual_leg_torque;
  float initial_plan[VMC_MPC_DECISION_COUNT];
  uint8_t valid;
  uint8_t saturated;
  uint8_t rate_limited;
  uint8_t torque_saturated;
} vmc_mpc_output_t;

typedef struct {
  float u_prev[VMC_MPC_INPUT_COUNT];
  float plan[VMC_MPC_DECISION_COUNT];
  uint8_t plan_valid;
  uint32_t solve_count;
  uint32_t failure_count;
  uint32_t saturation_count;
} vmc_mpc_axis_state_t;

void vmc_mpc_reset(uint8_t side);
void vmc_mpc_prime(uint8_t side, float wheel_torque,
                   float virtual_leg_torque);
uint8_t vmc_mpc_solve(uint8_t side, float leg_length,
                      const float state[VMC_MPC_STATE_COUNT],
                      const vmc_mpc_limits_t *limits,
                      vmc_mpc_output_t *output);
uint8_t vmc_mpc_solve_pair(
    float leg_length, const float left_state[VMC_MPC_STATE_COUNT],
    const float right_state[VMC_MPC_STATE_COUNT],
    const vmc_mpc_limits_t *limits, vmc_mpc_output_t *left_output,
    vmc_mpc_output_t *right_output);

#ifdef __cplusplus
}
#endif

#endif
