#include "vmc_mpc.h"

#include <math.h>
#include <string.h>

#define VMC_MPC_SIDE_COUNT 2U
#define VMC_MPC_ADMM_ITERATIONS 10U
#define VMC_MPC_ADMM_RHO        0.20f
#define VMC_MPC_SOLVER_TOLERANCE 1.0e-3f

static const float vmc_mpc_q[VMC_MPC_STATE_COUNT] = {
    1.0f, 100.0f, 400.0f, 100.0f, 5000.0f, 1.0f};
static const float vmc_mpc_delta_weight[VMC_MPC_INPUT_COUNT] = {
    1.0f, 0.05f};

static vmc_mpc_axis_state_t vmc_mpc_axis[VMC_MPC_SIDE_COUNT];

/* G and H are interpolated from the Flash schedule once per solve_pair() call.
 * The factor is then shared by both sides. */
static float vmc_mpc_g[VMC_MPC_PREDICTION_STATES][VMC_MPC_DECISION_COUNT];
static float vmc_mpc_base_left[VMC_MPC_PREDICTION_STATES];
static float vmc_mpc_base_right[VMC_MPC_PREDICTION_STATES];
static float vmc_mpc_h_factor[VMC_MPC_DECISION_COUNT]
                            [VMC_MPC_DECISION_COUNT];

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

static void interpolate_model(
    float leg_length,
    float a[VMC_MPC_STATE_COUNT][VMC_MPC_STATE_COUNT],
    float b[VMC_MPC_STATE_COUNT][VMC_MPC_INPUT_COUNT],
    float terminal_cost[VMC_MPC_STATE_COUNT][VMC_MPC_STATE_COUNT],
    float g[VMC_MPC_PREDICTION_STATES][VMC_MPC_DECISION_COUNT],
    float hessian[VMC_MPC_DECISION_COUNT][VMC_MPC_DECISION_COUNT])
{
  uint32_t upper = 1U;
  float fraction;
  const vmc_mpc_model_point_t *lo = &vmc_mpc_model_points[0];
  const vmc_mpc_model_point_t *hi = &vmc_mpc_model_points[1];
  uint32_t row;
  uint32_t column;

  if (leg_length <= vmc_mpc_model_points[0].leg_length) {
    lo = &vmc_mpc_model_points[0];
    hi = lo;
    fraction = 0.0f;
  } else if (leg_length >=
             vmc_mpc_model_points[VMC_MPC_MODEL_POINTS - 1U].leg_length) {
    lo = &vmc_mpc_model_points[VMC_MPC_MODEL_POINTS - 1U];
    hi = lo;
    fraction = 0.0f;
  } else {
    while (upper < VMC_MPC_MODEL_POINTS &&
           leg_length > vmc_mpc_model_points[upper].leg_length) {
      upper++;
    }
    lo = &vmc_mpc_model_points[upper - 1U];
    hi = &vmc_mpc_model_points[upper];
    fraction =
        (leg_length - lo->leg_length) / (hi->leg_length - lo->leg_length);
  }

  for (row = 0U; row < VMC_MPC_STATE_COUNT; row++) {
    for (column = 0U; column < VMC_MPC_STATE_COUNT; column++) {
      a[row][column] =
          lo->a[row][column] +
          fraction * (hi->a[row][column] - lo->a[row][column]);
      terminal_cost[row][column] =
          lo->terminal_cost[row][column] +
          fraction * (hi->terminal_cost[row][column] -
                      lo->terminal_cost[row][column]);
    }
    for (column = 0U; column < VMC_MPC_INPUT_COUNT; column++) {
      b[row][column] =
          lo->b[row][column] +
          fraction * (hi->b[row][column] - lo->b[row][column]);
    }
  }
  for (row = 0U; row < VMC_MPC_PREDICTION_STATES; row++) {
    for (column = 0U; column < VMC_MPC_DECISION_COUNT; column++) {
      g[row][column] =
          lo->g[row][column] +
          fraction * (hi->g[row][column] - lo->g[row][column]);
    }
  }
  for (row = 0U; row < VMC_MPC_DECISION_COUNT; row++) {
    for (column = 0U; column < VMC_MPC_DECISION_COUNT; column++) {
      hessian[row][column] =
          lo->hessian[row][column] +
          fraction * (hi->hessian[row][column] - lo->hessian[row][column]);
    }
  }
}

static void advance_state(
    const float a[VMC_MPC_STATE_COUNT][VMC_MPC_STATE_COUNT],
    float state[VMC_MPC_STATE_COUNT])
{
  float next[VMC_MPC_STATE_COUNT];
  uint32_t row;
  uint32_t column;

  for (row = 0U; row < VMC_MPC_STATE_COUNT; row++) {
    next[row] = 0.0f;
    for (column = 0U; column < VMC_MPC_STATE_COUNT; column++) {
      next[row] += a[row][column] * state[column];
    }
  }
  memcpy(state, next, sizeof(next));
}

static void build_base_predictions(
    const float a[VMC_MPC_STATE_COUNT][VMC_MPC_STATE_COUNT],
    const float left_state[VMC_MPC_STATE_COUNT],
    const float right_state[VMC_MPC_STATE_COUNT])
{
  float left_base[VMC_MPC_STATE_COUNT];
  float right_base[VMC_MPC_STATE_COUNT];
  uint32_t step;
  uint32_t row;

  memcpy(left_base, left_state, sizeof(left_base));
  memcpy(right_base, right_state, sizeof(right_base));
  memset(vmc_mpc_base_left, 0, sizeof(vmc_mpc_base_left));
  memset(vmc_mpc_base_right, 0, sizeof(vmc_mpc_base_right));

  for (step = 0U; step < VMC_MPC_HORIZON; step++) {
    advance_state(a, left_base);
    advance_state(a, right_base);
    for (row = 0U; row < VMC_MPC_STATE_COUNT; row++) {
      vmc_mpc_base_left[step * VMC_MPC_STATE_COUNT + row] = left_base[row];
      vmc_mpc_base_right[step * VMC_MPC_STATE_COUNT + row] = right_base[row];
    }
  }
}

static void add_admm_regularization(void)
{
  uint32_t step;
  uint32_t column;

  for (step = 0U; step < VMC_MPC_HORIZON; step++) {
    for (column = 0U; column < VMC_MPC_INPUT_COUNT; column++) {
      const uint32_t index = step * VMC_MPC_INPUT_COUNT + column;
      vmc_mpc_h_factor[index][index] += VMC_MPC_ADMM_RHO;
    }
  }
}

static uint8_t factor_hessian(void)
{
  uint32_t row;
  uint32_t column;
  uint32_t inner;

  for (row = 0U; row < VMC_MPC_DECISION_COUNT; row++) {
    for (column = 0U; column <= row; column++) {
      float value = vmc_mpc_h_factor[row][column];
      for (inner = 0U; inner < column; inner++) {
        value -= vmc_mpc_h_factor[row][inner] *
                 vmc_mpc_h_factor[column][inner];
      }
      if (column == row) {
        if (!isfinite(value) || value <= 1.0e-8f) {
          return 0U;
        }
        vmc_mpc_h_factor[row][column] = sqrtf(value);
      } else {
        vmc_mpc_h_factor[row][column] =
            value / vmc_mpc_h_factor[column][column];
      }
    }
    for (column = row + 1U; column < VMC_MPC_DECISION_COUNT; column++) {
      vmc_mpc_h_factor[row][column] = 0.0f;
    }
  }
  return 1U;
}

static void build_rhs(const float base[VMC_MPC_PREDICTION_STATES],
                      const float u_previous[VMC_MPC_INPUT_COUNT],
                      float f[VMC_MPC_DECISION_COUNT],
                      const float terminal_cost[VMC_MPC_STATE_COUNT]
                                               [VMC_MPC_STATE_COUNT])
{
  uint32_t step;
  uint32_t row;
  uint32_t column;
  uint32_t decision;

  memset(f, 0, VMC_MPC_DECISION_COUNT * sizeof(f[0]));
  for (step = 0U; step < VMC_MPC_HORIZON; step++) {
    const uint32_t prediction_row = step * VMC_MPC_STATE_COUNT;

    if (step + 1U == VMC_MPC_HORIZON) {
      for (row = 0U; row < VMC_MPC_STATE_COUNT; row++) {
        for (column = 0U; column < VMC_MPC_STATE_COUNT; column++) {
          const float weight = terminal_cost[row][column];
          for (decision = 0U; decision < VMC_MPC_DECISION_COUNT;
               decision++) {
            f[decision] +=
                vmc_mpc_g[prediction_row + row][decision] * weight *
                base[prediction_row + column];
          }
        }
      }
    } else {
      for (row = 0U; row < VMC_MPC_STATE_COUNT; row++) {
        const float weight = vmc_mpc_q[row];
        for (decision = 0U; decision < VMC_MPC_DECISION_COUNT; decision++) {
          f[decision] +=
              vmc_mpc_g[prediction_row + row][decision] * weight *
              base[prediction_row + row];
        }
      }
    }
  }

  for (step = 0U; step < VMC_MPC_HORIZON; step++) {
    for (column = 0U; column < VMC_MPC_INPUT_COUNT; column++) {
      const uint32_t index = step * VMC_MPC_INPUT_COUNT + column;
      f[index] -= vmc_mpc_delta_weight[column] * u_previous[column];
    }
  }
}

static void solve_factor(const float rhs[VMC_MPC_DECISION_COUNT],
                         float solution[VMC_MPC_DECISION_COUNT])
{
  float intermediate[VMC_MPC_DECISION_COUNT];
  uint32_t row;
  uint32_t column;

  for (row = 0U; row < VMC_MPC_DECISION_COUNT; row++) {
    float value = rhs[row];
    for (column = 0U; column < row; column++) {
      value -= vmc_mpc_h_factor[row][column] * intermediate[column];
    }
    intermediate[row] = value / vmc_mpc_h_factor[row][row];
  }
  for (row = VMC_MPC_DECISION_COUNT; row > 0U; row--) {
    const uint32_t index = row - 1U;
    float value = intermediate[index];
    for (column = index + 1U; column < VMC_MPC_DECISION_COUNT; column++) {
      value -= vmc_mpc_h_factor[column][index] * solution[column];
    }
    solution[index] = value / vmc_mpc_h_factor[index][index];
  }
}

static float variable_minimum(const vmc_mpc_limits_t *limits, uint32_t index)
{
  return (index % VMC_MPC_INPUT_COUNT == 0U) ? limits->wheel_torque_min
                                             : limits->leg_torque_min;
}

static float variable_maximum(const vmc_mpc_limits_t *limits, uint32_t index)
{
  return (index % VMC_MPC_INPUT_COUNT == 0U) ? limits->wheel_torque_max
                                             : limits->leg_torque_max;
}

static uint8_t solve_box_qp(
    vmc_mpc_axis_state_t *axis, const float f[VMC_MPC_DECISION_COUNT],
    const vmc_mpc_limits_t *limits,
    float solution[VMC_MPC_DECISION_COUNT])
{
  float z[VMC_MPC_DECISION_COUNT];
  float u[VMC_MPC_DECISION_COUNT];
  float y[VMC_MPC_DECISION_COUNT] = {0.0f};
  float rhs[VMC_MPC_DECISION_COUNT];
  uint32_t iteration;
  uint32_t row;

  if (axis->plan_valid != 0U) {
    for (row = 0U; row + VMC_MPC_INPUT_COUNT < VMC_MPC_DECISION_COUNT;
         row++) {
      solution[row] = axis->plan[row + VMC_MPC_INPUT_COUNT];
    }
    solution[VMC_MPC_DECISION_COUNT - 2U] = axis->u_prev[0];
    solution[VMC_MPC_DECISION_COUNT - 1U] = axis->u_prev[1];
  } else {
    for (row = 0U; row < VMC_MPC_HORIZON; row++) {
      solution[2U * row] = axis->u_prev[0];
      solution[2U * row + 1U] = axis->u_prev[1];
    }
  }
  for (row = 0U; row < VMC_MPC_DECISION_COUNT; row++) {
    z[row] = clamp_float(solution[row], variable_minimum(limits, row),
                         variable_maximum(limits, row));
  }

  for (iteration = 0U; iteration < VMC_MPC_ADMM_ITERATIONS; iteration++) {
    float primal_residual = 0.0f;
    float dual_residual = 0.0f;

    for (row = 0U; row < VMC_MPC_DECISION_COUNT; row++) {
      rhs[row] = VMC_MPC_ADMM_RHO * (z[row] - y[row]) - f[row];
    }
    solve_factor(rhs, u);
    for (row = 0U; row < VMC_MPC_DECISION_COUNT; row++) {
      const float next_z =
          clamp_float(u[row] + y[row], variable_minimum(limits, row),
                      variable_maximum(limits, row));
      const float primal = fabsf(u[row] - next_z);
      const float dual = fabsf(VMC_MPC_ADMM_RHO * (next_z - z[row]));
      y[row] += u[row] - next_z;
      z[row] = next_z;
      if (primal > primal_residual) {
        primal_residual = primal;
      }
      if (dual > dual_residual) {
        dual_residual = dual;
      }
    }
    if (primal_residual <= VMC_MPC_SOLVER_TOLERANCE &&
        dual_residual <= VMC_MPC_SOLVER_TOLERANCE) {
      break;
    }
  }
  memcpy(solution, z, sizeof(z));
  return 1U;
}

static uint8_t finalize_output(
    vmc_mpc_axis_state_t *axis, const float solution[VMC_MPC_DECISION_COUNT],
    const vmc_mpc_limits_t *limits, vmc_mpc_output_t *output)
{
  float previous_input[VMC_MPC_INPUT_COUNT];
  float next_input[VMC_MPC_INPUT_COUNT];
  float requested_delta[VMC_MPC_INPUT_COUNT];
  uint8_t saturated = 0U;

  previous_input[0] = axis->u_prev[0];
  previous_input[1] = axis->u_prev[1];
  next_input[0] = clamp_float(solution[0], limits->wheel_torque_min,
                              limits->wheel_torque_max);
  next_input[1] = clamp_float(solution[1], limits->leg_torque_min,
                              limits->leg_torque_max);
  requested_delta[0] = next_input[0] - previous_input[0];
  requested_delta[1] = next_input[1] - previous_input[1];
  next_input[0] =
      previous_input[0] +
      clamp_float(requested_delta[0], -limits->wheel_delta_max,
                  limits->wheel_delta_max);
  next_input[1] =
      previous_input[1] +
      clamp_float(requested_delta[1], -limits->leg_delta_max,
                  limits->leg_delta_max);
  next_input[0] = clamp_float(next_input[0], limits->wheel_torque_min,
                              limits->wheel_torque_max);
  next_input[1] = clamp_float(next_input[1], limits->leg_torque_min,
                              limits->leg_torque_max);

  if (fabsf(next_input[0] - solution[0]) > VMC_MPC_SOLVER_TOLERANCE ||
      fabsf(next_input[1] - solution[1]) > VMC_MPC_SOLVER_TOLERANCE) {
    saturated = 1U;
  }
  memcpy(axis->plan, solution, sizeof(axis->plan));
  axis->plan[0] = next_input[0];
  axis->plan[1] = next_input[1];
  axis->plan_valid = 1U;
  axis->u_prev[0] = next_input[0];
  axis->u_prev[1] = next_input[1];
  axis->solve_count++;
  if (saturated != 0U) {
    axis->saturation_count++;
  }

  memset(output, 0, sizeof(*output));
  output->wheel_torque = next_input[0];
  /* The plant's virtual-leg torque convention is opposite the firmware's
   * public Tp command. */
  output->virtual_leg_torque = -next_input[1];
  output->wheel_delta = next_input[0] - previous_input[0];
  output->virtual_leg_delta = next_input[1] - previous_input[1];
  output->valid = 1U;
  output->saturated = saturated;
  return 1U;
}

void vmc_mpc_reset(uint8_t side)
{
  if (side >= VMC_MPC_SIDE_COUNT) {
    return;
  }
  memset(&vmc_mpc_axis[side], 0, sizeof(vmc_mpc_axis[side]));
}

void vmc_mpc_prime(uint8_t side, float wheel_torque,
                   float virtual_leg_torque)
{
  uint32_t i;
  vmc_mpc_axis_state_t *axis;

  if (side >= VMC_MPC_SIDE_COUNT) {
    return;
  }
  axis = &vmc_mpc_axis[side];
  axis->u_prev[0] = wheel_torque;
  axis->u_prev[1] = -virtual_leg_torque;
  for (i = 0U; i < VMC_MPC_HORIZON; i++) {
    axis->plan[2U * i] = axis->u_prev[0];
    axis->plan[2U * i + 1U] = axis->u_prev[1];
  }
  axis->plan_valid = 1U;
}

uint8_t vmc_mpc_solve_pair(
    float leg_length, const float left_state[VMC_MPC_STATE_COUNT],
    const float right_state[VMC_MPC_STATE_COUNT],
    const vmc_mpc_limits_t *limits, vmc_mpc_output_t *left_output,
    vmc_mpc_output_t *right_output)
{
  float a[VMC_MPC_STATE_COUNT][VMC_MPC_STATE_COUNT];
  float b[VMC_MPC_STATE_COUNT][VMC_MPC_INPUT_COUNT];
  float terminal_cost[VMC_MPC_STATE_COUNT][VMC_MPC_STATE_COUNT];
  float f[VMC_MPC_DECISION_COUNT];
  float left_solution[VMC_MPC_DECISION_COUNT];
  float right_solution[VMC_MPC_DECISION_COUNT];
  uint32_t state_index;
  uint8_t left_valid;
  uint8_t right_valid;

  if (left_state == NULL || right_state == NULL || limits == NULL ||
      left_output == NULL || right_output == NULL) {
    return 0U;
  }
  memset(left_output, 0, sizeof(*left_output));
  memset(right_output, 0, sizeof(*right_output));
  for (state_index = 0U; state_index < VMC_MPC_STATE_COUNT;
       state_index++) {
    if (!isfinite(left_state[state_index]) ||
        !isfinite(right_state[state_index])) {
      return 0U;
    }
  }

  interpolate_model(leg_length, a, b, terminal_cost, vmc_mpc_g,
                    vmc_mpc_h_factor);
  build_base_predictions(a, left_state, right_state);
  add_admm_regularization();
  if (factor_hessian() == 0U) {
    vmc_mpc_axis[0].failure_count++;
    vmc_mpc_axis[1].failure_count++;
    return 0U;
  }

  build_rhs(vmc_mpc_base_left, vmc_mpc_axis[0].u_prev, f, terminal_cost);
  (void)solve_box_qp(&vmc_mpc_axis[0], f, limits, left_solution);
  left_valid =
      finalize_output(&vmc_mpc_axis[0], left_solution, limits, left_output);

  build_rhs(vmc_mpc_base_right, vmc_mpc_axis[1].u_prev, f, terminal_cost);
  (void)solve_box_qp(&vmc_mpc_axis[1], f, limits, right_solution);
  right_valid =
      finalize_output(&vmc_mpc_axis[1], right_solution, limits, right_output);
  return (left_valid != 0U && right_valid != 0U) ? 1U : 0U;
}

uint8_t vmc_mpc_solve(uint8_t side, float leg_length,
                      const float state[VMC_MPC_STATE_COUNT],
                      const vmc_mpc_limits_t *limits,
                      vmc_mpc_output_t *output)
{
  vmc_mpc_output_t ignored;

  if (side == 0U) {
    return vmc_mpc_solve_pair(leg_length, state, state, limits, output,
                              &ignored);
  }
  if (side == 1U) {
    return vmc_mpc_solve_pair(leg_length, state, state, limits, &ignored,
                              output);
  }
  return 0U;
}
