#include "vmc_lqr_rtt.h"

#include "SEGGER_RTT.h"
#include "board_comm.h"
#include "remote_control.h"
#include "stm32f4xx_hal.h"
#include "vmc_probe.h"
#include "vmc_math.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VMC_LQR_RTT_BUFFER_INDEX 0U
#define VMC_LQR_RTT_LINE_MAX     1536U
#define VMC_LQR_RTT_READ_CHUNK   64U
#define VMC_LQR_RTT_TELEM_PERIOD_MS 20U
#define VMC_LQR_RTT_TELEM_SCALE 1000L

#define VMC_LQR_RTT_ERR_NONE      0U
#define VMC_LQR_RTT_ERR_OVERFLOW  1U
#define VMC_LQR_RTT_ERR_TYPE      2U
#define VMC_LQR_RTT_ERR_K_COUNT   3U
#define VMC_LQR_RTT_ERR_PARAMS    4U

/* Bumped whenever the VMCVOFA layout or the encoding of an existing field
 * changes.  Field count alone is not enough: Q/R switched from a three-decimal
 * to a nine-decimal fixed point without adding or removing any field. */
#define VMC_LQR_RTT_SCHEMA_VERSION 8U

static char lqr_rtt_line[VMC_LQR_RTT_LINE_MAX];
static uint16_t lqr_rtt_line_len;
static uint8_t lqr_rtt_discard_until_newline;
static uint32_t lqr_rtt_last_telem_ms;

static void vmc_lqr_rtt_ack(const char *text)
{
  (void)SEGGER_RTT_WriteString(VMC_LQR_RTT_BUFFER_INDEX, text);
}

static void vmc_lqr_rtt_set_error(uint8_t error)
{
  vmc_probe_state.lqr.rtt_last_error = error;
  vmc_probe_state.lqr.rtt_error_count++;
}

static char *vmc_lqr_rtt_find_k_array(char *line)
{
  char *key = strstr(line, "k_flat_col_major");
  if (key == NULL) {
    key = strstr(line, "LQRK");
  }
  if (key == NULL) {
    return NULL;
  }

  while (*key != '\0' && *key != '[' && *key != ' ' && *key != '\t') {
    key++;
  }
  if (*key == '[') {
    key++;
  }
  return key;
}

static uint8_t vmc_lqr_rtt_parse_k(char *line, double k[12])
{
  char *cursor = vmc_lqr_rtt_find_k_array(line);
  char *endptr;
  uint8_t count = 0U;

  if (cursor == NULL) {
    return 0U;
  }

  while (*cursor != '\0' && count < 12U) {
    while (*cursor == '[' || *cursor == ',' || *cursor == ':' ||
           *cursor == ' ' || *cursor == '\t' || *cursor == '\r' ||
           *cursor == '\n') {
      cursor++;
    }
    if (*cursor == ']' || *cursor == '\0') {
      break;
    }

    k[count] = strtod(cursor, &endptr);
    if (endptr == cursor) {
      return 0U;
    }
    cursor = endptr;
    count++;
  }

  return (count == 12U) ? 1U : 0U;
}

static uint8_t vmc_lqr_rtt_parse_named_float(char *line,
                                             const char *name,
                                             float *value)
{
  char *cursor = line;
  char *endptr;
  double parsed;
  const size_t name_len = strlen(name);

  while ((cursor = strstr(cursor, name)) != NULL) {
    const char before = (cursor == line) ? '\0' : cursor[-1];
    const char after = cursor[name_len];
    const uint8_t quoted_key = (before == '"' && after == '"') ? 1U : 0U;
    const uint8_t bare_key =
        ((before == '\0' || before == '{' || before == ',' ||
          before == ' ' || before == '\t') &&
         (after == ':' || after == ' ' || after == '\t')) ? 1U : 0U;

    if (quoted_key != 0U || bare_key != 0U) {
      cursor += name_len;
      cursor = strchr(cursor, ':');
      if (cursor == NULL) {
        return 0U;
      }
      cursor++;

      parsed = strtod(cursor, &endptr);
      if (endptr == cursor) {
        return 0U;
      }

      *value = (float)parsed;
      return 1U;
    }
    cursor += name_len;
  }
  return 0U;
}

static float vmc_lqr_rtt_clamp_float(float value, float min_value, float max_value)
{
  if (value > max_value) {
    return max_value;
  }
  if (value < min_value) {
    return min_value;
  }
  return value;
}

static void vmc_lqr_rtt_apply_k(const double k[12])
{
  uint8_t i;

  for (i = 0U; i < 12U; i++) {
    vmc_probe_state.lqr.rtt_k[i] = k[i];
    vmc_probe_state.lqr.left_k[i] = k[i];
    vmc_probe_state.lqr.right_k[i] = k[i];
  }

  vmc_probe_state.lqr.rtt_override_active = 1U;
  vmc_probe_state.lqr.rtt_last_error = VMC_LQR_RTT_ERR_NONE;
  vmc_probe_state.lqr.rtt_update_count++;
  vmc_probe_state.lqr.rtt_last_update_ms = HAL_GetTick();
}

static uint8_t vmc_lqr_rtt_apply_balance_params(char *line)
{
  float value;
  uint8_t applied = 0U;

  /* The web tuner sends these four values as a separate
   * chassis_kalman_params message.  Keep the parser here so both the
   * combined balance payload and the dedicated Kalman payload use the same
   * validation and runtime storage. */
  if (vmc_lqr_rtt_parse_named_float(line, "q_v", &value) != 0U) {
    vmc_balance.kalman_q_v = vmc_lqr_rtt_clamp_float(value, 0.0f, 1.0e12f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "q_a", &value) != 0U) {
    vmc_balance.kalman_q_a = vmc_lqr_rtt_clamp_float(value, 0.0f, 1.0e12f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "r_wheel", &value) != 0U) {
    vmc_balance.kalman_r_wheel =
        vmc_lqr_rtt_clamp_float(value, 1.0e-9f, 1.0e12f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "r_imu", &value) != 0U) {
    vmc_balance.kalman_r_imu =
        vmc_lqr_rtt_clamp_float(value, 1.0e-9f, 1.0e12f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "slip_enable", &value) != 0U) {
    vmc_balance.kalman_slip_enable = (value > 0.5f) ? 1U : 0U;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "slip_accel", &value) != 0U) {
    vmc_balance.kalman_slip_accel = vmc_lqr_rtt_clamp_float(value, 0.0f, 1.0e4f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "slip_vel_diff", &value) != 0U) {
    vmc_balance.kalman_slip_vel_diff =
        vmc_lqr_rtt_clamp_float(value, 0.0f, 1.0e3f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "slip_yaw_track", &value) != 0U) {
    vmc_balance.kalman_slip_yaw_track = vmc_lqr_rtt_clamp_float(value, 0.0f, 2.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "slip_yaw", &value) != 0U) {
    vmc_balance.kalman_slip_yaw = vmc_lqr_rtt_clamp_float(value, 0.0f, 1.0e3f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "slip_hold_ms", &value) != 0U) {
    vmc_balance.kalman_slip_hold_ms =
        (uint32_t)vmc_lqr_rtt_clamp_float(value, 0.0f, 5000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "slip_confirm_ms", &value) != 0U) {
    vmc_balance.kalman_slip_confirm_ms =
        (uint32_t)vmc_lqr_rtt_clamp_float(value, 0.0f, 1000.0f);
    applied = 1U;
  }

  /* theta_target is the wheel-leg theta target; pitch_offset is the legacy
     internal field name kept for backwards-compatible RTT payloads. */
  if (vmc_lqr_rtt_parse_named_float(line, "theta_target", &value) != 0U ||
      vmc_lqr_rtt_parse_named_float(line, "target_theta", &value) != 0U ||
      vmc_lqr_rtt_parse_named_float(line, "pitch_offset", &value) != 0U) {
    vmc_balance.pitch_offset = vmc_lqr_rtt_clamp_float(value, -0.50f, 0.50f);
    applied = 1U;
  }
  /* pitch_target is the body pitch/phi target; phi_offset is the legacy
     internal field name kept for backwards-compatible RTT payloads. */
  if (vmc_lqr_rtt_parse_named_float(line, "pitch_target", &value) != 0U ||
      vmc_lqr_rtt_parse_named_float(line, "target_pitch", &value) != 0U ||
      vmc_lqr_rtt_parse_named_float(line, "phi_offset", &value) != 0U) {
    vmc_balance.phi_offset = vmc_lqr_rtt_clamp_float(value, -0.50f, 0.50f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "phi_deadband", &value) != 0U ||
      vmc_lqr_rtt_parse_named_float(line, "pitch_deadband", &value) != 0U) {
    vmc_balance.phi_deadband = vmc_lqr_rtt_clamp_float(value, 0.0f, 0.20f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "tp_output_enable", &value) != 0U) {
    vmc_balance.tp_output_enable = (value > 0.5f) ? 1U : 0U;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "wheel_output_user_enable", &value) != 0U ||
      vmc_lqr_rtt_parse_named_float(line, "wheel_output_enable", &value) != 0U) {
    vmc_balance.wheel_output_user_enable = (value > 0.5f) ? 1U : 0U;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "theta_deadband", &value) != 0U) {
    vmc_balance.theta_deadband = vmc_lqr_rtt_clamp_float(value, 0.0f, 0.20f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "lqr_dtheta_gain", &value) != 0U) {
    vmc_balance.lqr_dtheta_gain = vmc_lqr_rtt_clamp_float(value, -10.0f, 10.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "lqr_dtheta_deadband", &value) != 0U) {
    vmc_balance.lqr_dtheta_deadband = vmc_lqr_rtt_clamp_float(value, 0.0f, 5.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "lqr_dtheta_limit", &value) != 0U) {
    vmc_balance.lqr_dtheta_limit = vmc_lqr_rtt_clamp_float(value, 0.0f, 50.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "wheel_torque_limit", &value) != 0U) {
    vmc_balance.wheel_torque_limit = vmc_lqr_rtt_clamp_float(value, 0.0f, 20.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "wheel_current_limit", &value) != 0U) {
    vmc_balance.wheel_current_limit =
        (int16_t)vmc_lqr_rtt_clamp_float(
            value, 0.0f, (float)VMC_WHEEL_CURRENT_MAX_DEFAULT);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "airborne_detect_enable", &value) != 0U) {
    vmc_balance.airborne_detect_enable = (value > 0.5f) ? 1U : 0U;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "airborne_wheel_speed_threshold", &value) != 0U) {
    vmc_balance.airborne_wheel_speed_threshold =
        vmc_lqr_rtt_clamp_float(value, 0.1f, 30.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "airborne_joint_torque_threshold", &value) != 0U) {
    vmc_balance.airborne_joint_torque_threshold =
        vmc_lqr_rtt_clamp_float(value, 0.1f, 54.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "airborne_confirm_ms", &value) != 0U) {
    vmc_balance.airborne_confirm_ms =
        (uint32_t)vmc_lqr_rtt_clamp_float(value, 0.0f, 10000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "airborne_release_ms", &value) != 0U) {
    vmc_balance.airborne_release_ms =
        (uint32_t)vmc_lqr_rtt_clamp_float(value, 0.0f, 10000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "airborne_pitch_dev_max", &value) != 0U) {
    vmc_balance.airborne_pitch_dev_max =
        vmc_lqr_rtt_clamp_float(value, 0.01f, 3.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "airborne_pitch_rate_max", &value) != 0U) {
    vmc_balance.airborne_pitch_rate_max =
        vmc_lqr_rtt_clamp_float(value, 0.1f, 30.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "airborne_side_enable", &value) != 0U) {
    vmc_balance.airborne_side_enable = (value > 0.5f) ? 1U : 0U;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "airborne_side_confirm_ms", &value) != 0U) {
    vmc_balance.airborne_side_confirm_ms =
        (uint32_t)vmc_lqr_rtt_clamp_float(value, 0.0f, 10000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "airborne_side_release_ms", &value) != 0U) {
    vmc_balance.airborne_side_release_ms =
        (uint32_t)vmc_lqr_rtt_clamp_float(value, 0.0f, 10000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "tp_limit", &value) != 0U) {
    vmc_balance.tp_limit = vmc_lqr_rtt_clamp_float(value, 0.0f, VMC_LEG_TP_MAX);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "tp_phi_gain", &value) != 0U) {
    vmc_balance.tp_phi_gain = vmc_lqr_rtt_clamp_float(value, -2.0f, 2.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "tp_x_gain", &value) != 0U) {
    vmc_balance.tp_x_gain = vmc_lqr_rtt_clamp_float(value, -2.0f, 2.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "dpitch_lpf_alpha", &value) != 0U) {
    vmc_balance.dpitch_lpf_alpha = vmc_lqr_rtt_clamp_float(value, 0.001f, 1.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "disturbance_observer_enable", &value) != 0U) {
    vmc_balance.disturbance_observer_enable = (value > 0.5f) ? 1U : 0U;
    if (vmc_balance.disturbance_observer_enable == 0U) {
      vmc_balance.disturbance_observer_output = 0.0f;
    }
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "disturbance_observer_gain", &value) != 0U) {
    vmc_balance.disturbance_observer_gain =
        vmc_lqr_rtt_clamp_float(value, -20.0f, 20.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "disturbance_observer_leak", &value) != 0U) {
    vmc_balance.disturbance_observer_leak =
        vmc_lqr_rtt_clamp_float(value, 0.0f, 2.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "disturbance_observer_limit", &value) != 0U) {
    vmc_balance.disturbance_observer_limit =
        vmc_lqr_rtt_clamp_float(value, 0.0f, 0.5f);
    if (vmc_balance.disturbance_observer_output > vmc_balance.disturbance_observer_limit) {
      vmc_balance.disturbance_observer_output = vmc_balance.disturbance_observer_limit;
    } else if (vmc_balance.disturbance_observer_output <
               -vmc_balance.disturbance_observer_limit) {
      vmc_balance.disturbance_observer_output = -vmc_balance.disturbance_observer_limit;
    }
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "disturbance_observer_speed_limit", &value) != 0U) {
    vmc_balance.disturbance_observer_speed_limit =
        vmc_lqr_rtt_clamp_float(value, 0.01f, 2.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "disturbance_observer_delay", &value) != 0U) {
    vmc_balance.disturbance_observer_delay =
        vmc_lqr_rtt_clamp_float(value, 0.0f, 5.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "disturbance_observer_pitch_gain", &value) != 0U) {
    vmc_balance.disturbance_observer_pitch_gain =
        vmc_lqr_rtt_clamp_float(value, -2.0f, 2.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "disturbance_observer_theta_gain", &value) != 0U) {
    vmc_balance.disturbance_observer_theta_gain =
        vmc_lqr_rtt_clamp_float(value, -2.0f, 2.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "x_feedback_gain", &value) != 0U) {
    vmc_balance.x_feedback_gain = vmc_lqr_rtt_clamp_float(value, -200.0f, 200.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "dx_feedback_gain", &value) != 0U) {
    vmc_balance.dx_feedback_gain = vmc_lqr_rtt_clamp_float(value, -200.0f, 200.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "x_feedback_limit", &value) != 0U) {
    vmc_balance.x_feedback_limit =
        vmc_lqr_rtt_clamp_float(value, 0.0f, VMC_BODY_MOTION_X_LIMIT_DEFAULT);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "dx_feedback_limit", &value) != 0U) {
    vmc_balance.dx_feedback_limit = vmc_lqr_rtt_clamp_float(value, 0.0f, 30.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "speed_control_enable", &value) != 0U) {
    vmc_balance.speed_control_enable = (value > 0.5f) ? 1U : 0U;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "speed_target_max", &value) != 0U) {
    vmc_balance.speed_target_max = vmc_lqr_rtt_clamp_float(value, 0.0f, 3.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "speed_ramp_rate", &value) != 0U) {
    vmc_balance.speed_ramp_rate = vmc_lqr_rtt_clamp_float(value, 0.0f, 10.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "speed_deadband", &value) != 0U) {
    vmc_balance.speed_deadband = vmc_lqr_rtt_clamp_float(value, 0.0f, 0.5f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "stop_speed_threshold", &value) != 0U) {
    vmc_balance.stop_speed_threshold = vmc_lqr_rtt_clamp_float(value, 0.0f, 1.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "x_hold_max_stretch", &value) != 0U) {
    vmc_balance.x_hold_max_stretch =
        vmc_lqr_rtt_clamp_float(value, 0.0f, VMC_BODY_MOTION_X_LIMIT_DEFAULT);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "speed_pitch_gain", &value) != 0U) {
    vmc_balance.speed_pitch_gain = vmc_lqr_rtt_clamp_float(value, -1.0f, 1.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "speed_pitch_limit", &value) != 0U) {
    vmc_balance.speed_pitch_limit = vmc_lqr_rtt_clamp_float(value, 0.0f, 0.5f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "speed_pitch_ramp_rate", &value) != 0U) {
    vmc_balance.speed_pitch_ramp_rate = vmc_lqr_rtt_clamp_float(value, 0.0f, 5.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "kp_length", &value) != 0U) {
    vmc_control.kp_length = vmc_lqr_rtt_clamp_float(value, 0.0f, 5000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "kd_length", &value) != 0U) {
    vmc_control.kd_length = vmc_lqr_rtt_clamp_float(value, 0.0f, 1000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "max_force", &value) != 0U) {
    vmc_control.max_force = vmc_lqr_rtt_clamp_float(value, 0.0f, VMC_LEG_FORCE_MAX_DEFAULT);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "support_force", &value) != 0U) {
    vmc_control.support_force = vmc_lqr_rtt_clamp_float(value,
                                                       -VMC_LEG_SUPPORT_FORCE_MAX,
                                                       VMC_LEG_SUPPORT_FORCE_MAX);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "length_ramp_rate", &value) != 0U) {
    vmc_control.length_ramp_rate = vmc_lqr_rtt_clamp_float(value, 0.0f, 1.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_enable", &value) != 0U) {
    vmc_control.jump_enable = (value > 0.5f) ? 1U : 0U;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_moving_enable", &value) != 0U) {
    vmc_control.jump_moving_enable = (value > 0.5f) ? 1U : 0U;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_compress_length", &value) != 0U) {
    vmc_control.jump_compress_length =
        vmc_lqr_rtt_clamp_float(value, VMC_LEG_LENGTH_MIN, VMC_LEG_LENGTH_MAX);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_extend_length", &value) != 0U) {
    vmc_control.jump_extend_length =
        vmc_lqr_rtt_clamp_float(value, VMC_LEG_LENGTH_MIN, VMC_LEG_LENGTH_MAX);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_retract_length", &value) != 0U) {
    vmc_control.jump_retract_length =
        vmc_lqr_rtt_clamp_float(value, VMC_LEG_LENGTH_MIN, VMC_LEG_LENGTH_MAX);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_land_length", &value) != 0U) {
    vmc_control.jump_land_length =
        vmc_lqr_rtt_clamp_float(value, VMC_LEG_LENGTH_MIN, VMC_LEG_LENGTH_MAX);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_kp_length", &value) != 0U) {
    vmc_control.jump_kp_length = vmc_lqr_rtt_clamp_float(value, 0.0f, 5000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_kd_length", &value) != 0U) {
    vmc_control.jump_kd_length = vmc_lqr_rtt_clamp_float(value, 0.0f, 1000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_max_force", &value) != 0U) {
    vmc_control.jump_max_force =
        vmc_lqr_rtt_clamp_float(value, 0.0f, VMC_LEG_FORCE_MAX_DEFAULT);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_support_force", &value) != 0U) {
    vmc_control.jump_support_force =
        vmc_lqr_rtt_clamp_float(value,
                                -VMC_LEG_SUPPORT_FORCE_MAX,
                                VMC_LEG_SUPPORT_FORCE_MAX);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_compress_timeout_ms", &value) != 0U) {
    vmc_control.jump_compress_timeout_ms =
        (uint32_t)vmc_lqr_rtt_clamp_float(value, 1.0f, 2000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_extend_timeout_ms", &value) != 0U) {
    vmc_control.jump_extend_timeout_ms =
        (uint32_t)vmc_lqr_rtt_clamp_float(value, 1.0f, 2000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_retract_timeout_ms", &value) != 0U) {
    vmc_control.jump_retract_timeout_ms =
        (uint32_t)vmc_lqr_rtt_clamp_float(value, 1.0f, 2000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_land_timeout_ms", &value) != 0U) {
    vmc_control.jump_land_timeout_ms =
        (uint32_t)vmc_lqr_rtt_clamp_float(value, 1.0f, 3000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_cooldown_ms", &value) != 0U) {
    vmc_control.jump_cooldown_ms =
        (uint32_t)vmc_lqr_rtt_clamp_float(value, 0.0f, 5000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_total_timeout_ms", &value) != 0U) {
    vmc_control.jump_total_timeout_ms =
        (uint32_t)vmc_lqr_rtt_clamp_float(value, 100.0f, 5000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_attitude_limit", &value) != 0U) {
    vmc_control.jump_attitude_limit = vmc_lqr_rtt_clamp_float(value, 0.05f, 1.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_land_torque_threshold", &value) != 0U) {
    vmc_control.jump_land_torque_threshold =
        vmc_lqr_rtt_clamp_float(value, 0.1f, VMC_DM_J8009_TMAX);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "jump_land_confirm_ms", &value) != 0U) {
    vmc_control.jump_land_confirm_ms =
        (uint32_t)vmc_lqr_rtt_clamp_float(value, 0.0f, 1000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "roll_comp_enable", &value) != 0U) {
    vmc_control.roll_comp_enable = (value > 0.5f) ? 1U : 0U;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "roll_target", &value) != 0U) {
    vmc_control.roll_target = vmc_lqr_rtt_clamp_float(value, -0.50f, 0.50f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "roll_comp_gain", &value) != 0U) {
    vmc_control.roll_comp_gain = vmc_lqr_rtt_clamp_float(value, -0.50f, 0.50f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "roll_comp_limit", &value) != 0U) {
    vmc_control.roll_comp_limit = vmc_lqr_rtt_clamp_float(value, 0.0f, 0.10f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "roll_comp_ramp_rate", &value) != 0U) {
    vmc_control.roll_comp_ramp_rate = vmc_lqr_rtt_clamp_float(value, 0.0f, 1.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "kp_phi0", &value) != 0U) {
    vmc_control.kp_phi0 = vmc_lqr_rtt_clamp_float(value, 0.0f, 500.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "kd_phi0", &value) != 0U) {
    vmc_control.kd_phi0 = vmc_lqr_rtt_clamp_float(value, 0.0f, 50.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "max_phi0_torque", &value) != 0U) {
    vmc_control.max_phi0_torque = vmc_lqr_rtt_clamp_float(value, 0.0f, VMC_LEG_TP_MAX);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "yaw_hold_enable", &value) != 0U) {
    vmc_balance.yaw_hold_enable = (value > 0.5f) ? 1U : 0U;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "yaw_hold_kp", &value) != 0U) {
    vmc_balance.yaw_hold_kp = vmc_lqr_rtt_clamp_float(value, -5000.0f, 5000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "yaw_hold_kd", &value) != 0U) {
    vmc_balance.yaw_hold_kd = vmc_lqr_rtt_clamp_float(value, -5000.0f, 5000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "yaw_hold_deadband", &value) != 0U) {
    vmc_balance.yaw_hold_deadband = vmc_lqr_rtt_clamp_float(value, 0.0f, 0.50f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "yaw_target", &value) != 0U) {
    vmc_balance.yaw_target = value;
    vmc_balance.yaw_hold_active = 1U;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "yaw_target_rate_max", &value) != 0U ||
      vmc_lqr_rtt_parse_named_float(line, "turn_yaw_rate_max", &value) != 0U) {
    vmc_balance.yaw_target_rate_max = vmc_lqr_rtt_clamp_float(value, 0.0f, 10.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "yaw_hold_current_limit", &value) != 0U) {
    vmc_balance.yaw_hold_current_limit =
        (int16_t)vmc_lqr_rtt_clamp_float(value, 0.0f, VMC_WHEEL_CURRENT_MAX_DEFAULT);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "gimbal_follow_enable", &value) != 0U) {
    vmc_balance.gimbal_follow_enable = (value > 0.5f) ? 1U : 0U;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "gimbal_follow_front_ecd", &value) != 0U) {
    vmc_balance.gimbal_follow_front_ecd =
        (uint16_t)vmc_lqr_rtt_clamp_float(value, 0.0f, 8191.0f);
    vmc_balance.gimbal_follow_front_valid = 1U;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "gimbal_follow_front_valid", &value) != 0U) {
    vmc_balance.gimbal_follow_front_valid = (value > 0.5f) ? 1U : 0U;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "gimbal_follow_capture_front", &value) != 0U) {
    if (value <= 0.5f) {
      applied = 1U;
    } else if (board_comm_gimbal_state_is_valid() != 0U) {
      vmc_balance.gimbal_follow_front_ecd = board_comm_state.yaw_ecd;
      vmc_balance.gimbal_follow_front_valid = 1U;
      applied = 1U;
    }
  }
  if (vmc_lqr_rtt_parse_named_float(line, "gimbal_follow_kp", &value) != 0U) {
    vmc_balance.gimbal_follow_kp =
        vmc_lqr_rtt_clamp_float(value, 0.0f, 10000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "gimbal_follow_kd", &value) != 0U) {
    vmc_balance.gimbal_follow_kd =
        vmc_lqr_rtt_clamp_float(value, 0.0f, 10000.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "gimbal_follow_deadband", &value) != 0U) {
    vmc_balance.gimbal_follow_deadband =
        vmc_lqr_rtt_clamp_float(value, 0.0f, 4096.0f);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "gimbal_follow_current_limit", &value) != 0U) {
    vmc_balance.gimbal_follow_current_limit =
        (int16_t)vmc_lqr_rtt_clamp_float(value, 0.0f,
                                        VMC_WHEEL_CURRENT_MAX_DEFAULT);
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "gimbal_follow_direction", &value) != 0U) {
    vmc_balance.gimbal_follow_direction = (value < 0.0f) ? -1.0f : 1.0f;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "turn_current_max", &value) != 0U) {
    vmc_balance.turn_current_max =
        (int16_t)vmc_lqr_rtt_clamp_float(value, 0.0f, VMC_WHEEL_CURRENT_MAX_DEFAULT);
    applied = 1U;
  }

  if (applied != 0U) {
    vmc_probe_state.balance = vmc_balance;
    vmc_probe_state.lqr.rtt_last_error = VMC_LQR_RTT_ERR_NONE;
    vmc_probe_state.lqr.rtt_update_count++;
    vmc_probe_state.lqr.rtt_last_update_ms = HAL_GetTick();
  }

  return applied;
}

static void vmc_lqr_rtt_process_line(char *line)
{
  double k[12];

  if (strstr(line, "LQRCLEAR") != NULL ||
      strstr(line, "vmc_lqr_clear") != NULL) {
    vmc_probe_state.lqr.rtt_override_active = 0U;
    vmc_probe_state.lqr.rtt_last_error = VMC_LQR_RTT_ERR_NONE;
    vmc_lqr_rtt_ack("VMC_LQR_RTT CLEAR\n");
    return;
  }

  if (strstr(line, "vmc_balance_params") != NULL) {
    if (vmc_lqr_rtt_apply_balance_params(line) == 0U) {
      vmc_lqr_rtt_set_error(VMC_LQR_RTT_ERR_PARAMS);
      vmc_lqr_rtt_ack("VMC_LQR_RTT ERR PARAMS\n");
      return;
    }
    vmc_lqr_rtt_ack("VMC_BALANCE_PARAMS OK\n");
    return;
  }

  if (strstr(line, "chassis_kalman_params") != NULL) {
    if (vmc_lqr_rtt_apply_balance_params(line) == 0U) {
      vmc_lqr_rtt_set_error(VMC_LQR_RTT_ERR_PARAMS);
      vmc_lqr_rtt_ack("VMC_LQR_RTT ERR PARAMS\n");
      return;
    }
    vmc_lqr_rtt_ack("CHASSIS_KALMAN_PARAMS OK\n");
    return;
  }

  if (strstr(line, "vmc_lqr_k") == NULL &&
      strstr(line, "LQRK") == NULL) {
    vmc_lqr_rtt_set_error(VMC_LQR_RTT_ERR_TYPE);
    vmc_lqr_rtt_ack("VMC_LQR_RTT ERR TYPE\n");
    return;
  }

  if (vmc_lqr_rtt_parse_k(line, k) == 0U) {
    vmc_lqr_rtt_set_error(VMC_LQR_RTT_ERR_K_COUNT);
    vmc_lqr_rtt_ack("VMC_LQR_RTT ERR K\n");
    return;
  }

  vmc_lqr_rtt_apply_k(k);
  vmc_lqr_rtt_ack("VMC_LQR_RTT OK\n");
}

void vmc_lqr_rtt_init(void)
{
  SEGGER_RTT_Init();
  lqr_rtt_line_len = 0U;
  lqr_rtt_discard_until_newline = 0U;
}

void vmc_lqr_rtt_update(void)
{
  uint8_t data[VMC_LQR_RTT_READ_CHUNK];
  unsigned read_len;
  unsigned i;

  do {
    read_len = SEGGER_RTT_Read(VMC_LQR_RTT_BUFFER_INDEX, data, sizeof(data));
    if (read_len == 0U) {
      break;
    }

    vmc_probe_state.lqr.rtt_rx_count += read_len;
    for (i = 0U; i < read_len; i++) {
      const char c = (char)data[i];

      if (lqr_rtt_discard_until_newline != 0U) {
        if (c == '\n' || c == '\r') {
          lqr_rtt_discard_until_newline = 0U;
        }
        continue;
      }

      if (c == '\n' || c == '\r') {
        if (lqr_rtt_line_len > 0U) {
          lqr_rtt_line[lqr_rtt_line_len] = '\0';
          vmc_lqr_rtt_process_line(lqr_rtt_line);
          lqr_rtt_line_len = 0U;
        }
        continue;
      }

      if (lqr_rtt_line_len >= (VMC_LQR_RTT_LINE_MAX - 1U)) {
        lqr_rtt_line_len = 0U;
        lqr_rtt_discard_until_newline = 1U;
        vmc_lqr_rtt_set_error(VMC_LQR_RTT_ERR_OVERFLOW);
        vmc_lqr_rtt_ack("VMC_LQR_RTT ERR OVERFLOW\n");
        continue;
      }

      lqr_rtt_line[lqr_rtt_line_len++] = c;
    }
  } while (read_len == sizeof(data));
}

static int vmc_lqr_rtt_append_scaled(char *line, int offset, int max_len, double value)
{
  long scaled = (long)(value * (double)VMC_LQR_RTT_TELEM_SCALE);
  const int negative = (scaled < 0L || (value < 0.0 && scaled == 0L)) ? 1 : 0;
  long whole;
  long frac;

  if (scaled < 0L) {
    scaled = -scaled;
  } else if (negative != 0) {
    scaled = 1L;
  }

  whole = scaled / VMC_LQR_RTT_TELEM_SCALE;
  frac = scaled % VMC_LQR_RTT_TELEM_SCALE;

  return offset + snprintf(&line[offset],
                           (size_t)(max_len - offset),
                           negative != 0 ? ",-%ld.%03ld" : ",%ld.%03ld",
                           whole,
                           frac);
}

static int vmc_lqr_rtt_append_u32(char *line, int offset, int max_len,
                                  uint32_t value)
{
  return offset + snprintf(&line[offset],
                           (size_t)(max_len - offset),
                           ",%lu",
                           (unsigned long)value);
}

/* Frame age in milliseconds, or -1.0 when no frame has arrived yet.
 *
 * The producer timestamps are written from CAN/UART callbacks while "now" is
 * sampled at the top of the telemetry function.  A tick can therefore land
 * between the two reads and make the stored timestamp newer than "now"; the
 * unsigned subtraction then wraps to about 2^32, which overflows the 32-bit
 * scaler in vmc_lqr_rtt_append_scaled and reports a nonsense age.  Clamp that
 * case to zero: the frame arrived within the current tick. */
static double vmc_lqr_rtt_frame_age_ms(uint32_t now, uint32_t last_ms)
{
  if (last_ms == 0U) {
    return -1.0;
  }
  if (last_ms >= now) {
    return 0.0;
  }
  return (double)(now - last_ms);
}

/* Q/R span a huge dynamic range: a physically calibrated r_wheel is on the
 * order of 1e-4 m2/s2 while the legacy defaults were on the order of 1e2, and
 * the three-decimal wire format rounds the small end to zero.  Q/R therefore
 * use a nine-decimal fixed point.  newlib-nano has no "%lld", so the integer
 * part is split into two 32-bit chunks. */
#define VMC_LQR_RTT_QR_SCALE 1000000000LL

static int vmc_lqr_rtt_append_qr(char *line, int offset, int max_len,
                                 double value)
{
  long long scaled;
  unsigned long long magnitude;
  unsigned long long whole;
  unsigned long long frac;
  unsigned long long whole_high;
  unsigned long long whole_low;
  int negative;
  int written;

  if (value > 9.0e9) {
    value = 9.0e9;
  } else if (value < -9.0e9) {
    value = -9.0e9;
  }
  scaled = (long long)(value * (double)VMC_LQR_RTT_QR_SCALE);
  negative = (scaled < 0LL || (value < 0.0 && scaled == 0LL)) ? 1 : 0;

  if (scaled < 0LL) {
    magnitude = (unsigned long long)(-(scaled + 1LL)) + 1ULL;
  } else {
    magnitude = (unsigned long long)scaled;
  }

  whole = magnitude / (unsigned long long)VMC_LQR_RTT_QR_SCALE;
  frac = magnitude % (unsigned long long)VMC_LQR_RTT_QR_SCALE;
  whole_high = whole / 1000000000ULL;
  whole_low = whole % 1000000000ULL;

  written = snprintf(&line[offset], (size_t)(max_len - offset),
                     negative != 0 ? ",-" : ",");
  offset += written;
  if (whole_high != 0ULL) {
    written = snprintf(&line[offset], (size_t)(max_len - offset),
                       "%lu%09lu.%09lu",
                       (unsigned long)whole_high,
                       (unsigned long)whole_low,
                       (unsigned long)frac);
  } else {
    written = snprintf(&line[offset], (size_t)(max_len - offset),
                       "%lu.%09lu",
                       (unsigned long)whole_low,
                       (unsigned long)frac);
  }
  return offset + written;
}

void vmc_lqr_rtt_telemetry_update(void)
{
  const uint32_t now = HAL_GetTick();
  const volatile vmc_balance_t *b = &vmc_balance;
  const volatile vmc_motion_state_t *m = &vmc_probe_state.motion;
  const volatile vmc_body_state_t *body = &vmc_probe_state.body;
  /* Keep the large telemetry line out of the VMC task stack.  This function
   * runs from one task only, so a static buffer is safe here. */
  static char line[VMC_LQR_RTT_LINE_MAX];
  int offset;
  const double tp_phi_gain = b->tp_phi_gain;
  const double left_tp_phi_term = tp_phi_gain * vmc_probe_state.lqr.left_k[9] * b->phi_error;
  const double left_tp_dphi_term = tp_phi_gain * vmc_probe_state.lqr.left_k[11] * b->dpitch_lpf;
  const double right_tp_phi_term = tp_phi_gain * vmc_probe_state.lqr.right_k[9] * b->phi_error;
  const double right_tp_dphi_term = tp_phi_gain * vmc_probe_state.lqr.right_k[11] * b->dpitch_lpf;

  if ((uint32_t)(now - lqr_rtt_last_telem_ms) < VMC_LQR_RTT_TELEM_PERIOD_MS) {
    return;
  }
  lqr_rtt_last_telem_ms = now;

  offset = snprintf(line, sizeof(line), "VMCVOFA");
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), body->pitch);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), body->pitch_rate);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->lqr_theta);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->lqr_d_theta);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), m->kf_x);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), m->kf_dx);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), m->left_theta);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_lqr_d_theta_limited);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), m->right_theta);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_lqr_d_theta_limited);

  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_wheel_theta_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_wheel_dtheta_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_wheel_x_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_wheel_dx_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_wheel_phi_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_wheel_dphi_term);
  /* These legacy VOFA channels are now motor-shaft torque (N.m). */
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_wheel_torque_raw);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_wheel_current_send);

  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_wheel_theta_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_wheel_dtheta_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_wheel_x_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_wheel_dx_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_wheel_phi_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_wheel_dphi_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_wheel_torque_raw);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_wheel_current_send);

  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_tp_theta_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_tp_dtheta_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_tp_x_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_tp_dx_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), left_tp_phi_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), left_tp_dphi_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_tp_raw);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_tp_cmd);

  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_tp_theta_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_tp_dtheta_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_tp_x_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_tp_dx_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), right_tp_phi_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), right_tp_dphi_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_tp_raw);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->right_tp_cmd);

  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->x_offset);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->x_error);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->left_wheel_x_term);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line), b->dpitch_lpf);

  /* VMCVOFA fields 46..61: RC source, RC-frame age, state-frame age,
   * right-stick yaw, left-stick turn, yaw ECD, front ECD, front valid,
   * wrapped error, yaw speed RPM, follow active, P, D, follow current,
   * final left-wheel current and final right-wheel current. */
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)remote_control_source);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      vmc_lqr_rtt_frame_age_ms(now, board_comm_state.rc_last_rx_ms));
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      vmc_lqr_rtt_frame_age_ms(now, board_comm_state.state_last_rx_ms));
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)remote_control.ch[0]);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)remote_control.ch[2]);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)board_comm_state.yaw_ecd);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)b->gimbal_follow_front_ecd);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)b->gimbal_follow_front_valid);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     b->gimbal_follow_error_raw);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)board_comm_state.yaw_speed_rpm);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)b->gimbal_follow_active);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     b->gimbal_follow_p_output);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     b->gimbal_follow_d_output);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     b->gimbal_follow_output);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)vmc_wheel_control.left_current_output);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)vmc_wheel_control.right_current_output);

  /* VMCVOFA fields 62..69: disturbance observer enable, observer output,
   * configured gain, speed error (target velocity - measured velocity),
   * active state, neutral-stick elapsed time, effective pitch compensation
   * and effective virtual-leg theta compensation. */
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)b->disturbance_observer_enable);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)b->disturbance_observer_output);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)b->disturbance_observer_gain);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)b->disturbance_observer_residual);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)b->disturbance_observer_active);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)b->disturbance_observer_neutral_time);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)b->disturbance_observer_output *
          (double)b->disturbance_observer_pitch_gain);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)b->disturbance_observer_output *
          (double)b->disturbance_observer_theta_gain);

  /* VMCVOFA fields 70..74: get-up state, average/left/right theta and the
   * fully-fallen decision.  State 4 is the link-ground leg-recovery mode. */
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_control.getup_state);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     m->theta);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     m->left_theta);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     m->right_theta);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_getup_fully_fallen);

  /* VMCVOFA fields 75..77: measured left/right leg length and the ramped
   * target used by the recovery length controller. */
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     vmc_probe_state.left_leg.length);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     vmc_probe_state.right_leg.length);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_control.ramped_target_length);
  /* VMCVOFA field 78: LQR joint release is blocked until recovery length is
   * confirmed safe. */
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)vmc_control.getup_lqr_release_pending);
  /* VMCVOFA fields 79..80: theta held while shrinking the legs. */
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)vmc_control.getup_shrink_target_theta_left);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)vmc_control.getup_shrink_target_theta_right);

  /* VMCVOFA fields 81..84: runtime Kalman Q/R parameters.  These use the
   * nine-decimal fixed point above because the calibrated values are small. */
  offset = vmc_lqr_rtt_append_qr(line, offset, (int)sizeof(line),
                                 (double)b->kalman_q_v);
  offset = vmc_lqr_rtt_append_qr(line, offset, (int)sizeof(line),
                                 (double)b->kalman_q_a);
  offset = vmc_lqr_rtt_append_qr(line, offset, (int)sizeof(line),
                                 (double)b->kalman_r_wheel);
  offset = vmc_lqr_rtt_append_qr(line, offset, (int)sizeof(line),
                                 (double)b->kalman_r_imu);
  /* VMCVOFA fields 85..90: airborne detector diagnostics. */
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_probe_state.airborne_active);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_probe_state.airborne_candidate);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_probe_state.airborne_wheel_speed);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_probe_state.airborne_joint_torque);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_probe_state.airborne_candidate_ms);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_probe_state.airborne_release_candidate_ms);

  /* VMCVOFA fields 91..98: jump state diagnostics. */
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_control.jump_stage);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_control.jump_armed);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_control.jump_abort_reason);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_control.jump_target_length);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_control.jump_stage_ms);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_control.jump_land_contact_ms);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_control.jump_trigger_count);
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)vmc_control.jump_abort_count);

  /* VMCVOFA fields 99..105: MCU timebase and body-motion estimator
   * diagnostics.  These are appended after the legacy 0..98 block so old
   * parsers keep their original field indices. */
  offset = vmc_lqr_rtt_append_u32(line, offset, (int)sizeof(line), now);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), m->kf_wheel_measurement);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), m->kf_accel_measurement);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), m->kf_innovation_v);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), m->kf_innovation_a);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), m->kf_slip);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), m->kf_slip_metric);

  /* VMCVOFA field 106: resolved accelerometer sign convention (+1/-1), or 0
   * while gravity compensation is still calibrating.  While it is 0 the
   * acceleration measurement is withheld from the estimator. */
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)vmc_probe_state.body.gravity_sign);
  /* VMCVOFA fields 107..108: raw and gravity-compensated body-X acceleration
   * in g.  Watching these together shows whether the compensation removes a
   * real gravity projection or accidentally cancels the measurement. */
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)vmc_probe_state.body.ax);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)vmc_probe_state.body.ax_lin);
  /* VMCVOFA field 109: accelerometer corrections applied so far.  Convert the
   * delta between two frames to Hz to confirm the IMU fusion rate. */
  offset = vmc_lqr_rtt_append_u32(line, offset, (int)sizeof(line),
                                  m->kf_accel_update_count);
  /* VMCVOFA fields 110..114: operator command and the speed loop it drives.
   * ch[3] is the forward/back stick (the only channel that matters for
   * longitudinal tracking); ch[0] and ch[2] above are yaw and turn.
   * speed_target is the requested velocity, speed_target_ramped the value
   * actually handed to the LQR after the ramp limit, and dx_target/dx_error
   * show whether the position loop is contributing. */
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), (double)remote_control.ch[3]);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), (double)b->speed_target);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), (double)b->speed_target_ramped);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), b->dx_target);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), b->dx_error);
  /* VMCVOFA field 115: telemetry schema version. */
  offset = vmc_lqr_rtt_append_u32(line, offset, (int)sizeof(line),
                                  VMC_LQR_RTT_SCHEMA_VERSION);
  /* VMCVOFA fields 116..121: slip-gate inputs and per-side state.  The raw
   * per-wheel speeds are what condition C differentiates, so recording them
   * makes the trigger threshold verifiable offline. */
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), m->kf_slip_left);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), m->kf_slip_right);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), m->kf_wheel_left_raw);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), m->kf_wheel_right_raw);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), (double)b->kalman_slip_accel);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), (double)b->kalman_slip_enable);
  /* VMCVOFA fields 122..125: body attitude and roll rate.
   *
   * These were missing from the telemetry, which made an earlier lift/drop
   * analysis misread an attitude-holding artifact as a control-loop
   * instability: a machine held away from its balance pose while the wheels
   * are unloaded cannot correct the error, so the controller keeps commanding
   * torque and any roll error becomes differential wheel torque.  Without
   * pitch/roll in the recording that is indistinguishable from an oscillation
   * fault. */
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), (double)vmc_probe_state.body.roll);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), (double)vmc_probe_state.body.pitch);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), b->lqr_theta);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), (double)vmc_probe_state.body.wy);

  /* VMCVOFA fields 126..129: airborne-latch inputs and the pitch channel the
   * guard uses.  The latch needs all four legs unloaded at once, so the
   * weakest and strongest joint loads are recorded instead of the mean, and
   * the attitude reference shows what "the body did not move much" was
   * measured against.  body->wy above shares an axis with the roll rate on
   * this IMU mounting; the pitch rate the controller actually consumes is
   * field 129. */
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)vmc_probe_state.airborne_joint_torque_min);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)vmc_probe_state.airborne_joint_torque_max);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (vmc_probe_state.airborne_pitch_reference_valid != 0U)
          ? (double)vmc_probe_state.airborne_pitch_reference
          : (double)VMC_AIRBORNE_PITCH_REFERENCE_NONE);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), (double)vmc_probe_state.body.pitch_rate);

  /* VMCVOFA fields 130..133: per-side airborne latch.  One leg lifted while
   * the other still carries the body leaves the four-joint average high, so
   * the whole-machine test cannot see it; these record each side's own load
   * and latch state, and each latch gates only its own wheel. */
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)vmc_probe_state.airborne_side_torque[VMC_AIRBORNE_SIDE_LEFT]);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)vmc_probe_state.airborne_side_torque[VMC_AIRBORNE_SIDE_RIGHT]);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)vmc_probe_state.airborne_side_active[VMC_AIRBORNE_SIDE_LEFT]);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      (double)vmc_probe_state.airborne_side_active[VMC_AIRBORNE_SIDE_RIGHT]);

  /* VMCVOFA fields 134..135: measured control-loop timing.  The loop is not
   * fixed-rate, so its period is measured against TIM1 (the HAL tick source,
   * which keeps counting while the core sleeps) and fed to every time-based
   * term.  These two fields make that visible from a recording: loop_dt_us is
   * the instantaneous period, and loop_count gives the true average rate over
   * any window. */
  offset = vmc_lqr_rtt_append_u32(line, offset, (int)sizeof(line),
                                  vmc_probe_state.loop_dt_us);
  offset = vmc_lqr_rtt_append_u32(line, offset, (int)sizeof(line),
                                  vmc_probe_state.loop_count);

  if (offset < ((int)sizeof(line) - 2)) {
    line[offset++] = '\n';
    line[offset] = '\0';
    (void)SEGGER_RTT_WriteString(VMC_LQR_RTT_BUFFER_INDEX, line);
  }
}
