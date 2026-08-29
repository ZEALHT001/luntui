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
  if (vmc_lqr_rtt_parse_named_float(line, "tp_limit", &value) != 0U) {
    vmc_balance.tp_limit = vmc_lqr_rtt_clamp_float(value, 0.0f, VMC_DM_J8009_TMAX);
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
    vmc_control.max_phi0_torque = vmc_lqr_rtt_clamp_float(value, 0.0f, VMC_DM_J8009_TMAX);
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
    vmc_balance.yaw_target = vmc_rad_format(value);
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

  /* VMCVOFA fields 47..60: RC source, RC-frame age, state-frame age,
   * right-stick yaw, left-stick turn, yaw ECD, front ECD, front valid,
   * wrapped error, yaw speed RPM, follow active, P, D, follow current. */
  offset = vmc_lqr_rtt_append_scaled(line, offset, (int)sizeof(line),
                                     (double)remote_control_source);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      board_comm_state.rc_last_rx_ms == 0U
          ? -1.0
          : (double)(now - board_comm_state.rc_last_rx_ms));
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line),
      board_comm_state.state_last_rx_ms == 0U
          ? -1.0
          : (double)(now - board_comm_state.state_last_rx_ms));
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

  if (offset < ((int)sizeof(line) - 2)) {
    line[offset++] = '\n';
    line[offset] = '\0';
    (void)SEGGER_RTT_WriteString(VMC_LQR_RTT_BUFFER_INDEX, line);
  }
}
