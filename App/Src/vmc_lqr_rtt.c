#include "vmc_lqr_rtt.h"

#include "SEGGER_RTT.h"
#include "board_comm.h"
#include "remote_control.h"
#include "stm32f4xx_hal.h"
#include "vmc_probe.h"
#include "vmc_math.h"
#include "vmc_rtt_format.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VMC_LQR_RTT_BUFFER_INDEX 0U
#define VMC_LQR_RTT_LINE_MAX     1536U
#define VMC_LQR_RTT_FAST_LINE_MAX 768U
#define VMC_LQR_RTT_READ_CHUNK   64U
#define VMC_LQR_RTT_TELEM_PERIOD_MS 20U

/* The full VMCVOFA frame is large, so raising it to 250 Hz would spend a
 * material amount of control-loop time formatting and would stress the RTT
 * link.  Keep it at 50 Hz and add a compact second stream for the high-rate
 * mechanical channels needed by slip analysis. */
#define VMC_LQR_RTT_TELEM_PERIOD_MIN_MS 2U
#define VMC_LQR_RTT_TELEM_PERIOD_MAX_MS 200U
#define VMC_LQR_RTT_FAST_ENABLE_DEFAULT 0U
#define VMC_LQR_RTT_FAST_PERIOD_MIN_MS 2U
#define VMC_LQR_RTT_FAST_PERIOD_MAX_MS 100U
#define VMC_LQR_RTT_FAST_PERIOD_DEFAULT_MS 4U

/* Binary VMCFAST frame: 4 B header + 4 B seq + 80 B payload + 2 B CRC. */
#define VMC_LQR_RTT_FAST_SYNC0    0xFAU
#define VMC_LQR_RTT_FAST_SYNC1    0xCEU
#define VMC_LQR_RTT_FAST_VERSION  1U
#define VMC_LQR_RTT_FAST_FRAME_SIZE 90U

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
static uint16_t lqr_rtt_telem_period_ms = VMC_LQR_RTT_TELEM_PERIOD_MS;
static uint8_t lqr_rtt_fast_enable = VMC_LQR_RTT_FAST_ENABLE_DEFAULT;
static uint16_t lqr_rtt_fast_period_ms = VMC_LQR_RTT_FAST_PERIOD_DEFAULT_MS;
static uint32_t lqr_rtt_last_fast_ms;
static uint32_t lqr_rtt_fast_seq;

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
  /* Logging controls.  These are not control parameters, but keeping them on
   * the same payload lets a capture change rates without a reflash. */
  if (vmc_lqr_rtt_parse_named_float(line, "telem_period_ms", &value) != 0U) {
    const float clamped = vmc_lqr_rtt_clamp_float(
        value,
        (float)VMC_LQR_RTT_TELEM_PERIOD_MIN_MS,
        (float)VMC_LQR_RTT_TELEM_PERIOD_MAX_MS);
    lqr_rtt_telem_period_ms = (uint16_t)clamped;
    lqr_rtt_last_telem_ms = HAL_GetTick() - (uint32_t)lqr_rtt_telem_period_ms;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "fast_enable", &value) != 0U) {
    lqr_rtt_fast_enable = (value > 0.5f) ? 1U : 0U;
    lqr_rtt_last_fast_ms = HAL_GetTick() - (uint32_t)lqr_rtt_fast_period_ms;
    applied = 1U;
  }
  if (vmc_lqr_rtt_parse_named_float(line, "fast_period_ms", &value) != 0U) {
    const float clamped = vmc_lqr_rtt_clamp_float(
        value,
        (float)VMC_LQR_RTT_FAST_PERIOD_MIN_MS,
        (float)VMC_LQR_RTT_FAST_PERIOD_MAX_MS);
    lqr_rtt_fast_period_ms = (uint16_t)clamped;
    lqr_rtt_last_fast_ms = HAL_GetTick() - (uint32_t)lqr_rtt_fast_period_ms;
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
  lqr_rtt_telem_period_ms = VMC_LQR_RTT_TELEM_PERIOD_MS;
  lqr_rtt_fast_enable = VMC_LQR_RTT_FAST_ENABLE_DEFAULT;
  lqr_rtt_fast_period_ms = VMC_LQR_RTT_FAST_PERIOD_DEFAULT_MS;
  lqr_rtt_last_telem_ms = 0U;
  lqr_rtt_last_fast_ms = 0U;
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

/* Compact high-rate stream for slip experiments.
 *
 * VMCFAST carries raw wheel, IMU, leg and actuator observations.  It is
 * deliberately independent of the slow VMCVOFA stream so the full state
 * remains available at 50 Hz while the mechanical evidence is recorded at
 * 250 Hz.  The existing slip metric is published as a diagnostic, but it is
 * not allowed to gate or replace any value in this stream.
 *
 * The payload is little-endian fixed point rather than ASCII.  The same 43
 * fields cost 277 B/frame as text (mean 5.4 B/field) but 80 B here, which is
 * what buys back RTT buffer headroom: at 250 Hz the stream drops from
 * 69 kB/s to 22.5 kB/s.
 *
 * Wire format (90 bytes, fixed length):
 *   0     0xFA          sync
 *   1     0xCE          sync
 *   2     0x01          version
 *   3     90            total frame length
 *   4..7  seq           monotonic frame counter, little-endian u32
 *   8..11 t_ms          HAL tick
 *   12..15 loop_count   control-loop iteration counter
 *   16..17 loop_dt_us   measured control period
 *   18..27 10 flag bytes
 *   28..87 30 int16 scaled scalars
 *   88..89 CRC-16/CCITT over bytes 0..87
 *
 * The frame counter is the point of the header: it makes "the firmware never
 * wrote the frame" (seq jumps) distinguishable from "the host never read it"
 * (seq continuous, arrival gap).
 */

/* Scaled int16 payload, in wire order.
 *
 *   V(<field>, <scale>, <expression>)
 *
 * "expression * scale" is quantised to int16 with saturation and
 * round-half-away-from-zero.  A scale of 0 means "already an integer in
 * range".  tools/slip_capture.py carries the mirrored table and
 * tools/test_slip_capture.py fails if the two drift apart.
 *
 * Scale notes.  Wheel speed is the output shaft in rad/s: an M3508 reaches
 * ~60 rad/s at 9000 rpm and an unloaded wheel during a slip spins past the
 * ~33 rad/s that scale 1000 would allow, so it uses scale 100 (+/-327 rad/s).
 * That ceiling clipped real slip peaks in the first pellet run.  Joint torque
 * is reported at scale 100 (+/-327 N.m) against a 54 N.m motor rating. */
#define VMC_LQR_RTT_FAST_SCALARS(V)                                          \
  V(body_pitch,                  10000, (double)s->body.pitch)               \
  V(body_pitch_rate,              1000, (double)s->body.pitch_rate)          \
  V(body_roll,                   10000, (double)s->body.roll)                \
  V(body_roll_rate,               1000, (double)s->body.wy)                  \
  V(body_yaw_rate,                1000, (double)s->body.wz)                  \
  V(body_ax_g,                    1000, (double)s->body.ax)                  \
  V(body_ax_lin_g,                1000, (double)s->body.ax_lin)              \
  V(kf_dx,                        1000, s->motion.kf_dx)                     \
  V(wheel_left_speed,              100, (double)s->wheel_left.speed_rads)    \
  V(wheel_left_torque,            1000, (double)s->wheel_left.torque)        \
  V(wheel_left_current_output,       0,                                      \
    (double)vmc_wheel_control.left_current_output)                           \
  V(wheel_right_speed,             100, (double)s->wheel_right.speed_rads)   \
  V(wheel_right_torque,           1000, (double)s->wheel_right.torque)       \
  V(wheel_right_current_output,      0,                                      \
    (double)vmc_wheel_control.right_current_output)                          \
  V(leg_left_length,             10000, s->left_leg.length)                  \
  V(leg_left_d_length,            1000, s->left_leg.d_length)                \
  V(leg_left_phi0,               10000, s->left_leg.phi0)                    \
  V(leg_left_d_phi0,              1000, s->left_leg.d_phi0)                  \
  V(leg_right_length,            10000, s->right_leg.length)                 \
  V(leg_right_d_length,           1000, s->right_leg.d_length)               \
  V(leg_right_phi0,              10000, s->right_leg.phi0)                   \
  V(leg_right_d_phi0,             1000, s->right_leg.d_phi0)                 \
  V(joint_left_ahead_torque,       100, (double)s->joint_left_ahead.torque)  \
  V(joint_left_back_torque,        100, (double)s->joint_left_back.torque)   \
  V(joint_right_ahead_torque,      100, (double)s->joint_right_ahead.torque) \
  V(joint_right_back_torque,       100, (double)s->joint_right_back.torque)  \
  V(slip_metric,                   100, s->motion.kf_slip_metric)            \
  V(airborne_wheel_speed,         1000, (double)s->airborne_wheel_speed)     \
  V(airborne_joint_torque_min,     100, (double)s->airborne_joint_torque_min)\
  V(airborne_joint_torque_max,     100, (double)s->airborne_joint_torque_max)

/* Context flags, in wire order.  Written as raw bytes. */
#define VMC_LQR_RTT_FAST_FLAGS(V)                                            \
  V(enabled,                    (double)vmc_control.enabled)                 \
  V(dm_state,                   (double)s->dm_state)                         \
  V(feedback_timeout_source,    (double)s->feedback_timeout_source)          \
  V(airborne_active,            (double)s->airborne_active)                  \
  V(airborne_side_left,         (double)s->airborne_side_active[VMC_AIRBORNE_SIDE_LEFT])  \
  V(airborne_side_right,        (double)s->airborne_side_active[VMC_AIRBORNE_SIDE_RIGHT]) \
  V(jump_stage,                 (double)vmc_control.jump_stage)               \
  V(getup_state,                (double)vmc_control.getup_state)             \
  V(slip_left,                  s->motion.slip_raw_left)                     \
  V(slip_right,                 s->motion.slip_raw_right)

static void vmc_lqr_rtt_fast_telemetry_update(uint32_t now)
{
  static uint8_t frame[VMC_LQR_RTT_FAST_FRAME_SIZE];
  const volatile vmc_probe_state_t *s = &vmc_probe_state;
  int offset = 0;
  uint16_t crc;

  if (lqr_rtt_fast_enable == 0U) {
    return;
  }
  if ((uint32_t)(now - lqr_rtt_last_fast_ms) <
      (uint32_t)lqr_rtt_fast_period_ms) {
    return;
  }
  lqr_rtt_last_fast_ms = now;

  frame[offset++] = VMC_LQR_RTT_FAST_SYNC0;
  frame[offset++] = VMC_LQR_RTT_FAST_SYNC1;
  frame[offset++] = VMC_LQR_RTT_FAST_VERSION;
  frame[offset++] = (uint8_t)VMC_LQR_RTT_FAST_FRAME_SIZE;
  vmc_rtt_put_u32le(&frame[offset], lqr_rtt_fast_seq++);
  offset += 4;
  vmc_rtt_put_u32le(&frame[offset], now);
  offset += 4;
  vmc_rtt_put_u32le(&frame[offset], s->loop_count);
  offset += 4;
  vmc_rtt_put_u16le(&frame[offset], (uint16_t)s->loop_dt_us);
  offset += 2;

#define VMC_RTT_EMIT_FLAG(name, expr) \
  frame[offset++] = (uint8_t)vmc_rtt_scale_i16((expr), 0);
  VMC_LQR_RTT_FAST_FLAGS(VMC_RTT_EMIT_FLAG)
#undef VMC_RTT_EMIT_FLAG

#define VMC_RTT_EMIT_SCALAR(name, scale, expr) \
  vmc_rtt_put_i16le(&frame[offset], vmc_rtt_scale_i16((expr), (scale))); \
  offset += 2;
  VMC_LQR_RTT_FAST_SCALARS(VMC_RTT_EMIT_SCALAR)
#undef VMC_RTT_EMIT_SCALAR

  crc = vmc_rtt_crc16(frame, (uint32_t)(VMC_LQR_RTT_FAST_FRAME_SIZE - 2U));
  vmc_rtt_put_u16le(&frame[offset], crc);
  offset += 2;

  (void)SEGGER_RTT_Write(VMC_LQR_RTT_BUFFER_INDEX, frame, (unsigned)offset);
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

  /* Service the fast stream before the slow gate; otherwise a fast frame
   * could never be emitted on a cycle where the slow frame is not due. */
  vmc_lqr_rtt_fast_telemetry_update(now);

  if ((uint32_t)(now - lqr_rtt_last_telem_ms) <
      (uint32_t)lqr_rtt_telem_period_ms) {
    return;
  }
  lqr_rtt_last_telem_ms = now;

  /* Constant prefix; memcpy keeps the telemetry path free of printf. */
  memcpy(line, "VMCVOFA", 7U);
  offset = 7;
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
   * makes the trigger threshold verifiable offline.
   *
   * Fields 116/117 report the RAW per-side latch, not the post-gate state.
   * The gate is only allowed to reject wheel measurements when slip_enable is
   * set, so emitting kf_slip_left/right here would report a constant 0 during
   * a normal raw capture and the threshold under test would be unverifiable.
   * The filter action itself is still visible in field 104 (kf_slip). */
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), m->slip_raw_left);
  offset = vmc_lqr_rtt_append_scaled(
      line, offset, (int)sizeof(line), m->slip_raw_right);
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
