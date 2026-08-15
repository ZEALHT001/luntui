#include "vmc_lqr_rtt.h"

#include "SEGGER_RTT.h"
#include "stm32f4xx_hal.h"
#include "vmc_probe.h"

#include <stdlib.h>
#include <string.h>

#define VMC_LQR_RTT_BUFFER_INDEX 0U
#define VMC_LQR_RTT_LINE_MAX     640U
#define VMC_LQR_RTT_READ_CHUNK   64U

#define VMC_LQR_RTT_ERR_NONE      0U
#define VMC_LQR_RTT_ERR_OVERFLOW  1U
#define VMC_LQR_RTT_ERR_TYPE      2U
#define VMC_LQR_RTT_ERR_K_COUNT   3U

static char lqr_rtt_line[VMC_LQR_RTT_LINE_MAX];
static uint16_t lqr_rtt_line_len;

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
        vmc_lqr_rtt_set_error(VMC_LQR_RTT_ERR_OVERFLOW);
        vmc_lqr_rtt_ack("VMC_LQR_RTT ERR OVERFLOW\n");
        continue;
      }

      lqr_rtt_line[lqr_rtt_line_len++] = c;
    }
  } while (read_len == sizeof(data));
}
