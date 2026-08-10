#include "hi12_imu.h"

#include "usart.h"

#include <string.h>

volatile hi12_imu_t hi12_imu;

static uint8_t hi12_rx_buffers[2][HI12_IMU_FRAME_SIZE];
static uint8_t hi12_active_buffer;

static float hi12_read_float(const uint8_t *data)
{
  float value;
  memcpy(&value, data, sizeof(value));
  return value;
}

static void hi12_crc16_update(uint16_t *current_crc, const uint8_t *src,
                              uint32_t length)
{
  uint16_t crc = *current_crc;

  for (uint32_t j = 0U; j < length; j++) {
    crc ^= (uint16_t)src[j] << 8;
    for (uint32_t i = 0U; i < 8U; i++) {
      if ((crc & 0x8000U) != 0U) {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }

  *current_crc = crc;
}

static uint8_t hi12_verify_crc(const uint8_t *frame)
{
  uint16_t crc_calculated = 0U;
  const uint16_t payload_len = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
  const uint16_t crc_received = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);

  hi12_crc16_update(&crc_calculated, frame, 4U);
  hi12_crc16_update(&crc_calculated, frame + 6U, payload_len);

  return (crc_calculated == crc_received) ? 1U : 0U;
}

static void hi12_restart_rx(void)
{
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart6,
                                     hi12_rx_buffers[hi12_active_buffer],
                                     HI12_IMU_FRAME_SIZE);
  if (huart6.hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
  }
}

static void hi12_parse_frame(const uint8_t *data, uint32_t len)
{
  int32_t frame_start = -1;

  hi12_imu.last_rx_len = len;

  if (data == NULL || len < HI12_IMU_FRAME_SIZE) {
    hi12_imu.frame_error_count++;
    hi12_imu.valid = 0U;
    hi12_imu.last_frame_offset = -1;
    return;
  }

  for (uint32_t i = 0U; i <= len - HI12_IMU_FRAME_SIZE; i++) {
    const uint16_t payload_len = (uint16_t)data[i + 2U] |
                                 ((uint16_t)data[i + 3U] << 8);
    if (data[i] == 0x5AU && data[i + 1U] == 0xA5U &&
        payload_len == HI12_IMU_PAYLOAD_SIZE) {
      frame_start = (int32_t)i;
      break;
    }
  }

  hi12_imu.last_frame_offset = frame_start;

  if (frame_start < 0) {
    hi12_imu.frame_error_count++;
    hi12_imu.valid = 0U;
    return;
  }

  const uint8_t *frame = data + frame_start;
  if (hi12_verify_crc(frame) == 0U) {
    hi12_imu.frame_error_count++;
    hi12_imu.valid = 0U;
    return;
  }

  const uint8_t *payload = frame + 6U;

  hi12_imu.acc_mss[0] = hi12_read_float(payload + 12U);
  hi12_imu.acc_mss[1] = hi12_read_float(payload + 16U);
  hi12_imu.acc_mss[2] = hi12_read_float(payload + 20U);

  hi12_imu.gyro_dps[0] = hi12_read_float(payload + 24U);
  hi12_imu.gyro_dps[1] = hi12_read_float(payload + 28U);
  hi12_imu.gyro_dps[2] = hi12_read_float(payload + 32U);

  hi12_imu.angle_deg[0] = hi12_read_float(payload + 48U);
  hi12_imu.angle_deg[1] = hi12_read_float(payload + 52U);
  hi12_imu.angle_deg[2] = hi12_read_float(payload + 56U);

  hi12_imu.quaternion[0] = hi12_read_float(payload + 60U);
  hi12_imu.quaternion[1] = hi12_read_float(payload + 64U);
  hi12_imu.quaternion[2] = hi12_read_float(payload + 68U);
  hi12_imu.quaternion[3] = hi12_read_float(payload + 72U);

  for (uint8_t i = 0U; i < 3U; i++) {
    hi12_imu.gyro_radps[i] = hi12_imu.gyro_dps[i] * HI12_IMU_DEG_TO_RAD;
    hi12_imu.angle_rad[i] = hi12_imu.angle_deg[i] * HI12_IMU_DEG_TO_RAD;
  }

  hi12_imu.last_update_ms = HAL_GetTick();
  hi12_imu.connected = 1U;
  hi12_imu.valid = 1U;
  hi12_imu.rx_count++;
}

void hi12_imu_init(void)
{
  memset((void *)&hi12_imu, 0, sizeof(hi12_imu));
  memset(hi12_rx_buffers, 0, sizeof(hi12_rx_buffers));
  hi12_active_buffer = 0U;
  hi12_imu.last_frame_offset = -1;
  hi12_restart_rx();
}

void hi12_imu_update(void)
{
  if (hi12_imu.last_update_ms != 0U &&
      (HAL_GetTick() - hi12_imu.last_update_ms) > HI12_IMU_TIMEOUT_MS) {
    hi12_imu.connected = 0U;
    hi12_imu.valid = 0U;
  }
}

void hi12_imu_on_uart_rx_event(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart != &huart6) {
    return;
  }

  const uint8_t completed = hi12_active_buffer;
  hi12_parse_frame(hi12_rx_buffers[completed], size);
  hi12_active_buffer = (completed == 0U) ? 1U : 0U;
  hi12_restart_rx();
}
