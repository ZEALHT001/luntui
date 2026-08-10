#ifndef HI12_IMU_H
#define HI12_IMU_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HI12_IMU_FRAME_SIZE 82U
#define HI12_IMU_PAYLOAD_SIZE 76U
#define HI12_IMU_TIMEOUT_MS 100U
#define HI12_IMU_DEG_TO_RAD 0.01745329251994329577f

typedef struct {
  float acc_mss[3];
  float gyro_dps[3];
  float gyro_radps[3];
  float angle_deg[3];
  float angle_rad[3];
  float quaternion[4];
  uint32_t last_update_ms;
  uint32_t rx_count;
  uint32_t frame_error_count;
  uint32_t last_rx_len;
  int32_t last_frame_offset;
  uint8_t connected;
  uint8_t valid;
} hi12_imu_t;

extern volatile hi12_imu_t hi12_imu;

void hi12_imu_init(void);
void hi12_imu_update(void);
void hi12_imu_on_uart_rx_event(UART_HandleTypeDef *huart, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
