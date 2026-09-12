#ifndef TF02_PRO_H
#define TF02_PRO_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TF02-Pro standard UART output frame. */
#define TF02_PRO_FRAME_SIZE 9U
#define TF02_PRO_RX_DMA_SIZE 64U
#define TF02_PRO_TIMEOUT_MS 100U
#define TF02_PRO_FRAME_HEADER 0x59U
#define TF02_PRO_PROTOCOL_NONE 0U
#define TF02_PRO_PROTOCOL_BINARY_CM 1U
#define TF02_PRO_PROTOCOL_ASCII_M 2U

/*
 * These variables are intentionally global and volatile so they remain easy
 * to inspect in a debugger/Live Watch window.  Distance is in centimetres.
 */
extern volatile uint16_t tf02_distance_cm;
extern volatile uint16_t tf02_strength;
extern volatile uint16_t tf02_temperature_raw;
extern volatile uint32_t tf02_last_update_ms;
extern volatile uint32_t tf02_rx_count;
extern volatile uint32_t tf02_frame_error_count;
extern volatile uint32_t tf02_rx_event_count;
extern volatile uint32_t tf02_rx_byte_count;
extern volatile uint32_t tf02_rx_restart_count;
extern volatile uint32_t tf02_rx_start_error_count;
extern volatile uint16_t tf02_last_rx_size;
extern volatile uint8_t tf02_rx_start_result;
extern volatile uint8_t tf02_rx_started;
extern volatile uint32_t tf02_last_uart_error;
extern volatile uint16_t tf02_dma_remaining;
extern volatile uint32_t tf02_uart_status;
extern volatile uint8_t tf02_protocol;
extern volatile uint8_t tf02_last_rx_data[TF02_PRO_RX_DMA_SIZE];
extern volatile uint8_t tf02_connected;
extern volatile uint8_t tf02_last_frame[TF02_PRO_FRAME_SIZE];

void tf02_pro_init(void);
void tf02_pro_update(void);
void tf02_pro_on_uart_rx_event(UART_HandleTypeDef *huart, uint16_t size);
void tf02_pro_on_uart_error(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* TF02_PRO_H */
