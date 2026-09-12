#include "tf02_pro.h"

#include "usart.h"

#include <string.h>

#define TF02_PRO_UART (&huart7)

volatile uint16_t tf02_distance_cm;
volatile uint16_t tf02_strength;
volatile uint16_t tf02_temperature_raw;
volatile uint32_t tf02_last_update_ms;
volatile uint32_t tf02_rx_count;
volatile uint32_t tf02_frame_error_count;
volatile uint32_t tf02_rx_event_count;
volatile uint32_t tf02_rx_byte_count;
volatile uint32_t tf02_rx_restart_count;
volatile uint32_t tf02_rx_start_error_count;
volatile uint16_t tf02_last_rx_size;
volatile uint8_t tf02_rx_start_result;
volatile uint8_t tf02_rx_started;
volatile uint32_t tf02_last_uart_error;
volatile uint16_t tf02_dma_remaining;
volatile uint32_t tf02_uart_status;
volatile uint8_t tf02_protocol;
volatile uint8_t tf02_last_rx_data[TF02_PRO_RX_DMA_SIZE];
volatile uint8_t tf02_connected;
volatile uint8_t tf02_last_frame[TF02_PRO_FRAME_SIZE];

static uint8_t tf02_rx_dma[TF02_PRO_RX_DMA_SIZE];
static uint8_t tf02_frame[TF02_PRO_FRAME_SIZE];
static uint8_t tf02_frame_index;
static uint8_t tf02_ascii_buffer[16U];
static uint8_t tf02_ascii_length;

static uint8_t tf02_checksum(const uint8_t *frame)
{
  uint8_t checksum = 0U;

  for (uint8_t i = 0U; i < (TF02_PRO_FRAME_SIZE - 1U); i++) {
    checksum = (uint8_t)(checksum + frame[i]);
  }
  return checksum;
}

static void tf02_parse_frame(const uint8_t *frame)
{
  if (tf02_checksum(frame) != frame[TF02_PRO_FRAME_SIZE - 1U]) {
    tf02_frame_error_count++;
    return;
  }

  tf02_distance_cm = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
  tf02_strength = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
  tf02_temperature_raw = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
  memcpy((void *)tf02_last_frame, frame, TF02_PRO_FRAME_SIZE);
  tf02_last_update_ms = HAL_GetTick();
  tf02_rx_count++;
  tf02_protocol = TF02_PRO_PROTOCOL_BINARY_CM;
  tf02_connected = 1U;
}

static void tf02_parse_ascii_distance(void)
{
  uint32_t whole = 0U;
  uint32_t fractional = 0U;
  uint32_t scale = 1U;
  uint8_t have_digit = 0U;
  uint8_t after_decimal = 0U;

  for (uint8_t i = 0U; i < tf02_ascii_length; i++) {
    const uint8_t byte = tf02_ascii_buffer[i];

    if (byte == '.') {
      if (after_decimal != 0U) return;
      after_decimal = 1U;
    } else if (byte >= '0' && byte <= '9') {
      have_digit = 1U;
      if (after_decimal == 0U) {
        whole = whole * 10U + (uint32_t)(byte - '0');
      } else if (scale < 1000000U) {
        fractional = fractional * 10U + (uint32_t)(byte - '0');
        scale *= 10U;
      }
    } else {
      return;
    }
  }

  if (have_digit == 0U) return;

  {
    const uint32_t distance_cm = whole * 100U + (fractional * 100U) / scale;
    if (distance_cm > 0xFFFFU) return;
    tf02_distance_cm = (uint16_t)distance_cm;
  }

  tf02_last_update_ms = HAL_GetTick();
  tf02_rx_count++;
  tf02_protocol = TF02_PRO_PROTOCOL_ASCII_M;
  tf02_connected = 1U;
}

static void tf02_feed_ascii_byte(uint8_t byte)
{
  if (byte == '\r' || byte == '\n') {
    if (tf02_ascii_length != 0U) {
      tf02_parse_ascii_distance();
      tf02_ascii_length = 0U;
    }
    return;
  }

  if ((byte >= '0' && byte <= '9') || byte == '.') {
    if (tf02_ascii_length < sizeof(tf02_ascii_buffer)) {
      tf02_ascii_buffer[tf02_ascii_length++] = byte;
    } else {
      tf02_ascii_length = 0U;
    }
  } else {
    tf02_ascii_length = 0U;
  }
}

static void tf02_feed_byte(uint8_t byte)
{
  if (tf02_frame_index == 0U) {
    if (byte == TF02_PRO_FRAME_HEADER) {
      tf02_frame[tf02_frame_index++] = byte;
    }
    return;
  }

  if (tf02_frame_index == 1U) {
    if (byte == TF02_PRO_FRAME_HEADER) {
      tf02_frame[tf02_frame_index++] = byte;
    } else {
      tf02_frame_index = 0U;
    }
    return;
  }

  tf02_frame[tf02_frame_index++] = byte;
  if (tf02_frame_index >= TF02_PRO_FRAME_SIZE) {
    tf02_parse_frame(tf02_frame);
    tf02_frame_index = 0U;
  }
}

static void tf02_restart_rx(void)
{
  HAL_StatusTypeDef status;

  tf02_rx_restart_count++;
  status = HAL_UARTEx_ReceiveToIdle_DMA(TF02_PRO_UART, tf02_rx_dma,
                                        sizeof(tf02_rx_dma));
  tf02_rx_start_result = (uint8_t)status;
  if (status == HAL_OK) {
    tf02_rx_started = 1U;
  } else {
    tf02_rx_started = 0U;
    tf02_rx_start_error_count++;
  }
  if (status == HAL_OK && TF02_PRO_UART->hdmarx != NULL) {
    __HAL_DMA_DISABLE_IT(TF02_PRO_UART->hdmarx, DMA_IT_HT);
  }
}

void tf02_pro_init(void)
{
  tf02_distance_cm = 0U;
  tf02_strength = 0U;
  tf02_temperature_raw = 0U;
  tf02_last_update_ms = 0U;
  tf02_rx_count = 0U;
  tf02_frame_error_count = 0U;
  tf02_rx_event_count = 0U;
  tf02_rx_byte_count = 0U;
  tf02_rx_restart_count = 0U;
  tf02_rx_start_error_count = 0U;
  tf02_last_rx_size = 0U;
  tf02_rx_start_result = (uint8_t)HAL_ERROR;
  tf02_rx_started = 0U;
  tf02_last_uart_error = HAL_UART_ERROR_NONE;
  tf02_dma_remaining = 0U;
  tf02_uart_status = 0U;
  tf02_protocol = TF02_PRO_PROTOCOL_NONE;
  memset((void *)tf02_last_rx_data, 0, sizeof(tf02_last_rx_data));
  tf02_connected = 0U;
  memset((void *)tf02_last_frame, 0, sizeof(tf02_last_frame));
  memset(tf02_rx_dma, 0, sizeof(tf02_rx_dma));
  memset(tf02_frame, 0, sizeof(tf02_frame));
  memset(tf02_ascii_buffer, 0, sizeof(tf02_ascii_buffer));
  tf02_frame_index = 0U;
  tf02_ascii_length = 0U;
  tf02_restart_rx();
}

void tf02_pro_update(void)
{
  tf02_uart_status = TF02_PRO_UART->Instance->SR;
  if (TF02_PRO_UART->hdmarx != NULL) {
    tf02_dma_remaining = (uint16_t)__HAL_DMA_GET_COUNTER(TF02_PRO_UART->hdmarx);
  }
  if (tf02_rx_started == 0U) {
    tf02_restart_rx();
  }
  if (tf02_connected != 0U &&
      (HAL_GetTick() - tf02_last_update_ms) > TF02_PRO_TIMEOUT_MS) {
    tf02_connected = 0U;
  }
}

void tf02_pro_on_uart_rx_event(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart != TF02_PRO_UART) {
    return;
  }

  tf02_rx_event_count++;
  tf02_last_rx_size = size;
  tf02_rx_byte_count += size;
  if (size > sizeof(tf02_last_rx_data)) {
    size = sizeof(tf02_last_rx_data);
  }
  memcpy((void *)tf02_last_rx_data, tf02_rx_dma, size);
  for (uint16_t i = 0U; i < size; i++) {
    tf02_feed_byte(tf02_rx_dma[i]);
    tf02_feed_ascii_byte(tf02_rx_dma[i]);
  }
  tf02_restart_rx();
}

void tf02_pro_on_uart_error(UART_HandleTypeDef *huart)
{
  if (huart != TF02_PRO_UART) {
    return;
  }

  tf02_last_uart_error = TF02_PRO_UART->ErrorCode;
  tf02_frame_error_count++;
  (void)HAL_UART_DMAStop(TF02_PRO_UART);
  __HAL_UART_CLEAR_OREFLAG(TF02_PRO_UART);
  __HAL_UART_CLEAR_NEFLAG(TF02_PRO_UART);
  __HAL_UART_CLEAR_FEFLAG(TF02_PRO_UART);
  __HAL_UART_CLEAR_PEFLAG(TF02_PRO_UART);
  TF02_PRO_UART->ErrorCode = HAL_UART_ERROR_NONE;
  TF02_PRO_UART->RxState = HAL_UART_STATE_READY;
  tf02_restart_rx();
}
