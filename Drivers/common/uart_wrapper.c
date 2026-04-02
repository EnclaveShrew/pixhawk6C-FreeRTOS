/**
 * @file uart_wrapper.c
 * @brief Thin UART wrapper — HAL call abstraction
 *
 * GPS, telemetry, and other UART devices communicate through this wrapper.
 * Only this file needs modification when switching to DMA/interrupt.
 */

#include "uart_wrapper.h"

#define UART_DEFAULT_TIMEOUT_MS 10

/* ── Public API ─────────────────────────────────────── */

int uart_send(const uart_dev_t *dev, const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit(
        dev->huart, (uint8_t *)buf, len, UART_DEFAULT_TIMEOUT_MS);

    return (status == HAL_OK) ? 0 : -1;
}

int uart_receive(const uart_dev_t *dev, uint8_t *buf, uint16_t len, uint32_t timeout)
{
    HAL_StatusTypeDef status = HAL_UART_Receive(
        dev->huart, buf, len, timeout);

    return (status == HAL_OK) ? 0 : -1;
}

int uart_receive_byte(const uart_dev_t *dev, uint8_t *byte)
{
    HAL_StatusTypeDef status = HAL_UART_Receive(
        dev->huart, byte, 1, UART_DEFAULT_TIMEOUT_MS);

    return (status == HAL_OK) ? 0 : -1;
}
