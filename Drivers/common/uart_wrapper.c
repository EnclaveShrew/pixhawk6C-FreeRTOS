/**
 * @file uart_wrapper.c
 * @brief Thin UART wrapper — HAL 호출 추상화
 *
 * GPS, 텔레메트리 등 UART 기반 장치가 이 래퍼를 통해 통신.
 * 나중에 DMA/인터럽트 수신으로 전환 시 이 파일 내부만 수정.
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
