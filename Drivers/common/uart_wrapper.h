/**
 * @file uart_wrapper.h
 * @brief Thin UART wrapper — isolates drivers from HAL
 */

#ifndef UART_WRAPPER_H
#define UART_WRAPPER_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* ── UART device descriptor ──────────────────────────── */
typedef struct
{
    UART_HandleTypeDef *huart;
} uart_dev_t;

/**
 * @brief Send data over UART
 * @param dev  UART device descriptor
 * @param buf  Data buffer
 * @param len  Number of bytes
 * @return 0 on success, negative on error
 */
int uart_send(const uart_dev_t *dev, const uint8_t *buf, uint16_t len);

/**
 * @brief Receive data over UART (blocking)
 * @param dev      UART device descriptor
 * @param buf      Destination buffer
 * @param len      Number of bytes to receive
 * @param timeout  Timeout in ms
 * @return 0 on success, negative on error
 */
int uart_receive(const uart_dev_t *dev, uint8_t *buf, uint16_t len, uint32_t timeout);

/**
 * @brief Receive a single byte (blocking, short timeout)
 * @param dev   UART device descriptor
 * @param byte  Output: received byte
 * @return 0 on success, negative on error/timeout
 */
int uart_receive_byte(const uart_dev_t *dev, uint8_t *byte);

#endif /* UART_WRAPPER_H */
