/**
 * @file debug_uart.h
 * @brief Debug output via USART3 (PD8/PD9, 57600bps)
 */

#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "stm32h7xx_hal.h"

/**
 * @brief Initialize debug UART (USART3 핸들 등록)
 * @param huart  Pointer to USART3 handle
 */
void debug_init(UART_HandleTypeDef *huart);

/**
 * @brief printf-style debug output
 * @param fmt  Format string
 * @return Number of characters printed
 */
int debug_printf(const char *fmt, ...);

#endif /* DEBUG_UART_H */
