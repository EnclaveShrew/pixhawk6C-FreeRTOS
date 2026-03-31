/**
 * @file debug_uart.c
 * @brief Debug output via USART3
 */

#include "debug_uart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define DEBUG_BUF_SIZE 256
#define DEBUG_TIMEOUT_MS 50

static UART_HandleTypeDef *debug_huart = NULL;

void debug_init(UART_HandleTypeDef *huart)
{
    debug_huart = huart;
}

int debug_printf(const char *fmt, ...)
{
    if (debug_huart == NULL)
    {
        return -1;
    }

    char buf[DEBUG_BUF_SIZE];
    va_list args;

    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0)
    {
        uint16_t tx_len = (len < DEBUG_BUF_SIZE) ? (uint16_t)len : DEBUG_BUF_SIZE;
        HAL_UART_Transmit(debug_huart, (uint8_t *)buf, tx_len, DEBUG_TIMEOUT_MS);
    }

    return len;
}
