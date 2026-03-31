/**
 * @file spi_wrapper.h
 * @brief Thin SPI wrapper — isolates sensor drivers from HAL
 */

#ifndef SPI_WRAPPER_H
#define SPI_WRAPPER_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* ── SPI device descriptor (SPI 핸들 + CS 핀 정보) ── */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
} spi_dev_t;

/**
 * @brief Read a single register
 * @param dev  SPI device descriptor
 * @param reg  Register address
 * @param val  Output: register value
 * @return 0 on success, negative on error
 */
int spi_read_reg(const spi_dev_t *dev, uint8_t reg, uint8_t *val);

/**
 * @brief Write a single register
 * @param dev  SPI device descriptor
 * @param reg  Register address
 * @param val  Value to write
 * @return 0 on success, negative on error
 */
int spi_write_reg(const spi_dev_t *dev, uint8_t reg, uint8_t val);

/**
 * @brief Read multiple consecutive registers
 * @param dev  SPI device descriptor
 * @param reg  Starting register address
 * @param buf  Destination buffer
 * @param len  Number of bytes to read
 * @return 0 on success, negative on error
 */
int spi_read_bytes(const spi_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);

#endif /* SPI_WRAPPER_H */
