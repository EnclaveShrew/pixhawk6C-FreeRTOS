/**
 * @file i2c_wrapper.h
 * @brief Thin I2C wrapper — isolates sensor drivers from HAL
 */

#ifndef I2C_WRAPPER_H
#define I2C_WRAPPER_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* ── I2C device descriptor (I2C handle + slave address) ── */
typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t addr; /* 7-bit address (HAL internally shifts <<1) */
} i2c_dev_t;

/**
 * @brief Read a single register
 * @param dev  I2C device descriptor
 * @param reg  Register address
 * @param val  Output: register value
 * @return 0 on success, negative on error
 */
int i2c_read_reg(const i2c_dev_t *dev, uint8_t reg, uint8_t *val);

/**
 * @brief Write a single register
 * @param dev  I2C device descriptor
 * @param reg  Register address
 * @param val  Value to write
 * @return 0 on success, negative on error
 */
int i2c_write_reg(const i2c_dev_t *dev, uint8_t reg, uint8_t val);

/**
 * @brief Read multiple consecutive registers
 * @param dev  I2C device descriptor
 * @param reg  Starting register address
 * @param buf  Destination buffer
 * @param len  Number of bytes to read
 * @return 0 on success, negative on error
 */
int i2c_read_bytes(const i2c_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);

#endif /* I2C_WRAPPER_H */
