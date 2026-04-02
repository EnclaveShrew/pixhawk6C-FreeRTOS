/**
 * @file i2c_wrapper.c
 * @brief Thin I2C wrapper — Address management + HAL calls
 *
 * All I2C sensor drivers communicate through this wrapper.
 * Only this file needs modification when switching to DMA.
 */

#include "i2c_wrapper.h"

#define I2C_TIMEOUT_MS 10

/* ── Public API ─────────────────────────────────────── */

int i2c_read_reg(const i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    /*
     * HAL_I2C_Mem_Read: Send register address → receives data in one call
     * Internally: START -> addr+W -> reg -> RE-START -> addr+R -> data -> STOP
     */
    HAL_StatusTypeDef status =
        HAL_I2C_Mem_Read(dev->hi2c, dev->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, val, 1, I2C_TIMEOUT_MS);

    return (status == HAL_OK) ? 0 : -1;
}

int i2c_write_reg(const i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    /*
     * HAL_I2C_Mem_Write: START -> addr+W -> reg -> data -> STOP
     */
    HAL_StatusTypeDef status =
        HAL_I2C_Mem_Write(dev->hi2c, dev->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);

    return (status == HAL_OK) ? 0 : -1;
}

int i2c_read_bytes(const i2c_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status =
        HAL_I2C_Mem_Read(dev->hi2c, dev->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT_MS);

    return (status == HAL_OK) ? 0 : -1;
}
