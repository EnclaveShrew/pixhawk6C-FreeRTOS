/**
 * @file bmi055.c
 * @brief BMI055 6-axis IMU driver implementation
 *
 * Datasheet: BST-BMI055-DS000 (Bosch Sensortec)
 * SPI Mode 3 (CPOL=1, CPHA=1), max 10 MHz
 *
 * Accel (BMA280) and gyro (BMG160) are separate dies.
 * Each communicates via SPI with independent CS pin.
 */

#include "bmi055.h"
#include "stm32h7xx_hal.h"

/* ── Helper: raw 16-bit combine (little-endian: LSB first) ── */
static inline int16_t combine_bytes_le(uint8_t lsb, uint8_t msb)
{
    return (int16_t)((uint16_t)msb << 8 | lsb);
}

/*
 * Scale factor by FS (set during init).
 * NOTE: module-static, so only one BMI055 instance supported.
 * For multi-instance, move scale into a device context struct.
 */
static float s_accel_scale = BMI055_ACCEL_SCALE_16G;
static float s_gyro_scale = BMI055_GYRO_SCALE_2000DPS;

static float bmi055_get_accel_scale(uint8_t range)
{
    /* 12-bit signed, LSB/g varies by range */
    switch (range)
    {
    case BMI055_ACC_RANGE_2G:  return 9.80665f / 1024.0f;
    case BMI055_ACC_RANGE_4G:  return 9.80665f / 512.0f;
    case BMI055_ACC_RANGE_8G:  return 9.80665f / 256.0f;
    case BMI055_ACC_RANGE_16G: return 9.80665f / 128.0f;
    default:                   return 9.80665f / 128.0f;
    }
}

static float bmi055_get_gyro_scale(uint8_t range)
{
    /* 16-bit signed, LSB/dps, then deg→rad */
    switch (range)
    {
    case BMI055_GYRO_RANGE_125DPS:  return 0.01745329252f / 262.4f;
    case BMI055_GYRO_RANGE_250DPS:  return 0.01745329252f / 131.2f;
    case BMI055_GYRO_RANGE_500DPS:  return 0.01745329252f / 65.6f;
    case BMI055_GYRO_RANGE_1000DPS: return 0.01745329252f / 32.8f;
    case BMI055_GYRO_RANGE_2000DPS: return 0.01745329252f / 16.4f;
    default:                        return 0.01745329252f / 16.4f;
    }
}

/* ── Initialization ─────────────────────────────────────────── */

int bmi055_init(bmi055_dev_t *dev, const bmi055_config_t *cfg)
{
    bmi055_config_t default_cfg = BMI055_DEFAULT_CONFIG;
    if (cfg == NULL)
    {
        cfg = &default_cfg;
    }

    /* Set scale factor based on FS */
    s_accel_scale = bmi055_get_accel_scale(cfg->accel_range);
    s_gyro_scale = bmi055_get_gyro_scale(cfg->gyro_range);

    /* 1. Soft Reset -- accel and gyro separately */
    if (spi_write_reg(dev->accel_dev, BMI055_ACC_REG_SOFTRESET, BMI055_SOFT_RESET_CMD) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (spi_write_reg(dev->gyro_dev, BMI055_GYRO_REG_SOFTRESET, BMI055_SOFT_RESET_CMD) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    HAL_Delay(5); /* Datasheet: boot time after soft reset */

    /* 2. Chip ID verification -- accelerometer */
    uint8_t acc_id = 0;
    if (spi_read_reg(dev->accel_dev, BMI055_ACC_REG_CHIP_ID, &acc_id) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (acc_id != BMI055_ACC_CHIP_ID_VAL)
    {
        return SENSOR_ERR_ID;
    }

    /* 3. Chip ID verification -- gyroscope */
    uint8_t gyro_id = 0;
    if (spi_read_reg(dev->gyro_dev, BMI055_GYRO_REG_CHIP_ID, &gyro_id) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (gyro_id != BMI055_GYRO_CHIP_ID_VAL)
    {
        return SENSOR_ERR_ID;
    }

    /* 4. Accel config: Range → BW → Normal mode */
    if (spi_write_reg(dev->accel_dev, BMI055_ACC_REG_PMU_RANGE, cfg->accel_range) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (spi_write_reg(dev->accel_dev, BMI055_ACC_REG_PMU_BW, cfg->accel_bw) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (spi_write_reg(dev->accel_dev, BMI055_ACC_REG_PMU_LPW, BMI055_ACC_PM_NORMAL) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 5. Gyro config: Range → BW → Normal mode */
    if (spi_write_reg(dev->gyro_dev, BMI055_GYRO_REG_RANGE, cfg->gyro_range) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (spi_write_reg(dev->gyro_dev, BMI055_GYRO_REG_BW, cfg->gyro_bw) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (spi_write_reg(dev->gyro_dev, BMI055_GYRO_REG_LPM1, BMI055_GYRO_PM_NORMAL) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* Wait for power mode stabilization */
    HAL_Delay(5);

    /* 6. Config verification: accel Range readback */
    uint8_t readback = 0;
    if (spi_read_reg(dev->accel_dev, BMI055_ACC_REG_PMU_RANGE, &readback) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (readback != cfg->accel_range)
    {
        return SENSOR_ERR_CONFIG;
    }

    /* 7. Config verification: gyro Range readback */
    if (spi_read_reg(dev->gyro_dev, BMI055_GYRO_REG_RANGE, &readback) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (readback != cfg->gyro_range)
    {
        return SENSOR_ERR_CONFIG;
    }

    return SENSOR_OK;
}

/* ── Read accelerometer ────────────────────────────────────── */

int bmi055_read_accel(bmi055_dev_t *dev, vec3f_t *accel)
{
    /*
     * Accel data: LSB first, 12-bit
     * DATA_X_LSB(0x02) ~ DATA_Z_MSB(0x07) = 6 bytes consecutive
     *
     * Lower 4 bits of LSB register are new data flag etc.,
     * combine to 16-bit then >>4 to get 12-bit signed value
     */
    uint8_t buf[6];

    if (spi_read_bytes(dev->accel_dev, BMI055_ACC_REG_DATA_X_LSB, buf, 6) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    int16_t raw_x = combine_bytes_le(buf[0], buf[1]) >> 4;
    int16_t raw_y = combine_bytes_le(buf[2], buf[3]) >> 4;
    int16_t raw_z = combine_bytes_le(buf[4], buf[5]) >> 4;

    accel->x = (float)raw_x * s_accel_scale;
    accel->y = (float)raw_y * s_accel_scale;
    accel->z = (float)raw_z * s_accel_scale;

    return SENSOR_OK;
}

/* ── Read gyroscope ────────────────────────────────────── */

int bmi055_read_gyro(bmi055_dev_t *dev, vec3f_t *gyro)
{
    /*
     * Gyro data: LSB first, 16-bit
     * RATE_X_LSB(0x02) ~ RATE_Z_MSB(0x07) = 6 bytes consecutive
     */
    uint8_t buf[6];

    if (spi_read_bytes(dev->gyro_dev, BMI055_GYRO_REG_RATE_X_LSB, buf, 6) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    int16_t raw_x = combine_bytes_le(buf[0], buf[1]);
    int16_t raw_y = combine_bytes_le(buf[2], buf[3]);
    int16_t raw_z = combine_bytes_le(buf[4], buf[5]);

    gyro->x = (float)raw_x * s_gyro_scale;
    gyro->y = (float)raw_y * s_gyro_scale;
    gyro->z = (float)raw_z * s_gyro_scale;

    return SENSOR_OK;
}

/* ── Accel + gyro read ──────────────────────────── */

int bmi055_read_accel_gyro(bmi055_dev_t *dev, vec3f_t *accel, vec3f_t *gyro)
{
    /*
     * Separate dies require 2 SPI transactions.
     * 12-byte burst read like ICM-42688-P is not possible.
     */
    int ret = bmi055_read_accel(dev, accel);
    if (ret < 0)
    {
        return ret;
    }

    return bmi055_read_gyro(dev, gyro);
}
