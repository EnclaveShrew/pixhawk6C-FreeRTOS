/**
 * @file icm42688p.c
 * @brief ICM-42688-P 6-axis IMU driver implementation
 *
 * Datasheet: DS-000347 (ICM-42688-P)
 * SPI Mode 3 (CPOL=1, CPHA=1), max 24 MHz
 */

#include "icm42688p.h"
#include "stm32h7xx_hal.h" /* uses HAL_Delay */

/* ── Helper: raw 16-bit combine (big-endian) ───────────── */
static inline int16_t combine_bytes(uint8_t high, uint8_t low)
{
    return (int16_t)((uint16_t)high << 8 | low);
}

/*
 * Scale factor by FS (set during init).
 * NOTE: module-static, so only one ICM-42688-P instance supported.
 * For multi-instance, move scale into a device context struct.
 */
static float s_accel_scale = ICM42688P_ACCEL_SCALE_16G;
static float s_gyro_scale = ICM42688P_GYRO_SCALE_2000DPS;

static float get_accel_scale(uint8_t fs)
{
    /* 16-bit signed, LSB/g = 32768 / FS_g */
    switch (fs)
    {
    case ICM42688P_ACCEL_FS_2G:  return 9.80665f / 16384.0f;
    case ICM42688P_ACCEL_FS_4G:  return 9.80665f / 8192.0f;
    case ICM42688P_ACCEL_FS_8G:  return 9.80665f / 4096.0f;
    case ICM42688P_ACCEL_FS_16G: return 9.80665f / 2048.0f;
    default:                     return 9.80665f / 2048.0f;
    }
}

static float get_gyro_scale(uint8_t fs)
{
    /* 16-bit signed, LSB/dps = 32768 / FS_dps, then deg→rad */
    switch (fs)
    {
    case ICM42688P_GYRO_FS_250DPS:  return 0.01745329252f / 131.0f;
    case ICM42688P_GYRO_FS_500DPS:  return 0.01745329252f / 65.5f;
    case ICM42688P_GYRO_FS_1000DPS: return 0.01745329252f / 32.8f;
    case ICM42688P_GYRO_FS_2000DPS: return 0.01745329252f / 16.4f;
    default:                        return 0.01745329252f / 16.4f;
    }
}

/* ── Initialization ─────────────────────────────────────────── */

int icm42688p_init(spi_dev_t *dev, const icm42688p_config_t *cfg)
{
    icm42688p_config_t default_cfg = ICM42688P_DEFAULT_CONFIG;
    if (cfg == NULL)
    {
        cfg = &default_cfg;
    }

    /* Set scale factor based on FS */
    s_accel_scale = get_accel_scale(cfg->accel_fs);
    s_gyro_scale = get_gyro_scale(cfg->gyro_fs);

    /* 1. Soft Reset */
    if (spi_write_reg(dev, ICM42688P_REG_DEVICE_CONFIG, ICM42688P_SOFT_RESET) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    HAL_Delay(1); /* Datasheet: wait min 1ms after soft reset */

    /* 2. WHO_AM_I verification */
    uint8_t who = 0;
    if (spi_read_reg(dev, ICM42688P_REG_WHO_AM_I, &who) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (who != ICM42688P_WHO_AM_I_VAL)
    {
        return SENSOR_ERR_ID;
    }

    /* 3. Select Bank 0 (default, but explicit) */
    if (spi_write_reg(dev, ICM42688P_REG_BANK_SEL, 0x00) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 4. Gyro config: FS + ODR */
    uint8_t gyro_cfg = cfg->gyro_fs | cfg->gyro_odr;
    if (spi_write_reg(dev, ICM42688P_REG_GYRO_CONFIG0, gyro_cfg) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 5. Accel config: FS + ODR */
    uint8_t accel_cfg = cfg->accel_fs | cfg->accel_odr;
    if (spi_write_reg(dev, ICM42688P_REG_ACCEL_CONFIG0, accel_cfg) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 6. INT1 config: Data Ready interrupt (Push-Pull, Active High, Pulsed) */
    if (spi_write_reg(dev, ICM42688P_REG_INT_CONFIG, ICM42688P_INT1_ACTIVE_HIGH | ICM42688P_INT1_PUSH_PULL) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (spi_write_reg(dev, ICM42688P_REG_INT_SOURCE0, ICM42688P_INT_DRDY_EN) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 7. Enable Accel + Gyro Low Noise mode */
    if (spi_write_reg(dev, ICM42688P_REG_PWR_MGMT0, ICM42688P_PWR_GYRO_LN | ICM42688P_PWR_ACCEL_LN) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /*
     * Datasheet: first valid data after Accel/Gyro power on
     * Gyro: 30ms, Accel: 20ms
     */
    HAL_Delay(30);

    /* 8. Config verification: GYRO_CONFIG0 readback */
    uint8_t readback = 0;
    if (spi_read_reg(dev, ICM42688P_REG_GYRO_CONFIG0, &readback) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (readback != gyro_cfg)
    {
        return SENSOR_ERR_CONFIG;
    }

    return SENSOR_OK;
}

/* ── Read accelerometer ────────────────────────────────────── */

int icm42688p_read_accel(spi_dev_t *dev, vec3f_t *accel)
{
    uint8_t buf[6];

    if (spi_read_bytes(dev, ICM42688P_REG_ACCEL_DATA_X1, buf, 6) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    int16_t raw_x = combine_bytes(buf[0], buf[1]);
    int16_t raw_y = combine_bytes(buf[2], buf[3]);
    int16_t raw_z = combine_bytes(buf[4], buf[5]);

    accel->x = (float)raw_x * s_accel_scale;
    accel->y = (float)raw_y * s_accel_scale;
    accel->z = (float)raw_z * s_accel_scale;

    return SENSOR_OK;
}

/* ── Read gyroscope ────────────────────────────────────── */

int icm42688p_read_gyro(spi_dev_t *dev, vec3f_t *gyro)
{
    uint8_t buf[6];

    if (spi_read_bytes(dev, ICM42688P_REG_GYRO_DATA_X1, buf, 6) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    int16_t raw_x = combine_bytes(buf[0], buf[1]);
    int16_t raw_y = combine_bytes(buf[2], buf[3]);
    int16_t raw_z = combine_bytes(buf[4], buf[5]);

    gyro->x = (float)raw_x * s_gyro_scale;
    gyro->y = (float)raw_y * s_gyro_scale;
    gyro->z = (float)raw_z * s_gyro_scale;

    return SENSOR_OK;
}

/* ── Accel + gyro burst read ────────────────────── */

int icm42688p_read_accel_gyro(spi_dev_t *dev, vec3f_t *accel, vec3f_t *gyro)
{
    /*
     * ACCEL_DATA_X1(0x1F) ~ GYRO_DATA_Z0(0x2A) = 12 bytes consecutive
     * Single SPI transaction via burst read
     */
    uint8_t buf[12];

    if (spi_read_bytes(dev, ICM42688P_REG_ACCEL_DATA_X1, buf, 12) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* Accel: buf[0..5] */
    int16_t ax = combine_bytes(buf[0], buf[1]);
    int16_t ay = combine_bytes(buf[2], buf[3]);
    int16_t az = combine_bytes(buf[4], buf[5]);

    accel->x = (float)ax * s_accel_scale;
    accel->y = (float)ay * s_accel_scale;
    accel->z = (float)az * s_accel_scale;

    /* Gyro: buf[6..11] */
    int16_t gx = combine_bytes(buf[6], buf[7]);
    int16_t gy = combine_bytes(buf[8], buf[9]);
    int16_t gz = combine_bytes(buf[10], buf[11]);

    gyro->x = (float)gx * s_gyro_scale;
    gyro->y = (float)gy * s_gyro_scale;
    gyro->z = (float)gz * s_gyro_scale;

    return SENSOR_OK;
}

/* ── Read temperature ──────────────────────────────────────── */

int icm42688p_read_temp(spi_dev_t *dev, float *temp_c)
{
    uint8_t buf[2];

    if (spi_read_bytes(dev, ICM42688P_REG_TEMP_DATA1, buf, 2) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    int16_t raw = combine_bytes(buf[0], buf[1]);
    *temp_c = (float)raw * ICM42688P_TEMP_SCALE + ICM42688P_TEMP_OFFSET;

    return SENSOR_OK;
}
