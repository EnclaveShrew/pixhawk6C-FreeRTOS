/**
 * @file bmi055.c
 * @brief BMI055 6-axis IMU driver implementation
 *
 * 데이터시트: BST-BMI055-DS000 (Bosch Sensortec)
 * SPI Mode 3 (CPOL=1, CPHA=1), max 10 MHz
 *
 * 가속도계(BMA280)와 자이로(BMG160)가 별도 다이.
 * 각각 독립된 CS 핀으로 SPI 통신.
 */

#include "bmi055.h"
#include "stm32h7xx_hal.h"

/* ── Helper: raw 16-bit 조합 (little-endian: LSB first) ── */
static inline int16_t combine_bytes_le(uint8_t lsb, uint8_t msb)
{
    return (int16_t)((uint16_t)msb << 8 | lsb);
}

/* ── 초기화 ─────────────────────────────────────────── */

int bmi055_init(bmi055_dev_t *dev, const bmi055_config_t *cfg)
{
    bmi055_config_t default_cfg = BMI055_DEFAULT_CONFIG;
    if (cfg == NULL)
    {
        cfg = &default_cfg;
    }

    /* 1. Soft Reset — 가속도계, 자이로 각각 */
    if (spi_write_reg(dev->accel_dev, BMI055_ACC_REG_SOFTRESET, BMI055_SOFT_RESET_CMD) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (spi_write_reg(dev->gyro_dev, BMI055_GYRO_REG_SOFTRESET, BMI055_SOFT_RESET_CMD) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    HAL_Delay(5); /* 데이터시트: soft reset 후 부팅 시간 */

    /* 2. Chip ID 검증 — 가속도계 */
    uint8_t acc_id = 0;
    if (spi_read_reg(dev->accel_dev, BMI055_ACC_REG_CHIP_ID, &acc_id) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (acc_id != BMI055_ACC_CHIP_ID_VAL)
    {
        return SENSOR_ERR_ID;
    }

    /* 3. Chip ID 검증 — 자이로 */
    uint8_t gyro_id = 0;
    if (spi_read_reg(dev->gyro_dev, BMI055_GYRO_REG_CHIP_ID, &gyro_id) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (gyro_id != BMI055_GYRO_CHIP_ID_VAL)
    {
        return SENSOR_ERR_ID;
    }

    /* 4. 가속도계 설정: Range → BW → Normal mode */
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

    /* 5. 자이로 설정: Range → BW → Normal mode */
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

    /* 전원 모드 전환 안정화 대기 */
    HAL_Delay(5);

    /* 6. 설정 검증: 가속도계 Range readback */
    uint8_t readback = 0;
    if (spi_read_reg(dev->accel_dev, BMI055_ACC_REG_PMU_RANGE, &readback) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (readback != cfg->accel_range)
    {
        return SENSOR_ERR_CONFIG;
    }

    /* 7. 설정 검증: 자이로 Range readback */
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

/* ── 가속도 읽기 ────────────────────────────────────── */

int bmi055_read_accel(bmi055_dev_t *dev, vec3f_t *accel)
{
    /*
     * 가속도 데이터: LSB first, 12-bit
     * DATA_X_LSB(0x02) ~ DATA_Z_MSB(0x07) = 6바이트 연속
     *
     * LSB 레지스터 하위 4비트는 새 데이터 플래그 등이므로,
     * 16-bit로 조합 후 >>4 하여 12-bit signed 값을 얻음
     */
    uint8_t buf[6];

    if (spi_read_bytes(dev->accel_dev, BMI055_ACC_REG_DATA_X_LSB, buf, 6) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    int16_t raw_x = combine_bytes_le(buf[0], buf[1]) >> 4;
    int16_t raw_y = combine_bytes_le(buf[2], buf[3]) >> 4;
    int16_t raw_z = combine_bytes_le(buf[4], buf[5]) >> 4;

    accel->x = (float)raw_x * BMI055_ACCEL_SCALE_16G;
    accel->y = (float)raw_y * BMI055_ACCEL_SCALE_16G;
    accel->z = (float)raw_z * BMI055_ACCEL_SCALE_16G;

    return SENSOR_OK;
}

/* ── 자이로 읽기 ────────────────────────────────────── */

int bmi055_read_gyro(bmi055_dev_t *dev, vec3f_t *gyro)
{
    /*
     * 자이로 데이터: LSB first, 16-bit
     * RATE_X_LSB(0x02) ~ RATE_Z_MSB(0x07) = 6바이트 연속
     */
    uint8_t buf[6];

    if (spi_read_bytes(dev->gyro_dev, BMI055_GYRO_REG_RATE_X_LSB, buf, 6) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    int16_t raw_x = combine_bytes_le(buf[0], buf[1]);
    int16_t raw_y = combine_bytes_le(buf[2], buf[3]);
    int16_t raw_z = combine_bytes_le(buf[4], buf[5]);

    gyro->x = (float)raw_x * BMI055_GYRO_SCALE_2000DPS;
    gyro->y = (float)raw_y * BMI055_GYRO_SCALE_2000DPS;
    gyro->z = (float)raw_z * BMI055_GYRO_SCALE_2000DPS;

    return SENSOR_OK;
}

/* ── 가속도 + 자이로 읽기 ──────────────────────────── */

int bmi055_read_accel_gyro(bmi055_dev_t *dev, vec3f_t *accel, vec3f_t *gyro)
{
    /*
     * 별도 다이이므로 SPI 트랜잭션 2회 필요.
     * ICM-42688-P처럼 12바이트 버스트 읽기는 불가.
     */
    int ret = bmi055_read_accel(dev, accel);
    if (ret < 0)
    {
        return ret;
    }

    return bmi055_read_gyro(dev, gyro);
}
