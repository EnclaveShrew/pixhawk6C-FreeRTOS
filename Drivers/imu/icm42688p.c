/**
 * @file icm42688p.c
 * @brief ICM-42688-P 6-axis IMU driver implementation
 *
 * 데이터시트: DS-000347 (ICM-42688-P)
 * SPI Mode 3 (CPOL=1, CPHA=1), max 24 MHz
 */

#include "icm42688p.h"
#include "stm32h7xx_hal.h" /* HAL_Delay 사용 */

/* ── Helper: raw 16-bit 조합 (big-endian) ───────────── */
static inline int16_t combine_bytes(uint8_t high, uint8_t low)
{
    return (int16_t)((uint16_t)high << 8 | low);
}

/* ── 초기화 ─────────────────────────────────────────── */

int icm42688p_init(spi_dev_t *dev, const icm42688p_config_t *cfg)
{
    icm42688p_config_t default_cfg = ICM42688P_DEFAULT_CONFIG;
    if (cfg == NULL)
    {
        cfg = &default_cfg;
    }

    /* 1. Soft Reset */
    if (spi_write_reg(dev, ICM42688P_REG_DEVICE_CONFIG, ICM42688P_SOFT_RESET) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    HAL_Delay(1); /* 데이터시트: soft reset 후 최소 1ms 대기 */

    /* 2. WHO_AM_I 검증 */
    uint8_t who = 0;
    if (spi_read_reg(dev, ICM42688P_REG_WHO_AM_I, &who) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (who != ICM42688P_WHO_AM_I_VAL)
    {
        return SENSOR_ERR_ID;
    }

    /* 3. Bank 0 선택 (기본값이지만 명시적으로) */
    if (spi_write_reg(dev, ICM42688P_REG_BANK_SEL, 0x00) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 4. 자이로 설정: FS + ODR */
    uint8_t gyro_cfg = cfg->gyro_fs | cfg->gyro_odr;
    if (spi_write_reg(dev, ICM42688P_REG_GYRO_CONFIG0, gyro_cfg) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 5. 가속도계 설정: FS + ODR */
    uint8_t accel_cfg = cfg->accel_fs | cfg->accel_odr;
    if (spi_write_reg(dev, ICM42688P_REG_ACCEL_CONFIG0, accel_cfg) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 6. INT1 설정: Data Ready 인터럽트 (Push-Pull, Active High, Pulsed) */
    if (spi_write_reg(dev, ICM42688P_REG_INT_CONFIG, ICM42688P_INT1_ACTIVE_HIGH | ICM42688P_INT1_PUSH_PULL) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (spi_write_reg(dev, ICM42688P_REG_INT_SOURCE0, ICM42688P_INT_DRDY_EN) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 7. Accel + Gyro Low Noise 모드 활성화 */
    if (spi_write_reg(dev, ICM42688P_REG_PWR_MGMT0, ICM42688P_PWR_GYRO_LN | ICM42688P_PWR_ACCEL_LN) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /*
     * 데이터시트: Accel/Gyro 켜진 후 첫 유효 데이터까지
     * Gyro: 30ms, Accel: 20ms
     */
    HAL_Delay(30);

    /* 8. 설정 검증: GYRO_CONFIG0 readback */
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

/* ── 가속도 읽기 ────────────────────────────────────── */

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

    accel->x = (float)raw_x * ICM42688P_ACCEL_SCALE_16G;
    accel->y = (float)raw_y * ICM42688P_ACCEL_SCALE_16G;
    accel->z = (float)raw_z * ICM42688P_ACCEL_SCALE_16G;

    return SENSOR_OK;
}

/* ── 자이로 읽기 ────────────────────────────────────── */

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

    gyro->x = (float)raw_x * ICM42688P_GYRO_SCALE_2000DPS;
    gyro->y = (float)raw_y * ICM42688P_GYRO_SCALE_2000DPS;
    gyro->z = (float)raw_z * ICM42688P_GYRO_SCALE_2000DPS;

    return SENSOR_OK;
}

/* ── 가속도 + 자이로 버스트 읽기 ────────────────────── */

int icm42688p_read_accel_gyro(spi_dev_t *dev, vec3f_t *accel, vec3f_t *gyro)
{
    /*
     * ACCEL_DATA_X1(0x1F) ~ GYRO_DATA_Z0(0x2A) = 12바이트 연속
     * 버스트 읽기로 SPI 트랜잭션 1회로 처리
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

    accel->x = (float)ax * ICM42688P_ACCEL_SCALE_16G;
    accel->y = (float)ay * ICM42688P_ACCEL_SCALE_16G;
    accel->z = (float)az * ICM42688P_ACCEL_SCALE_16G;

    /* Gyro: buf[6..11] */
    int16_t gx = combine_bytes(buf[6], buf[7]);
    int16_t gy = combine_bytes(buf[8], buf[9]);
    int16_t gz = combine_bytes(buf[10], buf[11]);

    gyro->x = (float)gx * ICM42688P_GYRO_SCALE_2000DPS;
    gyro->y = (float)gy * ICM42688P_GYRO_SCALE_2000DPS;
    gyro->z = (float)gz * ICM42688P_GYRO_SCALE_2000DPS;

    return SENSOR_OK;
}

/* ── 온도 읽기 ──────────────────────────────────────── */

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
