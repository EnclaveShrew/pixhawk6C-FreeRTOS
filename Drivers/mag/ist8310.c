/**
 * @file ist8310.c
 * @brief IST8310 3-axis magnetometer driver implementation
 *
 * 데이터시트: IST8310 (iSentek)
 * I2C, 16-bit signed, little-endian
 * Single measurement 모드 사용
 */

#include "ist8310.h"
#include "stm32h7xx_hal.h"

/* ── Helper: raw 16-bit 조합 (little-endian) ────────── */
static inline int16_t combine_bytes_le(uint8_t lsb, uint8_t msb)
{
    return (int16_t)((uint16_t)msb << 8 | lsb);
}

/* ── 초기화 ─────────────────────────────────────────── */

int ist8310_init(i2c_dev_t *dev)
{
    /* 1. WHO_AM_I 검증 */
    uint8_t who = 0;
    if (i2c_read_reg(dev, IST8310_REG_WHO_AM_I, &who) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (who != IST8310_WHO_AM_I_VAL)
    {
        return SENSOR_ERR_ID;
    }

    /* 2. Standby 모드 진입 (설정 변경 전 필수) */
    if (i2c_write_reg(dev, IST8310_REG_CNTL1, IST8310_MODE_STANDBY) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 3. 평균 횟수 설정: 16회 (노이즈 저감) */
    if (i2c_write_reg(dev, IST8310_REG_AVGCNTL, IST8310_AVG_16) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 4. Set/Reset 펄스 설정 */
    if (i2c_write_reg(dev, IST8310_REG_PDCNTL, IST8310_PDCNTL_NORMAL) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 5. 설정 검증: AVGCNTL readback */
    uint8_t readback = 0;
    if (i2c_read_reg(dev, IST8310_REG_AVGCNTL, &readback) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    if (readback != IST8310_AVG_16)
    {
        return SENSOR_ERR_CONFIG;
    }

    return SENSOR_OK;
}

/* ── 자기장 읽기 ────────────────────────────────────── */

int ist8310_read(i2c_dev_t *dev, vec3f_t *mag)
{
    /* 1. Single measurement 트리거 */
    if (i2c_write_reg(dev, IST8310_REG_CNTL1, IST8310_MODE_SINGLE) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 2. DRDY 대기 (폴링) */
    uint8_t stat = 0;
    uint32_t start = HAL_GetTick();

    while (1)
    {
        if (i2c_read_reg(dev, IST8310_REG_STAT1, &stat) < 0)
        {
            return SENSOR_ERR_COMM;
        }
        if (stat & IST8310_STAT_DRDY)
        {
            break;
        }
        if ((HAL_GetTick() - start) >= IST8310_MEAS_TIMEOUT_MS)
        {
            return SENSOR_ERR_TIMEOUT;
        }
    }

    /* 3. 데이터 읽기: X_LSB(0x03) ~ Z_MSB(0x08) = 6바이트 */
    uint8_t buf[6];
    if (i2c_read_bytes(dev, IST8310_REG_DATA_X_LSB, buf, 6) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    int16_t raw_x = combine_bytes_le(buf[0], buf[1]);
    int16_t raw_y = combine_bytes_le(buf[2], buf[3]);
    int16_t raw_z = combine_bytes_le(buf[4], buf[5]);

    /* raw → uT 변환 */
    mag->x = (float)raw_x * IST8310_SCALE_UT;
    mag->y = (float)raw_y * IST8310_SCALE_UT;
    mag->z = (float)raw_z * IST8310_SCALE_UT;

    return SENSOR_OK;
}
