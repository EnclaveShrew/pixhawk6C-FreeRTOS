/**
 * @file ms5611.c
 * @brief MS5611 barometric pressure sensor driver implementation
 *
 * 데이터시트: MS5611-01BA03 (TE Connectivity)
 * I2C, 24-bit ADC, 공장 교정 PROM 포함
 *
 * MS5611은 일반적인 레지스터 R/W 방식이 아니라,
 * 커맨드 전송 → 변환 대기 → ADC 읽기 방식으로 동작.
 */

#include "ms5611.h"
#include "stm32h7xx_hal.h"

/* ── OSR별 변환 커맨드 및 대기 시간 ────────────────── */

/* D1(기압) 변환 커맨드 테이블 */
static const uint8_t d1_cmd[] = {
    MS5611_CMD_CONV_D1_256,  MS5611_CMD_CONV_D1_512,  MS5611_CMD_CONV_D1_1024,
    MS5611_CMD_CONV_D1_2048, MS5611_CMD_CONV_D1_4096,
};

/* D2(온도) 변환 커맨드 테이블 */
static const uint8_t d2_cmd[] = {
    MS5611_CMD_CONV_D2_256,  MS5611_CMD_CONV_D2_512,  MS5611_CMD_CONV_D2_1024,
    MS5611_CMD_CONV_D2_2048, MS5611_CMD_CONV_D2_4096,
};

/* 변환 대기 시간 (ms), OSR 인덱스 순 */
static const uint8_t conv_delay_ms[] = {1, 2, 3, 5, 10};

/* ── 내부 함수: 커맨드 전송 ────────────────────────── */

static int ms5611_send_cmd(ms5611_dev_t *dev, uint8_t cmd)
{
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(dev->i2c_dev->hi2c, dev->i2c_dev->addr << 1, &cmd, 1, 10);

    return (status == HAL_OK) ? 0 : -1;
}

/* ── 내부 함수: ADC 읽기 (24-bit) ──────────────────── */

static int ms5611_read_adc(ms5611_dev_t *dev, uint32_t *adc_val)
{
    uint8_t cmd = MS5611_CMD_ADC_READ;
    uint8_t buf[3] = {0};

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(dev->i2c_dev->hi2c, dev->i2c_dev->addr << 1, &cmd, 1, 10);
    if (status != HAL_OK)
    {
        return -1;
    }

    status = HAL_I2C_Master_Receive(dev->i2c_dev->hi2c, dev->i2c_dev->addr << 1, buf, 3, 10);
    if (status != HAL_OK)
    {
        return -1;
    }

    *adc_val = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    return 0;
}

/* ── 내부 함수: PROM 읽기 (16-bit) ─────────────────── */

static int ms5611_read_prom(ms5611_dev_t *dev, uint8_t cmd, uint16_t *val)
{
    uint8_t buf[2] = {0};

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(dev->i2c_dev->hi2c, dev->i2c_dev->addr << 1, &cmd, 1, 10);
    if (status != HAL_OK)
    {
        return -1;
    }

    status = HAL_I2C_Master_Receive(dev->i2c_dev->hi2c, dev->i2c_dev->addr << 1, buf, 2, 10);
    if (status != HAL_OK)
    {
        return -1;
    }

    *val = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}

/* ── 초기화 ─────────────────────────────────────────── */

int ms5611_init(ms5611_dev_t *dev, ms5611_osr_t osr)
{
    dev->osr = osr;

    /* 1. Soft Reset */
    if (ms5611_send_cmd(dev, MS5611_CMD_RESET) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    HAL_Delay(3); /* 데이터시트: reset 후 reload 시간 */

    /* 2. PROM 교정 계수 읽기 (C1~C6) */
    if (ms5611_read_prom(dev, MS5611_CMD_PROM_C1, &dev->c1) < 0 ||
        ms5611_read_prom(dev, MS5611_CMD_PROM_C2, &dev->c2) < 0 ||
        ms5611_read_prom(dev, MS5611_CMD_PROM_C3, &dev->c3) < 0 ||
        ms5611_read_prom(dev, MS5611_CMD_PROM_C4, &dev->c4) < 0 ||
        ms5611_read_prom(dev, MS5611_CMD_PROM_C5, &dev->c5) < 0 ||
        ms5611_read_prom(dev, MS5611_CMD_PROM_C6, &dev->c6) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 3. PROM 유효성 검증: 모두 0이거나 모두 0xFFFF면 통신 실패 */
    if ((dev->c1 == 0 && dev->c2 == 0 && dev->c3 == 0) || (dev->c1 == 0xFFFF && dev->c2 == 0xFFFF))
    {
        return SENSOR_ERR_ID;
    }

    return SENSOR_OK;
}

/* ── 온도 + 기압 읽기 (2nd order compensation) ──────── */

int ms5611_read(ms5611_dev_t *dev, float *temp_c, float *press_pa)
{
    uint32_t d1 = 0; /* raw pressure */
    uint32_t d2 = 0; /* raw temperature */

    /* 1. 온도 변환 시작 → 대기 → ADC 읽기 */
    if (ms5611_send_cmd(dev, d2_cmd[dev->osr]) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    HAL_Delay(conv_delay_ms[dev->osr]);

    if (ms5611_read_adc(dev, &d2) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /* 2. 기압 변환 시작 → 대기 → ADC 읽기 */
    if (ms5611_send_cmd(dev, d1_cmd[dev->osr]) < 0)
    {
        return SENSOR_ERR_COMM;
    }
    HAL_Delay(conv_delay_ms[dev->osr]);

    if (ms5611_read_adc(dev, &d1) < 0)
    {
        return SENSOR_ERR_COMM;
    }

    /*
     * 3. 2nd order temperature compensation
     *
     * 데이터시트 공식 (정수 연산):
     * dT = D2 - C5 * 2^8
     * TEMP = 2000 + dT * C6 / 2^23
     * OFF  = C2 * 2^16 + (C4 * dT) / 2^7
     * SENS = C1 * 2^15 + (C3 * dT) / 2^8
     *
     * 2nd order (TEMP < 20°C):
     *   T2    = dT^2 / 2^31
     *   OFF2  = 5 * (TEMP-2000)^2 / 2
     *   SENS2 = 5 * (TEMP-2000)^2 / 4
     *
     * 극저온 (TEMP < -15°C):
     *   OFF2  += 7 * (TEMP+1500)^2
     *   SENS2 += 11 * (TEMP+1500)^2 / 2
     */

    int64_t dT = (int64_t)d2 - ((int64_t)dev->c5 << 8);
    int64_t TEMP = 2000 + (dT * (int64_t)dev->c6) / (1LL << 23);

    int64_t OFF = ((int64_t)dev->c2 << 16) + ((int64_t)dev->c4 * dT) / (1LL << 7);
    int64_t SENS = ((int64_t)dev->c1 << 15) + ((int64_t)dev->c3 * dT) / (1LL << 8);

    /* 2nd order compensation */
    if (TEMP < 2000)
    {
        int64_t T2 = (dT * dT) / (1LL << 31);
        int64_t TEMP2 = TEMP - 2000;
        int64_t OFF2 = 5 * TEMP2 * TEMP2 / 2;
        int64_t SENS2 = 5 * TEMP2 * TEMP2 / 4;

        /* 극저온 보상 */
        if (TEMP < -1500)
        {
            int64_t TEMP3 = TEMP + 1500;
            OFF2 += 7 * TEMP3 * TEMP3;
            SENS2 += 11 * TEMP3 * TEMP3 / 2;
        }

        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    /* 4. 최종 기압 계산 */
    int64_t P = ((int64_t)d1 * SENS / (1LL << 21) - OFF) / (1LL << 15);

    /* TEMP: 1/100 °C 단위, P: 1/100 mbar 단위 → 변환 */
    if (temp_c != NULL)
    {
        *temp_c = (float)TEMP / 100.0f;
    }
    if (press_pa != NULL)
    {
        *press_pa = (float)P; /* 단위: Pa (= 1/100 mbar * 100) */
    }

    return SENSOR_OK;
}
