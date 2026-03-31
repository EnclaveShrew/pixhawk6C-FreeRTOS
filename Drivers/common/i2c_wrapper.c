/**
 * @file i2c_wrapper.c
 * @brief Thin I2C wrapper — 주소 관리 + HAL 호출
 *
 * 모든 I2C 센서 드라이버는 이 래퍼를 통해 통신.
 * 나중에 DMA로 전환 시 이 파일 내부만 수정하면 됨.
 */

#include "i2c_wrapper.h"

#define I2C_TIMEOUT_MS 10

/* ── Public API ─────────────────────────────────────── */

int i2c_read_reg(const i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    /*
     * HAL_I2C_Mem_Read: 레지스터 주소 전송 → 데이터 수신을 한 번에 처리
     * 내부적으로 START → 주소+W → 레지스터 → RE-START → 주소+R → 데이터 → STOP
     */
    HAL_StatusTypeDef status =
        HAL_I2C_Mem_Read(dev->hi2c, dev->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, val, 1, I2C_TIMEOUT_MS);

    return (status == HAL_OK) ? 0 : -1;
}

int i2c_write_reg(const i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    /*
     * HAL_I2C_Mem_Write: START → 주소+W → 레지스터 → 데이터 → STOP
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
