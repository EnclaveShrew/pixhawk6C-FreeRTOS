/**
 * @file spi_wrapper.c
 * @brief Thin SPI wrapper — CS 관리 + HAL 호출
 *
 * 모든 센서 드라이버는 이 래퍼를 통해 SPI 통신.
 * 나중에 DMA로 전환 시 이 파일 내부만 수정하면 됨.
 */

#include "spi_wrapper.h"

#define SPI_TIMEOUT_MS 10

/* SPI 읽기: 최상위 비트를 1로 설정 (대부분의 SPI 센서 규약) */
#define SPI_READ_FLAG 0x80

/* ── CS assert/deassert ─────────────────────────────── */
static inline void cs_low(const spi_dev_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(const spi_dev_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/* ── Public API ─────────────────────────────────────── */

int spi_read_reg(const spi_dev_t *dev, uint8_t reg, uint8_t *val)
{
    uint8_t tx[2] = {reg | SPI_READ_FLAG, 0x00};
    uint8_t rx[2] = {0};

    cs_low(dev);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, SPI_TIMEOUT_MS);
    cs_high(dev);

    if (status != HAL_OK)
    {
        return -1;
    }
    *val = rx[1];
    return 0;
}

int spi_write_reg(const spi_dev_t *dev, uint8_t reg, uint8_t val)
{
    /* 쓰기: 최상위 비트 0 (읽기 플래그 없음) */
    uint8_t tx[2] = {reg & 0x7F, val};

    cs_low(dev);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, tx, 2, SPI_TIMEOUT_MS);
    cs_high(dev);

    return (status == HAL_OK) ? 0 : -1;
}

int spi_read_bytes(const spi_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t tx_addr = reg | SPI_READ_FLAG;

    cs_low(dev);

    /* 레지스터 주소 전송 */
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, &tx_addr, 1, SPI_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        cs_high(dev);
        return -1;
    }

    /* 데이터 수신 */
    status = HAL_SPI_Receive(dev->hspi, buf, len, SPI_TIMEOUT_MS);
    cs_high(dev);

    return (status == HAL_OK) ? 0 : -1;
}
