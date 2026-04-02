/**
 * @file spi_wrapper.c
 * @brief Thin SPI wrapper — CS management + HAL calls
 *
 * All sensor drivers communicate via SPI through this wrapper.
 * Only this file needs modification when switching to DMA.
 */

#include "spi_wrapper.h"
#include <string.h>

#define SPI_TIMEOUT_MS 10

/* SPI read: set MSB to 1 (convention for most SPI sensors) */
#define SPI_READ_FLAG 0x80

/* Max burst read: 1 byte addr + 12 bytes data (IMU accel+gyro) */
#define SPI_BURST_MAX 13

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
    /* Write: MSB 0 (no read flag) */
    uint8_t tx[2] = {reg & 0x7F, val};

    cs_low(dev);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, tx, 2, SPI_TIMEOUT_MS);
    cs_high(dev);

    return (status == HAL_OK) ? 0 : -1;
}

int spi_read_bytes(const spi_dev_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    /*
     * Single TransmitReceive to avoid clock gap between address and data.
     * Split Transmit+Receive can cause SCK pause that some sensors don't tolerate,
     * especially during burst reads (e.g. 12-byte IMU accel+gyro).
     */
    if (len + 1 > SPI_BURST_MAX)
    {
        return -1;  /* Exceeds buffer, should not happen in normal use */
    }

    uint8_t tx[SPI_BURST_MAX];
    uint8_t rx[SPI_BURST_MAX];

    tx[0] = reg | SPI_READ_FLAG;
    memset(&tx[1], 0x00, len);

    cs_low(dev);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        dev->hspi, tx, rx, 1 + len, SPI_TIMEOUT_MS);
    cs_high(dev);

    if (status != HAL_OK)
    {
        return -1;
    }

    memcpy(buf, &rx[1], len);
    return 0;
}
