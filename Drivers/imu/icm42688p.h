/**
 * @file icm42688p.h
 * @brief ICM-42688-P 6-axis IMU driver (InvenSense/TDK)
 *
 * Interface: SPI (Mode 3, max 24 MHz)
 * Accel: ±16g, 1kHz ODR
 * Gyro:  ±2000 dps, 1kHz ODR
 */

#ifndef ICM42688P_H
#define ICM42688P_H

#include "sensor_types.h"
#include "spi_wrapper.h"

/* ══════════════════════════════════════════════════════
 *  Register Map — Bank 0 (default)
 * ══════════════════════════════════════════════════════ */

/* Device identification */
#define ICM42688P_REG_DEVICE_CONFIG 0x11
#define ICM42688P_REG_INT_CONFIG 0x14
#define ICM42688P_REG_FIFO_CONFIG 0x16
#define ICM42688P_REG_TEMP_DATA1 0x1D
#define ICM42688P_REG_TEMP_DATA0 0x1E

/* Accelerometer output registers */
#define ICM42688P_REG_ACCEL_DATA_X1 0x1F
#define ICM42688P_REG_ACCEL_DATA_X0 0x20
#define ICM42688P_REG_ACCEL_DATA_Y1 0x21
#define ICM42688P_REG_ACCEL_DATA_Y0 0x22
#define ICM42688P_REG_ACCEL_DATA_Z1 0x23
#define ICM42688P_REG_ACCEL_DATA_Z0 0x24

/* Gyroscope output registers */
#define ICM42688P_REG_GYRO_DATA_X1 0x25
#define ICM42688P_REG_GYRO_DATA_X0 0x26
#define ICM42688P_REG_GYRO_DATA_Y1 0x27
#define ICM42688P_REG_GYRO_DATA_Y0 0x28
#define ICM42688P_REG_GYRO_DATA_Z1 0x29
#define ICM42688P_REG_GYRO_DATA_Z0 0x2A

/* Status */
#define ICM42688P_REG_INT_STATUS 0x2D
#define ICM42688P_REG_INT_STATUS2 0x37
#define ICM42688P_REG_INT_STATUS3 0x38

/* Configuration */
#define ICM42688P_REG_INT_CONFIG0 0x63
#define ICM42688P_REG_INT_CONFIG1 0x64
#define ICM42688P_REG_INT_SOURCE0 0x65
#define ICM42688P_REG_INT_SOURCE1 0x66
#define ICM42688P_REG_GYRO_CONFIG0 0x4F
#define ICM42688P_REG_ACCEL_CONFIG0 0x50
#define ICM42688P_REG_GYRO_CONFIG1 0x51
#define ICM42688P_REG_ACCEL_CONFIG1 0x53
#define ICM42688P_REG_PWR_MGMT0 0x4E
#define ICM42688P_REG_WHO_AM_I 0x75
#define ICM42688P_REG_BANK_SEL 0x76

/* ══════════════════════════════════════════════════════
 *  Register Values / Bit Masks
 * ══════════════════════════════════════════════════════ */

/* WHO_AM_I expected value */
#define ICM42688P_WHO_AM_I_VAL 0x47

/* DEVICE_CONFIG */
#define ICM42688P_SOFT_RESET 0x01

/* PWR_MGMT0 */
#define ICM42688P_PWR_GYRO_LN (0x03 << 2)  /* Gyro Low Noise mode */
#define ICM42688P_PWR_ACCEL_LN (0x03 << 0) /* Accel Low Noise mode */

/* GYRO_CONFIG0: FS_SEL + ODR */
#define ICM42688P_GYRO_FS_2000DPS (0x00 << 5)
#define ICM42688P_GYRO_FS_1000DPS (0x01 << 5)
#define ICM42688P_GYRO_FS_500DPS (0x02 << 5)
#define ICM42688P_GYRO_FS_250DPS (0x03 << 5)
#define ICM42688P_GYRO_ODR_1KHZ 0x06

/* ACCEL_CONFIG0: FS_SEL + ODR */
#define ICM42688P_ACCEL_FS_16G (0x00 << 5)
#define ICM42688P_ACCEL_FS_8G (0x01 << 5)
#define ICM42688P_ACCEL_FS_4G (0x02 << 5)
#define ICM42688P_ACCEL_FS_2G (0x03 << 5)
#define ICM42688P_ACCEL_ODR_1KHZ 0x06

/* INT_CONFIG: INT1 push-pull, active high */
#define ICM42688P_INT1_ACTIVE_HIGH (0x00 << 0)
#define ICM42688P_INT1_PUSH_PULL (0x00 << 2)
#define ICM42688P_INT1_PULSED (0x00 << 3)

/* INT_SOURCE0 */
#define ICM42688P_INT_DRDY_EN (1 << 3)

/* ══════════════════════════════════════════════════════
 *  Scale factors
 * ══════════════════════════════════════════════════════ */

/* 가속도: raw → m/s² (±16g, 16-bit signed → 2048 LSB/g) */
#define ICM42688P_ACCEL_SCALE_16G (9.80665f / 2048.0f)

/* 자이로: raw → rad/s (±2000dps, 16-bit signed → 16.4 LSB/dps) */
#define ICM42688P_GYRO_SCALE_2000DPS (0.01745329252f / 16.4f)

/* 온도: raw → °C */
#define ICM42688P_TEMP_SCALE (1.0f / 132.48f)
#define ICM42688P_TEMP_OFFSET 25.0f

/* ══════════════════════════════════════════════════════
 *  Configuration structure
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    uint8_t accel_fs;  /* ACCEL_FS_xxG */
    uint8_t accel_odr; /* ACCEL_ODR_xxKHZ */
    uint8_t gyro_fs;   /* GYRO_FS_xxxxDPS */
    uint8_t gyro_odr;  /* GYRO_ODR_xxKHZ */
} icm42688p_config_t;

/* 기본 설정: ±16g, ±2000dps, 1kHz */
#define ICM42688P_DEFAULT_CONFIG                                                                                       \
    {                                                                                                                  \
        .accel_fs = ICM42688P_ACCEL_FS_16G,                                                                            \
        .accel_odr = ICM42688P_ACCEL_ODR_1KHZ,                                                                         \
        .gyro_fs = ICM42688P_GYRO_FS_2000DPS,                                                                          \
        .gyro_odr = ICM42688P_GYRO_ODR_1KHZ,                                                                           \
    }

/* ══════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════ */

/**
 * @brief Initialize ICM-42688-P IMU sensor
 * @param dev    SPI device descriptor (SPI handle + CS pin)
 * @param cfg    Configuration (NULL for defaults)
 * @return 0 on success, negative error code on failure
 *
 * Initialization sequence:
 * 1. Soft reset
 * 2. Verify WHO_AM_I register (expect 0x47)
 * 3. Configure accelerometer (FS, ODR)
 * 4. Configure gyroscope (FS, ODR)
 * 5. Enable Low Noise mode for both sensors
 */
int icm42688p_init(spi_dev_t *dev, const icm42688p_config_t *cfg);

/**
 * @brief Read accelerometer data
 * @param dev    SPI device descriptor
 * @param accel  Output: acceleration in m/s²
 * @return 0 on success, negative on error
 */
int icm42688p_read_accel(spi_dev_t *dev, vec3f_t *accel);

/**
 * @brief Read gyroscope data
 * @param dev    SPI device descriptor
 * @param gyro   Output: angular rate in rad/s
 * @return 0 on success, negative on error
 */
int icm42688p_read_gyro(spi_dev_t *dev, vec3f_t *gyro);

/**
 * @brief Read accelerometer and gyroscope in a single burst
 * @param dev    SPI device descriptor
 * @param accel  Output: acceleration in m/s²
 * @param gyro   Output: angular rate in rad/s
 * @return 0 on success, negative on error
 */
int icm42688p_read_accel_gyro(spi_dev_t *dev, vec3f_t *accel, vec3f_t *gyro);

/**
 * @brief Read die temperature
 * @param dev     SPI device descriptor
 * @param temp_c  Output: temperature in °C
 * @return 0 on success, negative on error
 */
int icm42688p_read_temp(spi_dev_t *dev, float *temp_c);

#endif /* ICM42688P_H */
