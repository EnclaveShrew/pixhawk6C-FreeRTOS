/**
 * @file bmi055.h
 * @brief BMI055 6-axis IMU driver (Bosch)
 *
 * Interface: SPI (Mode 3, max 10 MHz)
 * Accel: ±16g, 1kHz BW (BMA280 die, 12-bit)
 * Gyro:  ±2000 dps, 230Hz BW (BMG160 die, 16-bit)
 *
 * Accel and gyro are separate dies -> two CS pins needed
 */

#ifndef BMI055_H
#define BMI055_H

#include "sensor_types.h"
#include "spi_wrapper.h"

/* ══════════════════════════════════════════════════════
 *  Device descriptor — SPI devices for accel/gyro separately
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    spi_dev_t *accel_dev; /* Accelerometer SPI (separate CS) */
    spi_dev_t *gyro_dev;  /* Gyroscope SPI (separate CS) */
} bmi055_dev_t;

/* ══════════════════════════════════════════════════════
 *  Accelerometer Register Map (BMA280 die)
 * ══════════════════════════════════════════════════════ */

#define BMI055_ACC_REG_CHIP_ID 0x00
#define BMI055_ACC_REG_DATA_X_LSB 0x02
#define BMI055_ACC_REG_DATA_X_MSB 0x03
#define BMI055_ACC_REG_DATA_Y_LSB 0x04
#define BMI055_ACC_REG_DATA_Y_MSB 0x05
#define BMI055_ACC_REG_DATA_Z_LSB 0x06
#define BMI055_ACC_REG_DATA_Z_MSB 0x07
#define BMI055_ACC_REG_PMU_RANGE 0x0F
#define BMI055_ACC_REG_PMU_BW 0x10
#define BMI055_ACC_REG_PMU_LPW 0x11
#define BMI055_ACC_REG_SOFTRESET 0x14

/* ══════════════════════════════════════════════════════
 *  Gyroscope Register Map (BMG160 die)
 * ══════════════════════════════════════════════════════ */

#define BMI055_GYRO_REG_CHIP_ID 0x00
#define BMI055_GYRO_REG_RATE_X_LSB 0x02
#define BMI055_GYRO_REG_RATE_X_MSB 0x03
#define BMI055_GYRO_REG_RATE_Y_LSB 0x04
#define BMI055_GYRO_REG_RATE_Y_MSB 0x05
#define BMI055_GYRO_REG_RATE_Z_LSB 0x06
#define BMI055_GYRO_REG_RATE_Z_MSB 0x07
#define BMI055_GYRO_REG_RANGE 0x0F
#define BMI055_GYRO_REG_BW 0x10
#define BMI055_GYRO_REG_LPM1 0x11
#define BMI055_GYRO_REG_SOFTRESET 0x14

/* ══════════════════════════════════════════════════════
 *  Register Values / Bit Masks
 * ══════════════════════════════════════════════════════ */

/* Chip ID expected values */
#define BMI055_ACC_CHIP_ID_VAL 0xFA
#define BMI055_GYRO_CHIP_ID_VAL 0x0F

/* Soft reset command (same for both) */
#define BMI055_SOFT_RESET_CMD 0xB6

/* Accelerometer PMU_RANGE */
#define BMI055_ACC_RANGE_2G 0x03
#define BMI055_ACC_RANGE_4G 0x05
#define BMI055_ACC_RANGE_8G 0x08
#define BMI055_ACC_RANGE_16G 0x0C

/* Accelerometer PMU_BW (Data filter bandwidth) */
#define BMI055_ACC_BW_7_81HZ 0x08
#define BMI055_ACC_BW_15_63HZ 0x09
#define BMI055_ACC_BW_31_25HZ 0x0A
#define BMI055_ACC_BW_62_5HZ 0x0B
#define BMI055_ACC_BW_125HZ 0x0C
#define BMI055_ACC_BW_250HZ 0x0D
#define BMI055_ACC_BW_500HZ 0x0E
#define BMI055_ACC_BW_1000HZ 0x0F

/* Accelerometer PMU_LPW (Power mode) */
#define BMI055_ACC_PM_NORMAL 0x00
#define BMI055_ACC_PM_DEEP_SUSPEND 0x20
#define BMI055_ACC_PM_SUSPEND 0x80

/* Gyroscope RANGE */
#define BMI055_GYRO_RANGE_2000DPS 0x00
#define BMI055_GYRO_RANGE_1000DPS 0x01
#define BMI055_GYRO_RANGE_500DPS 0x02
#define BMI055_GYRO_RANGE_250DPS 0x03
#define BMI055_GYRO_RANGE_125DPS 0x04

/* Gyroscope BW (ODR + filter bandwidth) */
#define BMI055_GYRO_BW_523HZ 0x00 /* 2000Hz ODR, 523Hz BW */
#define BMI055_GYRO_BW_230HZ 0x01 /* 2000Hz ODR, 230Hz BW */
#define BMI055_GYRO_BW_116HZ 0x02 /* 1000Hz ODR, 116Hz BW */
#define BMI055_GYRO_BW_47HZ 0x03  /*  400Hz ODR,  47Hz BW */
#define BMI055_GYRO_BW_32HZ 0x07  /*  100Hz ODR,  32Hz BW */

/* Gyroscope LPM1 (Power mode) */
#define BMI055_GYRO_PM_NORMAL 0x00
#define BMI055_GYRO_PM_DEEP_SUSPEND 0x20
#define BMI055_GYRO_PM_SUSPEND 0x80

/* ══════════════════════════════════════════════════════
 *  Scale factors
 * ══════════════════════════════════════════════════════ */

/*
 * Accel: raw -> m/s^2
 * ±16g, 12-bit signed → 128 LSB/g
 * Note: 12-bit data stored in upper 12 bits of 16-bit register
 *       >>4 after read yields 12-bit signed value
 */
#define BMI055_ACCEL_SCALE_16G (9.80665f / 128.0f)

/* Gyro: raw -> rad/s (+/-2000dps, 16-bit signed -> 16.4 LSB/dps) */
#define BMI055_GYRO_SCALE_2000DPS (0.01745329252f / 16.4f)

/* ══════════════════════════════════════════════════════
 *  Configuration structure
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    uint8_t accel_range; /* BMI055_ACC_RANGE_xxG */
    uint8_t accel_bw;    /* BMI055_ACC_BW_xxxHZ */
    uint8_t gyro_range;  /* BMI055_GYRO_RANGE_xxxxDPS */
    uint8_t gyro_bw;     /* BMI055_GYRO_BW_xxxHZ */
} bmi055_config_t;

/* Default config: ±16g / 1kHz BW, ±2000dps / 230Hz BW */
#define BMI055_DEFAULT_CONFIG                                                                                          \
    {                                                                                                                  \
        .accel_range = BMI055_ACC_RANGE_16G,                                                                           \
        .accel_bw = BMI055_ACC_BW_1000HZ,                                                                              \
        .gyro_range = BMI055_GYRO_RANGE_2000DPS,                                                                       \
        .gyro_bw = BMI055_GYRO_BW_230HZ,                                                                               \
    }

/* ══════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════ */

/**
 * @brief Initialize BMI055 (both accel and gyro dies)
 * @param dev    BMI055 device descriptor (accel + gyro SPI devices)
 * @param cfg    Configuration (NULL for defaults)
 * @return 0 on success, negative error code on failure
 *
 * Initialization sequence:
 * 1. Soft reset both dies
 * 2. Verify Chip ID for both (accel: 0xFA, gyro: 0x0F)
 * 3. Configure accelerometer (range, bandwidth, normal mode)
 * 4. Configure gyroscope (range, bandwidth, normal mode)
 */
int bmi055_init(bmi055_dev_t *dev, const bmi055_config_t *cfg);

/**
 * @brief Read accelerometer data
 * @param dev    BMI055 device descriptor
 * @param accel  Output: acceleration in m/s²
 * @return 0 on success, negative on error
 */
int bmi055_read_accel(bmi055_dev_t *dev, vec3f_t *accel);

/**
 * @brief Read gyroscope data
 * @param dev    BMI055 device descriptor
 * @param gyro   Output: angular rate in rad/s
 * @return 0 on success, negative on error
 */
int bmi055_read_gyro(bmi055_dev_t *dev, vec3f_t *gyro);

/**
 * @brief Read accelerometer and gyroscope
 * @param dev    BMI055 device descriptor
 * @param accel  Output: acceleration in m/s²
 * @param gyro   Output: angular rate in rad/s
 * @return 0 on success, negative on error
 *
 * Note: BMI055 accel/gyro are separate dies, so
 * single burst read like ICM-42688-P is not possible.
 * Internally requires 2 SPI transactions.
 */
int bmi055_read_accel_gyro(bmi055_dev_t *dev, vec3f_t *accel, vec3f_t *gyro);

#endif /* BMI055_H */
