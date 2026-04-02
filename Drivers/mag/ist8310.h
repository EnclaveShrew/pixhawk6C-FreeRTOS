/**
 * @file ist8310.h
 * @brief IST8310 3-axis magnetometer driver (iSentek)
 *
 * Interface: I2C (address 0x0E)
 * Output: magnetic field in uT (micro-Tesla)
 * Mode: Single measurement (trigger per read)
 */

#ifndef IST8310_H
#define IST8310_H

#include "i2c_wrapper.h"
#include "sensor_types.h"

/* ══════════════════════════════════════════════════════
 *  I2C Address
 * ══════════════════════════════════════════════════════ */

#define IST8310_ADDR 0x0E

/* ══════════════════════════════════════════════════════
 *  Register Map
 * ══════════════════════════════════════════════════════ */

#define IST8310_REG_WHO_AM_I 0x00 /* Device ID */

/* Data output registers (16-bit, little-endian) */
#define IST8310_REG_DATA_X_LSB 0x03
#define IST8310_REG_DATA_X_MSB 0x04
#define IST8310_REG_DATA_Y_LSB 0x05
#define IST8310_REG_DATA_Y_MSB 0x06
#define IST8310_REG_DATA_Z_LSB 0x07
#define IST8310_REG_DATA_Z_MSB 0x08

/* Status */
#define IST8310_REG_STAT1 0x02 /* Contains DRDY bit */

/* Control */
#define IST8310_REG_CNTL1 0x0A   /* Operating mode control */
#define IST8310_REG_CNTL2 0x0B   /* Data output config */
#define IST8310_REG_AVGCNTL 0x41 /* Averaging count setting */
#define IST8310_REG_PDCNTL 0x42  /* Set/Reset pulse duration */

/* ══════════════════════════════════════════════════════
 *  Register Values / Bit Masks
 * ══════════════════════════════════════════════════════ */

/* WHO_AM_I expected value */
#define IST8310_WHO_AM_I_VAL 0x10

/* CNTL1: Operating mode */
#define IST8310_MODE_STANDBY 0x00
#define IST8310_MODE_SINGLE 0x01 /* Auto standby after single measurement */

/* STAT1 bits */
#define IST8310_STAT_DRDY 0x01 /* Data Ready */

/* AVGCNTL: Averaging count (noise reduction) */
#define IST8310_AVG_16 0x24 /* 16x averaging, ODR ~166Hz */

/* PDCNTL: Set/Reset pulse (low-noise mode) */
#define IST8310_PDCNTL_NORMAL 0xC0

/* ══════════════════════════════════════════════════════
 *  Scale factor
 * ══════════════════════════════════════════════════════ */

/* raw → uT: 330 LSB/Gauss, 1 Gauss = 100 uT → 3.3 LSB/uT */
#define IST8310_SCALE_UT (1.0f / 3.3f)

/* ══════════════════════════════════════════════════════
 *  DRDY max wait time
 * ══════════════════════════════════════════════════════ */

#define IST8310_MEAS_TIMEOUT_MS 10

/* ══════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════ */

/**
 * @brief Initialize IST8310 magnetometer
 * @param dev  I2C device descriptor
 * @return 0 on success, negative error code on failure
 *
 * Initialization sequence:
 * 1. Verify WHO_AM_I (expect 0x10)
 * 2. Set averaging (16x for low noise)
 * 3. Set pulse duration
 */
int ist8310_init(i2c_dev_t *dev);

/**
 * @brief Read magnetic field
 * @param dev  I2C device descriptor
 * @param mag  Output: magnetic field in uT (micro-Tesla)
 * @return 0 on success, negative on error
 *
 * Single measurement mode: trigger -> DRDY wait -> data read
 */
int ist8310_read(i2c_dev_t *dev, vec3f_t *mag);

#endif /* IST8310_H */
