/**
 * @file ms5611.h
 * @brief MS5611 barometric pressure sensor driver (TE Connectivity)
 *
 * Interface: I2C (address 0x77 default)
 * Output: temperature (°C), pressure (Pa)
 * 2nd order temperature compensation included
 */

#ifndef MS5611_H
#define MS5611_H

#include "i2c_wrapper.h"
#include "sensor_types.h"

/* ══════════════════════════════════════════════════════
 *  I2C Address
 * ══════════════════════════════════════════════════════ */

#define MS5611_ADDR_CSB_LOW 0x77  /* CSB → GND (default) */
#define MS5611_ADDR_CSB_HIGH 0x76 /* CSB → VCC */

/* ══════════════════════════════════════════════════════
 *  Commands
 * ══════════════════════════════════════════════════════ */

#define MS5611_CMD_RESET 0x1E

/* ADC read */
#define MS5611_CMD_ADC_READ 0x00

/* Pressure conversion (D1) — OSR select */
#define MS5611_CMD_CONV_D1_256 0x40
#define MS5611_CMD_CONV_D1_512 0x42
#define MS5611_CMD_CONV_D1_1024 0x44
#define MS5611_CMD_CONV_D1_2048 0x46
#define MS5611_CMD_CONV_D1_4096 0x48

/* Temperature conversion (D2) — OSR select */
#define MS5611_CMD_CONV_D2_256 0x50
#define MS5611_CMD_CONV_D2_512 0x52
#define MS5611_CMD_CONV_D2_1024 0x54
#define MS5611_CMD_CONV_D2_2048 0x56
#define MS5611_CMD_CONV_D2_4096 0x58

/* PROM read (calibration coefficients C1~C6) */
#define MS5611_CMD_PROM_C1 0xA2
#define MS5611_CMD_PROM_C2 0xA4
#define MS5611_CMD_PROM_C3 0xA6
#define MS5611_CMD_PROM_C4 0xA8
#define MS5611_CMD_PROM_C5 0xAA
#define MS5611_CMD_PROM_C6 0xAC

/* ══════════════════════════════════════════════════════
 *  OSR (Over Sampling Ratio)
 * ══════════════════════════════════════════════════════ */

/*
 * Higher OSR: better resolution, longer conversion time
 * | OSR  | Resolution(mbar) | Conv. time |
 * |------|-------------|----------|
 * |  256 | 0.065       | 1.0 ms   |
 * |  512 | 0.042       | 2.0 ms   |
 * | 1024 | 0.027       | 3.0 ms   |
 * | 2048 | 0.018       | 5.0 ms   |
 * | 4096 | 0.012       | 9.0 ms   |
 */

typedef enum
{
    MS5611_OSR_256 = 0,
    MS5611_OSR_512 = 1,
    MS5611_OSR_1024 = 2,
    MS5611_OSR_2048 = 3,
    MS5611_OSR_4096 = 4,
} ms5611_osr_t;

/* ══════════════════════════════════════════════════════
 *  Device structure
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    i2c_dev_t *i2c_dev;

    /* PROM calibration coefficients (Factory calibration values) */
    uint16_t c1; /* Pressure sensitivity (SENST1) */
    uint16_t c2; /* Pressure offset (OFFT1) */
    uint16_t c3; /* Temp coeff of pressure sensitivity (TCS) */
    uint16_t c4; /* Temp coeff of pressure offset (TCO) */
    uint16_t c5; /* Reference temperature (TREF) */
    uint16_t c6; /* Temp coeff of temperature (TEMPSENS) */

    /* OSR Configuration */
    ms5611_osr_t osr;
} ms5611_dev_t;

/* ══════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════ */

/**
 * @brief Initialize MS5611 barometer
 * @param dev  MS5611 device descriptor
 * @param osr  Over Sampling Ratio (resolution/speed tradeoff)
 * @return 0 on success, negative error code on failure
 *
 * Initialization sequence:
 * 1. Soft reset
 * 2. Read PROM calibration coefficients (C1~C6)
 */
int ms5611_init(ms5611_dev_t *dev, ms5611_osr_t osr);

/**
 * @brief Read temperature and pressure with 2nd order compensation
 * @param dev       MS5611 device descriptor
 * @param temp_c    Output: temperature in °C (nullable)
 * @param press_pa  Output: pressure in Pa (nullable)
 * @return 0 on success, negative on error
 *
 * Internally converts D2 (temperature) then D1 (pressure) + ADC read.
 * 2nd order temperature compensation applied.
 * Takes up to ~20ms depending on OSR.
 */
int ms5611_read(ms5611_dev_t *dev, float *temp_c, float *press_pa);

#endif /* MS5611_H */
