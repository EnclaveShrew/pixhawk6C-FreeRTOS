/**
 * @file sensor_types.h
 * @brief Common types shared across all sensor drivers
 */

#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>

/* ── 3D vector ──────────────────────────────────────── */
typedef struct
{
    float x, y, z;
} vec3f_t;

/* ── Quaternion ─────────────────────────────────────── */
typedef struct
{
    float w, x, y, z;
} quat_t;

/* ── Error codes (0 = success, negative = error) ───── */
typedef enum
{
    SENSOR_OK = 0,
    SENSOR_ERR_COMM = -1,    /* SPI/I2C communication failure */
    SENSOR_ERR_ID = -2,      /* WHO_AM_I mismatch */
    SENSOR_ERR_TIMEOUT = -3, /* response timeout */
    SENSOR_ERR_CONFIG = -4,  /* configuration verification failure */
    SENSOR_ERR_PARAM = -5,   /* invalid parameter */
} sensor_error_t;

#endif /* SENSOR_TYPES_H */
