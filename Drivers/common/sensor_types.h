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
    SENSOR_ERR_COMM = -1,    /* SPI/I2C 통신 실패 */
    SENSOR_ERR_ID = -2,      /* WHO_AM_I 불일치 */
    SENSOR_ERR_TIMEOUT = -3, /* 응답 타임아웃 */
    SENSOR_ERR_CONFIG = -4,  /* 설정 검증 실패 */
    SENSOR_ERR_PARAM = -5,   /* 잘못된 파라미터 */
} sensor_error_t;

#endif /* SENSOR_TYPES_H */
