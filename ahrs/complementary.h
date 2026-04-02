/**
 * @file complementary.h
 * @brief Complementary filter for attitude estimation
 *
 * Fuses gyroscope (short-term accurate) and accelerometer (long-term stable)
 * to estimate roll and pitch. Magnetometer fuses for yaw.
 *
 * Attitude represented as quaternion to avoid gimbal lock.
 */

#ifndef COMPLEMENTARY_H
#define COMPLEMENTARY_H

#include "sensor_types.h"

/* ══════════════════════════════════════════════════════
 *  Configuration
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    float alpha;        /* gyro trust factor (0.0~1.0, default 0.98) */
} complementary_config_t;

#define COMPLEMENTARY_DEFAULT_CONFIG { \
    .alpha = 0.98f, \
}

/* ══════════════════════════════════════════════════════
 *  Filter state
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    quat_t q;           /* current attitude (quaternion) */
    float roll;         /* current roll (rad) — convenience */
    float pitch;        /* current pitch (rad) */
    float yaw;          /* current yaw (rad) */
    float alpha;        /* gyro trust factor */
    uint8_t initialized;
} complementary_state_t;

/* ══════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════ */

/**
 * @brief Initialize complementary filter
 * @param state  Filter state
 * @param cfg    Configuration (NULL for defaults)
 */
void complementary_init(complementary_state_t *state, const complementary_config_t *cfg);

/**
 * @brief Update attitude estimate
 * @param state  Filter state
 * @param accel  Accelerometer reading (m/s²)
 * @param gyro   Gyroscope reading (rad/s)
 * @param mag    Magnetometer reading (uT, NULL if unavailable)
 * @param dt     Time step (seconds)
 *
 * Call rate: 1kHz recommended, matching sensor ODR
 */
void complementary_update(complementary_state_t *state,
                          const vec3f_t *accel,
                          const vec3f_t *gyro,
                          const vec3f_t *mag,
                          float dt);

/**
 * @brief Get current attitude as euler angles
 * @param state   Filter state
 * @param roll    Output: roll in rad (nullable)
 * @param pitch   Output: pitch in rad (nullable)
 * @param yaw     Output: yaw in rad (nullable)
 */
void complementary_get_euler(const complementary_state_t *state,
                             float *roll, float *pitch, float *yaw);

/**
 * @brief Get current attitude as quaternion
 * @param state  Filter state
 * @param q      Output: quaternion
 */
void complementary_get_quat(const complementary_state_t *state, quat_t *q);

#endif /* COMPLEMENTARY_H */
