/**
 * @file ekf.h
 * @brief Extended Kalman Filter for attitude estimation
 *
 * State vector (7):
 *   [q0, q1, q2, q3] — attitude quaternion
 *   [bx, by, bz]     — gyroscope bias (rad/s)
 *
 * Prediction: gyroscope (angular rate)
 * Measurement update: accelerometer (gravity) + magnetometer (heading)
 */

#ifndef EKF_H
#define EKF_H

#include "sensor_types.h"

/* ══════════════════════════════════════════════════════
 *  Configuration
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    float gyro_noise;       /* gyro process noise (rad/s) */
    float gyro_bias_noise;  /* bias drift noise (rad/s^2) */
    float accel_noise;      /* accelerometer measurement noise (m/s^2) */
    float mag_noise;        /* magnetometer measurement noise (uT) */
} ekf_config_t;

#define EKF_DEFAULT_CONFIG { \
    .gyro_noise      = 0.01f, \
    .gyro_bias_noise = 0.001f, \
    .accel_noise     = 0.5f, \
    .mag_noise       = 1.0f, \
}

/* ══════════════════════════════════════════════════════
 *  EKF state
 * ══════════════════════════════════════════════════════ */

#define EKF_STATE_DIM   7   /* quaternion(4) + gyro bias(3) */
#define EKF_ACCEL_DIM   3   /* accelerometer measurement */
#define EKF_MAG_DIM     3   /* magnetometer measurement */

typedef struct
{
    /* State vector: [q0, q1, q2, q3, bx, by, bz] */
    float x[EKF_STATE_DIM];

    /* Covariance matrix (7x7) — estimation uncertainty */
    float P[EKF_STATE_DIM][EKF_STATE_DIM];

    /* Noise parameters */
    float gyro_noise;
    float gyro_bias_noise;
    float accel_noise;
    float mag_noise;

    /* Euler angles (convenience, updated after each step) */
    float roll;
    float pitch;
    float yaw;

    uint8_t initialized;
} ekf_state_t;

/* ══════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════ */

/**
 * @brief Initialize EKF
 * @param state  EKF state
 * @param cfg    Configuration (NULL for defaults)
 */
void ekf_init(ekf_state_t *state, const ekf_config_t *cfg);

/**
 * @brief Prediction step — gyroscope
 * @param state  EKF state
 * @param gyro   Gyroscope reading (rad/s)
 * @param dt     Time step (seconds)
 *
 * Predict state (quaternion) from gyro angular rate + propagate covariance
 */
void ekf_predict(ekf_state_t *state, const vec3f_t *gyro, float dt);

/**
 * @brief Measurement update — accelerometer
 * @param state  EKF state
 * @param accel  Accelerometer reading (m/s²)
 *
 * Correct roll/pitch using gravity direction
 */
void ekf_update_accel(ekf_state_t *state, const vec3f_t *accel);

/**
 * @brief Measurement update — magnetometer
 * @param state  EKF state
 * @param mag    Magnetometer reading (uT)
 *
 * Correct yaw using magnetic field direction
 */
void ekf_update_mag(ekf_state_t *state, const vec3f_t *mag);

/**
 * @brief Get current attitude as euler angles
 * @param state   EKF state
 * @param roll    Output: roll in rad (nullable)
 * @param pitch   Output: pitch in rad (nullable)
 * @param yaw     Output: yaw in rad (nullable)
 */
void ekf_get_euler(const ekf_state_t *state, float *roll, float *pitch, float *yaw);

/**
 * @brief Get current attitude as quaternion
 * @param state  EKF state
 * @param q      Output: quaternion
 */
void ekf_get_quat(const ekf_state_t *state, quat_t *q);

/**
 * @brief Get estimated gyroscope bias
 * @param state  EKF state
 * @param bias   Output: gyro bias in rad/s
 */
void ekf_get_gyro_bias(const ekf_state_t *state, vec3f_t *bias);

#endif /* EKF_H */
