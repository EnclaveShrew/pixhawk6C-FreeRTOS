/**
 * @file indi_controller.h
 * @brief INDI (Incremental Nonlinear Dynamic Inversion) controller
 *
 * Mathematical background:
 *   Standard NDI: w_dot = f(w) + G*u -> u = G^(-1)*(w_dot_des - f(w))
 *   -> Requires accurate model f(w) (vulnerable to modeling errors)
 *
 *   INDI: incremental approach
 *   du = G^(-1) * (w_dot_des - w_dot_measured)
 *   u = u_prev + du
 *
 *   -> No f(w) model needed. Only uses previous input and current angular acceleration.
 *   -> Robust to modeling errors and disturbances.
 *   -> However, angular acceleration (w_dot) must be accurately measured/estimated.
 *
 * G matrix: control effectiveness matrix
 *   Effect of each axis torque on angular acceleration.
 *   Proportional to inverse of vehicle moment of inertia.
 */

#ifndef INDI_CONTROLLER_H
#define INDI_CONTROLLER_H

#include "controller_interface.h"

/* ══════════════════════════════════════════════════════
 *  Configuration
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    /* Attitude loop P gain (attitude error -> target angular rate) */
    float att_kp_roll;
    float att_kp_pitch;
    float att_kp_yaw;

    /*
     * G_inv: control effectiveness inverse (3x3, diagonal approx)
     * G_inv[i] = 1 / G[i][i]
     * G[i][i] ≈ 1 / J[i] (inverse of moment of inertia)
     *
     * Actual values determined by vehicle parameters.
     */
    float g_inv_roll;
    float g_inv_pitch;
    float g_inv_yaw;

    /* Angular acceleration filter coefficient (LPF) */
    float gyro_dot_filter_alpha;

    float output_limit;
} indi_config_t;

#define INDI_DEFAULT_CONFIG { \
    .att_kp_roll  = 6.0f, \
    .att_kp_pitch = 6.0f, \
    .att_kp_yaw   = 3.0f, \
    \
    .g_inv_roll  = 1.0f, \
    .g_inv_pitch = 1.0f, \
    .g_inv_yaw   = 1.0f, \
    \
    .gyro_dot_filter_alpha = 0.05f, \
    .output_limit = 1.0f, \
}

/* ══════════════════════════════════════════════════════
 *  Context (instance state)
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    /* Attitude loop P gains */
    float att_kp[3];

    /* Control effectiveness inverse matrix (diagonal) */
    float g_inv[3];

    /* Previous gyro value (for angular acceleration computation) */
    vec3f_t gyro_prev;

    /* Filtered angular acceleration */
    vec3f_t gyro_dot_filtered;

    /* Previous control output */
    float u_prev[3];

    /* Configuration */
    float gyro_dot_filter_alpha;
    float output_limit;

    uint8_t initialized;
} indi_ctx_t;

/* ══════════════════════════════════════════════════════
 *  Public — controller_t instance
 * ══════════════════════════════════════════════════════ */

extern controller_t indi_controller;

#endif /* INDI_CONTROLLER_H */
