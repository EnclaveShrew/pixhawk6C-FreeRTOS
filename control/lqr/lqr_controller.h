/**
 * @file lqr_controller.h
 * @brief LQR (Linear-Quadratic Regulator) controller
 *
 * Mathematical background:
 *   For linear system x_dot = Ax + Bu,
 *   find optimal gain K that minimizes cost J = integral(x'Qx + u'Ru)dt
 *   and control with u = -Kx.
 *
 *   Q: state error weight (how fast to reduce attitude error)
 *   R: control input weight (how much to conserve motor usage)
 *
 * Application in FC:
 *   State x = [attitude error(3), angular rate error(3)] = 6-dim
 *   Input u = [roll, pitch, yaw torque] = 3-dim
 *   K matrix (3x6) is computed offline via MATLAB/Python and entered as constants.
 */

#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include "controller_interface.h"

/* ══════════════════════════════════════════════════════
 *  Configuration
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    /*
     * K matrix (3x6): u = -K * x
     *
     * Rows: [roll, pitch, yaw]
     * Cols: [e_roll, e_pitch, e_yaw, e_p, e_q, e_r]
     *        ─── attitude error ──  ── rate error ──
     *
     * MATLAB: K = lqr(A, B, Q, R)
     * Python: K = scipy.linalg.solve_continuous_are(...)
     */
    float K[3][6];
    float output_limit;     /* output clamping (default 1.0) */
} lqr_config_t;

/*
 * Default K values: hover condition approximation.
 * Must be recomputed based on vehicle inertia for actual operation.
 */
#define LQR_DEFAULT_CONFIG { \
    .K = { \
        { 6.0f, 0.0f, 0.0f, 0.25f, 0.0f,  0.0f  }, \
        { 0.0f, 6.0f, 0.0f, 0.0f,  0.25f, 0.0f  }, \
        { 0.0f, 0.0f, 3.0f, 0.0f,  0.0f,  0.15f }, \
    }, \
    .output_limit = 1.0f, \
}

/* ══════════════════════════════════════════════════════
 *  Context (instance state)
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    float K[3][6];
    float output_limit;
} lqr_ctx_t;

/* ══════════════════════════════════════════════════════
 *  Public — controller_t instance
 * ══════════════════════════════════════════════════════ */

extern controller_t lqr_controller;

#endif /* LQR_CONTROLLER_H */
