/**
 * @file cascade_pid.h
 * @brief Cascade PID controller (attitude loop → rate loop)
 *
 * Structure:
 *   Outer loop (attitude): target attitude -> target angular rate
 *   Inner loop (rate): target angular rate -> control output
 *
 * Independent PID applied per axis (roll, pitch, yaw).
 * Includes anti-windup and derivative filtering.
 */

#ifndef CASCADE_PID_H
#define CASCADE_PID_H

#include "controller_interface.h"

/* ══════════════════════════════════════════════════════
 *  Single-axis PID state
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;         /* integral accumulator */
    float prev_error;       /* previous error (for derivative) */
    float integral_limit;   /* anti-windup upper limit */
    float d_filter_alpha;   /* derivative filter coeff (0~1, 1=no filter) */
    float d_filtered;       /* filtered derivative value */
    float output_limit;     /* output upper limit */
} pid_axis_t;

/* ══════════════════════════════════════════════════════
 *  Cascade PID configuration
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    /* Outer loop (attitude -> target rate) — P only */
    float att_kp_roll;
    float att_kp_pitch;
    float att_kp_yaw;

    /* Inner loop (rate -> control output) — PID */
    float rate_kp_roll;
    float rate_ki_roll;
    float rate_kd_roll;

    float rate_kp_pitch;
    float rate_ki_pitch;
    float rate_kd_pitch;

    float rate_kp_yaw;
    float rate_ki_yaw;
    float rate_kd_yaw;

    /* Common settings */
    float integral_limit;   /* anti-windup limit (default 0.3) */
    float d_filter_alpha;   /* derivative LPF coeff (default 0.1, smaller = stronger filter) */
    float output_limit;     /* output clamping (default 1.0) */
} cascade_pid_config_t;

#define CASCADE_PID_DEFAULT_CONFIG { \
    .att_kp_roll  = 6.0f, \
    .att_kp_pitch = 6.0f, \
    .att_kp_yaw   = 3.0f, \
    \
    .rate_kp_roll  = 0.15f, \
    .rate_ki_roll  = 0.05f, \
    .rate_kd_roll  = 0.003f, \
    \
    .rate_kp_pitch = 0.15f, \
    .rate_ki_pitch = 0.05f, \
    .rate_kd_pitch = 0.003f, \
    \
    .rate_kp_yaw   = 0.20f, \
    .rate_ki_yaw   = 0.02f, \
    .rate_kd_yaw   = 0.0f, \
    \
    .integral_limit = 0.3f, \
    .d_filter_alpha = 0.1f, \
    .output_limit   = 1.0f, \
}

/* ══════════════════════════════════════════════════════
 *  Instance context (allows multiple PID instances with different gains)
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    /* Attitude loop P gains */
    float att_kp[3];    /* [roll, pitch, yaw] */

    /* Rate loop PID */
    pid_axis_t rate[3]; /* [roll, pitch, yaw] */
} cascade_pid_ctx_t;

/* ══════════════════════════════════════════════════════
 *  Public — controller_t instance
 * ══════════════════════════════════════════════════════ */

/**
 * Default instance. For A/B testing, declare your own cascade_pid_ctx_t
 * and create a second controller_t with the same function pointers.
 */
extern controller_t cascade_pid_controller;

#endif /* CASCADE_PID_H */
