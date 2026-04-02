/**
 * @file controller_interface.h
 * @brief Common interface for all flight controllers
 *
 * Abstracts PID, LQR, INDI controllers behind a common interface.
 * Function pointer table allows runtime controller switching.
 *
 * Usage:
 *   controller_t *ctrl = &cascade_pid_controller;  // or lqr, indi
 *   ctrl->init(ctrl->ctx, cfg);
 *   ctrl->update(ctrl->ctx, &input, &output);
 *   ctrl->set_param(ctrl->ctx, "kp_roll", 5.0f);
 *   ctrl->reset(ctrl->ctx);
 */

#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

#include "sensor_types.h"

/* ══════════════════════════════════════════════════════
 *  Controller input (AHRS → Controller)
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    quat_t target_attitude;     /* target attitude (quaternion) */
    quat_t current_attitude;    /* current attitude (quaternion, AHRS output) */
    vec3f_t gyro;               /* current angular rate (rad/s, sensor output) */
    float thrust;               /* target throttle (0.0~1.0) */
    float dt;                   /* time step (seconds) */
} controller_input_t;

/* ══════════════════════════════════════════════════════
 *  Controller output (Controller → Mixer)
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    float roll;     /* roll control output (-1.0 ~ +1.0) */
    float pitch;    /* pitch control output (-1.0 ~ +1.0) */
    float yaw;      /* yaw control output (-1.0 ~ +1.0) */
    float thrust;   /* thrust output (0.0 ~ 1.0) */
} controller_output_t;

/* ══════════════════════════════════════════════════════
 *  Controller type enum
 * ══════════════════════════════════════════════════════ */

typedef enum
{
    CTRL_TYPE_PID,
    CTRL_TYPE_LQR,
    CTRL_TYPE_INDI,
} controller_type_t;

/* ══════════════════════════════════════════════════════
 *  Controller interface (function pointer table)
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    const char *name;           /* controller name (for logging/debug) */
    controller_type_t type;     /* controller type */
    void *ctx;                  /* instance context (state), enables multi-instance */

    /**
     * @brief Initialize controller
     * @param ctx     Instance context
     * @param config  Controller-specific config (each controller casts to its own type)
     * @return 0 on success, negative on error
     */
    int (*init)(void *ctx, void *config);

    /**
     * @brief Run one control loop iteration
     * @param ctx  Instance context
     * @param in   Controller input (attitude, angular rate, targets, etc.)
     * @param out  Controller output (roll/pitch/yaw/thrust)
     * @return 0 on success, negative on error
     */
    int (*update)(void *ctx, const controller_input_t *in, controller_output_t *out);

    /**
     * @brief Set a parameter by name
     * @param ctx    Instance context
     * @param key    Parameter name (e.g. "kp_roll", "ki_pitch")
     * @param value  Parameter value
     * @return 0 on success, -1 if key not found
     */
    int (*set_param)(void *ctx, const char *key, float value);

    /**
     * @brief Reset controller state (clear integrators, etc.)
     * @param ctx  Instance context
     * @return 0 on success
     */
    int (*reset)(void *ctx);
} controller_t;

#endif /* CONTROLLER_INTERFACE_H */
