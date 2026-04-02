/**
 * @file motor_mixer.h
 * @brief Motor mixing matrix for quadcopter
 *
 * Converts controller output (roll, pitch, yaw, thrust) to individual motor outputs.
 *
 * Quadcopter X layout (front view):
 *
 *     Front
 *      1     2       Motor 1: front-left (CCW)
 *       \   /        Motor 2: front-right (CW)
 *        [X]         Motor 3: rear-left (CW)
 *       /   \        Motor 4: rear-right (CCW)
 *      3     4
 *
 * Mixing formula:
 *   motor[i] = thrust + roll_mix[i]*roll + pitch_mix[i]*pitch + yaw_mix[i]*yaw
 */

#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include "controller_interface.h"
#include <stdint.h>

/* Maximum number of motors */
#define MIXER_MAX_MOTORS    8

/* ══════════════════════════════════════════════════════
 *  Mixer configuration
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    uint8_t num_motors;

    /*
     * Mixing coefficients [motor index][axis]
     * Roll, pitch, yaw contribution for each motor.
     * Values: -1.0 ~ +1.0
     */
    float mix[MIXER_MAX_MOTORS][3];  /* [motor][roll, pitch, yaw] */

    float output_min;   /* motor min output (default 0.0) */
    float output_max;   /* motor max output (default 1.0) */
    float idle_throttle; /* idle throttle (default 0.05) */
} mixer_config_t;

/*
 * Quad-X default configuration
 * Motor 1 (front-left CCW): +roll, +pitch, -yaw
 * Motor 2 (front-right CW):  -roll, +pitch, +yaw
 * Motor 3 (rear-left CW):  +roll, -pitch, +yaw
 * Motor 4 (rear-right CCW): -roll, -pitch, -yaw
 */
#define MIXER_QUAD_X_CONFIG { \
    .num_motors = 4, \
    .mix = { \
        { +1.0f, +1.0f, -1.0f }, \
        { -1.0f, +1.0f, +1.0f }, \
        { +1.0f, -1.0f, +1.0f }, \
        { -1.0f, -1.0f, -1.0f }, \
    }, \
    .output_min = 0.0f, \
    .output_max = 1.0f, \
    .idle_throttle = 0.05f, \
}

/* ══════════════════════════════════════════════════════
 *  Mixer output
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    float motor[MIXER_MAX_MOTORS];  /* individual motor output (0.0~1.0) */
    uint8_t num_motors;
} mixer_output_t;

/* ══════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════ */

/**
 * @brief Initialize motor mixer
 * @param cfg  Mixer configuration (NULL for quad-X default)
 * @return 0 on success
 */
int mixer_init(const mixer_config_t *cfg);

/**
 * @brief Mix controller output to individual motor outputs
 * @param ctrl_out  Controller output (roll, pitch, yaw, thrust)
 * @param mix_out   Motor outputs
 * @return 0 on success
 *
 * motor[i] = thrust + mix[i][0]*roll + mix[i][1]*pitch + mix[i][2]*yaw
 * Output is clamped to [idle_throttle, output_max].
 * If thrust is 0, all motors are 0 (disarmed).
 */
int mixer_mix(const controller_output_t *ctrl_out, mixer_output_t *mix_out);

#endif /* MOTOR_MIXER_H */
