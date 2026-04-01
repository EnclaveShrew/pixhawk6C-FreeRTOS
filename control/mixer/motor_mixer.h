/**
 * @file motor_mixer.h
 * @brief Motor mixing matrix for quadcopter
 *
 * 제어기 출력(roll, pitch, yaw, thrust)을 개별 모터 출력으로 변환.
 *
 * 쿼드콥터 X 배치 (앞에서 볼 때):
 *
 *     전방 (Front)
 *      1     2       Motor 1: 좌전 (CCW)
 *       \   /        Motor 2: 우전 (CW)
 *        [X]         Motor 3: 좌후 (CW)
 *       /   \        Motor 4: 우후 (CCW)
 *      3     4
 *
 * 믹싱 공식:
 *   motor[i] = thrust + roll_mix[i]*roll + pitch_mix[i]*pitch + yaw_mix[i]*yaw
 */

#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include "controller_interface.h"
#include <stdint.h>

/* 최대 모터 수 */
#define MIXER_MAX_MOTORS    8

/* ══════════════════════════════════════════════════════
 *  Mixer configuration
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    uint8_t num_motors;

    /*
     * 믹싱 계수 [모터 인덱스][축]
     * 각 모터에 대해 roll, pitch, yaw 기여도.
     * 값: -1.0 ~ +1.0
     */
    float mix[MIXER_MAX_MOTORS][3];  /* [motor][roll, pitch, yaw] */

    float output_min;   /* 모터 최소 출력 (기본 0.0) */
    float output_max;   /* 모터 최대 출력 (기본 1.0) */
    float idle_throttle; /* 아이들 스로틀 (기본 0.05) */
} mixer_config_t;

/*
 * 쿼드콥터 X 배치 기본 설정
 * Motor 1 (좌전 CCW): +roll, +pitch, -yaw
 * Motor 2 (우전 CW):  -roll, +pitch, +yaw
 * Motor 3 (좌후 CW):  +roll, -pitch, +yaw
 * Motor 4 (우후 CCW): -roll, -pitch, -yaw
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
    float motor[MIXER_MAX_MOTORS];  /* 개별 모터 출력 (0.0~1.0) */
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
 * 출력은 [idle_throttle, output_max] 범위로 클램핑.
 * thrust가 0이면 모든 모터 0 (시동 꺼짐).
 */
int mixer_mix(const controller_output_t *ctrl_out, mixer_output_t *mix_out);

#endif /* MOTOR_MIXER_H */
