/**
 * @file controller_interface.h
 * @brief Common interface for all flight controllers
 *
 * PID, LQR, INDI 등 제어기를 공통 인터페이스로 추상화.
 * 함수 포인터 테이블로 런타임에 제어기를 교체할 수 있음.
 *
 * 사용 예:
 *   controller_t *ctrl = &cascade_pid_controller;  // 또는 lqr, indi
 *   ctrl->init(cfg);
 *   ctrl->update(&input, &output);
 *   ctrl->set_param("kp_roll", 5.0f);
 *   ctrl->reset();
 */

#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

#include "sensor_types.h"

/* ══════════════════════════════════════════════════════
 *  Controller input (AHRS → Controller)
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    quat_t target_attitude;     /* 목표 자세 (쿼터니언) */
    quat_t current_attitude;    /* 현재 자세 (쿼터니언, AHRS 출력) */
    vec3f_t gyro;               /* 현재 각속도 (rad/s, 센서 출력) */
    float thrust;               /* 목표 스로틀 (0.0~1.0) */
    float dt;                   /* 시간 간격 (초) */
} controller_input_t;

/* ══════════════════════════════════════════════════════
 *  Controller output (Controller → Mixer)
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    float roll;     /* roll 제어 출력 (-1.0 ~ +1.0) */
    float pitch;    /* pitch 제어 출력 (-1.0 ~ +1.0) */
    float yaw;      /* yaw 제어 출력 (-1.0 ~ +1.0) */
    float thrust;   /* thrust 출력 (0.0 ~ 1.0) */
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
 *  Controller interface (함수 포인터 테이블)
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    const char *name;           /* 제어기 이름 (로깅/디버그용) */
    controller_type_t type;     /* 제어기 타입 */

    /**
     * @brief Initialize controller
     * @param config  Controller-specific config (각 제어기가 캐스팅해서 사용)
     * @return 0 on success, negative on error
     */
    int (*init)(void *config);

    /**
     * @brief Run one control loop iteration
     * @param in   Controller input (자세, 각속도, 목표 등)
     * @param out  Controller output (roll/pitch/yaw/thrust)
     * @return 0 on success, negative on error
     */
    int (*update)(const controller_input_t *in, controller_output_t *out);

    /**
     * @brief Set a parameter by name
     * @param key    Parameter name (e.g. "kp_roll", "ki_pitch")
     * @param value  Parameter value
     * @return 0 on success, -1 if key not found
     */
    int (*set_param)(const char *key, float value);

    /**
     * @brief Reset controller state (적분기 초기화 등)
     * @return 0 on success
     */
    int (*reset)(void);
} controller_t;

#endif /* CONTROLLER_INTERFACE_H */
