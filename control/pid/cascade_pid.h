/**
 * @file cascade_pid.h
 * @brief Cascade PID controller (attitude loop → rate loop)
 *
 * 구조:
 *   외부 루프 (자세): 목표 자세 → 목표 각속도
 *   내부 루프 (각속도): 목표 각속도 → 제어 출력
 *
 * 각 축(roll, pitch, yaw)에 대해 독립적으로 PID 적용.
 * Anti-windup, derivative filtering 포함.
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
    float integral;         /* 적분 누적값 */
    float prev_error;       /* 이전 오차 (미분용) */
    float integral_limit;   /* Anti-windup 상한 */
    float d_filter_alpha;   /* 미분 필터 계수 (0~1, 1=필터 없음) */
    float d_filtered;       /* 필터링된 미분값 */
    float output_limit;     /* 출력 상한 */
} pid_axis_t;

/* ══════════════════════════════════════════════════════
 *  Cascade PID configuration
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    /* 외부 루프 (자세 → 목표 각속도) — P만 사용 */
    float att_kp_roll;
    float att_kp_pitch;
    float att_kp_yaw;

    /* 내부 루프 (각속도 → 제어 출력) — PID */
    float rate_kp_roll;
    float rate_ki_roll;
    float rate_kd_roll;

    float rate_kp_pitch;
    float rate_ki_pitch;
    float rate_kd_pitch;

    float rate_kp_yaw;
    float rate_ki_yaw;
    float rate_kd_yaw;

    /* 공통 설정 */
    float integral_limit;   /* Anti-windup 상한 (기본 0.3) */
    float d_filter_alpha;   /* 미분 LPF 계수 (기본 0.1, 작을수록 강한 필터) */
    float output_limit;     /* 출력 클램핑 (기본 1.0) */
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
 *  Public — controller_t 인스턴스
 * ══════════════════════════════════════════════════════ */

/**
 * controller_interface.h의 함수 포인터 테이블을 구현한 전역 인스턴스.
 *
 * 사용법:
 *   controller_t *ctrl = &cascade_pid_controller;
 *   ctrl->init(&my_config);
 *   ctrl->update(&input, &output);
 */
extern controller_t cascade_pid_controller;

#endif /* CASCADE_PID_H */
