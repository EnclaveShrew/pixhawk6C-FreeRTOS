/**
 * @file indi_controller.h
 * @brief INDI (Incremental Nonlinear Dynamic Inversion) controller
 *
 * 수학적 배경:
 *   일반 NDI: ω_dot = f(ω) + G·u → u = G^(-1)·(ω_dot_des - f(ω))
 *   → 정확한 모델 f(ω)가 필요 (모델링 오차에 취약)
 *
 *   INDI: 증분(incremental) 접근
 *   Δu = G^(-1) · (ω_dot_des - ω_dot_measured)
 *   u = u_prev + Δu
 *
 *   → f(ω) 모델이 불필요. 이전 입력과 현재 각가속도 측정값만 사용.
 *   → 모델링 오차, 외란에 강인.
 *   → 단, 각가속도(ω_dot)를 정확히 측정/추정해야 함.
 *
 * G 행렬: 제어 효과 행렬 (control effectiveness)
 *   각 축 토크가 각가속도에 미치는 영향.
 *   기체 관성 모멘트의 역수에 비례.
 */

#ifndef INDI_CONTROLLER_H
#define INDI_CONTROLLER_H

#include "controller_interface.h"

/* ══════════════════════════════════════════════════════
 *  Configuration
 * ══════════════════════════════════════════════════════ */

typedef struct
{
    /* 자세 루프 P 게인 (자세 오차 → 목표 각속도) */
    float att_kp_roll;
    float att_kp_pitch;
    float att_kp_yaw;

    /*
     * G_inv: 제어 효과 역행렬 (3x3, 대각 근사)
     * G_inv[i] = 1 / G[i][i]
     * G[i][i] ≈ 1 / J[i] (관성 모멘트의 역수)
     *
     * 실제 값은 기체 파라미터에서 결정.
     */
    float g_inv_roll;
    float g_inv_pitch;
    float g_inv_yaw;

    /* 각가속도 필터 계수 (LPF) */
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
 *  Public — controller_t 인스턴스
 * ══════════════════════════════════════════════════════ */

extern controller_t indi_controller;

#endif /* INDI_CONTROLLER_H */
