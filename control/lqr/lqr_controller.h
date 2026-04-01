/**
 * @file lqr_controller.h
 * @brief LQR (Linear-Quadratic Regulator) controller
 *
 * 수학적 배경:
 *   선형 시스템 x_dot = Ax + Bu 에서,
 *   비용함수 J = ∫(x'Qx + u'Ru)dt 를 최소화하는 최적 게인 K를 구해
 *   u = -Kx 로 제어.
 *
 *   Q: 상태 오차 가중치 (자세 오차를 얼마나 빨리 줄일지)
 *   R: 제어 입력 가중치 (모터 사용을 얼마나 아낄지)
 *
 * FC에서의 적용:
 *   상태 x = [자세 오차(3), 각속도 오차(3)] = 6차원
 *   입력 u = [roll, pitch, yaw 토크] = 3차원
 *   K 행렬(3x6)은 오프라인에서 MATLAB/Python으로 계산하여 상수로 입력.
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
     * K 행렬 (3x6): u = -K * x
     *
     * 행: [roll, pitch, yaw]
     * 열: [e_roll, e_pitch, e_yaw, e_p, e_q, e_r]
     *      ──── 자세 오차 ────  ── 각속도 오차 ──
     *
     * MATLAB: K = lqr(A, B, Q, R)
     * Python: K = scipy.linalg.solve_continuous_are(...)
     */
    float K[3][6];
    float output_limit;     /* 출력 클램핑 (기본 1.0) */
} lqr_config_t;

/*
 * 기본 K 값: 호버링 조건 근사.
 * 실제 운용 시 기체 관성 모멘트 기반으로 재계산 필요.
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
 *  Public — controller_t 인스턴스
 * ══════════════════════════════════════════════════════ */

extern controller_t lqr_controller;

#endif /* LQR_CONTROLLER_H */
