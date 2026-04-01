/**
 * @file ekf.c
 * @brief Extended Kalman Filter implementation for attitude estimation
 *
 * EKF 사이클 (매 루프):
 *   1. Predict: 자이로로 상태 예측 + 공분산 전파
 *   2. Update (accel): 중력 방향으로 roll/pitch 보정
 *   3. Update (mag):   자기장 방향으로 yaw 보정
 *
 * State: [q0, q1, q2, q3, bx, by, bz]
 *   q = 자세 쿼터니언
 *   b = 자이로 바이어스 (자동 추정됨)
 *
 * 자이로 입력에서 바이어스를 빼서 예측:
 *   ω_corrected = ω_measured - bias
 */

#include "ekf.h"
#include "quaternion.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

/* ── 행렬 유틸리티 (7x7 고정 크기) ─────────────────── */

/**
 * @brief 7x7 행렬 덧셈: C = A + B
 */
static void mat7_add(float C[7][7], const float A[7][7], const float B[7][7])
{
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

/**
 * @brief 7x7 행렬 곱셈: C = A * B
 */
static void mat7_mul(float C[7][7], const float A[7][7], const float B[7][7])
{
    float tmp[7][7];
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            tmp[i][j] = 0.0f;
            for (int k = 0; k < 7; k++)
            {
                tmp[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    memcpy(C, tmp, sizeof(tmp));
}

/**
 * @brief 7x7 전치행렬: B = A^T
 */
static void mat7_transpose(float B[7][7], const float A[7][7])
{
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            B[i][j] = A[j][i];
        }
    }
}

/* ── 소규모 행렬 (측정 업데이트용) ─────────────────── */

/**
 * @brief 3x3 행렬 역행렬 (Cramer's rule)
 * @return 0 on success, -1 if singular
 */
static int mat3_inverse(float out[3][3], const float in[3][3])
{
    float det = in[0][0] * (in[1][1] * in[2][2] - in[1][2] * in[2][1])
              - in[0][1] * (in[1][0] * in[2][2] - in[1][2] * in[2][0])
              + in[0][2] * (in[1][0] * in[2][1] - in[1][1] * in[2][0]);

    if (fabsf(det) < 1e-10f)
    {
        return -1;
    }

    float inv_det = 1.0f / det;

    out[0][0] = (in[1][1] * in[2][2] - in[1][2] * in[2][1]) * inv_det;
    out[0][1] = (in[0][2] * in[2][1] - in[0][1] * in[2][2]) * inv_det;
    out[0][2] = (in[0][1] * in[1][2] - in[0][2] * in[1][1]) * inv_det;
    out[1][0] = (in[1][2] * in[2][0] - in[1][0] * in[2][2]) * inv_det;
    out[1][1] = (in[0][0] * in[2][2] - in[0][2] * in[2][0]) * inv_det;
    out[1][2] = (in[0][2] * in[1][0] - in[0][0] * in[1][2]) * inv_det;
    out[2][0] = (in[1][0] * in[2][1] - in[1][1] * in[2][0]) * inv_det;
    out[2][1] = (in[0][1] * in[2][0] - in[0][0] * in[2][1]) * inv_det;
    out[2][2] = (in[0][0] * in[1][1] - in[0][1] * in[1][0]) * inv_det;

    return 0;
}

/* ── 쿼터니언 정규화 (state 내부) ──────────────────── */

static void ekf_normalize_quat(ekf_state_t *state)
{
    float n = sqrtf(state->x[0] * state->x[0] + state->x[1] * state->x[1] +
                    state->x[2] * state->x[2] + state->x[3] * state->x[3]);
    if (n < 1e-8f)
    {
        return;
    }
    float inv = 1.0f / n;
    state->x[0] *= inv;
    state->x[1] *= inv;
    state->x[2] *= inv;
    state->x[3] *= inv;
}

/* ── 오일러 각도 갱신 ──────────────────────────────── */

static void ekf_update_euler(ekf_state_t *state)
{
    quat_t q = { state->x[0], state->x[1], state->x[2], state->x[3] };
    quat_to_euler(&q, &state->roll, &state->pitch, &state->yaw);
}

/* ── 초기화 ─────────────────────────────────────────── */

void ekf_init(ekf_state_t *state, const ekf_config_t *cfg)
{
    ekf_config_t default_cfg = EKF_DEFAULT_CONFIG;
    if (cfg == NULL)
    {
        cfg = &default_cfg;
    }

    memset(state, 0, sizeof(ekf_state_t));

    /* 초기 쿼터니언: 항등 (회전 없음) */
    state->x[0] = 1.0f;

    /* 초기 공분산: 대각 행렬 (초기 불확실성) */
    for (int i = 0; i < 4; i++)
    {
        state->P[i][i] = 0.1f;     /* 쿼터니언 불확실성 */
    }
    for (int i = 4; i < 7; i++)
    {
        state->P[i][i] = 0.01f;    /* 바이어스 불확실성 */
    }

    state->gyro_noise = cfg->gyro_noise;
    state->gyro_bias_noise = cfg->gyro_bias_noise;
    state->accel_noise = cfg->accel_noise;
    state->mag_noise = cfg->mag_noise;
    state->initialized = 1;
}

/* ── Prediction step ────────────────────────────────── */

void ekf_predict(ekf_state_t *state, const vec3f_t *gyro, float dt)
{
    float q0 = state->x[0], q1 = state->x[1];
    float q2 = state->x[2], q3 = state->x[3];

    /* 자이로에서 바이어스 제거 */
    float wx = gyro->x - state->x[4];
    float wy = gyro->y - state->x[5];
    float wz = gyro->z - state->x[6];

    /*
     * 쿼터니언 미분 방정식:
     * q_dot = 0.5 * q ⊗ [0, wx, wy, wz]
     *
     * 1차 오일러 적분: q_new = q + q_dot * dt
     */
    float half_dt = 0.5f * dt;
    state->x[0] += (-q1 * wx - q2 * wy - q3 * wz) * half_dt;
    state->x[1] += ( q0 * wx + q2 * wz - q3 * wy) * half_dt;
    state->x[2] += ( q0 * wy - q1 * wz + q3 * wx) * half_dt;
    state->x[3] += ( q0 * wz + q1 * wy - q2 * wx) * half_dt;

    /* 바이어스는 변하지 않는다고 가정 (random walk) */
    /* state->x[4..6] 유지 */

    ekf_normalize_quat(state);

    /*
     * 상태 전이 야코비안 F (7x7)
     * F = ∂f/∂x
     *
     * 쿼터니언 부분: 자이로 각속도에 의한 회전
     * 바이어스 부분: 항등 (변화 없음)
     */
    float F[7][7];
    memset(F, 0, sizeof(F));

    /* 쿼터니언 → 쿼터니언 (상위 4x4) */
    F[0][0] = 1.0f;
    F[0][1] = -half_dt * wx;
    F[0][2] = -half_dt * wy;
    F[0][3] = -half_dt * wz;
    F[1][0] =  half_dt * wx;
    F[1][1] = 1.0f;
    F[1][2] =  half_dt * wz;
    F[1][3] = -half_dt * wy;
    F[2][0] =  half_dt * wy;
    F[2][1] = -half_dt * wz;
    F[2][2] = 1.0f;
    F[2][3] =  half_dt * wx;
    F[3][0] =  half_dt * wz;
    F[3][1] =  half_dt * wy;
    F[3][2] = -half_dt * wx;
    F[3][3] = 1.0f;

    /* 쿼터니언 → 바이어스 (4x3) */
    F[0][4] =  half_dt * q1;
    F[0][5] =  half_dt * q2;
    F[0][6] =  half_dt * q3;
    F[1][4] = -half_dt * q0;
    F[1][5] =  half_dt * q3;
    F[1][6] = -half_dt * q2;
    F[2][4] = -half_dt * q3;
    F[2][5] = -half_dt * q0;
    F[2][6] =  half_dt * q1;
    F[3][4] =  half_dt * q2;
    F[3][5] = -half_dt * q1;
    F[3][6] = -half_dt * q0;

    /* 바이어스 → 바이어스 (3x3 항등) */
    F[4][4] = 1.0f;
    F[5][5] = 1.0f;
    F[6][6] = 1.0f;

    /*
     * 프로세스 노이즈 Q (7x7 대각)
     */
    float Q[7][7];
    memset(Q, 0, sizeof(Q));
    float gn2 = state->gyro_noise * state->gyro_noise * dt;
    float bn2 = state->gyro_bias_noise * state->gyro_bias_noise * dt;
    Q[0][0] = gn2; Q[1][1] = gn2; Q[2][2] = gn2; Q[3][3] = gn2;
    Q[4][4] = bn2; Q[5][5] = bn2; Q[6][6] = bn2;

    /*
     * 공분산 전파: P = F * P * F^T + Q
     */
    float Ft[7][7];
    float FP[7][7];

    mat7_transpose(Ft, F);
    mat7_mul(FP, F, state->P);
    mat7_mul(state->P, FP, Ft);
    mat7_add(state->P, state->P, Q);

    ekf_update_euler(state);
}

/* ── Measurement update: Accelerometer ──────────────── */

void ekf_update_accel(ekf_state_t *state, const vec3f_t *accel)
{
    float q0 = state->x[0], q1 = state->x[1];
    float q2 = state->x[2], q3 = state->x[3];

    /* 가속도 벡터 정규화 (중력 방향만 추출) */
    float anorm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    if (anorm < 1e-6f)
    {
        return;
    }
    float ax = accel->x / anorm;
    float ay = accel->y / anorm;
    float az = accel->z / anorm;

    /*
     * 예측 중력 방향 (쿼터니언으로 [0,0,1]을 회전체 좌표로 변환)
     * h = R^T * [0, 0, 1]
     */
    float hx = 2.0f * (q1 * q3 - q0 * q2);
    float hy = 2.0f * (q2 * q3 + q0 * q1);
    float hz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 잔차 (innovation): z - h(x) */
    float y[3];
    y[0] = ax - hx;
    y[1] = ay - hy;
    y[2] = az - hz;

    /*
     * 측정 야코비안 H (3x7)
     * H = ∂h/∂x (중력 벡터의 쿼터니언 편미분)
     */
    float H[3][7];
    memset(H, 0, sizeof(H));

    H[0][0] = -2.0f * q2;  H[0][1] =  2.0f * q3;
    H[0][2] = -2.0f * q0;  H[0][3] =  2.0f * q1;
    H[1][0] =  2.0f * q1;  H[1][1] =  2.0f * q0;
    H[1][2] =  2.0f * q3;  H[1][3] =  2.0f * q2;
    H[2][0] =  2.0f * q0;  H[2][1] = -2.0f * q1;
    H[2][2] = -2.0f * q2;  H[2][3] =  2.0f * q3;

    /*
     * 칼만 게인 계산:
     * S = H * P * H^T + R     (3x3)
     * K = P * H^T * S^(-1)    (7x3)
     */

    /* PHt = P * H^T (7x3) */
    float PHt[7][3];
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            PHt[i][j] = 0.0f;
            for (int k = 0; k < 7; k++)
            {
                PHt[i][j] += state->P[i][k] * H[j][k];
            }
        }
    }

    /* S = H * PHt + R (3x3) */
    float S[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            S[i][j] = 0.0f;
            for (int k = 0; k < 7; k++)
            {
                S[i][j] += H[i][k] * PHt[k][j];
            }
            if (i == j)
            {
                S[i][j] += state->accel_noise * state->accel_noise;
            }
        }
    }

    /* S^(-1) */
    float Si[3][3];
    if (mat3_inverse(Si, S) < 0)
    {
        return;
    }

    /* K = PHt * Si (7x3) */
    float K[7][3];
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            K[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                K[i][j] += PHt[i][k] * Si[k][j];
            }
        }
    }

    /* 상태 업데이트: x = x + K * y */
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            state->x[i] += K[i][j] * y[j];
        }
    }

    /* 공분산 업데이트: P = (I - K*H) * P */
    float KH[7][7];
    memset(KH, 0, sizeof(KH));
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    float P_new[7][7];
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            float I_KH = ((i == j) ? 1.0f : 0.0f) - KH[i][j];
            P_new[i][j] = 0.0f;
            for (int k = 0; k < 7; k++)
            {
                P_new[i][j] += I_KH * state->P[k][j];
            }
        }
    }
    memcpy(state->P, P_new, sizeof(P_new));

    ekf_normalize_quat(state);
    ekf_update_euler(state);
}

/* ── Measurement update: Magnetometer ───────────────── */

void ekf_update_mag(ekf_state_t *state, const vec3f_t *mag)
{
    float q0 = state->x[0], q1 = state->x[1];
    float q2 = state->x[2], q3 = state->x[3];

    /* 자력계 벡터 정규화 */
    float mnorm = sqrtf(mag->x * mag->x + mag->y * mag->y + mag->z * mag->z);
    if (mnorm < 1e-6f)
    {
        return;
    }
    float mx = mag->x / mnorm;
    float my = mag->y / mnorm;
    float mz = mag->z / mnorm;

    /*
     * 자력계 벡터를 지구 좌표계로 변환 후, 수평 성분만 사용 (yaw만 보정)
     * h = R * m  (body → earth)
     */
    float hx = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * mx
             + 2.0f * (q1 * q2 - q0 * q3) * my
             + 2.0f * (q1 * q3 + q0 * q2) * mz;
    float hy = 2.0f * (q1 * q2 + q0 * q3) * mx
             + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * my
             + 2.0f * (q2 * q3 - q0 * q1) * mz;

    /* 지구 좌표계의 자기 북극 방향 (수평 성분) */
    float bx = sqrtf(hx * hx + hy * hy);
    float bz_earth = 2.0f * (q1 * q3 - q0 * q2) * mx
                   + 2.0f * (q2 * q3 + q0 * q1) * my
                   + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * mz;

    /*
     * 예측 자력계 값: 지구 기준 [bx, 0, bz]를 body로 역변환
     */
    float pred_mx = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * bx
                  + 2.0f * (q1 * q3 + q0 * q2) * bz_earth;
    float pred_my = 2.0f * (q1 * q2 + q0 * q3) * bx
                  + 2.0f * (q2 * q3 - q0 * q1) * bz_earth;
    float pred_mz = 2.0f * (q1 * q3 - q0 * q2) * bx
                  + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * bz_earth;

    /* 잔차 */
    float y[3];
    y[0] = mx - pred_mx;
    y[1] = my - pred_my;
    y[2] = mz - pred_mz;

    /*
     * 자력계 야코비안 H는 복잡하므로 수치 근사 대신
     * 가속도계와 동일한 구조의 간소화된 업데이트 사용.
     *
     * 수치적으로 H를 구성:
     * 여기서는 가속도계 업데이트와 동일한 칼만 게인 구조를 사용하되,
     * 예측값과 잔차만 자력계 기반으로 교체.
     */

    /* 간소화된 H (3x7): 쿼터니언 편미분 수치 근사 */
    float H[3][7];
    memset(H, 0, sizeof(H));
    float delta = 1e-4f;

    for (int j = 0; j < 4; j++)
    {
        float x_save = state->x[j];

        /* +delta */
        state->x[j] = x_save + delta;
        ekf_normalize_quat(state);
        float qq0p = state->x[0], qq1p = state->x[1];
        float qq2p = state->x[2], qq3p = state->x[3];

        float hxp = (qq0p * qq0p + qq1p * qq1p - qq2p * qq2p - qq3p * qq3p) * bx
                  + 2.0f * (qq1p * qq3p + qq0p * qq2p) * bz_earth;
        float hyp = 2.0f * (qq1p * qq2p + qq0p * qq3p) * bx
                  + 2.0f * (qq2p * qq3p - qq0p * qq1p) * bz_earth;
        float hzp = 2.0f * (qq1p * qq3p - qq0p * qq2p) * bx
                  + (qq0p * qq0p - qq1p * qq1p - qq2p * qq2p + qq3p * qq3p) * bz_earth;

        /* -delta */
        state->x[j] = x_save - delta;
        ekf_normalize_quat(state);
        float qq0m = state->x[0], qq1m = state->x[1];
        float qq2m = state->x[2], qq3m = state->x[3];

        float hxm = (qq0m * qq0m + qq1m * qq1m - qq2m * qq2m - qq3m * qq3m) * bx
                  + 2.0f * (qq1m * qq3m + qq0m * qq2m) * bz_earth;
        float hym = 2.0f * (qq1m * qq2m + qq0m * qq3m) * bx
                  + 2.0f * (qq2m * qq3m - qq0m * qq1m) * bz_earth;
        float hzm = 2.0f * (qq1m * qq3m - qq0m * qq2m) * bx
                  + (qq0m * qq0m - qq1m * qq1m - qq2m * qq2m + qq3m * qq3m) * bz_earth;

        float inv_2delta = 1.0f / (2.0f * delta);
        H[0][j] = (hxp - hxm) * inv_2delta;
        H[1][j] = (hyp - hym) * inv_2delta;
        H[2][j] = (hzp - hzm) * inv_2delta;

        /* 복원 */
        state->x[j] = x_save;
    }
    ekf_normalize_quat(state);

    /* 칼만 게인 계산 (가속도계 업데이트와 동일한 구조) */

    float PHt[7][3];
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            PHt[i][j] = 0.0f;
            for (int k = 0; k < 7; k++)
            {
                PHt[i][j] += state->P[i][k] * H[j][k];
            }
        }
    }

    float S[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            S[i][j] = 0.0f;
            for (int k = 0; k < 7; k++)
            {
                S[i][j] += H[i][k] * PHt[k][j];
            }
            if (i == j)
            {
                S[i][j] += state->mag_noise * state->mag_noise;
            }
        }
    }

    float Si[3][3];
    if (mat3_inverse(Si, S) < 0)
    {
        return;
    }

    float K[7][3];
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            K[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                K[i][j] += PHt[i][k] * Si[k][j];
            }
        }
    }

    /* 상태 업데이트 */
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            state->x[i] += K[i][j] * y[j];
        }
    }

    /* 공분산 업데이트 */
    float KH[7][7];
    memset(KH, 0, sizeof(KH));
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    float P_new[7][7];
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            float I_KH = ((i == j) ? 1.0f : 0.0f) - KH[i][j];
            P_new[i][j] = 0.0f;
            for (int k = 0; k < 7; k++)
            {
                P_new[i][j] += I_KH * state->P[k][j];
            }
        }
    }
    memcpy(state->P, P_new, sizeof(P_new));

    ekf_normalize_quat(state);
    ekf_update_euler(state);
}

/* ── Getter 함수 ────────────────────────────────────── */

void ekf_get_euler(const ekf_state_t *state, float *roll, float *pitch, float *yaw)
{
    if (roll != NULL)
    {
        *roll = state->roll;
    }
    if (pitch != NULL)
    {
        *pitch = state->pitch;
    }
    if (yaw != NULL)
    {
        *yaw = state->yaw;
    }
}

void ekf_get_quat(const ekf_state_t *state, quat_t *q)
{
    q->w = state->x[0];
    q->x = state->x[1];
    q->y = state->x[2];
    q->z = state->x[3];
}

void ekf_get_gyro_bias(const ekf_state_t *state, vec3f_t *bias)
{
    bias->x = state->x[4];
    bias->y = state->x[5];
    bias->z = state->x[6];
}
