/**
 * @file complementary.c
 * @brief Complementary filter implementation
 *
 * 알고리즘:
 * 1. 자이로 각속도로 쿼터니언 적분 (단기 예측)
 * 2. 가속도계에서 roll/pitch 계산 (장기 기준)
 * 3. 자력계에서 yaw 계산 (방위 기준)
 * 4. α 비율로 자이로 예측과 센서 기준을 융합
 *
 * 쿼터니언 사용 이유:
 * - 오일러 각도는 pitch ±90°에서 짐벌 락 발생
 * - 쿼터니언은 특이점 없이 3D 회전 표현 가능
 * - 회전 합성이 곱셈 한 번으로 효율적
 */

#include "complementary.h"
#include "quaternion.h"
#include <math.h>
#include <stddef.h>

/* ── 가속도계에서 roll/pitch 계산 ──────────────────── */

/**
 * @brief 가속도 벡터에서 기울기(roll, pitch) 추출
 * 정지 상태에서 가속도 = 중력만 측정된다는 가정
 */
static void accel_to_roll_pitch(const vec3f_t *accel, float *roll, float *pitch)
{
    *roll = atan2f(accel->y, accel->z);
    *pitch = atan2f(-accel->x, sqrtf(accel->y * accel->y + accel->z * accel->z));
}

/* ── 자력계에서 yaw 계산 (tilt-compensated) ────────── */

/**
 * @brief 자력계 + 현재 roll/pitch로 yaw 계산
 * 기울기 보정 없이 자력계만 쓰면 기체가 기울었을 때 yaw가 틀어짐
 */
static float mag_to_yaw(const vec3f_t *mag, float roll, float pitch)
{
    /* 자력계 벡터를 수평면으로 회전 (기울기 보정) */
    float cos_r = cosf(roll);
    float sin_r = sinf(roll);
    float cos_p = cosf(pitch);
    float sin_p = sinf(pitch);

    float mx = mag->x * cos_p + mag->y * sin_r * sin_p + mag->z * cos_r * sin_p;
    float my = mag->y * cos_r - mag->z * sin_r;

    return atan2f(-my, mx);
}

/* ── Public API ─────────────────────────────────────── */

void complementary_init(complementary_state_t *state, const complementary_config_t *cfg)
{
    complementary_config_t default_cfg = COMPLEMENTARY_DEFAULT_CONFIG;
    if (cfg == NULL)
    {
        cfg = &default_cfg;
    }

    state->alpha = cfg->alpha;
    state->q.w = 1.0f;
    state->q.x = 0.0f;
    state->q.y = 0.0f;
    state->q.z = 0.0f;
    state->roll = 0.0f;
    state->pitch = 0.0f;
    state->yaw = 0.0f;
    state->initialized = 0;
}

void complementary_update(complementary_state_t *state,
                          const vec3f_t *accel,
                          const vec3f_t *gyro,
                          const vec3f_t *mag,
                          float dt)
{
    /*
     * 첫 호출: 가속도계(+자력계)로 초기 자세 설정
     * 자이로는 상대 변화만 알려주므로, 최초 기준점이 필요
     */
    if (!state->initialized)
    {
        float init_roll, init_pitch, init_yaw;
        accel_to_roll_pitch(accel, &init_roll, &init_pitch);

        init_yaw = 0.0f;
        if (mag != NULL)
        {
            init_yaw = mag_to_yaw(mag, init_roll, init_pitch);
        }

        state->q = euler_to_quat(init_roll, init_pitch, init_yaw);
        state->roll = init_roll;
        state->pitch = init_pitch;
        state->yaw = init_yaw;
        state->initialized = 1;
        return;
    }

    /*
     * Step 1: 자이로 적분으로 쿼터니언 예측
     *
     * 각속도 벡터 ω = (gx, gy, gz)로부터 미소 회전 쿼터니언:
     * dq = (1, ωx*dt/2, ωy*dt/2, ωz*dt/2) (1차 근사)
     * q_pred = q * dq
     */
    float half_dt = 0.5f * dt;
    quat_t dq;
    dq.w = 1.0f;
    dq.x = gyro->x * half_dt;
    dq.y = gyro->y * half_dt;
    dq.z = gyro->z * half_dt;

    quat_t q_gyro = quat_multiply(&state->q, &dq);
    quat_normalize(&q_gyro);

    /*
     * Step 2: 가속도계에서 기준 roll/pitch 계산
     */
    float accel_roll, accel_pitch;
    accel_to_roll_pitch(accel, &accel_roll, &accel_pitch);

    /*
     * Step 3: 자이로 예측에서 오일러 추출
     */
    float gyro_roll, gyro_pitch, gyro_yaw;
    quat_to_euler(&q_gyro, &gyro_roll, &gyro_pitch, &gyro_yaw);

    /*
     * Step 4: 상보 필터 융합
     * roll  = α × gyro_roll  + (1-α) × accel_roll
     * pitch = α × gyro_pitch + (1-α) × accel_pitch
     * yaw: 자력계 있으면 보정, 없으면 자이로만
     */
    float alpha = state->alpha;
    float fused_roll = alpha * gyro_roll + (1.0f - alpha) * accel_roll;
    float fused_pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;
    float fused_yaw = gyro_yaw;

    if (mag != NULL)
    {
        float mag_yaw = mag_to_yaw(mag, fused_roll, fused_pitch);
        fused_yaw = alpha * gyro_yaw + (1.0f - alpha) * mag_yaw;
    }

    /*
     * Step 5: 융합된 오일러 → 쿼터니언으로 저장
     */
    state->q = euler_to_quat(fused_roll, fused_pitch, fused_yaw);
    quat_normalize(&state->q);

    state->roll = fused_roll;
    state->pitch = fused_pitch;
    state->yaw = fused_yaw;
}

void complementary_get_euler(const complementary_state_t *state,
                             float *roll, float *pitch, float *yaw)
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

void complementary_get_quat(const complementary_state_t *state, quat_t *q)
{
    *q = state->q;
}
