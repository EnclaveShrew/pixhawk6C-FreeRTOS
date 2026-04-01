/**
 * @file indi_controller.c
 * @brief INDI controller implementation
 *
 * PID/LQR과의 차이:
 *   PID: 오차 기반, 모델 불필요, 튜닝 수동
 *   LQR: 오차 기반, 모델 필요(선형화), 게인 최적 계산
 *   INDI: 증분 기반, 모델 최소화, 외란에 강인
 *
 * INDI 핵심:
 *   Δu = G^(-1) · (ω_dot_desired - ω_dot_measured)
 *   u = u_prev + Δu
 *
 *   ω_dot_measured는 자이로 미분으로 추정 (LPF 적용)
 */

#include "indi_controller.h"
#include "quaternion.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

/* ── 내부 상태 ─────────────────────────────────────── */

static struct
{
    /* 자세 루프 P 게인 */
    float att_kp[3];

    /* 제어 효과 역행렬 (대각) */
    float g_inv[3];

    /* 이전 자이로 값 (각가속도 계산용) */
    vec3f_t gyro_prev;

    /* 필터링된 각가속도 */
    vec3f_t gyro_dot_filtered;

    /* 이전 제어 출력 */
    float u_prev[3];

    /* 설정 */
    float gyro_dot_filter_alpha;
    float output_limit;

    uint8_t initialized;
} s_indi;

/* ── 자세 오차 ─────────────────────────────────────── */

static void attitude_error(const quat_t *target, const quat_t *current, vec3f_t *error)
{
    quat_t q_inv;
    q_inv.w =  current->w;
    q_inv.x = -current->x;
    q_inv.y = -current->y;
    q_inv.z = -current->z;

    quat_t q_err = quat_multiply(&q_inv, target);

    float sign = (q_err.w >= 0.0f) ? 1.0f : -1.0f;

    error->x = 2.0f * sign * q_err.x;
    error->y = 2.0f * sign * q_err.y;
    error->z = 2.0f * sign * q_err.z;
}

/* ── controller_t 인터페이스 구현 ──────────────────── */

static int indi_init(void *config)
{
    indi_config_t default_cfg = INDI_DEFAULT_CONFIG;
    indi_config_t *cfg = (config != NULL) ? (indi_config_t *)config : &default_cfg;

    memset(&s_indi, 0, sizeof(s_indi));

    s_indi.att_kp[0] = cfg->att_kp_roll;
    s_indi.att_kp[1] = cfg->att_kp_pitch;
    s_indi.att_kp[2] = cfg->att_kp_yaw;

    s_indi.g_inv[0] = cfg->g_inv_roll;
    s_indi.g_inv[1] = cfg->g_inv_pitch;
    s_indi.g_inv[2] = cfg->g_inv_yaw;

    s_indi.gyro_dot_filter_alpha = cfg->gyro_dot_filter_alpha;
    s_indi.output_limit = cfg->output_limit;
    s_indi.initialized = 0;

    return 0;
}

static int indi_update(const controller_input_t *in, controller_output_t *out)
{
    /* 첫 호출: 이전 자이로/출력 초기화 */
    if (!s_indi.initialized)
    {
        s_indi.gyro_prev = in->gyro;
        s_indi.u_prev[0] = 0.0f;
        s_indi.u_prev[1] = 0.0f;
        s_indi.u_prev[2] = 0.0f;
        s_indi.initialized = 1;

        out->roll = 0.0f;
        out->pitch = 0.0f;
        out->yaw = 0.0f;
        out->thrust = in->thrust;
        return 0;
    }

    /*
     * Step 1: 각가속도 추정 (자이로 미분 + LPF)
     * ω_dot ≈ (ω - ω_prev) / dt
     */
    float alpha = s_indi.gyro_dot_filter_alpha;

    float gyro_dot_raw_x = (in->gyro.x - s_indi.gyro_prev.x) / in->dt;
    float gyro_dot_raw_y = (in->gyro.y - s_indi.gyro_prev.y) / in->dt;
    float gyro_dot_raw_z = (in->gyro.z - s_indi.gyro_prev.z) / in->dt;

    s_indi.gyro_dot_filtered.x += alpha * (gyro_dot_raw_x - s_indi.gyro_dot_filtered.x);
    s_indi.gyro_dot_filtered.y += alpha * (gyro_dot_raw_y - s_indi.gyro_dot_filtered.y);
    s_indi.gyro_dot_filtered.z += alpha * (gyro_dot_raw_z - s_indi.gyro_dot_filtered.z);

    s_indi.gyro_prev = in->gyro;

    /*
     * Step 2: 자세 루프 (P) → 목표 각속도
     */
    vec3f_t att_err;
    attitude_error(&in->target_attitude, &in->current_attitude, &att_err);

    float target_rate[3];
    target_rate[0] = s_indi.att_kp[0] * att_err.x;
    target_rate[1] = s_indi.att_kp[1] * att_err.y;
    target_rate[2] = s_indi.att_kp[2] * att_err.z;

    /*
     * Step 3: 목표 각가속도
     * 단순 P 제어: ω_dot_des = Kp_rate * (ω_target - ω_current)
     * 여기서는 자세 P 게인이 이미 각속도 단위이므로 바로 사용.
     *
     * 더 정밀하게는 별도 rate P 게인을 둘 수 있지만,
     * INDI 자체가 각가속도 레벨에서 동작하므로 자세 P로 충분.
     */
    float gyro_dot_des[3];
    gyro_dot_des[0] = target_rate[0] - in->gyro.x;
    gyro_dot_des[1] = target_rate[1] - in->gyro.y;
    gyro_dot_des[2] = target_rate[2] - in->gyro.z;

    /*
     * Step 4: INDI 증분 계산
     * Δu = G^(-1) · (ω_dot_des - ω_dot_measured)
     * u = u_prev + Δu
     */
    float du[3];
    du[0] = s_indi.g_inv[0] * (gyro_dot_des[0] - s_indi.gyro_dot_filtered.x);
    du[1] = s_indi.g_inv[1] * (gyro_dot_des[1] - s_indi.gyro_dot_filtered.y);
    du[2] = s_indi.g_inv[2] * (gyro_dot_des[2] - s_indi.gyro_dot_filtered.z);

    float u[3];
    u[0] = s_indi.u_prev[0] + du[0];
    u[1] = s_indi.u_prev[1] + du[1];
    u[2] = s_indi.u_prev[2] + du[2];

    /* 출력 클램핑 */
    for (int i = 0; i < 3; i++)
    {
        if (u[i] > s_indi.output_limit)
        {
            u[i] = s_indi.output_limit;
        }
        if (u[i] < -s_indi.output_limit)
        {
            u[i] = -s_indi.output_limit;
        }
    }

    /* 이전 출력 저장 */
    s_indi.u_prev[0] = u[0];
    s_indi.u_prev[1] = u[1];
    s_indi.u_prev[2] = u[2];

    out->roll   = u[0];
    out->pitch  = u[1];
    out->yaw    = u[2];
    out->thrust = in->thrust;

    return 0;
}

static int indi_set_param(const char *key, float value)
{
    if (strcmp(key, "att_kp_roll") == 0)       { s_indi.att_kp[0] = value; return 0; }
    if (strcmp(key, "att_kp_pitch") == 0)      { s_indi.att_kp[1] = value; return 0; }
    if (strcmp(key, "att_kp_yaw") == 0)        { s_indi.att_kp[2] = value; return 0; }
    if (strcmp(key, "g_inv_roll") == 0)         { s_indi.g_inv[0] = value; return 0; }
    if (strcmp(key, "g_inv_pitch") == 0)        { s_indi.g_inv[1] = value; return 0; }
    if (strcmp(key, "g_inv_yaw") == 0)          { s_indi.g_inv[2] = value; return 0; }
    if (strcmp(key, "gyro_dot_alpha") == 0)     { s_indi.gyro_dot_filter_alpha = value; return 0; }

    return -1;
}

static int indi_reset(void)
{
    s_indi.gyro_prev.x = 0.0f;
    s_indi.gyro_prev.y = 0.0f;
    s_indi.gyro_prev.z = 0.0f;
    s_indi.gyro_dot_filtered.x = 0.0f;
    s_indi.gyro_dot_filtered.y = 0.0f;
    s_indi.gyro_dot_filtered.z = 0.0f;
    s_indi.u_prev[0] = 0.0f;
    s_indi.u_prev[1] = 0.0f;
    s_indi.u_prev[2] = 0.0f;
    s_indi.initialized = 0;

    return 0;
}

/* ── 전역 controller_t 인스턴스 ────────────────────── */

controller_t indi_controller = {
    .name      = "INDI",
    .type      = CTRL_TYPE_INDI,
    .init      = indi_init,
    .update    = indi_update,
    .set_param = indi_set_param,
    .reset     = indi_reset,
};
