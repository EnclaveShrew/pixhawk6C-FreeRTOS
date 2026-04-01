/**
 * @file lqr_controller.c
 * @brief LQR controller implementation
 *
 * PID와의 차이:
 *   PID: 각 축을 독립적으로 제어, 게인을 수동 튜닝
 *   LQR: 전체 상태를 동시에 고려, 게인을 수학적으로 최적 계산
 *
 * 구조가 PID보다 단순:
 *   u = -K * [자세 오차; 각속도 오차]
 *   행렬-벡터 곱 한 번으로 끝.
 *   적분기가 없어서 정상상태 오차가 남을 수 있음.
 */

#include "lqr_controller.h"
#include "quaternion.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

/* ── 내부 상태 ─────────────────────────────────────── */

static struct
{
    float K[3][6];
    float output_limit;
} s_lqr;

/* ── 자세 오차 (PID와 동일한 방식) ─────────────────── */

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

static int lqr_init(void *config)
{
    lqr_config_t default_cfg = LQR_DEFAULT_CONFIG;
    lqr_config_t *cfg = (config != NULL) ? (lqr_config_t *)config : &default_cfg;

    memcpy(s_lqr.K, cfg->K, sizeof(s_lqr.K));
    s_lqr.output_limit = cfg->output_limit;

    return 0;
}

static int lqr_update(const controller_input_t *in, controller_output_t *out)
{
    /* 상태 벡터 x = [e_roll, e_pitch, e_yaw, e_p, e_q, e_r] */
    vec3f_t att_err;
    attitude_error(&in->target_attitude, &in->current_attitude, &att_err);

    /* 각속도 오차: 호버링 목표 각속도 = 0 → 오차 = -현재 각속도 */
    float x[6] = {
        att_err.x,      /* 자세 오차 roll */
        att_err.y,      /* 자세 오차 pitch */
        att_err.z,      /* 자세 오차 yaw */
        in->gyro.x,     /* 각속도 (목표=0이므로 오차=현재값) */
        in->gyro.y,
        in->gyro.z,
    };

    /*
     * u = -K * x
     * u[0] = roll 출력, u[1] = pitch 출력, u[2] = yaw 출력
     */
    float u[3] = { 0.0f, 0.0f, 0.0f };
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            u[i] += s_lqr.K[i][j] * x[j];
        }
        u[i] = -u[i];

        /* 출력 클램핑 */
        if (u[i] > s_lqr.output_limit)
        {
            u[i] = s_lqr.output_limit;
        }
        if (u[i] < -s_lqr.output_limit)
        {
            u[i] = -s_lqr.output_limit;
        }
    }

    out->roll   = u[0];
    out->pitch  = u[1];
    out->yaw    = u[2];
    out->thrust = in->thrust;

    return 0;
}

static int lqr_set_param(const char *key, float value)
{
    /*
     * K 행렬 원소를 "K_0_0", "K_0_1", ... "K_2_5" 형태로 설정
     */
    if (key[0] == 'K' && key[1] == '_' && key[3] == '_')
    {
        int row = key[2] - '0';
        int col = key[4] - '0';
        if (row >= 0 && row < 3 && col >= 0 && col < 6)
        {
            s_lqr.K[row][col] = value;
            return 0;
        }
    }

    return -1;
}

static int lqr_reset(void)
{
    /* LQR은 내부 상태(적분기 등)가 없으므로 할 것 없음 */
    return 0;
}

/* ── 전역 controller_t 인스턴스 ────────────────────── */

controller_t lqr_controller = {
    .name      = "LQR",
    .type      = CTRL_TYPE_LQR,
    .init      = lqr_init,
    .update    = lqr_update,
    .set_param = lqr_set_param,
    .reset     = lqr_reset,
};
