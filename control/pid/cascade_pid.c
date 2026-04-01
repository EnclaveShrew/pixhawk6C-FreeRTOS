/**
 * @file cascade_pid.c
 * @brief Cascade PID controller implementation
 *
 * 캐스케이드 구조:
 *   1. 자세 루프 (P): 자세 오차 → 목표 각속도
 *   2. 각속도 루프 (PID): 각속도 오차 → 제어 출력
 *
 * 수학적 배경:
 *   자세 오차를 쿼터니언으로 계산하여 축-각도(axis-angle) 오차 벡터를 추출.
 *   e_att = 2 * sign(q_err.w) * [q_err.x, q_err.y, q_err.z]
 *   이 오차에 P 게인을 곱하면 목표 각속도가 됨.
 */

#include "cascade_pid.h"
#include "quaternion.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

/* ── 내부 상태 (모듈 static) ───────────────────────── */

static struct
{
    /* 자세 루프 P 게인 */
    float att_kp[3];    /* [roll, pitch, yaw] */

    /* 각속도 루프 PID */
    pid_axis_t rate[3]; /* [roll, pitch, yaw] */
} s_pid;

/* ── 단축 PID 연산 ─────────────────────────────────── */

static float pid_compute(pid_axis_t *pid, float error, float dt)
{
    /* P */
    float p_term = pid->kp * error;

    /* I (anti-windup: 적분값 클램핑) */
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)
    {
        pid->integral = pid->integral_limit;
    }
    if (pid->integral < -pid->integral_limit)
    {
        pid->integral = -pid->integral_limit;
    }
    float i_term = pid->ki * pid->integral;

    /*
     * D (derivative filtering)
     * 오차 미분 대신 측정값 미분을 쓰면 setpoint kick 방지 가능하지만,
     * 여기서는 오차 미분 + LPF로 노이즈 억제
     */
    float d_raw = (error - pid->prev_error) / dt;
    pid->d_filtered = pid->d_filtered + pid->d_filter_alpha * (d_raw - pid->d_filtered);
    float d_term = pid->kd * pid->d_filtered;
    pid->prev_error = error;

    /* 출력 클램핑 */
    float output = p_term + i_term + d_term;
    if (output > pid->output_limit)
    {
        output = pid->output_limit;
    }
    if (output < -pid->output_limit)
    {
        output = -pid->output_limit;
    }

    return output;
}

/* ── 쿼터니언 자세 오차 → 오차 벡터 ───────────────── */

/**
 * @brief 두 쿼터니언 사이의 자세 오차 벡터 계산
 * @param target   목표 자세
 * @param current  현재 자세
 * @param error    출력: 오차 벡터 [roll, pitch, yaw] (rad)
 *
 * q_err = q_current^(-1) * q_target
 * error = 2 * sign(q_err.w) * [q_err.x, q_err.y, q_err.z]
 */
static void attitude_error(const quat_t *target, const quat_t *current, vec3f_t *error)
{
    /* q_current의 켤레(역) 계산 (단위 쿼터니언이므로 켤레 = 역) */
    quat_t q_inv;
    q_inv.w =  current->w;
    q_inv.x = -current->x;
    q_inv.y = -current->y;
    q_inv.z = -current->z;

    /* q_err = q_inv * q_target */
    quat_t q_err = quat_multiply(&q_inv, target);

    /* 최단 경로 보장: w < 0이면 부호 반전 */
    float sign = (q_err.w >= 0.0f) ? 1.0f : -1.0f;

    error->x = 2.0f * sign * q_err.x;
    error->y = 2.0f * sign * q_err.y;
    error->z = 2.0f * sign * q_err.z;
}

/* ── controller_t 인터페이스 구현 ──────────────────── */

static int pid_init(void *config)
{
    cascade_pid_config_t default_cfg = CASCADE_PID_DEFAULT_CONFIG;
    cascade_pid_config_t *cfg = (config != NULL) ? (cascade_pid_config_t *)config : &default_cfg;

    memset(&s_pid, 0, sizeof(s_pid));

    /* 자세 루프 P 게인 */
    s_pid.att_kp[0] = cfg->att_kp_roll;
    s_pid.att_kp[1] = cfg->att_kp_pitch;
    s_pid.att_kp[2] = cfg->att_kp_yaw;

    /* 각속도 루프 PID — Roll */
    s_pid.rate[0].kp = cfg->rate_kp_roll;
    s_pid.rate[0].ki = cfg->rate_ki_roll;
    s_pid.rate[0].kd = cfg->rate_kd_roll;
    s_pid.rate[0].integral_limit = cfg->integral_limit;
    s_pid.rate[0].d_filter_alpha = cfg->d_filter_alpha;
    s_pid.rate[0].output_limit = cfg->output_limit;

    /* 각속도 루프 PID — Pitch */
    s_pid.rate[1].kp = cfg->rate_kp_pitch;
    s_pid.rate[1].ki = cfg->rate_ki_pitch;
    s_pid.rate[1].kd = cfg->rate_kd_pitch;
    s_pid.rate[1].integral_limit = cfg->integral_limit;
    s_pid.rate[1].d_filter_alpha = cfg->d_filter_alpha;
    s_pid.rate[1].output_limit = cfg->output_limit;

    /* 각속도 루프 PID — Yaw */
    s_pid.rate[2].kp = cfg->rate_kp_yaw;
    s_pid.rate[2].ki = cfg->rate_ki_yaw;
    s_pid.rate[2].kd = cfg->rate_kd_yaw;
    s_pid.rate[2].integral_limit = cfg->integral_limit;
    s_pid.rate[2].d_filter_alpha = cfg->d_filter_alpha;
    s_pid.rate[2].output_limit = cfg->output_limit;

    return 0;
}

static int pid_update(const controller_input_t *in, controller_output_t *out)
{
    /*
     * Step 1: 자세 루프 (외부 P)
     * 자세 오차 → 목표 각속도
     */
    vec3f_t att_err;
    attitude_error(&in->target_attitude, &in->current_attitude, &att_err);

    float target_rate[3];
    target_rate[0] = s_pid.att_kp[0] * att_err.x;  /* 목표 roll rate */
    target_rate[1] = s_pid.att_kp[1] * att_err.y;  /* 목표 pitch rate */
    target_rate[2] = s_pid.att_kp[2] * att_err.z;  /* 목표 yaw rate */

    /*
     * Step 2: 각속도 루프 (내부 PID)
     * 각속도 오차 = 목표 각속도 - 현재 각속도
     */
    float rate_err[3];
    rate_err[0] = target_rate[0] - in->gyro.x;
    rate_err[1] = target_rate[1] - in->gyro.y;
    rate_err[2] = target_rate[2] - in->gyro.z;

    out->roll  = pid_compute(&s_pid.rate[0], rate_err[0], in->dt);
    out->pitch = pid_compute(&s_pid.rate[1], rate_err[1], in->dt);
    out->yaw   = pid_compute(&s_pid.rate[2], rate_err[2], in->dt);
    out->thrust = in->thrust;

    return 0;
}

static int pid_set_param(const char *key, float value)
{
    if (strcmp(key, "att_kp_roll") == 0)       { s_pid.att_kp[0] = value; return 0; }
    if (strcmp(key, "att_kp_pitch") == 0)      { s_pid.att_kp[1] = value; return 0; }
    if (strcmp(key, "att_kp_yaw") == 0)        { s_pid.att_kp[2] = value; return 0; }
    if (strcmp(key, "rate_kp_roll") == 0)       { s_pid.rate[0].kp = value; return 0; }
    if (strcmp(key, "rate_ki_roll") == 0)       { s_pid.rate[0].ki = value; return 0; }
    if (strcmp(key, "rate_kd_roll") == 0)       { s_pid.rate[0].kd = value; return 0; }
    if (strcmp(key, "rate_kp_pitch") == 0)      { s_pid.rate[1].kp = value; return 0; }
    if (strcmp(key, "rate_ki_pitch") == 0)      { s_pid.rate[1].ki = value; return 0; }
    if (strcmp(key, "rate_kd_pitch") == 0)      { s_pid.rate[1].kd = value; return 0; }
    if (strcmp(key, "rate_kp_yaw") == 0)        { s_pid.rate[2].kp = value; return 0; }
    if (strcmp(key, "rate_ki_yaw") == 0)        { s_pid.rate[2].ki = value; return 0; }
    if (strcmp(key, "rate_kd_yaw") == 0)        { s_pid.rate[2].kd = value; return 0; }

    return -1;
}

static int pid_reset(void)
{
    for (int i = 0; i < 3; i++)
    {
        s_pid.rate[i].integral = 0.0f;
        s_pid.rate[i].prev_error = 0.0f;
        s_pid.rate[i].d_filtered = 0.0f;
    }
    return 0;
}

/* ── 전역 controller_t 인스턴스 ────────────────────── */

controller_t cascade_pid_controller = {
    .name      = "Cascade PID",
    .type      = CTRL_TYPE_PID,
    .init      = pid_init,
    .update    = pid_update,
    .set_param = pid_set_param,
    .reset     = pid_reset,
};
