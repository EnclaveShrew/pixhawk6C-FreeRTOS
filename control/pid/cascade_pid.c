/**
 * @file cascade_pid.c
 * @brief Cascade PID controller implementation
 *
 * Cascade structure:
 *   1. Attitude loop (P): attitude error -> target angular rate
 *   2. Rate loop (PID): rate error -> control output
 *
 * Mathematical background:
 *   Attitude error is computed via quaternion, extracting an axis-angle error vector.
 *   e_att = 2 * sign(q_err.w) * [q_err.x, q_err.y, q_err.z]
 *   Multiplying this error by P gain yields the target angular rate.
 */

#include "cascade_pid.h"
#include "quaternion.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

/* ── Default instance context ──────────────────────── */

static cascade_pid_ctx_t cascade_pid_default_ctx;

/* ── Single-axis PID computation ──────────────────── */

static float pid_compute(pid_axis_t *pid, float error, float dt)
{
    /* P */
    float p_term = pid->kp * error;

    /* I (anti-windup: clamp integral) */
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
     * Using measurement derivative instead of error derivative prevents setpoint kick,
     * but here we use error derivative + LPF for noise rejection
     */
    float d_term = 0.0f;
    if (dt > 1e-6f)
    {
        float d_raw = (error - pid->prev_error) / dt;
        pid->d_filtered = pid->d_filtered + pid->d_filter_alpha * (d_raw - pid->d_filtered);
        d_term = pid->kd * pid->d_filtered;
    }
    pid->prev_error = error;

    /* Output clamping */
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

/* ── controller_t interface implementation ────────── */

static int pid_init(void *ctx, void *config)
{
    cascade_pid_ctx_t *p = (cascade_pid_ctx_t *)ctx;
    cascade_pid_config_t default_cfg = CASCADE_PID_DEFAULT_CONFIG;
    cascade_pid_config_t *cfg = (config != NULL) ? (cascade_pid_config_t *)config : &default_cfg;

    memset(p, 0, sizeof(cascade_pid_ctx_t));

    p->att_kp[0] = cfg->att_kp_roll;
    p->att_kp[1] = cfg->att_kp_pitch;
    p->att_kp[2] = cfg->att_kp_yaw;

    p->rate[0].kp = cfg->rate_kp_roll;
    p->rate[0].ki = cfg->rate_ki_roll;
    p->rate[0].kd = cfg->rate_kd_roll;
    p->rate[0].integral_limit = cfg->integral_limit;
    p->rate[0].d_filter_alpha = cfg->d_filter_alpha;
    p->rate[0].output_limit = cfg->output_limit;

    p->rate[1].kp = cfg->rate_kp_pitch;
    p->rate[1].ki = cfg->rate_ki_pitch;
    p->rate[1].kd = cfg->rate_kd_pitch;
    p->rate[1].integral_limit = cfg->integral_limit;
    p->rate[1].d_filter_alpha = cfg->d_filter_alpha;
    p->rate[1].output_limit = cfg->output_limit;

    p->rate[2].kp = cfg->rate_kp_yaw;
    p->rate[2].ki = cfg->rate_ki_yaw;
    p->rate[2].kd = cfg->rate_kd_yaw;
    p->rate[2].integral_limit = cfg->integral_limit;
    p->rate[2].d_filter_alpha = cfg->d_filter_alpha;
    p->rate[2].output_limit = cfg->output_limit;

    return 0;
}

static int pid_update(void *ctx, const controller_input_t *in, controller_output_t *out)
{
    cascade_pid_ctx_t *p = (cascade_pid_ctx_t *)ctx;

    vec3f_t att_err;
    quat_attitude_error(&in->target_attitude, &in->current_attitude, &att_err);

    float target_rate[3];
    target_rate[0] = p->att_kp[0] * att_err.x;
    target_rate[1] = p->att_kp[1] * att_err.y;
    target_rate[2] = p->att_kp[2] * att_err.z;

    float rate_err[3];
    rate_err[0] = target_rate[0] - in->gyro.x;
    rate_err[1] = target_rate[1] - in->gyro.y;
    rate_err[2] = target_rate[2] - in->gyro.z;

    out->roll  = pid_compute(&p->rate[0], rate_err[0], in->dt);
    out->pitch = pid_compute(&p->rate[1], rate_err[1], in->dt);
    out->yaw   = pid_compute(&p->rate[2], rate_err[2], in->dt);
    out->thrust = in->thrust;

    return 0;
}

static int pid_set_param(void *ctx, const char *key, float value)
{
    cascade_pid_ctx_t *p = (cascade_pid_ctx_t *)ctx;

    if (strcmp(key, "att_kp_roll") == 0)       { p->att_kp[0] = value; return 0; }
    if (strcmp(key, "att_kp_pitch") == 0)      { p->att_kp[1] = value; return 0; }
    if (strcmp(key, "att_kp_yaw") == 0)        { p->att_kp[2] = value; return 0; }
    if (strcmp(key, "rate_kp_roll") == 0)       { p->rate[0].kp = value; return 0; }
    if (strcmp(key, "rate_ki_roll") == 0)       { p->rate[0].ki = value; return 0; }
    if (strcmp(key, "rate_kd_roll") == 0)       { p->rate[0].kd = value; return 0; }
    if (strcmp(key, "rate_kp_pitch") == 0)      { p->rate[1].kp = value; return 0; }
    if (strcmp(key, "rate_ki_pitch") == 0)      { p->rate[1].ki = value; return 0; }
    if (strcmp(key, "rate_kd_pitch") == 0)      { p->rate[1].kd = value; return 0; }
    if (strcmp(key, "rate_kp_yaw") == 0)        { p->rate[2].kp = value; return 0; }
    if (strcmp(key, "rate_ki_yaw") == 0)        { p->rate[2].ki = value; return 0; }
    if (strcmp(key, "rate_kd_yaw") == 0)        { p->rate[2].kd = value; return 0; }

    return -1;
}

static int pid_reset(void *ctx)
{
    cascade_pid_ctx_t *p = (cascade_pid_ctx_t *)ctx;
    for (int i = 0; i < 3; i++)
    {
        p->rate[i].integral = 0.0f;
        p->rate[i].prev_error = 0.0f;
        p->rate[i].d_filtered = 0.0f;
    }
    return 0;
}

/* ── Global controller_t instance ─────────────────── */

controller_t cascade_pid_controller = {
    .name      = "Cascade PID",
    .type      = CTRL_TYPE_PID,
    .ctx       = &cascade_pid_default_ctx,
    .init      = pid_init,
    .update    = pid_update,
    .set_param = pid_set_param,
    .reset     = pid_reset,
};
