/**
 * @file indi_controller.c
 * @brief INDI controller implementation
 *
 * Differences from PID/LQR:
 *   PID: error-based, no model needed, manual tuning
 *   LQR: error-based, model required (linearized), optimal gain
 *   INDI: increment-based, minimal model, robust to disturbances
 *
 * INDI core:
 *   Δu = G^(-1) · (ω_dot_desired - ω_dot_measured)
 *   u = u_prev + du
 *
 *   w_dot_measured is estimated from gyro differentiation (with LPF)
 */

#include "indi_controller.h"
#include "quaternion.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

/* ── Instance state ── */

static indi_ctx_t indi_default_ctx;


/* ── controller_t interface implementation ──────────────────── */

static int indi_init(void *ctx, void *config)
{
    indi_ctx_t *indi = (indi_ctx_t *)ctx;
    indi_config_t default_cfg = INDI_DEFAULT_CONFIG;
    indi_config_t *cfg = (config != NULL) ? (indi_config_t *)config : &default_cfg;

    memset(indi, 0, sizeof(*indi));

    indi->att_kp[0] = cfg->att_kp_roll;
    indi->att_kp[1] = cfg->att_kp_pitch;
    indi->att_kp[2] = cfg->att_kp_yaw;

    indi->g_inv[0] = cfg->g_inv_roll;
    indi->g_inv[1] = cfg->g_inv_pitch;
    indi->g_inv[2] = cfg->g_inv_yaw;

    indi->gyro_dot_filter_alpha = cfg->gyro_dot_filter_alpha;
    indi->output_limit = cfg->output_limit;
    indi->initialized = 0;

    return 0;
}

static int indi_update(void *ctx, const controller_input_t *in, controller_output_t *out)
{
    indi_ctx_t *indi = (indi_ctx_t *)ctx;

    /* First call: initialize previous gyro/output */
    if (!indi->initialized)
    {
        indi->gyro_prev = in->gyro;
        indi->u_prev[0] = 0.0f;
        indi->u_prev[1] = 0.0f;
        indi->u_prev[2] = 0.0f;
        indi->initialized = 1;

        out->roll = 0.0f;
        out->pitch = 0.0f;
        out->yaw = 0.0f;
        out->thrust = in->thrust;
        return 0;
    }

    /*
     * Step 1: Estimate angular acceleration (gyro differentiation + LPF)
     * ω_dot ≈ (ω - ω_prev) / dt
     *
     * If dt ~ 0, derivative is undefined. Hold previous output (du = 0).
     * Computing with stale gyro_dot_filtered would produce incorrect increments.
     * No goto — MISRA-C compliant.
     */
    if (in->dt < 1e-6f)
    {
        indi->gyro_prev = in->gyro;
        out->roll   = indi->u_prev[0];
        out->pitch  = indi->u_prev[1];
        out->yaw    = indi->u_prev[2];
        out->thrust = in->thrust;
        return 0;
    }

    float alpha = indi->gyro_dot_filter_alpha;

    float gyro_dot_raw_x = (in->gyro.x - indi->gyro_prev.x) / in->dt;
    float gyro_dot_raw_y = (in->gyro.y - indi->gyro_prev.y) / in->dt;
    float gyro_dot_raw_z = (in->gyro.z - indi->gyro_prev.z) / in->dt;

    indi->gyro_dot_filtered.x += alpha * (gyro_dot_raw_x - indi->gyro_dot_filtered.x);
    indi->gyro_dot_filtered.y += alpha * (gyro_dot_raw_y - indi->gyro_dot_filtered.y);
    indi->gyro_dot_filtered.z += alpha * (gyro_dot_raw_z - indi->gyro_dot_filtered.z);

    indi->gyro_prev = in->gyro;

    /*
     * Step 2: Attitude loop (P) -> target angular rate
     */
    vec3f_t att_err;
    quat_attitude_error(&in->target_attitude, &in->current_attitude, &att_err);

    float target_rate[3];
    target_rate[0] = indi->att_kp[0] * att_err.x;
    target_rate[1] = indi->att_kp[1] * att_err.y;
    target_rate[2] = indi->att_kp[2] * att_err.z;

    /*
     * Step 3: Desired angular acceleration
     * Simple P control: w_dot_des = Kp_rate * (w_target - w_current)
     * Here, attitude P gain already produces angular rate units, so used directly.
     *
     * A separate rate P gain could be added for more precision,
     * but INDI operates at the angular acceleration level, so attitude P suffices.
     */
    float gyro_dot_des[3];
    gyro_dot_des[0] = target_rate[0] - in->gyro.x;
    gyro_dot_des[1] = target_rate[1] - in->gyro.y;
    gyro_dot_des[2] = target_rate[2] - in->gyro.z;

    /*
     * Step 4: INDI increment computation
     * Δu = G^(-1) · (ω_dot_des - ω_dot_measured)
     * u = u_prev + Δu
     */
    float du[3];
    du[0] = indi->g_inv[0] * (gyro_dot_des[0] - indi->gyro_dot_filtered.x);
    du[1] = indi->g_inv[1] * (gyro_dot_des[1] - indi->gyro_dot_filtered.y);
    du[2] = indi->g_inv[2] * (gyro_dot_des[2] - indi->gyro_dot_filtered.z);

    float u[3];
    u[0] = indi->u_prev[0] + du[0];
    u[1] = indi->u_prev[1] + du[1];
    u[2] = indi->u_prev[2] + du[2];

    /* Output clamping */
    for (int i = 0; i < 3; i++)
    {
        if (u[i] > indi->output_limit)
        {
            u[i] = indi->output_limit;
        }
        if (u[i] < -indi->output_limit)
        {
            u[i] = -indi->output_limit;
        }
    }

    /* Store previous output */
    indi->u_prev[0] = u[0];
    indi->u_prev[1] = u[1];
    indi->u_prev[2] = u[2];

    out->roll   = u[0];
    out->pitch  = u[1];
    out->yaw    = u[2];
    out->thrust = in->thrust;

    return 0;
}

static int indi_set_param(void *ctx, const char *key, float value)
{
    indi_ctx_t *indi = (indi_ctx_t *)ctx;

    if (strcmp(key, "att_kp_roll") == 0)       { indi->att_kp[0] = value; return 0; }
    if (strcmp(key, "att_kp_pitch") == 0)      { indi->att_kp[1] = value; return 0; }
    if (strcmp(key, "att_kp_yaw") == 0)        { indi->att_kp[2] = value; return 0; }
    if (strcmp(key, "g_inv_roll") == 0)         { indi->g_inv[0] = value; return 0; }
    if (strcmp(key, "g_inv_pitch") == 0)        { indi->g_inv[1] = value; return 0; }
    if (strcmp(key, "g_inv_yaw") == 0)          { indi->g_inv[2] = value; return 0; }
    if (strcmp(key, "gyro_dot_alpha") == 0)     { indi->gyro_dot_filter_alpha = value; return 0; }

    return -1;
}

static int indi_reset(void *ctx)
{
    indi_ctx_t *indi = (indi_ctx_t *)ctx;

    indi->gyro_prev.x = 0.0f;
    indi->gyro_prev.y = 0.0f;
    indi->gyro_prev.z = 0.0f;
    indi->gyro_dot_filtered.x = 0.0f;
    indi->gyro_dot_filtered.y = 0.0f;
    indi->gyro_dot_filtered.z = 0.0f;
    indi->u_prev[0] = 0.0f;
    indi->u_prev[1] = 0.0f;
    indi->u_prev[2] = 0.0f;
    indi->initialized = 0;

    return 0;
}

/* ── Global controller_t instance ────────────────────── */

controller_t indi_controller = {
    .name      = "INDI",
    .type      = CTRL_TYPE_INDI,
    .ctx       = &indi_default_ctx,
    .init      = indi_init,
    .update    = indi_update,
    .set_param = indi_set_param,
    .reset     = indi_reset,
};
