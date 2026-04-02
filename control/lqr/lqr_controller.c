/**
 * @file lqr_controller.c
 * @brief LQR controller implementation
 *
 * Differences from PID:
 *   PID: controls each axis independently, manually tuned gains
 *   LQR: considers full state simultaneously, mathematically optimal gains
 *
 * Simpler structure than PID:
 *   u = -K * [attitude error; rate error]
 *   Single matrix-vector multiplication.
 *   No integrator, so steady-state error may remain.
 */

#include "lqr_controller.h"
#include "quaternion.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

/* ── Instance state ── */

static lqr_ctx_t lqr_default_ctx;


/* ── controller_t interface implementation ────────── */

static int lqr_init(void *ctx, void *config)
{
    lqr_ctx_t *lqr = (lqr_ctx_t *)ctx;
    lqr_config_t default_cfg = LQR_DEFAULT_CONFIG;
    lqr_config_t *cfg = (config != NULL) ? (lqr_config_t *)config : &default_cfg;

    memcpy(lqr->K, cfg->K, sizeof(lqr->K));
    lqr->output_limit = cfg->output_limit;

    return 0;
}

static int lqr_update(void *ctx, const controller_input_t *in, controller_output_t *out)
{
    lqr_ctx_t *lqr = (lqr_ctx_t *)ctx;

    /* State vector x = [e_roll, e_pitch, e_yaw, e_p, e_q, e_r] */
    vec3f_t att_err;
    quat_attitude_error(&in->target_attitude, &in->current_attitude, &att_err);

    /* Rate error: hover target rate = 0, so error = current rate */
    float x[6] = {
        att_err.x,      /* attitude error roll */
        att_err.y,      /* attitude error pitch */
        att_err.z,      /* attitude error yaw */
        in->gyro.x,     /* angular rate (target=0, so error=current) */
        in->gyro.y,
        in->gyro.z,
    };

    /*
     * u = -K * x
     * u[0] = roll output, u[1] = pitch output, u[2] = yaw output
     */
    float u[3] = { 0.0f, 0.0f, 0.0f };
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            u[i] += lqr->K[i][j] * x[j];
        }
        u[i] = -u[i];

        /* Output clamping */
        if (u[i] > lqr->output_limit)
        {
            u[i] = lqr->output_limit;
        }
        if (u[i] < -lqr->output_limit)
        {
            u[i] = -lqr->output_limit;
        }
    }

    out->roll   = u[0];
    out->pitch  = u[1];
    out->yaw    = u[2];
    out->thrust = in->thrust;

    return 0;
}

static int lqr_set_param(void *ctx, const char *key, float value)
{
    lqr_ctx_t *lqr = (lqr_ctx_t *)ctx;

    /*
     * Set K matrix elements as "K_0_0", "K_0_1", ... "K_2_5"
     * Format: exactly 5 chars "K_R_C" where R=0~2, C=0~5
     */
    if (key == NULL)
    {
        return -1;
    }

    size_t len = strlen(key);
    if (len == 5 &&
        key[0] == 'K' && key[1] == '_' && key[3] == '_' &&
        key[2] >= '0' && key[2] <= '2' &&
        key[4] >= '0' && key[4] <= '5')
    {
        int row = key[2] - '0';
        int col = key[4] - '0';
        lqr->K[row][col] = value;
        return 0;
    }

    return -1;
}

static int lqr_reset(void *ctx)
{
    (void)ctx;
    /* LQR has no internal state (integrators, etc.), nothing to reset */
    return 0;
}

/* ── Global controller_t instance ─────────────────── */

controller_t lqr_controller = {
    .name      = "LQR",
    .type      = CTRL_TYPE_LQR,
    .ctx       = &lqr_default_ctx,
    .init      = lqr_init,
    .update    = lqr_update,
    .set_param = lqr_set_param,
    .reset     = lqr_reset,
};
