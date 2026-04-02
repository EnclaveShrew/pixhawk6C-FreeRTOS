/**
 * @file motor_mixer.c
 * @brief Motor mixing matrix implementation
 */

#include "motor_mixer.h"
#include <string.h>
#include <stddef.h>

/* ── Internal state ─────────────────────────────────────── */

static mixer_config_t s_mixer;

/* ── Public API ─────────────────────────────────────── */

int mixer_init(const mixer_config_t *cfg)
{
    if (cfg != NULL)
    {
        s_mixer = *cfg;
    }
    else
    {
        mixer_config_t default_cfg = MIXER_QUAD_X_CONFIG;
        s_mixer = default_cfg;
    }

    /* Clamp to prevent array overflow in mix loop */
    if (s_mixer.num_motors > MIXER_MAX_MOTORS)
    {
        s_mixer.num_motors = MIXER_MAX_MOTORS;
    }

    return 0;
}

int mixer_mix(const controller_output_t *ctrl_out, mixer_output_t *mix_out)
{
    mix_out->num_motors = s_mixer.num_motors;

    /* If thrust is 0, stop all motors (disarmed) */
    if (ctrl_out->thrust <= 0.0f)
    {
        for (int i = 0; i < s_mixer.num_motors; i++)
        {
            mix_out->motor[i] = 0.0f;
        }
        return 0;
    }

    /*
     * Priority-based mixing with saturation handling:
     *   Priority: roll/pitch (attitude safety) > yaw > thrust
     *
     * 1. Compute raw mix with roll/pitch only
     * 2. Scale thrust down if needed to preserve roll/pitch authority
     * 3. Add yaw, scale yaw down if saturated
     * 4. Clamp to [idle_throttle, output_max]
     */

    float roll = ctrl_out->roll;
    float pitch = ctrl_out->pitch;
    float yaw = ctrl_out->yaw;
    float thrust = ctrl_out->thrust;

    /* Step 1: Compute roll/pitch contribution range */
    float rp_min = 0.0f;
    float rp_max = 0.0f;
    for (int i = 0; i < s_mixer.num_motors; i++)
    {
        float rp = s_mixer.mix[i][0] * roll + s_mixer.mix[i][1] * pitch;
        if (rp < rp_min) rp_min = rp;
        if (rp > rp_max) rp_max = rp;
    }

    /* Step 2: Adjust thrust to keep roll/pitch within [idle, max] */
    float thrust_max = s_mixer.output_max - rp_max;
    float thrust_min = s_mixer.idle_throttle - rp_min;

    if (thrust > thrust_max)
    {
        thrust = thrust_max;
    }
    if (thrust < thrust_min)
    {
        thrust = thrust_min;
    }

    /*
     * Step 3: Add yaw, scale down if any motor would saturate.
     * Yaw is lowest priority — reduced or zeroed when roll/pitch + thrust
     * already consume the full output range.
     * If base alone exceeds limits (extreme roll/pitch), yaw_scale = 0
     * and yaw control is lost. This is intentional: attitude safety > yaw.
     */
    float yaw_scale = 1.0f;
    for (int i = 0; i < s_mixer.num_motors; i++)
    {
        float base = thrust + s_mixer.mix[i][0] * roll + s_mixer.mix[i][1] * pitch;
        float yaw_contrib = s_mixer.mix[i][2] * yaw;

        /* Only scale if yaw_contrib is pushing past the limit */
        if (yaw_contrib > 1e-6f)
        {
            float headroom = s_mixer.output_max - base;
            if (headroom < yaw_contrib)
            {
                float s = (headroom > 0.0f) ? headroom / yaw_contrib : 0.0f;
                if (s < yaw_scale) yaw_scale = s;
            }
        }
        else if (yaw_contrib < -1e-6f)
        {
            float headroom = base - s_mixer.idle_throttle;
            if (headroom < -yaw_contrib)
            {
                float s = (headroom > 0.0f) ? headroom / (-yaw_contrib) : 0.0f;
                if (s < yaw_scale) yaw_scale = s;
            }
        }
    }
    if (yaw_scale < 0.0f) yaw_scale = 0.0f;
    if (yaw_scale > 1.0f) yaw_scale = 1.0f;

    /* Step 4: Final motor output */
    for (int i = 0; i < s_mixer.num_motors; i++)
    {
        float m = thrust
                + s_mixer.mix[i][0] * roll
                + s_mixer.mix[i][1] * pitch
                + s_mixer.mix[i][2] * yaw * yaw_scale;

        if (m < s_mixer.idle_throttle)
        {
            m = s_mixer.idle_throttle;
        }
        if (m > s_mixer.output_max)
        {
            m = s_mixer.output_max;
        }

        mix_out->motor[i] = m;
    }

    return 0;
}
