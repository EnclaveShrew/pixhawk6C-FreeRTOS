/**
 * @file motor_mixer.c
 * @brief Motor mixing matrix implementation
 */

#include "motor_mixer.h"
#include <string.h>
#include <stddef.h>

/* ── 내부 상태 ─────────────────────────────────────── */

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

    return 0;
}

int mixer_mix(const controller_output_t *ctrl_out, mixer_output_t *mix_out)
{
    mix_out->num_motors = s_mixer.num_motors;

    /* thrust가 0이면 모든 모터 정지 (시동 꺼짐) */
    if (ctrl_out->thrust <= 0.0f)
    {
        for (int i = 0; i < s_mixer.num_motors; i++)
        {
            mix_out->motor[i] = 0.0f;
        }
        return 0;
    }

    /*
     * 믹싱:
     * motor[i] = thrust + mix[i][0]*roll + mix[i][1]*pitch + mix[i][2]*yaw
     */
    for (int i = 0; i < s_mixer.num_motors; i++)
    {
        float m = ctrl_out->thrust
                + s_mixer.mix[i][0] * ctrl_out->roll
                + s_mixer.mix[i][1] * ctrl_out->pitch
                + s_mixer.mix[i][2] * ctrl_out->yaw;

        /* 클램핑: [idle_throttle, output_max] */
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
