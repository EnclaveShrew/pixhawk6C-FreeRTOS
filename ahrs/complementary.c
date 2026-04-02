/**
 * @file complementary.c
 * @brief Complementary filter implementation
 *
 * Algorithm:
 * 1. Integrate gyro angular rate via quaternion (short-term prediction)
 * 2. Compute roll/pitch from accelerometer (long-term reference)
 * 3. Compute yaw from magnetometer (heading reference)
 * 4. Fuse gyro prediction and sensor reference at ratio alpha
 *
 * Why quaternions:
 * - Euler angles suffer from gimbal lock at pitch +/-90 deg
 * - Quaternions represent 3D rotation without singularities
 * - Rotation composition is a single multiplication
 */

#include "complementary.h"
#include "quaternion.h"
#include <math.h>
#include <stddef.h>

/* ── Compute roll/pitch from accelerometer ─────────── */

/**
 * @brief Extract tilt (roll, pitch) from acceleration vector
 * Assumes static condition where acceleration = gravity only
 */
static void accel_to_roll_pitch(const vec3f_t *accel, float *roll, float *pitch)
{
    *roll = atan2f(accel->y, accel->z);
    *pitch = atan2f(-accel->x, sqrtf(accel->y * accel->y + accel->z * accel->z));
}

/* ── Compute yaw from magnetometer (tilt-compensated)  */

/**
 * @brief Compute yaw from magnetometer + current roll/pitch
 * Without tilt compensation, yaw drifts when the vehicle is tilted
 */
static float mag_to_yaw(const vec3f_t *mag, float roll, float pitch)
{
    /* Rotate magnetometer vector to horizontal plane (tilt compensation) */
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
     * First call: set initial attitude from accelerometer (+magnetometer)
     * Gyro only provides relative changes, so an initial reference is needed
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
     * Step 1: Predict quaternion via gyro integration
     *
     * Small rotation quaternion from angular rate w = (gx, gy, gz):
     * dq = (1, wx*dt/2, wy*dt/2, wz*dt/2) (first-order approximation)
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
     * Step 2: Compute reference roll/pitch from accelerometer
     */
    float accel_roll, accel_pitch;
    accel_to_roll_pitch(accel, &accel_roll, &accel_pitch);

    /*
     * Step 3: Extract euler angles from gyro prediction
     *
     * NOTE: fusion is done in Euler space (quat->euler->fuse->quat).
     * This reintroduces gimbal lock near pitch ±90°.
     * A proper implementation would use quaternion SLERP:
     *   q_accel = euler_to_quat(accel_roll, accel_pitch, gyro_yaw)
     *   q_fused = slerp(q_accel, q_gyro, alpha)
     * Kept as Euler fusion for simplicity; use EKF for production.
     */
    float gyro_roll, gyro_pitch, gyro_yaw;
    quat_to_euler(&q_gyro, &gyro_roll, &gyro_pitch, &gyro_yaw);

    /*
     * Step 4: Complementary filter fusion
     * roll  = a * gyro_roll  + (1-a) * accel_roll
     * pitch = a * gyro_pitch + (1-a) * accel_pitch
     * yaw: corrected with magnetometer if available, gyro-only otherwise
     */
    float alpha = state->alpha;
    float fused_roll = alpha * gyro_roll + (1.0f - alpha) * accel_roll;
    float fused_pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_pitch;
    float fused_yaw = gyro_yaw;

    if (mag != NULL)
    {
        float mag_yaw = mag_to_yaw(mag, fused_roll, fused_pitch);

        /* Wrap yaw difference to [-pi, pi] before interpolation.
         * Without this, 179° vs -179° averages to ~0° instead of ±180°. */
        float yaw_diff = mag_yaw - gyro_yaw;
        if (yaw_diff > 3.14159265f)  yaw_diff -= 6.28318530f;
        if (yaw_diff < -3.14159265f) yaw_diff += 6.28318530f;
        fused_yaw = gyro_yaw + (1.0f - alpha) * yaw_diff;
    }

    /*
     * Step 5: Store fused euler angles as quaternion
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
