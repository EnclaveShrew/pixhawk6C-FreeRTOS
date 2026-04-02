/**
 * @file quaternion.c
 * @brief Quaternion math utilities implementation
 */

#include "quaternion.h"
#include <math.h>

void quat_normalize(quat_t *q)
{
    float n = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (n < 1e-8f)
    {
        return;
    }
    float inv = 1.0f / n;
    q->w *= inv;
    q->x *= inv;
    q->y *= inv;
    q->z *= inv;
}

quat_t quat_multiply(const quat_t *a, const quat_t *b)
{
    quat_t r;
    r.w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    r.x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    r.y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    r.z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
    return r;
}

void quat_to_euler(const quat_t *q, float *roll, float *pitch, float *yaw)
{
    /* Roll (X-axis) */
    float sinr = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    *roll = atan2f(sinr, cosr);

    /* Pitch (Y-axis) -- asin clamping for numerical stability */
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (sinp > 1.0f)
    {
        sinp = 1.0f;
    }
    if (sinp < -1.0f)
    {
        sinp = -1.0f;
    }
    *pitch = asinf(sinp);

    /* Yaw (Z-axis) */
    float siny = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    *yaw = atan2f(siny, cosy);
}

quat_t euler_to_quat(float roll, float pitch, float yaw)
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    quat_t q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

void quat_attitude_error(const quat_t *target, const quat_t *current, vec3f_t *error)
{
    /* Conjugate of unit quaternion = inverse */
    quat_t q_inv;
    q_inv.w =  current->w;
    q_inv.x = -current->x;
    q_inv.y = -current->y;
    q_inv.z = -current->z;

    quat_t q_err = quat_multiply(&q_inv, target);

    /* Ensure shortest path: flip sign if w < 0 */
    float sign = (q_err.w >= 0.0f) ? 1.0f : -1.0f;

    error->x = 2.0f * sign * q_err.x;
    error->y = 2.0f * sign * q_err.y;
    error->z = 2.0f * sign * q_err.z;
}
