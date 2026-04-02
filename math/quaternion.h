/**
 * @file quaternion.h
 * @brief Quaternion math utilities for attitude representation
 *
 * Quaternion operations shared by AHRS, EKF, controllers, etc.
 * ZYX Euler angle order (yaw -> pitch -> roll).
 */

#ifndef QUATERNION_H
#define QUATERNION_H

#include "sensor_types.h"

/**
 * @brief Normalize quaternion to unit length
 * @param q  Quaternion (in-place)
 */
void quat_normalize(quat_t *q);

/**
 * @brief Quaternion multiplication: result = a * b
 * @param a  First quaternion
 * @param b  Second quaternion
 * @return Product quaternion
 *
 * Composition of two rotations. Not commutative (a*b != b*a)
 */
quat_t quat_multiply(const quat_t *a, const quat_t *b);

/**
 * @brief Quaternion → Euler angles (ZYX order)
 * @param q      Input quaternion
 * @param roll   Output: roll in rad
 * @param pitch  Output: pitch in rad
 * @param yaw    Output: yaw in rad
 */
void quat_to_euler(const quat_t *q, float *roll, float *pitch, float *yaw);

/**
 * @brief Euler angles → Quaternion (ZYX order)
 * @param roll   Roll in rad
 * @param pitch  Pitch in rad
 * @param yaw    Yaw in rad
 * @return Quaternion
 */
quat_t euler_to_quat(float roll, float pitch, float yaw);

/**
 * @brief Compute attitude error vector between two quaternions
 * @param target   Target attitude
 * @param current  Current attitude
 * @param error    Output: error vector [roll, pitch, yaw] (rad)
 *
 * q_err = q_current^(-1) * q_target
 * error = 2 * sign(q_err.w) * [q_err.x, q_err.y, q_err.z]
 * Sign flip ensures shortest rotation path.
 */
void quat_attitude_error(const quat_t *target, const quat_t *current, vec3f_t *error);

#endif /* QUATERNION_H */
