/**
 * @file quaternion.h
 * @brief Quaternion math utilities for attitude representation
 *
 * AHRS, EKF, 제어기 등에서 공통으로 사용하는 쿼터니언 연산.
 * ZYX 오일러 각도 순서 (yaw → pitch → roll).
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
 * 두 회전의 합성. 교환법칙 성립하지 않음 (a*b ≠ b*a)
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

#endif /* QUATERNION_H */
