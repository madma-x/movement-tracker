#ifndef INC_COMPENSATION_H_
#define INC_COMPENSATION_H_

#include <stdint.h>

/* Quaternion type */
typedef struct {
    float q0, q1, q2, q3;
} quaternion_t;

/**
 * @brief Quaternion normalization
 */
void quat_normalize(quaternion_t *q);

/**
 * @brief Quaternion conjugate (inverse for unit quaternions)
 */
quaternion_t quat_conjugate(quaternion_t q);

/**
 * @brief Quaternion multiplication
 */
quaternion_t quat_multiply(quaternion_t q1, quaternion_t q2);

/**
 * @brief Rotate a 3D vector by a quaternion
 * Applies: v_rotated = q * v * q_conjugate
 */
void quat_rotate_vector(quaternion_t q, float *vx, float *vy, float *vz);

/**
 * @brief Compensate PAA translation delta for sensor rotation
 * Converts PAA 2D delta (sensor frame) to world frame by rotating through quaternion
 * 
 * @param dx_mm: PAA delta X in sensor frame (mm)
 * @param dy_mm: PAA delta Y in sensor frame (mm)
 * @param q: Current quaternion (world orientation)
 * @param wx: Output world-frame X translation (mm)
 * @param wy: Output world-frame Y translation (mm)
 */
void compensate_paa_delta(float dx_mm, float dy_mm, quaternion_t q, 
                          float *wx, float *wy);

#endif /* INC_COMPENSATION_H_ */
