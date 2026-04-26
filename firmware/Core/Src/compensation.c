#include "compensation.h"
#include <math.h>

/**
 * @brief Normalize a quaternion to unit length
 */
void quat_normalize(quaternion_t *q) {
    float norm = sqrtf(q->q0 * q->q0 + q->q1 * q->q1 + 
                       q->q2 * q->q2 + q->q3 * q->q3);
    
    if (norm > 0.0f) {
        q->q0 /= norm;
        q->q1 /= norm;
        q->q2 /= norm;
        q->q3 /= norm;
    }
}

/**
 * @brief Return the conjugate of a quaternion
 */
quaternion_t quat_conjugate(quaternion_t q) {
    quaternion_t conj;
    conj.q0 = q.q0;
    conj.q1 = -q.q1;
    conj.q2 = -q.q2;
    conj.q3 = -q.q3;
    return conj;
}

/**
 * @brief Multiply two quaternions: result = q1 * q2
 */
quaternion_t quat_multiply(quaternion_t q1, quaternion_t q2) {
    quaternion_t result;
    
    result.q0 = q1.q0 * q2.q0 - q1.q1 * q2.q1 - q1.q2 * q2.q2 - q1.q3 * q2.q3;
    result.q1 = q1.q0 * q2.q1 + q1.q1 * q2.q0 + q1.q2 * q2.q3 - q1.q3 * q2.q2;
    result.q2 = q1.q0 * q2.q2 - q1.q1 * q2.q3 + q1.q2 * q2.q0 + q1.q3 * q2.q1;
    result.q3 = q1.q0 * q2.q3 + q1.q1 * q2.q2 - q1.q2 * q2.q1 + q1.q3 * q2.q0;
    
    return result;
}

/**
 * @brief Rotate a 3D vector by a quaternion
 * v_rotated = q * v * q_conjugate
 * where v is represented as a quaternion with q0=0, (q1,q2,q3)=vector
 */
void quat_rotate_vector(quaternion_t q, float *vx, float *vy, float *vz) {
    /* Create quaternion from vector: (0, vx, vy, vz) */
    quaternion_t v_quat;
    v_quat.q0 = 0.0f;
    v_quat.q1 = *vx;
    v_quat.q2 = *vy;
    v_quat.q3 = *vz;
    
    /* Compute q * v */
    quaternion_t qv = quat_multiply(q, v_quat);
    
    /* Compute q * v * q_conjugate */
    quaternion_t q_conj = quat_conjugate(q);
    quaternion_t result = quat_multiply(qv, q_conj);
    
    /* Extract vector part */
    *vx = result.q1;
    *vy = result.q2;
    *vz = result.q3;
}

/**
 * @brief Compensate PAA delta for rotation
 * Takes PAA motion in sensor frame and rotates it to world frame
 */
void compensate_paa_delta(float dx_mm, float dy_mm, quaternion_t q,
                          float *wx, float *wy) {
    /*
     * For planar odometry we only want heading compensation.
     * Using the full 3D quaternion and then discarding Z shrinks X/Y when
     * the board has pitch/roll, which corrupts ground-plane distance.
     */
    float yaw = atan2f(2.0f * (q.q0 * q.q3 + q.q1 * q.q2),
                       1.0f - 2.0f * (q.q2 * q.q2 + q.q3 * q.q3));
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    *wx = cy * dx_mm - sy * dy_mm;
    *wy = sy * dx_mm + cy * dy_mm;
}
