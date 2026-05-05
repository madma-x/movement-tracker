#include "kalman.h"
#include <string.h>
#include <math.h>

/* Helper: 6x6 matrix operations */
static void mat6x6_mult(const float *a, const float *b, float *result) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 6; k++) {
                sum += a[i*6+k] * b[k*6+j];
            }
            result[i*6+j] = sum;
        }
    }
}

static void mat6x6_add(const float *a, const float *b, float *result) {
    for (int i = 0; i < 36; i++) {
        result[i] = a[i] + b[i];
    }
}

static void mat2x2_inv(float *m, float *inv) {
    /* 2x2 inverse: [[a,b],[c,d]] -> det = ad-bc */
    float det = m[0]*m[3] - m[1]*m[2];
    if (fabs(det) < 1e-8f) return;
    
    inv[0] = m[3] / det;
    inv[1] = -m[1] / det;
    inv[2] = -m[2] / det;
    inv[3] = m[0] / det;
}

/**
 * @brief Initialize Kalman filter with identity covariance
 */
void kalman_init(kalman_filter_t *kf) {
    kf->state.x = 0.0f;
    kf->state.y = 0.0f;
    kf->state.vx = 0.0f;
    kf->state.vy = 0.0f;
    kf->state.bx = 0.0f;
    kf->state.by = 0.0f;
    
    /* Initialize covariance to identity scaled by 10 (high uncertainty) */
    memset(kf->cov.data, 0, sizeof(kf->cov.data));
    kf->cov.data[0] = 10.0f;    /* x variance */
    kf->cov.data[7] = 10.0f;    /* y variance */
    kf->cov.data[14] = 10.0f;   /* vx variance */
    kf->cov.data[21] = 10.0f;   /* vy variance */
    kf->cov.data[28] = 0.5f;    /* bx variance */
    kf->cov.data[35] = 0.5f;    /* by variance */
    
    kf->last_update_us = 0;
}

/**
 * @brief Prediction step: propagate state with IMU acceleration
 * State vector: [x, y, vx, vy, bx, by]
 * Dynamics: x(k+1) = x(k) + vx(k)*dt + 0.5*ax*dt²
 *           y(k+1) = y(k) + vy(k)*dt + 0.5*ay*dt²
 *           vx(k+1) = vx(k) + ax*dt
 *           vy(k+1) = vy(k) + ay*dt
 *           bx(k+1) = bx(k)
 *           by(k+1) = by(k)
 */
void kalman_predict(kalman_filter_t *kf, float ax, float ay, uint32_t now_us) {
    float dt = (now_us - kf->last_update_us) * 1e-6f;  /* Convert us to seconds */
    if (dt > 1.0f) dt = 1.0f;  /* Cap dt to 1 second */
    if (dt < 0.0f) return;     /* Ignore negative dt */
    
    kf->last_update_us = now_us;
    
    if (dt == 0.0f) return;
    
    /* State transition matrix F (6x6) */
    float F[36] = {
        1, 0, dt, 0, -0.5f*dt*dt, 0,
        0, 1, 0, dt, 0, -0.5f*dt*dt,
        0, 0, 1, 0, -dt, 0,
        0, 0, 0, 1, 0, -dt,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1
    };

    /* Process noise matrix Q (diagonal, scaled by dt) */
    float Q[36];
    memset(Q, 0, sizeof(Q));
    Q[0] = KALMAN_Q_POS * dt * dt;
    Q[7] = KALMAN_Q_POS * dt * dt;
    Q[14] = KALMAN_Q_VEL * dt * dt;
    Q[21] = KALMAN_Q_VEL * dt * dt;
    Q[28] = KALMAN_Q_BIAS * dt;
    Q[35] = KALMAN_Q_BIAS * dt;

    /* Propagate state using bias-compensated acceleration */
    float ax_corr = ax - kf->state.bx;
    float ay_corr = ay - kf->state.by;
    kalman_state_t s_new;
    s_new.x = kf->state.x + kf->state.vx * dt + 0.5f * ax_corr * dt * dt;
    s_new.y = kf->state.y + kf->state.vy * dt + 0.5f * ay_corr * dt * dt;
    s_new.vx = kf->state.vx + ax_corr * dt;
    s_new.vy = kf->state.vy + ay_corr * dt;
    s_new.bx = kf->state.bx;
    s_new.by = kf->state.by;
    kf->state = s_new;

    /* Propagate covariance: P = F*P*F' + Q */
    float FP[36];
    float FPFt[36];
    mat6x6_mult(F, kf->cov.data, FP);

    float Ft[36];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            Ft[i*6+j] = F[j*6+i];
        }
    }
    mat6x6_mult(FP, Ft, FPFt);
    mat6x6_add(FPFt, Q, kf->cov.data);
}

/**
 * @brief Update step: correct with PAA measurement (position delta)
 * Measurement model: z = [x, y] (direct observation of position)
 */
void kalman_update_position(kalman_filter_t *kf, float z_x, float z_y, float dt) {
    /* Measurement matrix H: select x and y from 6-state */
    /* Measurement covariance R (2x2 diagonal) */
    float R[4] = {
        KALMAN_R_POS, 0,
        0, KALMAN_R_POS
    };
    
    /* Innovation: y = z - H*x */
    float innov_x = z_x - kf->state.x;
    float innov_y = z_y - kf->state.y;
    
    /* Innovation covariance: S = H*P*H' + R (2x2) */
    float S[4] = {
        kf->cov.data[0] + R[0], kf->cov.data[1],
        kf->cov.data[6], kf->cov.data[7] + R[3]
    };
    
    /* Kalman gain: K = P*H'*S^-1 (6x2) */
    float S_inv[4];
    mat2x2_inv(S, S_inv);

    float K[12];
    for (int i = 0; i < 6; i++) {
        float p0 = kf->cov.data[i*6 + 0];
        float p1 = kf->cov.data[i*6 + 1];
        K[i*2 + 0] = p0 * S_inv[0] + p1 * S_inv[2];
        K[i*2 + 1] = p0 * S_inv[1] + p1 * S_inv[3];
    }

    /* State update: x = x + K*innovation */
    kf->state.x  += K[0]  * innov_x + K[1]  * innov_y;
    kf->state.y  += K[2]  * innov_x + K[3]  * innov_y;
    kf->state.vx += K[4]  * innov_x + K[5]  * innov_y;
    kf->state.vy += K[6]  * innov_x + K[7]  * innov_y;
    kf->state.bx += K[8]  * innov_x + K[9]  * innov_y;
    kf->state.by += K[10] * innov_x + K[11] * innov_y;

    /* Covariance update – Joseph form: P = (I-KH)*P*(I-KH)^T + K*R*K^T
     * Numerically stable: keeps P symmetric and positive-definite. */
    float I_KH[36];
    memset(I_KH, 0, sizeof(I_KH));
    for (int i = 0; i < 6; i++) {
        I_KH[i*6+i] = 1.0f;
        I_KH[i*6+0] -= K[i*2 + 0];
        I_KH[i*6+1] -= K[i*2 + 1];
    }

    /* (I-KH)*P */
    float IKH_P[36];
    mat6x6_mult(I_KH, kf->cov.data, IKH_P);

    /* (I-KH)*P*(I-KH)^T */
    float I_KH_t[36];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            I_KH_t[i*6+j] = I_KH[j*6+i];
        }
    }
    float P_new[36];
    mat6x6_mult(IKH_P, I_KH_t, P_new);

    /* + K*R*K^T  (K is 6x2, R is 2x2) */
    float KR[12];
    for (int i = 0; i < 6; i++) {
        KR[i*2 + 0] = K[i*2+0]*R[0] + K[i*2+1]*R[2];
        KR[i*2 + 1] = K[i*2+0]*R[1] + K[i*2+1]*R[3];
    }
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P_new[i*6+j] += KR[i*2+0]*K[j*2+0] + KR[i*2+1]*K[j*2+1];
        }
    }

    memcpy(kf->cov.data, P_new, sizeof(P_new));
}

/**
 * @brief Update step using measured velocity (from optical flow: wx_mm/dt, wy_mm/dt).
 * H_vel = [[0,0,1,0,0,0],[0,0,0,1,0,0]]
 * Directly constrains vx/vy in each optical update cycle, preventing accel-noise
 * from accumulating into the velocity state between position corrections.
 */
void kalman_update_velocity(kalman_filter_t *kf, float vx, float vy) {
    /* Measurement noise for velocity (m/s)^2 */
    float R[4] = {
        KALMAN_R_VEL, 0,
        0,            KALMAN_R_VEL
    };

    /* Innovation: z - H*x */
    float innov_vx = vx - kf->state.vx;
    float innov_vy = vy - kf->state.vy;

    /* S = H_vel * P * H_vel^T + R  →  picks rows/cols 2,3 of P */
    float S[4] = {
        kf->cov.data[2*6+2] + R[0], kf->cov.data[2*6+3],
        kf->cov.data[3*6+2],        kf->cov.data[3*6+3] + R[3]
    };

    float S_inv[4];
    mat2x2_inv(S, S_inv);

    /* K = P * H_vel^T * S_inv  →  K is 6x2, H_vel^T selects columns 2,3 of P */
    float K[12];
    for (int i = 0; i < 6; i++) {
        float p0 = kf->cov.data[i*6 + 2];
        float p1 = kf->cov.data[i*6 + 3];
        K[i*2 + 0] = p0 * S_inv[0] + p1 * S_inv[2];
        K[i*2 + 1] = p0 * S_inv[1] + p1 * S_inv[3];
    }

    /* State update */
    kf->state.x  += K[0]  * innov_vx + K[1]  * innov_vy;
    kf->state.y  += K[2]  * innov_vx + K[3]  * innov_vy;
    kf->state.vx += K[4]  * innov_vx + K[5]  * innov_vy;
    kf->state.vy += K[6]  * innov_vx + K[7]  * innov_vy;
    kf->state.bx += K[8]  * innov_vx + K[9]  * innov_vy;
    kf->state.by += K[10] * innov_vx + K[11] * innov_vy;

    /* Covariance update – Joseph form */
    float I_KH[36];
    memset(I_KH, 0, sizeof(I_KH));
    for (int i = 0; i < 6; i++) {
        I_KH[i*6+i] = 1.0f;
        I_KH[i*6+2] -= K[i*2 + 0];
        I_KH[i*6+3] -= K[i*2 + 1];
    }

    float IKH_P[36];
    mat6x6_mult(I_KH, kf->cov.data, IKH_P);

    float I_KH_t[36];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            I_KH_t[i*6+j] = I_KH[j*6+i];
        }
    }
    float P_new[36];
    mat6x6_mult(IKH_P, I_KH_t, P_new);

    float KR[12];
    for (int i = 0; i < 6; i++) {
        KR[i*2 + 0] = K[i*2+0]*R[0] + K[i*2+1]*R[2];
        KR[i*2 + 1] = K[i*2+0]*R[1] + K[i*2+1]*R[3];
    }
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P_new[i*6+j] += KR[i*2+0]*K[j*2+0] + KR[i*2+1]*K[j*2+1];
        }
    }

    memcpy(kf->cov.data, P_new, sizeof(P_new));
}

/**
 * @brief Get current state
 */
kalman_state_t kalman_get_state(kalman_filter_t *kf) {
    return kf->state;
}

/**
 * @brief Reset filter
 */
void kalman_reset(kalman_filter_t *kf) {
    kalman_init(kf);
}
