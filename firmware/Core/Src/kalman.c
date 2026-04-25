#include "kalman.h"
#include <string.h>
#include <math.h>

/* Helper: 4x4 matrix operations */
static void mat4x4_mult(float *a, float *b, float *result) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result[i*4+j] = 0;
            for (int k = 0; k < 4; k++) {
                result[i*4+j] += a[i*4+k] * b[k*4+j];
            }
        }
    }
}

static void mat4x4_add(float *a, float *b, float *result) {
    for (int i = 0; i < 16; i++) {
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
    
    /* Initialize covariance to identity scaled by 10 (high uncertainty) */
    memset(kf->cov.data, 0, sizeof(kf->cov.data));
    kf->cov.data[0] = 10.0f;   /* x variance */
    kf->cov.data[5] = 10.0f;   /* y variance */
    kf->cov.data[10] = 10.0f;  /* vx variance */
    kf->cov.data[15] = 10.0f;  /* vy variance */
    
    kf->last_update_us = 0;
}

/**
 * @brief Prediction step: propagate state with IMU acceleration
 * State vector: [x, y, vx, vy]
 * Dynamics: x(k+1) = x(k) + vx(k)*dt + 0.5*ax*dt²
 *           y(k+1) = y(k) + vy(k)*dt + 0.5*ay*dt²
 *           vx(k+1) = vx(k) + ax*dt
 *           vy(k+1) = vy(k) + ay*dt
 */
void kalman_predict(kalman_filter_t *kf, float ax, float ay, uint32_t now_us) {
    float dt = (now_us - kf->last_update_us) * 1e-6f;  /* Convert us to seconds */
    if (dt > 1.0f) dt = 1.0f;  /* Cap dt to 1 second */
    if (dt < 0.0f) return;     /* Ignore negative dt */
    
    kf->last_update_us = now_us;
    
    if (dt == 0.0f) return;
    
    /* State transition matrix F (4x4) */
    float F[16] = {
        1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    
    /* Process noise matrix Q (diagonal, scaled by dt²) */
    float Q[16];
    memset(Q, 0, sizeof(Q));
    Q[0] = KALMAN_Q_POS * dt * dt;   /* x process noise */
    Q[5] = KALMAN_Q_POS * dt * dt;   /* y process noise */
    Q[10] = KALMAN_Q_VEL * dt * dt;  /* vx process noise */
    Q[15] = KALMAN_Q_VEL * dt * dt;  /* vy process noise */
    
    /* Propagate state: s = F * s */
    kalman_state_t s_new;
    s_new.x = F[0]*kf->state.x + F[2]*kf->state.vx + 0.5f*ax*dt*dt;
    s_new.y = F[5]*kf->state.y + F[7]*kf->state.vy + 0.5f*ay*dt*dt;
    s_new.vx = F[10]*kf->state.vx + ax*dt;
    s_new.vy = F[15]*kf->state.vy + ay*dt;
    
    kf->state = s_new;
    
    /* Propagate covariance: P = F*P*F' + Q (simplified, diagonal approximation) */
    float P_new[16];
    memset(P_new, 0, sizeof(P_new));
    
    /* Diagonal elements only (simplified for 4-state) */
    P_new[0] = kf->cov.data[0] + kf->cov.data[10]*dt*dt + Q[0];
    P_new[5] = kf->cov.data[5] + kf->cov.data[15]*dt*dt + Q[5];
    P_new[10] = kf->cov.data[10] + Q[10];
    P_new[15] = kf->cov.data[15] + Q[15];
    
    memcpy(kf->cov.data, P_new, sizeof(P_new));
}

/**
 * @brief Update step: correct with PAA measurement (position delta)
 * Measurement model: z = [x, y] (direct observation of position)
 */
void kalman_update_position(kalman_filter_t *kf, float z_x, float z_y, float dt) {
    /* Measurement matrix H: measure x and y directly */
    float H[8] = {
        1, 0, 0, 0,  /* Observe x */
        0, 1, 0, 0   /* Observe y */
    };
    
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
        kf->cov.data[4], kf->cov.data[5] + R[3]
    };
    
    /* Kalman gain: K = P*H'*S^-1 (4x2) */
    float S_inv[4];
    mat2x2_inv(S, S_inv);
    
    /* K = P*H'*S_inv, simplified for 4x2*2x2 */
    float K[8] = {
        kf->cov.data[0]*S_inv[0] + kf->cov.data[1]*S_inv[2], 
        kf->cov.data[0]*S_inv[1] + kf->cov.data[1]*S_inv[3],
        kf->cov.data[4]*S_inv[0] + kf->cov.data[5]*S_inv[2],
        kf->cov.data[4]*S_inv[1] + kf->cov.data[5]*S_inv[3],
        kf->cov.data[8]*S_inv[0] + kf->cov.data[9]*S_inv[2],
        kf->cov.data[8]*S_inv[1] + kf->cov.data[9]*S_inv[3],
        kf->cov.data[12]*S_inv[0] + kf->cov.data[13]*S_inv[2],
        kf->cov.data[12]*S_inv[1] + kf->cov.data[13]*S_inv[3]
    };
    
    /* State update: x = x + K*innovation */
    kf->state.x += K[0]*innov_x + K[1]*innov_y;
    kf->state.y += K[2]*innov_x + K[3]*innov_y;
    kf->state.vx += K[4]*innov_x + K[5]*innov_y;
    kf->state.vy += K[6]*innov_x + K[7]*innov_y;
    
    /* Covariance update: P = (I - K*H)*P */
    float I_KH[16];
    memset(I_KH, 0, sizeof(I_KH));
    for (int i = 0; i < 4; i++) I_KH[i*4+i] = 1.0f;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            I_KH[i*4+j] -= K[i*2]*H[j] + K[i*2+1]*H[4+j];
        }
    }
    
    float P_new[16];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P_new[i*4+j] = 0;
            for (int k = 0; k < 4; k++) {
                P_new[i*4+j] += I_KH[i*4+k] * kf->cov.data[k*4+j];
            }
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
