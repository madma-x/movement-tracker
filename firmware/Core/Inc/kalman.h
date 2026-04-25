#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include <stdint.h>

/* Tuning parameters for Kalman filter */
#define KALMAN_Q_POS   1e-4f   /* Process noise for position (m²/s⁴) small = trust model */
#define KALMAN_Q_VEL   1e-3f   /* Process noise for velocity (m/s²) */
#define KALMAN_R_POS   0.01f   /* Measurement noise for position (m²) bigger = less trust measured */
#define KALMAN_R_VEL   0.05f   /* Measurement noise for velocity (m²/s²) */
#define KALMAN_GRAVITY 9.81f   /* Gravitational acceleration (m/s²) */

/* State: [x, y, vx, vy] in meters and m/s */
typedef struct {
    float x, y;           /* Position (m) */
    float vx, vy;         /* Velocity (m/s) */
} kalman_state_t;

/* Covariance matrix (4x4 for 4D state, stored as 16 floats) */
typedef struct {
    float data[16];  /* Symmetric, stored row-major */
} kalman_cov_t;

typedef struct {
    kalman_state_t state;
    kalman_cov_t cov;
    uint32_t last_update_us;  /* Last update timestamp */
} kalman_filter_t;

/**
 * @brief Initialize Kalman filter
 */
void kalman_init(kalman_filter_t *kf);

/**
 * @brief Prediction step using IMU acceleration
 * Propagates state and covariance forward in time
 * 
 * @param kf: Kalman filter state
 * @param ax, ay: Linear accelerations (m/s²) in world frame
 * @param now_us: Current timestamp (microseconds)
 */
void kalman_predict(kalman_filter_t *kf, float ax, float ay, uint32_t now_us);

/**
 * @brief Update step using measured position
 * Corrects state based on PAA measurement (compensated deltax, deltay)
 * 
 * @param kf: Kalman filter state
 * @param z_x, z_y: Measured position delta (m)
 * @param dt: Time step (s) from last predict
 */
void kalman_update_position(kalman_filter_t *kf, float z_x, float z_y, float dt);

/**
 * @brief Get current state
 */
kalman_state_t kalman_get_state(kalman_filter_t *kf);

/**
 * @brief Reset filter (zero position and velocity, reset covariance)
 */
void kalman_reset(kalman_filter_t *kf);

#endif /* INC_KALMAN_H_ */
