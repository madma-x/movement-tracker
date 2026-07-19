#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include <stdint.h>
#include <string.h>

#define KALMAN3_SIZE 3

/* 3-state Kalman filter: state = [position, velocity, acceleration] */
typedef struct {
    float f[KALMAN3_SIZE * KALMAN3_SIZE];  /* state transition matrix */
    float q[KALMAN3_SIZE * KALMAN3_SIZE];  /* process noise covariance */
    float p[KALMAN3_SIZE * KALMAN3_SIZE];  /* error covariance */
    float x[KALMAN3_SIZE];                 /* [position, velocity, acceleration] */
} kalman3_t;

void kalman3_init(kalman3_t *kf, float q00, float q11, float q22);
void kalman3_reset(kalman3_t *kf);
void kalman3_predict(kalman3_t *kf, float dt);
void kalman3_updateVel(kalman3_t *kf, float vel, float velVar);
void kalman3_updateAcc(kalman3_t *kf, float acc, float accVar);

#endif /* INC_KALMAN_H_ */
