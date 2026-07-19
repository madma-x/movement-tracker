#include "kalman.h"

/* Flat 3x3 index helpers */
#define I00 0
#define I01 1
#define I02 2
#define I10 3
#define I11 4
#define I12 5
#define I20 6
#define I21 7
#define I22 8

void kalman3_init(kalman3_t *kf, float q00, float q11, float q22)
{
    memset(kf, 0, sizeof(kalman3_t));
    /* Identity state transition */
    kf->f[I00] = 1.0f;
    kf->f[I11] = 1.0f;
    kf->f[I22] = 1.0f;
    /* Diagonal process noise */
    kf->q[I00] = q00;
    kf->q[I11] = q11;
    kf->q[I22] = q22;
}

void kalman3_reset(kalman3_t *kf)
{
    memset(kf->x, 0, sizeof(kf->x));
    memset(kf->p, 0, sizeof(kf->p));
}

void kalman3_predict(kalman3_t *kf, float dt)
{
    kf->f[I01] = dt;
    kf->f[I12] = dt;

    /* State: pos += vel*dt, vel += acc*dt */
    kf->x[0] += kf->x[1] * dt;
    kf->x[1] += kf->x[2] * dt;

    /* Covariance: P = F*P*F' + Q (dt^2 terms dropped as negligible) */
    kf->p[I00] += (kf->p[I01] + kf->p[I10]) * dt + kf->q[I00];
    kf->p[I01] += (kf->p[I02] + kf->p[I11]) * dt;
    kf->p[I02] += kf->p[I12] * dt;
    kf->p[I10] += (kf->p[I11] + kf->p[I20]) * dt;
    kf->p[I11] += (kf->p[I12] + kf->p[I21]) * dt + kf->q[I11];
    kf->p[I12] += kf->p[I22] * dt;
    kf->p[I20] += kf->p[I21] * dt;
    kf->p[I21] += kf->p[I22] * dt;
    kf->p[I22] += kf->q[I22];
}

static void kalman3_update(kalman3_t *kf, float measurement, float variance, uint8_t idx)
{
    float y    = measurement - kf->x[idx];
    float sInv = 1.0f / (kf->p[KALMAN3_SIZE * idx + idx] + variance);

    float k[KALMAN3_SIZE];
    k[0] = kf->p[I00 + idx] * sInv;
    k[1] = kf->p[I10 + idx] * sInv;
    k[2] = kf->p[I20 + idx] * sInv;

    kf->x[0] += k[0] * y;
    kf->x[1] += k[1] * y;
    kf->x[2] += k[2] * y;

    for (int i = 0; i < KALMAN3_SIZE; i++) {
        for (int j = 0; j < KALMAN3_SIZE; j++) {
            kf->p[i * KALMAN3_SIZE + j] -= kf->p[idx * KALMAN3_SIZE + j] * k[i];
        }
    }
}

void kalman3_updateVel(kalman3_t *kf, float vel, float velVar)
{
    kalman3_update(kf, vel, velVar, 1);
}

void kalman3_updateAcc(kalman3_t *kf, float acc, float accVar)
{
    kalman3_update(kf, acc, accVar, 2);
}
