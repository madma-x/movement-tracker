#include "runtime.h"
#include "main.h"

#include "PAA5163.h"
#include "lsm6dsv16x_reg.h"
#include "lsm6dsv16x_hal_glue.h"
#include "micros.h"
#include "kalman.h"
#include "lut.h"
#include "output.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

typedef struct { float q0, q1, q2, q3; } quaternion_t;
static inline void quat_normalize(quaternion_t *q) {
    float mag = sqrtf(q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3);
    if (mag > 0.0f) { q->q0 /= mag; q->q1 /= mag; q->q2 /= mag; q->q3 /= mag; }
}

/* ── Tuning ─────────────────────────────────────────────────────────────── */
#define DEBUG_UART            1
#define ENABLE_BUS_OUTPUT     1
#define BUS_OUTPUT_PERIOD_MS  20U   /* 50 Hz */
#define DEBUG_PRINT_PERIOD_MS 50U

/* Set to 1 to apply bilinear LUT speed calibration, 0 to bypass */
#define ENABLE_LUT  1

/* Kalman measurement noise variances */
#define KF_VEL_VAR   6e-3f
#define KF_GZ_VAR    1e-3f

/* FIFO / timing */
#define GYRO_FIFO_RATE_HZ        960.0f
#define GYRO_FIFO_DT_S_FALLBACK  (1.0f / GYRO_FIFO_RATE_HZ)
#define GYRO_FIFO_SAMPLE_US      (1000000.0f / GYRO_FIFO_RATE_HZ)
#define IMU_TIMESTAMP_TICK_US    25.0f
#define GYRO_MAX_DT_S            0.01f
#define PAA_READ_PERIOD_US       2083U

/* Optical distance calibration */
#define PAA_DISTANCE_SCALE 1.0f

/* ── Sensor instances ────────────────────────────────────────────────────── */
paa5163_t paa = {
    .spi        = &hspi1,
    .NCS_Port   = CS_PAA_GPIO_Port,
    .NCS_Pin    = CS_PAA_Pin,
    .NRST_Port  = RST_PAA_GPIO_Port,
    .NRST_Pin   = RST_PAA_Pin,
    .invert_x   = 1,
    /* resolution = 0 → paaInit uses DEFAULT_RESOLUTION (20000 CPI) */
};

lsm6dsv16x_ctx_t lsm6_ctx;

/* ── Kalman filters ───────────────────────────────────────────────────────── */
static kalman3_t xKf, yKf, hKf;

/* ── SFLP quaternion (kept for debug display only) ───────────────────────── */
static quaternion_t imu_q = { .q0=1.0f, .q1=0.0f, .q2=0.0f, .q3=0.0f };
static float yaw_sflp_rad = 0.0f;

/* ── Gyro / timing state ─────────────────────────────────────────────────── */
static float gz_bias_dps       = 0.0f;
static float latest_gz_dps     = 0.0f;
static float latest_gz_raw_dps = 0.0f;
static bool  gyro_ts_valid     = false;
static float fifo_ts_us        = 0.0f;
static float next_gyro_ts_us   = 0.0f;
static uint32_t fifo_ts_host_us = 0U;

/* ── PAA state ───────────────────────────────────────────────────────────── */
static float   cpi_to_mm     = 0.0f;
static int16_t last_dx_cpi   = 0;
static int16_t last_dy_cpi   = 0;
static float   dbg_dx_mm_sum = 0.0f;
static float   dbg_dy_mm_sum = 0.0f;
static float   last_vx       = 0.0f;
static float   last_vy       = 0.0f;

/* ── helpers ─────────────────────────────────────────────────────────────── */
static float wrap_pi(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/* ── setup ───────────────────────────────────────────────────────────────── */
void setup(void) {
    if (DEBUG_UART) printf("=== Movement Tracker Startup ===\n");

    lsm6dsv16x_ctx_init(&lsm6_ctx, &hspi1);

    uint8_t whoami = 0;
    lsm6dsv16x_device_id_get(&lsm6_ctx, &whoami);
    if (DEBUG_UART) printf("LSM6DSV16X WHO_AM_I: 0x%02X\n", whoami);

    lsm6dsv16x_reset_set(&lsm6_ctx, 0x02);
    HAL_Delay(100);
    lsm6dsv16x_auto_increment_set(&lsm6_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_block_data_update_set(&lsm6_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_xl_data_rate_set(&lsm6_ctx, LSM6DSV16X_ODR_AT_480Hz);
    lsm6dsv16x_gy_data_rate_set(&lsm6_ctx, LSM6DSV16X_ODR_AT_960Hz);
    lsm6dsv16x_xl_mode_set(&lsm6_ctx, LSM6DSV16X_XL_HIGH_PERFORMANCE_MD);
    lsm6dsv16x_gy_mode_set(&lsm6_ctx, LSM6DSV16X_GY_HIGH_ACCURACY_ODR_MD);
    lsm6dsv16x_xl_full_scale_set(&lsm6_ctx, LSM6DSV16X_4g);
    lsm6dsv16x_gy_full_scale_set(&lsm6_ctx, LSM6DSV16X_2000dps);
    lsm6dsv16x_timestamp_set(&lsm6_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_fifo_timestamp_batch_set(&lsm6_ctx, LSM6DSV16X_TMSTMP_DEC_1);
    lsm6dsv16x_sflp_data_rate_set(&lsm6_ctx, LSM6DSV16X_SFLP_120Hz);
    lsm6dsv16x_sflp_game_rotation_set(&lsm6_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_fifo_gy_batch_set(&lsm6_ctx, LSM6DSV16X_GY_BATCHED_AT_960Hz);
    lsm6dsv16x_fifo_sflp_raw_t sflp_cfg = {0};
    sflp_cfg.game_rotation = 1;
    lsm6dsv16x_fifo_sflp_batch_set(&lsm6_ctx, sflp_cfg);
    lsm6dsv16x_fifo_mode_set(&lsm6_ctx, LSM6DSV16X_STREAM_MODE);

    lsm6dsv16x_emb_func_init_a_t emb = {0};
    lsm6dsv16x_mem_bank_set(&lsm6_ctx, LSM6DSV16X_EMBED_FUNC_MEM_BANK);
    lsm6dsv16x_read_reg(&lsm6_ctx, LSM6DSV16X_EMB_FUNC_INIT_A, (uint8_t*)&emb, 1);
    emb.sflp_game_init = 1;
    lsm6dsv16x_write_reg(&lsm6_ctx, LSM6DSV16X_EMB_FUNC_INIT_A, (uint8_t*)&emb, 1);
    lsm6dsv16x_mem_bank_set(&lsm6_ctx, LSM6DSV16X_MAIN_MEM_BANK);
    HAL_Delay(50);

    if (DEBUG_UART) printf("LSM6DSV16X init complete\n");

    if (DEBUG_UART) printf("PAA5163 Init... ");
    paa_err_t paa_ret = paaInit(&paa);
    if (DEBUG_UART) printf("%d (%s)  res=%u CPI\n",
                           paa_ret, paa_ret == paa_ok ? "OK" : "ERROR",
                           (unsigned)paa.resolution);
    if (paa_ret != paa_ok) {
        if (DEBUG_UART) printf("FATAL: PAA init failed, rebooting...\n");
        HAL_Delay(500);
        NVIC_SystemReset();
    }

    cpi_to_mm = (25.4f / (float)paa.resolution) * PAA_DISTANCE_SCALE;
    if (DEBUG_UART) printf("cpi_to_mm=%.6f  LUT=%s\n", cpi_to_mm, ENABLE_LUT ? "ON" : "OFF");

    output_init(&hi2c2);

    /* Gyro Z bias calibration */
    {
        const int CAL_N = 500;
        double gz_sum = 0.0;
        int16_t g_raw[3] = {0};
        for (int i = 0; i < CAL_N; i++) {
            HAL_Delay(2);
            lsm6dsv16x_angular_rate_raw_get(&lsm6_ctx, g_raw);
            gz_sum += lsm6dsv16x_from_fs2000_to_mdps(g_raw[2]) / 1000.0;
        }
        gz_bias_dps = (float)(gz_sum / CAL_N);
        if (DEBUG_UART) printf("Gyro Z bias: %.4f dps\n", gz_bias_dps);
    }

    /* Flush PAA counts accumulated during gyro calibration */
    paaReadMotion(&paa);
    paa.dx_cpi = 0;
    paa.dy_cpi = 0;

    /* Kalman filters */
    kalman3_init(&xKf, 1e-8f, 1e-6f, 1e-4f);
    kalman3_init(&yKf, 1e-8f, 1e-6f, 1e-4f);
    kalman3_init(&hKf, 1e-10f, 1e-2f, 1e-1f);
    xKf.p[4] = 0.1f;
    yKf.p[4] = 0.1f;

    if (DEBUG_UART) printf("Setup complete.\n");
}

/* ── loop ────────────────────────────────────────────────────────────────── */
void loop(void) {
    static uint32_t last_print_ms       = 0;
    static uint32_t last_paa_us         = 0;
    static float    last_h_sample_ts_us = 0.0f;

    output_process();

    uint32_t now_ms = HAL_GetTick();
    uint32_t now_us = micros();

    /* ── FIFO drain ─────────────────────────────────────────────────────── */
    lsm6dsv16x_fifo_status_t fifo_status = {0};
    lsm6dsv16x_fifo_status_get(&lsm6_ctx, &fifo_status);
    uint16_t budget = (uint16_t)(fifo_status.fifo_level + 8U);

    for (uint16_t n = 0; n < budget; n++) {
        lsm6dsv16x_fifo_out_raw_t raw = {0};
        if (lsm6dsv16x_fifo_out_raw_get(&lsm6_ctx, &raw) != 0) break;
        if (raw.tag == LSM6DSV16X_FIFO_EMPTY) break;

        if (raw.tag == LSM6DSV16X_TIMESTAMP_TAG) {
            uint32_t ts_raw = (uint32_t)raw.data[0]
                            | ((uint32_t)raw.data[1] << 8)
                            | ((uint32_t)raw.data[2] << 16)
                            | ((uint32_t)raw.data[3] << 24);
            fifo_ts_us      = (float)ts_raw * IMU_TIMESTAMP_TICK_US;
            next_gyro_ts_us = fifo_ts_us;
            fifo_ts_host_us = now_us;
            gyro_ts_valid   = true;

        } else if (raw.tag == LSM6DSV16X_GY_NC_TAG     ||
                   raw.tag == LSM6DSV16X_GY_NC_T_1_TAG  ||
                   raw.tag == LSM6DSV16X_GY_NC_T_2_TAG  ||
                   raw.tag == LSM6DSV16X_GY_2XC_TAG     ||
                   raw.tag == LSM6DSV16X_GY_3XC_TAG) {

            int16_t gx_r = (int16_t)((raw.data[1] << 8) | raw.data[0]);
            int16_t gy_r = (int16_t)((raw.data[3] << 8) | raw.data[2]);
            int16_t gz_r = (int16_t)((raw.data[5] << 8) | raw.data[4]);
            (void)gx_r; (void)gy_r;

            latest_gz_raw_dps = lsm6dsv16x_from_fs2000_to_mdps(gz_r) / 1000.0f;
            latest_gz_dps     = latest_gz_raw_dps - gz_bias_dps;

            float gz_rps_hkf = latest_gz_dps * ((float)M_PI / 180.0f);
            float sample_ts  = gyro_ts_valid ? next_gyro_ts_us : (float)now_us;

            if (gyro_ts_valid) {
                next_gyro_ts_us += GYRO_FIFO_SAMPLE_US;
                fifo_ts_us       = sample_ts;
                fifo_ts_host_us  = now_us;
            }

            /* Heading Kalman: per-sample FIFO timestamp so all 960 Hz gyro
               samples integrate correctly within a single FIFO drain call. */
            if (last_h_sample_ts_us > 0.0f) {
                float dt_h = (sample_ts - last_h_sample_ts_us) * 1e-6f;
                if (dt_h > 0.0f && dt_h < GYRO_MAX_DT_S) {
                    kalman3_predict(&hKf, dt_h);
                    kalman3_updateVel(&hKf, gz_rps_hkf, KF_GZ_VAR);
                    hKf.x[0] = wrap_pi(hKf.x[0]);
                }
            }
            last_h_sample_ts_us = sample_ts;

        } else if (raw.tag == LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG) {
            /* Decode SFLP quaternion for debug display only — not used for fusion */
            uint16_t q1h = (uint16_t)raw.data[0] | ((uint16_t)raw.data[1] << 8);
            uint16_t q2h = (uint16_t)raw.data[2] | ((uint16_t)raw.data[3] << 8);
            uint16_t q3h = (uint16_t)raw.data[4] | ((uint16_t)raw.data[5] << 8);
            imu_q.q1 = float16_to_float32(q1h);
            imu_q.q2 = float16_to_float32(q2h);
            imu_q.q3 = float16_to_float32(q3h);
            float q0sq = 1.0f - (imu_q.q1*imu_q.q1 + imu_q.q2*imu_q.q2 + imu_q.q3*imu_q.q3);
            imu_q.q0 = (q0sq > 0.0f) ? sqrtf(q0sq) : 0.0f;
            quat_normalize(&imu_q);
            yaw_sflp_rad = atan2f(2.0f*(imu_q.q0*imu_q.q3 + imu_q.q1*imu_q.q2),
                                  1.0f - 2.0f*(imu_q.q2*imu_q.q2 + imu_q.q3*imu_q.q3));
        }
    }

    /* ── PAA polling ────────────────────────────────────────────────────── */
    if ((uint32_t)(now_us - last_paa_us) >= PAA_READ_PERIOD_US) {
        float paa_dt = ((float)(now_us - last_paa_us)) * 1e-6f;

        /* DWT wraps every ~25 s — skip this cycle if dt is absurd */
        if (paa_dt < 0.0005f || paa_dt > 0.1f) {
            last_paa_us = now_us;
            return;
        }

        paaReadMotion(&paa);

        /* Keep displacement in native sensor frame (invert_x handled by driver).
           LUT must be applied BEFORE the axis swap to match SFE's calibration. */
        float sensor_dx_mm = (float)paa.dx_cpi * cpi_to_mm;
        float sensor_dy_mm = (float)paa.dy_cpi * cpi_to_mm;

#if ENABLE_LUT
        float vx_s = sensor_dx_mm * 1e-3f / paa_dt;
        float vy_s = sensor_dy_mm * 1e-3f / paa_dt;
        float xScalar = 1.0f, yScalar = 1.0f;
        bilinearInterp(vx_s * 39.37f, vy_s * 39.37f, &xScalar, &yScalar);
        sensor_dx_mm *= xScalar;
        sensor_dy_mm *= yScalar;
#endif

        /* Axis swap: logical X = -sensor Y, logical Y = sensor X */
        float dx_mm = -sensor_dy_mm;
        float dy_mm =  sensor_dx_mm;

        last_dx_cpi = (int16_t)(-paa.dy_cpi);
        last_dy_cpi = (int16_t)( paa.dx_cpi);

        /* Accumulate for debug print (all 480 Hz samples) */
        dbg_dx_mm_sum += dx_mm;
        dbg_dy_mm_sum += dy_mm;

        /* Rotate displacement into world frame using current heading (SFE approach:
           no SE(2) needed — correction is < 0.01 mm at 480 Hz for any realistic rotation) */
        float cth  = cosf(hKf.x[0]);
        float sth  = sinf(hKf.x[0]);
        float wx_m = (cth * dx_mm - sth * dy_mm) * 1e-3f;
        float wy_m = (sth * dx_mm + cth * dy_mm) * 1e-3f;

        last_vx = wx_m / paa_dt;
        last_vy = wy_m / paa_dt;

        kalman3_predict(&xKf, paa_dt);
        kalman3_predict(&yKf, paa_dt);
        kalman3_updateVel(&xKf, last_vx, KF_VEL_VAR);
        kalman3_updateVel(&yKf, last_vy, KF_VEL_VAR);

        paa.dx_cpi  = 0;
        paa.dy_cpi  = 0;
        last_paa_us = now_us;
    }

    /* ── I2C output at 50 Hz ────────────────────────────────────────────── */
    if (ENABLE_BUS_OUTPUT) {
        static uint32_t last_out_ms = 0;
        if (now_ms - last_out_ms >= BUS_OUTPUT_PERIOD_MS) {
            float yaw_rate_z = latest_gz_dps * ((float)M_PI / 180.0f);
            output_send(xKf.x[0], yKf.x[0],
                        xKf.x[1], yKf.x[1],
                        imu_q.q1, imu_q.q2, imu_q.q3, imu_q.q0,
                        yaw_rate_z);
            last_out_ms = now_ms;
        }
    }

    /* ── UART debug at 20 Hz ────────────────────────────────────────────── */
    if (DEBUG_UART && (now_ms - last_print_ms) >= DEBUG_PRINT_PERIOD_MS) {
        float yaw_deg  = hKf.x[0] * 57.2957795f;
        float sflp_deg = yaw_sflp_rad * 57.2957795f;
        printf("x:%.3f y:%.3f | vx:%.6f vy:%.6f | yaw:%.1f sflp:%.1f | gz:%.1f dps | dx:%d dy:%d | dmm:%.3f,%.3f\r\n",
               xKf.x[0] * 1000.0f, yKf.x[0] * 1000.0f,
               last_vx, last_vy,
               yaw_deg, sflp_deg, latest_gz_dps,
               last_dx_cpi, last_dy_cpi,
               dbg_dx_mm_sum, dbg_dy_mm_sum);
        dbg_dx_mm_sum = 0.0f;
        dbg_dy_mm_sum = 0.0f;
        last_print_ms = now_ms;
    }
}
