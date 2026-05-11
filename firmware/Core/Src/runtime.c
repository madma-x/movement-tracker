#include "runtime.h"
#include "main.h"

#include "PAA5163.h"
#include "lsm6dsv16x_reg.h"
#include "lsm6dsv16x_hal_glue.h"
#include "compensation.h"
#include "kalman.h"
#include "output.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

/* Debug output control */
#define DEBUG_UART 1
#define ENABLE_BUS_OUTPUT 1
#define BUS_OUTPUT_PERIOD_MS 20U  /* 50 Hz */
#define DEBUG_PRINT_PERIOD_MS 50U
/* Optical distance calibration: measured 96.4 mm for true 100 mm => scale by 100/96.4 */
#define PAA_DISTANCE_SCALE 1.08056f
/* Lever arm: distance from optical sensor to robot rotation centre (mm).
 * Positive X = forward, positive Y = left. Set 0 if sensor is at centre. */
#define PAA_OFFSET_X_MM  0.0f
#define PAA_OFFSET_Y_MM  0.0f

/* Sensor instances */
paa5163_t paa = {
	.spi = &hspi1,
	.NCS_Port = CS_PAA_GPIO_Port,
	.NCS_Pin = CS_PAA_Pin,
	.NRST_Port = RST_PAA_GPIO_Port,
	.NRST_Pin = RST_PAA_Pin,
	.invert_x = 1,
};

lsm6dsv16x_ctx_t lsm6_ctx;

/* Filter state */
kalman_filter_t kalman;

/* Latest orientation from IMU game rotation vector */
static quaternion_t imu_q = {
    .q0 = 1.0f,
    .q1 = 0.0f,
    .q2 = 0.0f,
    .q3 = 0.0f,
};
static float yaw_gyro_rad = 0.0f;
static uint32_t last_gyro_ms = 0;

/* Integrated compensated position in mm */
static float world_x_mm = 0.0f;
static float world_y_mm = 0.0f;
static float world_vx_ms = 0.0f;  /* PAA-derived velocity X (m/s) */
static float world_vy_ms = 0.0f;  /* PAA-derived velocity Y (m/s) */
static float accel_vx_ms = 0.0f;  /* Accel-derived velocity X (m/s) */
static float accel_vy_ms = 0.0f;  /* Accel-derived velocity Y (m/s) */
static int16_t last_raw_dx_cpi = 0;
static int16_t last_raw_dy_cpi = 0;
static int32_t raw_accum_dx_cpi = 0;
static int32_t raw_accum_dy_cpi = 0;
static float last_raw_dx_mm = 0.0f; /* Uncompensated PAA delta X (mm) */
static float last_raw_dy_mm = 0.0f; /* Uncompensated PAA delta Y (mm) */
static float gz_bias_dps = 0.0f;    /* Gyro Z bias measured at startup (dps) */


void setup(void){
    if (DEBUG_UART) printf("=== Movement Tracker V5 Startup ===\n");


    // Initialize ST context for LSM6DSV16X
    lsm6dsv16x_ctx_init(&lsm6_ctx, &hspi1);

    // Check WHO_AM_I
    uint8_t whoami = 0;
    lsm6dsv16x_device_id_get(&lsm6_ctx, &whoami);
    if (DEBUG_UART) printf("LSM6DSV16X WHO_AM_I: 0x%02X\n", whoami);

    // Reset and configure sensor
    lsm6dsv16x_reset_set(&lsm6_ctx, 0x02); // SW_RESET
    HAL_Delay(100);
    lsm6dsv16x_auto_increment_set(&lsm6_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_block_data_update_set(&lsm6_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_xl_data_rate_set(&lsm6_ctx, 0x07); // 416Hz
    lsm6dsv16x_gy_data_rate_set(&lsm6_ctx, 0x07); // 416Hz
    lsm6dsv16x_xl_mode_set(&lsm6_ctx, LSM6DSV16X_XL_HIGH_ACCURACY_ODR_MD);
    lsm6dsv16x_gy_mode_set(&lsm6_ctx, LSM6DSV16X_GY_HIGH_ACCURACY_ODR_MD); // High performance
    
    // Set full scale with official API to match conversion helpers.
    lsm6dsv16x_xl_full_scale_set(&lsm6_ctx, LSM6DSV16X_4g);
    lsm6dsv16x_gy_full_scale_set(&lsm6_ctx, LSM6DSV16X_2000dps);

    // Enable SFLP game rotation vector and batch it in FIFO
    lsm6dsv16x_sflp_data_rate_set(&lsm6_ctx, 0x06);
    lsm6dsv16x_sflp_game_rotation_set(&lsm6_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_fifo_sflp_raw_t sflp_fifo_cfg = {0};
    sflp_fifo_cfg.game_rotation = 1;
    lsm6dsv16x_fifo_sflp_batch_set(&lsm6_ctx, sflp_fifo_cfg);
    lsm6dsv16x_fifo_mode_set(&lsm6_ctx, LSM6DSV16X_STREAM_MODE);

    // Trigger SFLP game init bit in embedded function bank
    lsm6dsv16x_emb_func_init_a_t emb_init_a = {0};
    lsm6dsv16x_mem_bank_set(&lsm6_ctx, LSM6DSV16X_EMBED_FUNC_MEM_BANK);
    lsm6dsv16x_read_reg(&lsm6_ctx, LSM6DSV16X_EMB_FUNC_INIT_A, (uint8_t*)&emb_init_a, 1);
    emb_init_a.sflp_game_init = 1;
    lsm6dsv16x_write_reg(&lsm6_ctx, LSM6DSV16X_EMB_FUNC_INIT_A, (uint8_t*)&emb_init_a, 1);
    lsm6dsv16x_mem_bank_set(&lsm6_ctx, LSM6DSV16X_MAIN_MEM_BANK);

    HAL_Delay(50);

    if (DEBUG_UART) printf("LSM6DSV16X init complete\n");

    /* Initialize PAA5163 */
    if (DEBUG_UART) printf("PAA5163 Init... ");
    paa_err_t paa_ret = paaInit(&paa);
    if (DEBUG_UART) printf("%d (%s)\n", paa_ret, (paa_ret == paa_ok ? "OK" : "ERROR"));

    /* Fatal error if PAA sensor fails */
    if (paa_ret != paa_ok) {
        if (DEBUG_UART) printf("FATAL: Sensor init failed, rebooting...\n");
        HAL_Delay(500);
        NVIC_SystemReset();
    }

    /* Initialize output drivers */
    output_init(&hi2c2);

    /* Initialize Kalman filter */
    kalman_init(&kalman);

    /* Gyro Z bias calibration: average 500 samples at rest (~2.4 s at 208 Hz) */
    {
        const int CAL_SAMPLES = 500;
        double gz_sum = 0.0;
        int16_t g_raw[3] = {0};
        for (int i = 0; i < CAL_SAMPLES; i++) {
            HAL_Delay(5);
            lsm6dsv16x_angular_rate_raw_get(&lsm6_ctx, g_raw);
            gz_sum += lsm6dsv16x_from_fs2000_to_mdps(g_raw[2]) / 1000.0;
        }
        gz_bias_dps = (float)(gz_sum / CAL_SAMPLES);
        if (DEBUG_UART) printf("Gyro Z bias: %.4f dps\n", gz_bias_dps);
    }

    if (DEBUG_UART) {
        printf("Setup complete. Starting main loop.\n");
        printf("Debug: bus output %s\n", ENABLE_BUS_OUTPUT ? "ENABLED" : "DISABLED");
    }

    last_gyro_ms = HAL_GetTick();
}

void loop(void){
	static uint32_t last_print_ms = 0;

    output_process();  /* Re-arm I2C slave listen if peripheral got stuck */

    uint32_t now_ms = HAL_GetTick();
    uint32_t now_us = now_ms * 1000U;
    int16_t accel_raw[3] = {0};
    int16_t gyro_raw[3] = {0};
    float dx_mm = 0.0f;
    float dy_mm = 0.0f;
    float wx_mm = 0.0f;
    float wy_mm = 0.0f;
    int32_t gyro_ret = 0;
    int32_t accel_ret = 0;

    // Read IMU gyro and accel samples.
    gyro_ret = lsm6dsv16x_angular_rate_raw_get(&lsm6_ctx, gyro_raw);
    accel_ret = lsm6dsv16x_acceleration_raw_get(&lsm6_ctx, accel_raw);

    // Integrate heading from gyro Z (robust against linear translation).
    float gz_dps = lsm6dsv16x_from_fs2000_to_mdps(gyro_raw[2]) / 1000.0f - gz_bias_dps;
    float yaw_rate_rad_s = gz_dps * ((float)M_PI / 180.0f);
    float dt_s = ((float)(now_ms - last_gyro_ms)) * 1e-3f;
    last_gyro_ms = now_ms;
    if (dt_s > 0.0f && dt_s < 0.1f) {
        yaw_gyro_rad += gz_dps * (float)M_PI / 180.0f * dt_s;
        while (yaw_gyro_rad > (float)M_PI) yaw_gyro_rad -= 2.0f * (float)M_PI;
        while (yaw_gyro_rad < -(float)M_PI) yaw_gyro_rad += 2.0f * (float)M_PI;
    }
    // Feed Kalman prediction with accel projected into yaw-compensated world XY.
    float ax_mps2 = lsm6dsv16x_from_fs4_to_mg(accel_raw[0]) * 9.80665f / 1000.0f;
    float ay_mps2 = lsm6dsv16x_from_fs4_to_mg(accel_raw[1]) * 9.80665f / 1000.0f;

    // Subtract gravity projected onto sensor XY axes using SFLP quaternion (roll/pitch compensation).
    // For a body-to-world quaternion q, gravity in sensor frame = R^T * [0,0,g]:
    //   gx = 2*(q1*q3 - q0*q2)*g
    //   gy = 2*(q2*q3 + q0*q1)*g
    float grav_x = 2.0f * (imu_q.q1 * imu_q.q3 - imu_q.q0 * imu_q.q2) * 9.80665f;
    float grav_y = 2.0f * (imu_q.q2 * imu_q.q3 + imu_q.q0 * imu_q.q1) * 9.80665f;
    float ax_linear = ax_mps2 - grav_x;
    float ay_linear = ay_mps2 - grav_y;
    /* Reset velocity integrator once at 3s to discard unstable startup transient */
    if (now_ms >= 3000U) {
        static bool accel_v_reset_done = false;
        if (!accel_v_reset_done) {
            accel_vx_ms = 0.0f;
            accel_vy_ms = 0.0f;
            accel_v_reset_done = true;
            if (DEBUG_UART) printf("Accel velocity reset at 3s\n");
        }
        accel_vx_ms += ax_linear * dt_s;
        accel_vy_ms += ay_linear * dt_s;
    }
    float cy = cosf(yaw_gyro_rad);
    float sy = sinf(yaw_gyro_rad);
    float ax_world = cy * ax_linear - sy * ay_linear;
    float ay_world = sy * ax_linear + cy * ay_linear;
    if (accel_ret != 0) {
        ax_world = 0.0f;
        ay_world = 0.0f;
    }
    kalman_predict(&kalman, ax_world, ay_world, now_us);

    /* Use gz_dps (already computed) for ZUPT rotation check */
    float gyro_yaw_dps = fabsf(gz_dps);


    // Drain FIFO first so imu_q reflects the end of the current PAA window.
    // fifo_level is not a count of complete tagged samples, so refresh status
    // each iteration and stop only when the FIFO reports empty.
    lsm6dsv16x_fifo_status_t fifo_status = {0};
    lsm6dsv16x_fifo_status_get(&lsm6_ctx, &fifo_status);
    while (fifo_status.fifo_level > 0U) {
        lsm6dsv16x_fifo_out_raw_t fifo_raw = {0};
        if (lsm6dsv16x_fifo_out_raw_get(&lsm6_ctx, &fifo_raw) != 0) {
            break;
        }

        if (fifo_raw.tag == LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG) {
            uint16_t q1_h = (uint16_t)fifo_raw.data[0] | ((uint16_t)fifo_raw.data[1] << 8);
            uint16_t q2_h = (uint16_t)fifo_raw.data[2] | ((uint16_t)fifo_raw.data[3] << 8);
            uint16_t q3_h = (uint16_t)fifo_raw.data[4] | ((uint16_t)fifo_raw.data[5] << 8);

            imu_q.q1 = float16_to_float32(q1_h);
            imu_q.q2 = float16_to_float32(q2_h);
            imu_q.q3 = float16_to_float32(q3_h);

            float q0_sq = 1.0f - (imu_q.q1 * imu_q.q1 + imu_q.q2 * imu_q.q2 + imu_q.q3 * imu_q.q3);
            imu_q.q0 = (q0_sq > 0.0f) ? sqrtf(q0_sq) : 0.0f;
            quat_normalize(&imu_q);
        }

        lsm6dsv16x_fifo_status_get(&lsm6_ctx, &fifo_status);
    }

    // Read PAA5163 motion
    static uint32_t last_paa_read_ms = 0;
    /* Step 1+5: gyro heading continuously integrated above; save window-start value here */
    static float yaw_gyro_paa_start = 0.0f;
    if (now_ms - last_paa_read_ms >= 5) { // 5ms between reads (200Hz max)
        float paa_dt_s = (float)(now_ms - last_paa_read_ms) * 1e-3f;
        paaReadMotion(&paa);

        // Consume the latest delta once so it is not integrated again on later loops.
        last_raw_dx_cpi = -paa.dy_cpi;
        last_raw_dy_cpi = paa.dx_cpi;

        /* Zero Velocity Update: PAA reports no motion AND gyro is below threshold */
        if (last_raw_dx_cpi == 0 && last_raw_dy_cpi == 0 && gyro_yaw_dps < 2.0f) {
            kalman_zupt(&kalman);
        }
        dx_mm = (((float)last_raw_dx_cpi * 25.4f) / (float)paa.resolution) * PAA_DISTANCE_SCALE;
        dy_mm = (((float)last_raw_dy_cpi * 25.4f) / (float)paa.resolution) * PAA_DISTANCE_SCALE;
        raw_accum_dx_cpi += last_raw_dx_cpi;
        raw_accum_dy_cpi += last_raw_dy_cpi;
        last_raw_dx_mm += dx_mm;
        last_raw_dy_mm += dy_mm;

        /* Step 1: dtheta from gyro integrated at full IMU rate — no fusion latency */
        float dtheta = yaw_gyro_rad - yaw_gyro_paa_start;
        while (dtheta >  (float)M_PI) dtheta -= 2.0f * (float)M_PI;
        while (dtheta < -(float)M_PI) dtheta += 2.0f * (float)M_PI;

        /* Step 2: lever-arm correction (fake translation when sensor is off-centre) */
        float dx_c = dx_mm + PAA_OFFSET_Y_MM * dtheta;
        float dy_c = dy_mm - PAA_OFFSET_X_MM * dtheta;

        /* Step 3: exact SE(2) rigid-body integration
         *   A = sin(dtheta)/dtheta,  B = (1-cos(dtheta))/dtheta
         * Taylor expansion for |dtheta| < 1e-6 avoids 0/0. */
        float A_se2, B_se2;
        if (fabsf(dtheta) > 1e-6f) {
            A_se2 = sinf(dtheta) / dtheta;
            B_se2 = (1.0f - cosf(dtheta)) / dtheta;
        } else {
            A_se2 = 1.0f - dtheta * dtheta * (1.0f / 6.0f);
            B_se2 = dtheta * 0.5f;
        }
        float u_se2 = A_se2 * dx_c - B_se2 * dy_c;
        float v_se2 = B_se2 * dx_c + A_se2 * dy_c;
        float cth = cosf(yaw_gyro_paa_start);
        float sth = sinf(yaw_gyro_paa_start);
        wx_mm = cth * u_se2 - sth * v_se2;
        wy_mm = sth * u_se2 + cth * v_se2;
        world_x_mm += wx_mm;
        world_y_mm += wy_mm;

        /* Measure velocity from compensated PAA optical flow in world frame */
        if (paa_dt_s > 0.001f) {
            float vx_meas = (wx_mm * 1e-3f) / paa_dt_s;
            float vy_meas = (wy_mm * 1e-3f) / paa_dt_s;
            world_vx_ms = vx_meas;
            world_vy_ms = vy_meas;
            
            /* Kalman: use compensated PAA velocity as measurement only (no position) */
            kalman_update_velocity(&kalman, vx_meas, vy_meas);
        }

        paa.dx_cpi = 0;
        paa.dy_cpi = 0;
        yaw_gyro_paa_start = yaw_gyro_rad;  /* Step 5: save gyro heading at window end */
        last_paa_read_ms = now_ms;
    }

    float yaw_sflp_rad = atan2f(2.0f * (imu_q.q0 * imu_q.q3 + imu_q.q1 * imu_q.q2),
                                1.0f - 2.0f * (imu_q.q2 * imu_q.q2 + imu_q.q3 * imu_q.q3));

    if (ENABLE_BUS_OUTPUT) {
        static uint32_t last_output_ms = 0;
        if (now_ms - last_output_ms >= BUS_OUTPUT_PERIOD_MS) {
            output_send(world_x_mm * 1e-3f, world_y_mm * 1e-3f,
                        world_vx_ms, world_vy_ms,
                        imu_q.q1, imu_q.q2, imu_q.q3, imu_q.q0,
                        yaw_rate_rad_s);
            last_output_ms = now_ms;
        }
    }

    if (DEBUG_UART && (now_ms - last_print_ms) >= DEBUG_PRINT_PERIOD_MS) {
        kalman_state_t kstate = kalman_get_state(&kalman);
        float kx_mm = kstate.x * 1000.0f;
        float ky_mm = kstate.y * 1000.0f;
        float kvx_mms = kstate.vx * 1000.0f;
        float kvy_mms = kstate.vy * 1000.0f;
        float yaw_sflp_deg = yaw_sflp_rad * 57.2957795f;
        float yaw_gyro_deg = yaw_gyro_rad * 57.2957795f;
        float gx_dps = lsm6dsv16x_from_fs2000_to_mdps(gyro_raw[0]) / 1000.0f;
        float gy_dps = lsm6dsv16x_from_fs2000_to_mdps(gyro_raw[1]) / 1000.0f;
        printf("classic x:%.2f y:%.2f | kalman x:%.2f y:%.2f vx:%.2f vy:%.2f mm/s | bias ax:%.4f ay:%.4f | dx:%d dy:%d | raw_mm:(%.3f,%.3f) | raw_cpi:(%ld,%ld) | yaw_gyro:%.1f yaw_sflp:%.1f | gyro_xyz:(%.1f,%.1f,%.1f)dps | imu r[g:%ld a:%ld] axy=(%.3f,%.3f)m/s2\r\n",
			world_x_mm, world_y_mm,
			kx_mm, ky_mm, world_vx_ms, world_vy_ms,
			kstate.bx, kstate.by,
			last_raw_dx_cpi, last_raw_dy_cpi,
            last_raw_dx_mm, last_raw_dy_mm,
            (long)raw_accum_dx_cpi, (long)raw_accum_dy_cpi,
			yaw_gyro_deg, yaw_sflp_deg,
			gx_dps, gy_dps, gz_dps,
			(long)gyro_ret, (long)accel_ret, ax_world, ay_world);
        last_print_ms = now_ms;
    }
}
