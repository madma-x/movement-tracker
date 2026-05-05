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
static float last_raw_dx_mm = 0.0f; /* Uncompensated PAA delta X (mm) */
static float last_raw_dy_mm = 0.0f; /* Uncompensated PAA delta Y (mm) */


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
    lsm6dsv16x_xl_data_rate_set(&lsm6_ctx, 0x06); // 208Hz
    lsm6dsv16x_gy_data_rate_set(&lsm6_ctx, 0x06); // 208Hz
    lsm6dsv16x_xl_mode_set(&lsm6_ctx, 0x01); // High performance
    lsm6dsv16x_gy_mode_set(&lsm6_ctx, 0x01); // High performance
    // Set full scale with official API to match conversion helpers.
    lsm6dsv16x_xl_full_scale_set(&lsm6_ctx, LSM6DSV16X_4g);
    lsm6dsv16x_gy_full_scale_set(&lsm6_ctx, LSM6DSV16X_2000dps);

    // Enable SFLP game rotation vector and batch it in FIFO
    lsm6dsv16x_sflp_data_rate_set(&lsm6_ctx, LSM6DSV16X_SFLP_120Hz);
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
    float gz_dps = lsm6dsv16x_from_fs2000_to_mdps(gyro_raw[2]) / 1000.0f;
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


    // Drain FIFO first so imu_q reflects the end of the current PAA window.
    lsm6dsv16x_fifo_status_t fifo_status = {0};
    lsm6dsv16x_fifo_status_get(&lsm6_ctx, &fifo_status);
    uint16_t fifo_level = fifo_status.fifo_level;
    while (fifo_level--) {
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
    }

    // Read PAA5163 motion
    static uint32_t last_paa_read_ms = 0;
    // Stores SFLP yaw at the end of the previous PAA window (= start of this window).
    static quaternion_t imu_q_paa_start = {.q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f};
    if (now_ms - last_paa_read_ms >= 5) { // 5ms between reads (200Hz max)
        float paa_dt_s = (float)(now_ms - last_paa_read_ms) * 1e-3f;
        paaReadMotion(&paa);

        // Consume the latest delta once so it is not integrated again on later loops.
        last_raw_dx_cpi = paa.dx_cpi;  /* sensor X maps to robot Y — swap axes */
        last_raw_dy_cpi = paa.dy_cpi;
        dx_mm = ((float)last_raw_dx_cpi * 25.4f) / (float)paa.resolution;
        dy_mm = ((float)last_raw_dy_cpi * 25.4f) / (float)paa.resolution;
        last_raw_dx_mm += dx_mm;
        last_raw_dy_mm += dy_mm;

        // Midpoint heading: average SFLP yaw at window-start and window-end.
        float yaw_start = atan2f(2.0f * (imu_q_paa_start.q0 * imu_q_paa_start.q3 + imu_q_paa_start.q1 * imu_q_paa_start.q2),
                                  1.0f - 2.0f * (imu_q_paa_start.q2 * imu_q_paa_start.q2 + imu_q_paa_start.q3 * imu_q_paa_start.q3));
        float yaw_end   = atan2f(2.0f * (imu_q.q0 * imu_q.q3 + imu_q.q1 * imu_q.q2),
                                  1.0f - 2.0f * (imu_q.q2 * imu_q.q2 + imu_q.q3 * imu_q.q3));
        float dyaw = yaw_end - yaw_start;
        while (dyaw >  (float)M_PI) dyaw -= 2.0f * (float)M_PI;
        while (dyaw < -(float)M_PI) dyaw += 2.0f * (float)M_PI;
        float yaw_mid = yaw_start + 0.5f * dyaw;
        quaternion_t mid_q = {
            .q0 = cosf(0.5f * yaw_mid),
            .q1 = 0.0f,
            .q2 = 0.0f,
            .q3 = sinf(0.5f * yaw_mid),
        };
        compensate_paa_delta(dx_mm, dy_mm, mid_q, &wx_mm, &wy_mm);
        world_x_mm += wx_mm;
        world_y_mm += wy_mm;

        // Update Kalman with measured position from classic yaw-compensated odometry.
        kalman_update_position(&kalman, world_x_mm * 1e-3f, world_y_mm * 1e-3f, paa_dt_s);

        // Update Kalman velocity directly from optical flow (wx/dt, wy/dt).
        // This is the primary fix for velocity oscillation: directly constrains vx/vy
        // instead of relying on off-diagonal covariance terms from position updates only.
        if (paa_dt_s > 0.001f) {
            float vx_opt = (wx_mm * 1e-3f) / paa_dt_s;
            float vy_opt = (wy_mm * 1e-3f) / paa_dt_s;
            world_vx_ms = vx_opt;
            world_vy_ms = vy_opt;
            kalman_update_velocity(&kalman, vx_opt, vy_opt);
        }

        paa.dx_cpi = 0;
        paa.dy_cpi = 0;
        imu_q_paa_start = imu_q;  // save end-of-window heading for next iteration
        last_paa_read_ms = now_ms;
    }

    float yaw_sflp_rad = atan2f(2.0f * (imu_q.q0 * imu_q.q3 + imu_q.q1 * imu_q.q2),
                                1.0f - 2.0f * (imu_q.q2 * imu_q.q2 + imu_q.q3 * imu_q.q3));

    if (ENABLE_BUS_OUTPUT) {
        static uint32_t last_output_ms = 0;
        if (now_ms - last_output_ms >= BUS_OUTPUT_PERIOD_MS) {
            output_send(world_x_mm * 1e-3f, world_y_mm * 1e-3f,
                        world_vx_ms, world_vy_ms,
                        yaw_sflp_rad);
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
        printf("classic x:%.2f y:%.2f | kalman x:%.2f y:%.2f vx:%.2f vy:%.2f m/s | bias ax:%.4f ay:%.4f | dx:%d dy:%d | raw_mm:(%.3f,%.3f) | yaw_gyro:%.1f yaw_sflp:%.1f | imu r[g:%ld a:%ld] axy=(%.3f,%.3f)m/s2\r\n",
			world_x_mm, world_y_mm,
			kx_mm, ky_mm, accel_vx_ms, accel_vy_ms,
			kstate.bx, kstate.by,
			last_raw_dx_cpi, last_raw_dy_cpi,
            last_raw_dx_mm, last_raw_dy_mm,
			yaw_gyro_deg, yaw_sflp_deg,
			(long)gyro_ret, (long)accel_ret, ax_world, ay_world);
        last_print_ms = now_ms;
    }
}
