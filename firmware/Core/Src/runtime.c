#include "runtime.h"
#include "main.h"

#include "PAA5163.h"
#include "lsm6dsv16x_reg.h"
#include "lsm6dsv16x_hal_glue.h"
#include "micros.h"
#include "compensation.h"
#include "output.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

/* Debug output control */
#define DEBUG_UART 1
#define ENABLE_BUS_OUTPUT 1
#define BUS_OUTPUT_PERIOD_MS 20U  /* 50 Hz */
#define DEBUG_PRINT_PERIOD_MS 50U
/* Fused heading: integrate gyro continuously for low-latency rotation
 * compensation, then correct slow drift toward GRV while stationary. */
#define USE_HYBRID_YAW 1
#define YAW_STATIONARY_GZ_DPS_TH 2.0f
#define YAW_BIAS_TAU_S 2.5f
#define YAW_BIAS_MAX_STEP_RAD 0.02f
#define YAW_BIAS_OUTLIER_REJECT_RAD 0.61f
#define GYRO_FIFO_RATE_HZ 960.0f
#define GYRO_FIFO_DT_S_FALLBACK (1.0f / GYRO_FIFO_RATE_HZ)
#define GYRO_FIFO_SAMPLE_US (1000000.0f / GYRO_FIFO_RATE_HZ)
#define IMU_TIMESTAMP_TICK_US 25.0f
#define GYRO_MAX_DT_S 0.01f
#define GYRO_SAMPLE_QUEUE_CAPACITY 2048U
#define PAA_READ_PERIOD_US 2000U
/* Gyro yaw scale from turn calibration: 80 deg real vs 90 deg measured => 0.8889. */
#define GYRO_YAW_SCALE 0.8889f
/* Optical distance calibration: measured 96.4 mm for true 100 mm => scale by 100/96.4 */
#define PAA_DISTANCE_SCALE 1.08056f
/* Lever arm: distance from optical sensor to rotation center (mm). */
#define PAA_OFFSET_X_MM 0.0f
#define PAA_OFFSET_Y_MM 0.0f

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

/* Latest orientation from IMU game rotation vector */
static quaternion_t imu_q = {
    .q0 = 1.0f,
    .q1 = 0.0f,
    .q2 = 0.0f,
    .q3 = 0.0f,
};
static float yaw_gyro_rad = 0.0f;

/* Direct gyro-compensated odometry state */
static float world_x_mm = 0.0f;
static float world_y_mm = 0.0f;
static float world_vx_ms = 0.0f;  /* PAA-derived velocity X (m/s) */
static float world_vy_ms = 0.0f;  /* PAA-derived velocity Y (m/s) */
static float cpi_to_mm = 0.0f;     /* Precomputed optical scale: CPI -> mm */
static int16_t last_raw_dx_cpi = 0;
static int16_t last_raw_dy_cpi = 0;
static int32_t raw_accum_dx_cpi = 0;
static int32_t raw_accum_dy_cpi = 0;
static float last_raw_dx_mm = 0.0f; /* Uncompensated PAA delta X (mm) */
static float last_raw_dy_mm = 0.0f; /* Uncompensated PAA delta Y (mm) */
static float gz_bias_dps = 0.0f;    /* Gyro Z bias measured at startup (dps) */
static float yaw_bias_corr_rad = 0.0f;
static float latest_gx_dps = 0.0f;
static float latest_gy_dps = 0.0f;
static float latest_gz_raw_dps = 0.0f;
static float latest_gz_dps = 0.0f;

typedef struct {
    float timestamp_us;
    float dtheta_rad;
} gyro_sample_t;

static gyro_sample_t gyro_sample_queue[GYRO_SAMPLE_QUEUE_CAPACITY];
static uint16_t gyro_sample_head = 0;
static uint16_t gyro_sample_tail = 0;
static bool gyro_timestamp_sync_valid = false;
static float latest_fifo_timestamp_us = 0.0f;
static float next_gyro_sample_timestamp_us = 0.0f;
static float last_gyro_sample_timestamp_us = 0.0f;
static uint32_t latest_fifo_timestamp_host_us = 0U;

static float wrap_pi(float a) {
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static void push_gyro_sample(float timestamp_us, float dtheta_rad) {
    uint16_t next_tail = (uint16_t)((gyro_sample_tail + 1U) % GYRO_SAMPLE_QUEUE_CAPACITY);
    if (next_tail == gyro_sample_head) {
        gyro_sample_head = (uint16_t)((gyro_sample_head + 1U) % GYRO_SAMPLE_QUEUE_CAPACITY);
    }
    gyro_sample_queue[gyro_sample_tail].timestamp_us = timestamp_us;
    gyro_sample_queue[gyro_sample_tail].dtheta_rad = dtheta_rad;
    gyro_sample_tail = next_tail;
}

static float consume_gyro_window(float start_us, float end_us) {
    float dtheta_sum = 0.0f;

    while (gyro_sample_head != gyro_sample_tail) {
        gyro_sample_t sample = gyro_sample_queue[gyro_sample_head];
        if (sample.timestamp_us > end_us) {
            break;
        }
        if (sample.timestamp_us > start_us) {
            dtheta_sum += sample.dtheta_rad;
        }
        gyro_sample_head = (uint16_t)((gyro_sample_head + 1U) % GYRO_SAMPLE_QUEUE_CAPACITY);
    }

    return dtheta_sum;
}

static float estimate_current_imu_time_us(uint32_t host_now_us) {
    if (!gyro_timestamp_sync_valid) {
        return (float)host_now_us;
    }
    return latest_fifo_timestamp_us + (float)(host_now_us - latest_fifo_timestamp_host_us);
}
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
    lsm6dsv16x_xl_data_rate_set(&lsm6_ctx, LSM6DSV16X_ODR_AT_120Hz);
    lsm6dsv16x_gy_data_rate_set(&lsm6_ctx, LSM6DSV16X_ODR_AT_960Hz);
    lsm6dsv16x_xl_mode_set(&lsm6_ctx, LSM6DSV16X_XL_HIGH_ACCURACY_ODR_MD);
    lsm6dsv16x_gy_mode_set(&lsm6_ctx, LSM6DSV16X_GY_HIGH_ACCURACY_ODR_MD); // High performance
    
    // Set full scale with official API to match conversion helpers.
    lsm6dsv16x_xl_full_scale_set(&lsm6_ctx, LSM6DSV16X_4g);
    lsm6dsv16x_gy_full_scale_set(&lsm6_ctx, LSM6DSV16X_2000dps);

    // Batch gyro, timestamps, and GRV into the FIFO.
    lsm6dsv16x_timestamp_set(&lsm6_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_fifo_timestamp_batch_set(&lsm6_ctx, LSM6DSV16X_TMSTMP_DEC_1);
    lsm6dsv16x_sflp_data_rate_set(&lsm6_ctx, LSM6DSV16X_SFLP_120Hz);
    lsm6dsv16x_sflp_game_rotation_set(&lsm6_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_fifo_gy_batch_set(&lsm6_ctx, LSM6DSV16X_GY_BATCHED_AT_960Hz);
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

    /* Precompute CPI->mm scale to avoid divisions in the loop hot path. */
    if (paa.resolution > 0U) {
        cpi_to_mm = (25.4f / (float)paa.resolution) * PAA_DISTANCE_SCALE;
    } else {
        cpi_to_mm = 0.0f;
    }

    /* Initialize output drivers */
    output_init(&hi2c2);

    /* Gyro Z bias calibration: average 500 samples at rest. */
    {
        const int CAL_SAMPLES = 500;
        double gz_sum = 0.0;
        int16_t g_raw[3] = {0};
        for (int i = 0; i < CAL_SAMPLES; i++) {
            HAL_Delay(2);
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
}

void loop(void){
	static uint32_t last_print_ms = 0;
	static uint32_t last_paa_read_us = 0;
    static float last_paa_read_imu_us = 0.0f;
	static uint32_t last_fusion_update_us = 0;
	static bool odom_window_init = false;
	static float yaw_fused_paa_start_rad = 0.0f;

    output_process();  /* Re-arm I2C slave listen if peripheral got stuck */

    uint32_t now_ms = HAL_GetTick();
    uint32_t now_us = micros();
    float dx_mm = 0.0f;
    float dy_mm = 0.0f;
    float wx_mm = 0.0f;
    float wy_mm = 0.0f;
    float yaw_rate_rad_s = latest_gz_dps * ((float)M_PI / 180.0f) * GYRO_YAW_SCALE;


    // Drain FIFO first so gyro/GRV state reflects the end of the current PAA window.
    // Read status once, then drain up to that many samples plus guard margin,
    // stopping early if FIFO_EMPTY tag appears.
    lsm6dsv16x_fifo_status_t fifo_status = {0};
    lsm6dsv16x_fifo_status_get(&lsm6_ctx, &fifo_status);
    uint16_t drain_budget = (uint16_t)(fifo_status.fifo_level + 8U);
    for (uint16_t n = 0; n < drain_budget; n++) {
        lsm6dsv16x_fifo_out_raw_t fifo_raw = {0};
        if (lsm6dsv16x_fifo_out_raw_get(&lsm6_ctx, &fifo_raw) != 0) {
            break;
        }

        if (fifo_raw.tag == LSM6DSV16X_FIFO_EMPTY) {
            break;
        }

        if (fifo_raw.tag == LSM6DSV16X_TIMESTAMP_TAG) {
            uint32_t timestamp_raw = (uint32_t)fifo_raw.data[0]
                                   | ((uint32_t)fifo_raw.data[1] << 8)
                                   | ((uint32_t)fifo_raw.data[2] << 16)
                                   | ((uint32_t)fifo_raw.data[3] << 24);
            latest_fifo_timestamp_us = (float)timestamp_raw * IMU_TIMESTAMP_TICK_US;
            next_gyro_sample_timestamp_us = latest_fifo_timestamp_us;
            latest_fifo_timestamp_host_us = now_us;
            gyro_timestamp_sync_valid = true;
        } else if (fifo_raw.tag == LSM6DSV16X_GY_NC_TAG ||
            fifo_raw.tag == LSM6DSV16X_GY_NC_T_1_TAG ||
            fifo_raw.tag == LSM6DSV16X_GY_NC_T_2_TAG ||
            fifo_raw.tag == LSM6DSV16X_GY_2XC_TAG ||
            fifo_raw.tag == LSM6DSV16X_GY_3XC_TAG) {
            int16_t gx_raw = (int16_t)((fifo_raw.data[1] << 8) | fifo_raw.data[0]);
            int16_t gy_raw = (int16_t)((fifo_raw.data[3] << 8) | fifo_raw.data[2]);
            int16_t gz_raw = (int16_t)((fifo_raw.data[5] << 8) | fifo_raw.data[4]);

            latest_gx_dps = lsm6dsv16x_from_fs2000_to_mdps(gx_raw) / 1000.0f;
            latest_gy_dps = lsm6dsv16x_from_fs2000_to_mdps(gy_raw) / 1000.0f;
            latest_gz_raw_dps = lsm6dsv16x_from_fs2000_to_mdps(gz_raw) / 1000.0f;
            latest_gz_dps = latest_gz_raw_dps - gz_bias_dps;
            yaw_rate_rad_s = latest_gz_dps * ((float)M_PI / 180.0f) * GYRO_YAW_SCALE;
            float sample_timestamp_us = gyro_timestamp_sync_valid ? next_gyro_sample_timestamp_us : (float)now_us;
            float gyro_dt_s = GYRO_FIFO_DT_S_FALLBACK;

            if (last_gyro_sample_timestamp_us > 0.0f) {
                float gyro_dt_from_timestamp_s = (sample_timestamp_us - last_gyro_sample_timestamp_us) * 1e-6f;
                if (gyro_dt_from_timestamp_s > 0.0f && gyro_dt_from_timestamp_s < GYRO_MAX_DT_S) {
                    gyro_dt_s = gyro_dt_from_timestamp_s;
                }
            }
            float dtheta_sample = yaw_rate_rad_s * gyro_dt_s;

            yaw_gyro_rad = wrap_pi(yaw_gyro_rad + dtheta_sample);
            push_gyro_sample(sample_timestamp_us, dtheta_sample);
            last_gyro_sample_timestamp_us = sample_timestamp_us;

            if (gyro_timestamp_sync_valid) {
                next_gyro_sample_timestamp_us += GYRO_FIFO_SAMPLE_US;
                latest_fifo_timestamp_us = sample_timestamp_us;
                latest_fifo_timestamp_host_us = now_us;
            }
        } else if (fifo_raw.tag == LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG) {
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

    float yaw_sflp_rad = atan2f(2.0f * (imu_q.q0 * imu_q.q3 + imu_q.q1 * imu_q.q2),
                                1.0f - 2.0f * (imu_q.q2 * imu_q.q2 + imu_q.q3 * imu_q.q3));
    float yaw_fused_rad = wrap_pi(yaw_gyro_rad + yaw_bias_corr_rad);
#if USE_HYBRID_YAW
    {
        float fusion_dt_s = 0.0f;
        if (last_fusion_update_us != 0U) {
            fusion_dt_s = ((float)(now_us - last_fusion_update_us)) * 1e-6f;
        }
        last_fusion_update_us = now_us;

        bool is_stationary = (last_raw_dx_cpi == 0 && last_raw_dy_cpi == 0 && fabsf(latest_gz_dps) < YAW_STATIONARY_GZ_DPS_TH);
        if (is_stationary && fusion_dt_s > 0.0f && fusion_dt_s < 0.1f) {
            float yaw_err = wrap_pi(yaw_sflp_rad - yaw_fused_rad);
            if (fabsf(yaw_err) <= YAW_BIAS_OUTLIER_REJECT_RAD) {
                float yaw_step = (fusion_dt_s / YAW_BIAS_TAU_S) * yaw_err;
                if (yaw_step > YAW_BIAS_MAX_STEP_RAD) yaw_step = YAW_BIAS_MAX_STEP_RAD;
                if (yaw_step < -YAW_BIAS_MAX_STEP_RAD) yaw_step = -YAW_BIAS_MAX_STEP_RAD;
                yaw_bias_corr_rad = wrap_pi(yaw_bias_corr_rad + yaw_step);
                yaw_fused_rad = wrap_pi(yaw_gyro_rad + yaw_bias_corr_rad);
            }
        }
    }
#endif
    float current_imu_time_us = estimate_current_imu_time_us(now_us);
    if (!odom_window_init) {
        last_paa_read_us = now_us;
        last_paa_read_imu_us = current_imu_time_us;
        yaw_fused_paa_start_rad = yaw_fused_rad;
        odom_window_init = true;
    }

    // Read PAA5163 motion faster using microsecond timing.
    if ((uint32_t)(now_us - last_paa_read_us) >= PAA_READ_PERIOD_US) {
        float paa_dt_s = ((float)(now_us - last_paa_read_us)) * 1e-6f;
        float imu_window_end_us = current_imu_time_us;
        paaReadMotion(&paa);

        // Consume the latest delta once so it is not integrated again on later loops.
        last_raw_dx_cpi = -paa.dy_cpi;
        last_raw_dy_cpi = paa.dx_cpi;
        dx_mm = (float)last_raw_dx_cpi * cpi_to_mm;
        dy_mm = (float)last_raw_dy_cpi * cpi_to_mm;
        raw_accum_dx_cpi += last_raw_dx_cpi;
        raw_accum_dy_cpi += last_raw_dy_cpi;
        last_raw_dx_mm += dx_mm;
        last_raw_dy_mm += dy_mm;

        /* Integrate only gyro samples whose FIFO timestamps fall in the PAA window. */
        float dtheta = consume_gyro_window(last_paa_read_imu_us, imu_window_end_us);

        /* Exact SE(2) rigid-body integration
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
        float u_se2 = A_se2 * dx_mm - B_se2 * dy_mm;
        float v_se2 = B_se2 * dx_mm + A_se2 * dy_mm;
        float cth = cosf(yaw_fused_paa_start_rad);
        float sth = sinf(yaw_fused_paa_start_rad);
        wx_mm = cth * u_se2 - sth * v_se2;
        wy_mm = sth * u_se2 + cth * v_se2;
        world_x_mm += wx_mm;
        world_y_mm += wy_mm;

        /* Velocity from gyro-window-compensated optical flow in world frame. */
        if (paa_dt_s > 0.001f) {
            float vx_meas = (wx_mm * 1e-3f) / paa_dt_s;
            float vy_meas = (wy_mm * 1e-3f) / paa_dt_s;
            world_vx_ms = vx_meas;
            world_vy_ms = vy_meas;
        }

        paa.dx_cpi = 0;
        paa.dy_cpi = 0;
        last_paa_read_imu_us = imu_window_end_us;
        yaw_fused_paa_start_rad = yaw_fused_rad;
        last_paa_read_us = now_us;
    }

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
        float odom_x_mm = world_x_mm;
        float odom_y_mm = world_y_mm;
        float odom_vx_mms = world_vx_ms * 1000.0f;
        float odom_vy_mms = world_vy_ms * 1000.0f;
        float yaw_sflp_deg = yaw_sflp_rad * 57.2957795f;
        float yaw_gyro_deg = yaw_gyro_rad * 57.2957795f;
        float yaw_fused_deg = yaw_fused_rad * 57.2957795f;
        /* Keep legacy field labels for the existing serial plot/parser. */
        printf("classic x:%.2f y:%.2f| dx:%d dy:%d | raw_mm:(%.3f,%.3f) | raw_cpi:(%ld,%ld) | yaw_gyro:%.1f yaw_sflp:%.1f yaw_fused:%.1f | gyro_xyz:(%.1f,%.1f,%.1f)dps | gyro_raw_z:%.1fdps\r\n",
			odom_x_mm, odom_y_mm,
			last_raw_dx_cpi, last_raw_dy_cpi,
            last_raw_dx_mm, last_raw_dy_mm,
            (long)raw_accum_dx_cpi, (long)raw_accum_dy_cpi,
			yaw_gyro_deg, yaw_sflp_deg, yaw_fused_deg,
			latest_gx_dps, latest_gy_dps, latest_gz_dps, latest_gz_raw_dps);
        last_print_ms = now_ms;
    }
}
