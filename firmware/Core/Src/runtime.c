#include "runtime.h"
#include "main.h"

#include "PAA5163.h"
#include "lsm6dsv.h"
#include "compensation.h"
#include "kalman.h"
#include "output.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

/* Debug output control */
#define DEBUG_UART 1

/* Sensor instances */
paa5163_t paa = {
	.spi = &hspi1,
	.NCS_Port = CS_PAA_GPIO_Port,
	.NCS_Pin = CS_PAA_Pin,
	.NRST_Port = RST_PAA_GPIO_Port,
	.NRST_Pin = RST_PAA_Pin,
	.invert_x = 1,
};

lsm6dsv_t lsm6dsv = {
	.spi = &hspi1,
	.NCS_Port = GPIOA,        /* PA0 */
	.NCS_Pin = GPIO_PIN_0,
};

/* Filter state */
kalman_filter_t kalman;

/* Last PAA readings */
static int16_t paa_dx_cpi = 0;
static int16_t paa_dy_cpi = 0;


void setup(void){
	if (DEBUG_UART) printf("=== Movement Tracker V5 Startup ===\n");
	
	/* Initialize LSM6DSV */
	if (DEBUG_UART) printf("LSM6DSV Init... ");
	int lsm_ret = lsm6dsvInit(&lsm6dsv);
	if (DEBUG_UART) printf("%d (%s)\n", lsm_ret, (lsm_ret == 0 ? "OK" : "ERROR"));
	
	/* Initialize PAA5163 */
	if (DEBUG_UART) printf("PAA5163 Init... ");
	paa_err_t paa_ret = paaInit(&paa);
	if (DEBUG_UART) printf("%d (%s)\n", paa_ret, (paa_ret == paa_ok ? "OK" : "ERROR"));
	
	/* Fatal error if either sensor fails */
	if (lsm_ret != 0 || paa_ret != paa_ok) {
		if (DEBUG_UART) printf("FATAL: Sensor init failed, rebooting...\n");
		HAL_Delay(500);
		NVIC_SystemReset();
	}
	
	/* Initialize output drivers */
	output_init(&hi2c2, &hfdcan2);
	
	/* Initialize Kalman filter */
	kalman_init(&kalman);
	
	if (DEBUG_UART) printf("Setup complete. Starting main loop.\n");
}

void loop(void){
	static uint32_t last_print_us = 0;
	
	/* Read sensors */
	lsm6dsv_data_t lsm_data;
	paa_err_t paa_err = paaReadMotion(&paa);
	int lsm_ok = lsm6dsvRead(&lsm6dsv, &lsm_data);
	
	if (lsm_ok != 0) {
		if (DEBUG_UART) printf("LSM6DSV read error\n");
		return;
	}
	
	if (paa_err != paa_ok) {
		if (DEBUG_UART) printf("PAA5163 read error\n");
		return;
	}
	
	/* Get latest PAA delta motion (accumulated since last read) */
	paa_dx_cpi = paa.dx_cpi;
	paa_dy_cpi = paa.dy_cpi;
	
	/* Convert PAA delta to mm */
	float paa_dx_mm = (paa_dx_cpi * 25.4f) / paa.resolution;
	float paa_dy_mm = (paa_dy_cpi * 25.4f) / paa.resolution;
	
	/* Compensate PAA delta for IMU rotation */
	quaternion_t q = {
		.q0 = lsm_data.quat.q0,
		.q1 = lsm_data.quat.q1,
		.q2 = lsm_data.quat.q2,
		.q3 = lsm_data.quat.q3
	};
	
	float compensated_dx, compensated_dy;
	compensate_paa_delta(paa_dx_mm, paa_dy_mm, q, &compensated_dx, &compensated_dy);
	
	/* Convert mm to m for Kalman filter */
	float z_x = compensated_dx / 1000.0f;
	float z_y = compensated_dy / 1000.0f;
	
	/* Convert accel mg to m/s² */
	float ax = (lsm_data.accel.x / 1000.0f) * 9.81f;
	float ay = (lsm_data.accel.y / 1000.0f) * 9.81f;
	
	/* Kalman prediction and update */
	kalman_predict(&kalman, ax, ay, lsm_data.timestamp_us);
	kalman_update_position(&kalman, z_x, z_y, 0.005f);  /* Assume ~5ms per sample */
	
	/* Get filtered state */
	kalman_state_t state = kalman_get_state(&kalman);
	
	/* Output data (convert back to mm/s for output) */
	float x_mm = state.x * 1000.0f;
	float y_mm = state.y * 1000.0f;
	float vx_mms = state.vx * 1000.0f;
	float vy_mms = state.vy * 1000.0f;
	
	output_send(x_mm, y_mm, vx_mms, vy_mms, q.q0, q.q1, q.q2, q.q3);
	
	/* Debug output (10 Hz) */
	if (DEBUG_UART && (lsm_data.timestamp_us - last_print_us) > 100000) {
		printf("POS: %.1f, %.1f | VEL: %.1f, %.1f | Q: %.2f, %.2f, %.2f, %.2f\n",
		       x_mm, y_mm, vx_mms, vy_mms, q.q0, q.q1, q.q2, q.q3);
		last_print_us = lsm_data.timestamp_us;
	}
}
