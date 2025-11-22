#include "runtime.h"
#include "main.h"

#include "PAA5163.h"
#include "BNO08x.h"
#include "micros.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

paa5163_t paa = {
	.spi = &hspi1,

	.NCS_Port = CS_PAA_GPIO_Port,
	.NCS_Pin = CS_PAA_Pin,
	.NRST_Port = RST_PAA_GPIO_Port,
	.NRST_Pin = RST_PAA_Pin,

	.invert_x = 1,
};

bno08x_t bno = {
	.spi = &hspi1,

	.NCS_Port = CS_IMU_GPIO_Port,
	.NCS_Pin = CS_IMU_Pin,
	.NINT_Port = INT_IMU_GPIO_Port,
	.NINT_Pin = INT_IMU_Pin,
	.NRST_Port = RST_IMU_GPIO_Port,
	.NRST_Pin = RST_IMU_Pin,
};
sh2_SensorValue_t sensorValue;

// flag set by EXTI when BNO indicates data ready (set in ISR)
static volatile bool imu_data_ready = false;
// last measured duration of a bnoProcess() call (microseconds)
static uint32_t last_bno_proc_us = 0;
// state for odometry/heading
static float theta_mid_prev = 0.0f;
static float theta_mid = 0.0f;
static float yaw_prev = 0.0f;
static float x = 0.0f;
static float y = 0.0f;

#define constrain(x, floor, ceiling) ((x < floor ? floor : x) > ceiling ? ceiling : x)

// WARNING : Both libraries could attempt to access the spi bus at the same time if read operations are done inside the interrupts !

void setBnoReports(){
	printf("Setting reports... ");
	if (!bnoEnableReportInterval(SH2_ROTATION_VECTOR, 2500)) {
		printf("ERROR\n");
		return;
	}
	printf("OK\n");
}

// EXTI callback from HAL - set a flag for main loop to process the IMU.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == INT_IMU_Pin) {
		imu_data_ready = true;
	}
}

void setup(void){
	printf("BNO Init... ");
	bno_err_t bno_init_exit = bnoInit(&bno);
	printf("%d (%s) \nPAA Init... ", bno_init_exit, (bno_init_exit == bno_ok ? "OK" : "ERROR"));
	paa_err_t paa_init_exit = paaInit(&paa);
	printf("%d (%s)\n",paa_init_exit, (paa_init_exit == paa_ok ? "OK" : "ERROR"));

	if(paa_init_exit != paa_ok || bno_init_exit != bno_ok) { // for now, if one of the sensors could not be initialized properly, reboot to try again
		printf("FATAL : a sensor could not be initialized, rebooting...\n");
		HAL_Delay(500);
		NVIC_SystemReset();
	}

	sh2_ProductIds_t* ids = bnoGetProdIds();
	printf("BNO080 Sensors :\n");
	for (int n = 0; n < ids->numEntries; n++) {
		printf("\tPart %ld, ",ids->entry[n].swPartNumber);
		printf("Version %d.%d.%d, ",ids->entry[n].swVersionMajor,ids->entry[n].swVersionMinor,ids->entry[n].swVersionPatch);
		printf("Build %ld\n",ids->entry[n].swBuildNumber);
	}

	setBnoReports();
}

void loop(void){

	// Run a 400Hz scheduler (every 2500us). On each tick:
	// 1) If IMU EXTI indicated data ready, process BNO and measure its processing time.
	// 2) Then call paaReadMotion() to fetch optical deltas.

		// Handle BNO only when EXTI has signaled data ready. Keep ISR minimal.
		if (imu_data_ready) {
			imu_data_ready = false;
			uint32_t bno_start = micros();
			if (bnoProcess(&sensorValue)) {
				uint32_t bno_end = micros();
				last_bno_proc_us = (uint32_t)(bno_end - bno_start);
				// Read optical sensor after handling IMU
				paaReadMotion(&paa);
				
				//if delay is around 2.5ms and the loop is at 400hz, we can use shift the yaw
				//reading by 1 period and have a output delay of 2.5ms
				//it is better to change the imu and do an ekf without any delay at maximum
				//supported rate on the SPI bus. 
				float yaw = bnoRotationVectorToYaw(&sensorValue);
				theta_mid_prev = theta_mid;
    			float theta_mid = yaw_prev + 0.5f * yaw;
				yaw_prev = bnoNormalizeAngle(yaw);
				
    			float c = cosf(theta_mid_prev);
    			float s = sinf(theta_mid_prev);
				float dx_world = paa.dx * c - paa.dy * s;
    			float dy_world = paa.dx * s + paa.dy * c;
				x += dx_world;
				y += dy_world;

				
				printf("BNO process time: %lu us\n", (unsigned long)last_bno_proc_us);
				printf("delay: %lu \n",(unsigned long)sensorValue.delay);
				printf("Yaw: %.3f rad, X: %.3f mm, Y: %.3f mm\n", yaw, x, y);



			}
		}

	uint8_t rst = bnoWasReset();
	if (rst) {
		printf("Reset : %s (%d)\n", bno_reset_reason[constrain(rst, 0, 5)], rst);
		setBnoReports();
	}

}
