#include "main.h"
#include "BNO08x.h"

#include "SH2_Inc/sh2_hal.h"
#include "SH2_Inc/sh2.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef MODIFIED_SHTP_SH2
#error Modification to file required ! sh2.c -> executableDeviceHdlr -> add "sh2AsyncEvent.shtpEvent = payload[1];" below "sh2AsyncEvent.eventId = SH2_RESET;" in the "EXECUTABLE_DEVICE_RESP_RESET_COMPLETE" case
#endif


#define SPI_TIMEOUT 1000 // ms, timeout fed to HAL_SPI functions
#define INT_TIMEOUT 500 // ms, time before the sensor is considered unresponsive when the program expects it to assert INT

// ==================== Variables ==================== //

SPI_HandleTypeDef* _spi;

GPIO_TypeDef *_NCS_Port;
uint16_t _NCS_Pin;

GPIO_TypeDef *_NINT_Port;
uint16_t _NINT_Pin;

GPIO_TypeDef *_NRST_Port;
uint16_t _NRST_Pin;

struct sh2_Hal_s _hal;

sh2_ProductIds_t prod_ids = {0};

static sh2_SensorValue_t *_sensor_value = NULL;

uint8_t _rst_reason = 0;

// ==================== Hardware abstraction ==================== //

// CS Pin
static inline void _select(){
	HAL_GPIO_WritePin(_NCS_Port, _NCS_Pin, GPIO_PIN_RESET);
}
static inline void _deselect(){
	HAL_GPIO_WritePin(_NCS_Port, _NCS_Pin, GPIO_PIN_SET);
}

// INT Pin
static inline bool _sensorReady(){
	return !HAL_GPIO_ReadPin(_NINT_Port, _NINT_Pin);
}
static bno_err_t _waitForSensRdy(uint16_t timeout){
	for(uint16_t i = 0; i < timeout; i++){
		if(_sensorReady()){
			//printf("waited %d\n", i);
			return bno_ok; // sensor is still not ready
		}
		HAL_Delay(1);
	}
	return bno_timeout;
	/*
	uint32_t t = HAL_GetTick();
	while(!_sensorReady() && HAL_GetTick() - t < timeout);

	if(!_sensorReady()) return bno_timeout; // sensor is still not ready

	return bno_ok;
	*/
}

// RESET Pin
static void _reset(){
	HAL_GPIO_WritePin(_NRST_Port, _NRST_Pin, GPIO_PIN_RESET);
}
static void _release(){
	HAL_GPIO_WritePin(_NRST_Port, _NRST_Pin, GPIO_PIN_SET);
}

// ==================== Low Level ==================== //

uint32_t _getTimeUs(sh2_Hal_t *self){
	return HAL_GetTick() * 1000;
}

static int _write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len){
	if(_waitForSensRdy(INT_TIMEOUT) != bno_ok) return 0;

	_select();

	if(HAL_SPI_Transmit(_spi, pBuffer, len, SPI_TIMEOUT) != HAL_OK){
		len = 0;
	}

	_deselect();

	return len;
}

static int _read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us){
	if(_waitForSensRdy(INT_TIMEOUT) != bno_ok) return 0;

	uint16_t packet_size = 0;

	_select();
	if(HAL_SPI_Receive(_spi, pBuffer, 4, SPI_TIMEOUT) != HAL_OK){
		_deselect();
		return 0;
	}
	_deselect();

	// Determine amount to read
	packet_size = (uint16_t)pBuffer[0] | (uint16_t)pBuffer[1] << 8;
	// Unset the "continue" bit
	packet_size &= ~0x8000;

	if (packet_size > len){
		return 0;
	}

	if(_waitForSensRdy(INT_TIMEOUT) != bno_ok){
		printf("no read\n");
		return 0;
	}

	_select();
	if(HAL_SPI_Receive(_spi, pBuffer, packet_size, SPI_TIMEOUT) != HAL_OK){
		_deselect();
		return 0;
	}
	_deselect();

	//if(packet_size > 100)printf("%d\n", packet_size);

	//*t_us = _getTimeUs(self);

	return packet_size;
}

static int _open(sh2_Hal_t *self){
	uint8_t dummy[1] = {0};
	HAL_SPI_Transmit(_spi, dummy, 1, SPI_TIMEOUT);
	HAL_SPI_Receive(_spi, dummy, 1, SPI_TIMEOUT); // make sure the SPI interface is in a knwon state

	_release(); // turn on

	_waitForSensRdy(500); // wait for the sensor to respond

	return 0;
}

static void _close(sh2_Hal_t *self){
	_reset(); // turn off the sensor
	_deselect();
}

// non-sensor events
static void _eventCallback(void *cookie, sh2_AsyncEvent_t *pEvent) {
	// If we see a reset, set a flag so that sensors will be reconfigured.
	switch (pEvent->eventId) {
	case SH2_RESET :
		printf("EventHandler : RST (%d)\n", pEvent->shtpEvent);
		//printf("RST, %d\n",pEvent->shtpEvent);
	    _rst_reason = pEvent->shtpEvent;
		break;
	case SH2_SHTP_EVENT :
		printf("EventHandler : SHTP %d\n", pEvent->shtpEvent);
		break;
	case SH2_GET_FEATURE_RESP :
		printf("EventHandler : Set feature ACK \n");

		//printf("Response to set reports :");
		//printf("id %d", pEvent->sh2SensorConfigResp.sensorId);
		//printf("change sensitivity : %d, en %d, rela %d\n", pEvent->sh2SensorConfigResp.sensorConfig.changeSensitivity, pEvent->sh2SensorConfigResp.sensorConfig.changeSensitivityEnabled, pEvent->sh2SensorConfigResp.sensorConfig.changeSensitivityRelative);
		//printf("wkup en %d\n", pEvent->sh2SensorConfigResp.sensorConfig.wakeupEnabled);
		//printf("alw on en %d\n", pEvent->sh2SensorConfigResp.sensorConfig.alwaysOnEnabled);
		//printf("rprt interval %ld, batch interval %ld (us)\n", pEvent->sh2SensorConfigResp.sensorConfig.reportInterval_us, pEvent->sh2SensorConfigResp.sensorConfig.batchInterval_us);
		//printf("sensor specific %ld\n", pEvent->sh2SensorConfigResp.sensorConfig.sensorSpecific);
		break;
	default :
		printf("EventHandler : unknown event, Id : %ld\n", pEvent->eventId);
		break;
	}
}

// sensor events (data), with event given by the lib when sensor is serviced and _sensor_value the memory space to store the information
static void _reportHandler(void *cookie, sh2_SensorEvent_t *event) {
	//printf("report\n");
	if (sh2_decodeSensorEvent(_sensor_value, event) != SH2_OK) {
		printf("err reading event\n");
		_sensor_value->timestamp = 0;
		return;
  }
}

// ==================== High Level ==================== //

bno_err_t bnoInit(bno08x_t* b){
	_reset(); // turn off the sensor for now
	_deselect();

	_spi = b->spi;
	_NCS_Port = b->NCS_Port;
	_NCS_Pin = b->NCS_Pin;
	_NINT_Port = b->NINT_Port;
	_NINT_Pin = b->NINT_Pin;
	_NRST_Port = b->NRST_Port;
	_NRST_Pin = b->NRST_Pin;

	_hal.write = _write;
	_hal.read = _read;
	_hal.open = _open;
	_hal.close = _close;
	_hal.getTimeUs = _getTimeUs;

	bno_err_t err = 0;

	// Open SH2 interface (also registers non-sensor event handler.)
	err = sh2_open(&_hal, _eventCallback, NULL);
	if(err != SH2_OK) return err;

	// Register sensor listener
	sh2_setSensorCallback(_reportHandler, NULL);

	// Check connection partially by getting the product id's
	err = sh2_getProdIds(&prod_ids);
	if(err != SH2_OK) return err;

	_rst_reason = 0; // we get a false positive when the senor boots up

	return bno_ok;
}

bool bnoProcess(sh2_SensorValue_t *value) {
	_sensor_value = value;

	value->timestamp = 0;

	sh2_service();

	if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
		// no new events
		return false;
	}

	return true;
}

void bnoGetData(){

}

// ==================== State Getters ==================== //

uint8_t bnoWasReset(){
	uint8_t reason = _rst_reason;
	_rst_reason = 0;
	return reason;
}

sh2_ProductIds_t* bnoGetProdIds(){
	return &prod_ids;
}

// ==================== Setters/Getters ==================== //

/**
 * @brief Enable the given report type
 *
 * @param sensorId The report ID to enable
 * @param interval_us The update interval for reports to be generated, in
 * microseconds
 * @return true: success false: failure
 */
bool bnoEnableReportInterval(sh2_SensorId_t sensorId, uint32_t interval_us) {
  static sh2_SensorConfig_t config = {0};

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = 0;

  config.reportInterval_us = interval_us;

  if (sh2_setSensorConfig(sensorId, &config) != SH2_OK)
	  return false;

  return true;
}
bool bnoEnableReport(sh2_SensorId_t sensorId) {
	return bnoEnableReportInterval(sensorId, 10000);
}

/**
 * @brief Convert a rotation-vector quaternion (from sh2_SensorValue_t) to yaw (radians).
 *
 * Uses the quaternion-to-yaw conversion (Z axis rotation):
 *   yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
 * The quaternion is normalized before conversion.
 *
 * @param value Pointer to a populated sh2_SensorValue_t whose .un.rotationVector fields are valid
 * @return yaw angle in radians
 */
float bnoRotationVectorToYaw(const sh2_SensorValue_t *value){
	if(value == NULL) return 0.0f;
	float w = value->un.rotationVector.real;
	float x = value->un.rotationVector.i;
	float y = value->un.rotationVector.j;
	float z = value->un.rotationVector.k;

	float norm = sqrtf(w*w + x*x + y*y + z*z);
	if(norm == 0.0f) return 0.0f;
	w /= norm; x /= norm; y /= norm; z /= norm;

	float t3 = 2.0f * (w * z + x * y);
	float t4 = 1.0f - 2.0f * (y * y + z * z);
	return atan2f(t3, t4);
}

/**
 * @brief Normalize angle to (-PI, PI]
 *
 * @param a angle in radians
 * @return normalized angle in radians
 */
float bnoNormalizeAngle(float a){
    while (a > M_PI) a -= 2.0f * M_PI;
    while (a <= -M_PI) a += 2.0f * M_PI;
    return a;
}

