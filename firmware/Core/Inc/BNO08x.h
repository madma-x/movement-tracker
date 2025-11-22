#ifndef INC_BNO08X_H_
#define INC_BNO08X_H_

#include <stdbool.h>

#include "main.h"

#include "SH2_Inc/sh2_err.h"
#include "SH2_Inc/sh2.h"
#include "SH2_Inc/sh2_SensorValue.h"

// Additional Activities not listed in SH-2 lib
#define PAC_ON_STAIRS 8 ///< Activity code for being on stairs
#define PAC_OPTION_COUNT 9 ///< The number of current options for the activity classifier

// IMPORTANT NOTE : Sadly, CEVA's SH2 library was not designed with multiple sensors in mind. Thus, as this library relies on it, it only supports one sensor

typedef enum {
	bno_ok = SH2_OK,
	bno_err = SH2_ERR,
	bno_bad_param = SH2_ERR_BAD_PARAM,
	bno_op_in_progress = SH2_ERR_OP_IN_PROGRESS,
	bno_io = SH2_ERR_IO,
	bno_hub = SH2_ERR_HUB,
	bno_timeout = SH2_ERR_TIMEOUT,

	bno_decod,
} bno_err_t;

static char* bno_reset_reason[] = {"not applicable","power on","internal system reset","watchdog timeout", "external reset", "other"};

typedef struct { // Note : this structure is only used as a descriptor for all used pins and interface, to be fed to bnoInit, any instance of it can be destroyed after calling the function
	SPI_HandleTypeDef* spi;

	GPIO_TypeDef *NCS_Port;
	uint16_t NCS_Pin;

	GPIO_TypeDef *NINT_Port;
	uint16_t NINT_Pin;

	GPIO_TypeDef *NRST_Port;
	uint16_t NRST_Pin;
} bno08x_t;

// ========== High level ========== //

bno_err_t bnoInit(bno08x_t* b);
bool bnoProcess();

// ========== State getters ========== //

uint8_t bnoWasReset(); // returns the reason, if there is one (see bno_reset_reason above)
sh2_ProductIds_t* bnoGetProdIds();

// ========== Setters/Getters ========== //

bool bnoEnableReportInterval(sh2_SensorId_t sensorId, uint32_t interval_us);
bool bnoEnableReport(sh2_SensorId_t sensorId);

// Convert a rotation-vector quaternion contained in a sensor value to yaw (radians)
float bnoRotationVectorToYaw(const sh2_SensorValue_t *value);

// Normalize an angle (radians) into the range (-PI, PI]
float bnoNormalizeAngle(float a);

#endif /* INC_BNO08X_H_ */
