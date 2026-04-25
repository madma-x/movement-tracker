#ifndef INC_OUTPUT_H_
#define INC_OUTPUT_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"

/* Output frame format (20 bytes total) */
typedef struct {
    float x;         /* Position X (mm) */
    float y;         /* Position Y (mm) */
    float vx;        /* Velocity X (mm/s) */
    float vy;        /* Velocity Y (mm/s) */
    uint16_t q0;     /* Quaternion q0 as float16 */
    uint16_t q1;     /* Quaternion q1 as float16 */
    uint16_t q2;     /* Quaternion q2 as float16 */
    uint16_t q3;     /* Quaternion q3 as float16 */
} output_frame_t;

/* CAN frame IDs */
#define OUTPUT_CAN_ID_POSVEL   0x100  /* Position + Velocity frame */
#define OUTPUT_CAN_ID_ORIENT   0x101  /* Orientation (quaternion) frame */

/**
 * @brief Initialize output drivers (I2C, CAN)
 */
int output_init(I2C_HandleTypeDef *hi2c, FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief Send position/velocity/orientation data via I2C and CAN
 */
int output_send(float x_mm, float y_mm, float vx_mms, float vy_mms,
                float q0, float q1, float q2, float q3);

/**
 * @brief Convert float32 to float16 (half precision)
 * Simplified conversion (not IEEE 754 compliant, but close enough for telemetry)
 */
uint16_t float32_to_float16(float f);

/**
 * @brief Convert float16 to float32
 */
float float16_to_float32(uint16_t h);

#endif /* INC_OUTPUT_H_ */
