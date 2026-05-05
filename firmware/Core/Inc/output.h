#ifndef INC_OUTPUT_H_
#define INC_OUTPUT_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"

/* Output frame: x, y, vx, vy, yaw_rad — 20 bytes, all float32 */
typedef struct {
    float x;         /* Position X (m) */
    float y;         /* Position Y (m) */
    float vx;        /* Velocity X (m/s) */
    float vy;        /* Velocity Y (m/s) */
    float yaw_rad;   /* SFLP yaw angle (radians) */
} output_frame_t;

/**
 * @brief Initialize I2C slave at address 0x42 and start listening.
 */
void output_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Call every main loop iteration to keep the slave alive.
 */
void output_process(void);

/**
 * @brief Update the transmit buffer (non-blocking, safe to call at 50 Hz).
 */
void output_send(float x_m, float y_m, float vx_ms, float vy_ms, float yaw_rad);

/* float16 helpers used by runtime.c */
uint16_t float32_to_float16(float f);
float    float16_to_float32(uint16_t h);

#endif /* INC_OUTPUT_H_ */
