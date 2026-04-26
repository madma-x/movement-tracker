#ifndef LSM6DSV16X_HAL_GLUE_H
#define LSM6DSV16X_HAL_GLUE_H

#include "lsm6dsv16x_reg.h"
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void lsm6dsv16x_ctx_init(lsm6dsv16x_ctx_t *ctx, SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif // LSM6DSV16X_HAL_GLUE_H
