#ifndef INC_LSM6DSV_H_
#define INC_LSM6DSV_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"

/* LSM6DSV Register Map */
#define LSM6DSV_WHO_AM_I            0x0F
#define LSM6DSV_WHO_AM_I_VAL         0x70

/* CTRL Registers */
#define LSM6DSV_CTRL1_XL            0x10  /* Accel control */
#define LSM6DSV_CTRL2_G             0x11  /* Gyro control */
#define LSM6DSV_CTRL3_C             0x12  /* General control */
#define LSM6DSV_CTRL4_C             0x13
#define LSM6DSV_CTRL5_C             0x14
#define LSM6DSV_CTRL6_C             0x15
#define LSM6DSV_CTRL7_G             0x16
#define LSM6DSV_CTRL8_XL            0x17
#define LSM6DSV_CTRL9_XL            0x18
#define LSM6DSV_CTRL10_C            0x19
#define LSM6DSV_FUNC_CFG_ACCESS     0x01  /* Register bank select */

/* SFLP (Sensor Fusion Low Power) - Game Rotation Vector registers */
#define LSM6DSV_SFLP_ODR            0x5E  /* SFLP output data rate config */
#define LSM6DSV_SFLP_GAME_RV_QI     0x08  /* Quaternion I (imaginary X) LSB */
#define LSM6DSV_SFLP_GAME_RV_QJ     0x0A  /* Quaternion J (imaginary Y) LSB */
#define LSM6DSV_SFLP_GAME_RV_QK     0x0C  /* Quaternion K (imaginary Z) LSB */
#define LSM6DSV_SFLP_GAME_RV_QR     0x0E  /* Quaternion R (real/scalar) LSB */

/* Data Registers */
#define LSM6DSV_OUTX_L_G            0x22  /* Gyro X LSB */
#define LSM6DSV_OUTX_H_G            0x23  /* Gyro X MSB */
#define LSM6DSV_OUTY_L_G            0x24  /* Gyro Y LSB */
#define LSM6DSV_OUTY_H_G            0x25  /* Gyro Y MSB */
#define LSM6DSV_OUTZ_L_G            0x26  /* Gyro Z LSB */
#define LSM6DSV_OUTZ_H_G            0x27  /* Gyro Z MSB */

#define LSM6DSV_OUTX_L_A            0x28  /* Accel X LSB */
#define LSM6DSV_OUTX_H_A            0x29  /* Accel X MSB */
#define LSM6DSV_OUTY_L_A            0x2A  /* Accel Y LSB */
#define LSM6DSV_OUTY_H_A            0x2B  /* Accel Y MSB */
#define LSM6DSV_OUTZ_L_A            0x2C  /* Accel Z LSB */
#define LSM6DSV_OUTZ_H_A            0x2D  /* Accel Z MSB */

/* Status Register */
#define LSM6DSV_STATUS              0x1D
#define LSM6DSV_STATUS_XLDA         0x01  /* Accel data available */
#define LSM6DSV_STATUS_GDA          0x02  /* Gyro data available */

/* FIFO Registers */
#define LSM6DSV_FIFO_STATUS1        0x3A
#define LSM6DSV_FIFO_STATUS2        0x3B

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *NCS_Port;
    uint16_t NCS_Pin;
} lsm6dsv_t;

/* Data structures */
typedef struct {
    int16_t x, y, z;  /* Raw 16-bit values */
} lsm6dsv_raw_data_t;

typedef struct {
    float x, y, z;  /* Converted to standard units (mg for accel, dps for gyro) */
} lsm6dsv_float_data_t;

typedef struct {
    float q0, q1, q2, q3;  /* Quaternion (game rotation vector from SFLP) */
} lsm6dsv_quat_t;

typedef struct {
    lsm6dsv_float_data_t accel;  /* Acceleration in mg */
    lsm6dsv_float_data_t gyro;   /* Angular velocity in dps */
    lsm6dsv_quat_t quat;         /* Quaternion */
    uint32_t timestamp_us;
} lsm6dsv_data_t;

/* Function prototypes */
int lsm6dsvInit(lsm6dsv_t *dev);
int lsm6dsvRead(lsm6dsv_t *dev, lsm6dsv_data_t *data);
int lsm6dsvWriteReg(lsm6dsv_t *dev, uint8_t reg, uint8_t value);
int lsm6dsvReadReg(lsm6dsv_t *dev, uint8_t reg, uint8_t *value);
int lsm6dsvReadRegs(lsm6dsv_t *dev, uint8_t reg, uint8_t *data, uint8_t len);

#endif /* INC_LSM6DSV_H_ */
