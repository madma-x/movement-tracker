#include "lsm6dsv.h"
#include "main.h"
#include "micros.h"
#include <math.h>

/* Private functions */
static void _ncsLow(lsm6dsv_t *dev) {
    HAL_GPIO_WritePin(dev->NCS_Port, dev->NCS_Pin, GPIO_PIN_RESET);
}

static void _ncsHigh(lsm6dsv_t *dev) {
    HAL_GPIO_WritePin(dev->NCS_Port, dev->NCS_Pin, GPIO_PIN_SET);
}

static int _spiTransfer(lsm6dsv_t *dev, uint8_t *txData, uint8_t *rxData, uint16_t len) {
    return HAL_SPI_TransmitReceive(dev->spi, txData, rxData, len, 1000);
}

/* Public functions */

/**
 * @brief Write a single register
 */
int lsm6dsvWriteReg(lsm6dsv_t *dev, uint8_t reg, uint8_t value) {
    uint8_t txBuf[2] = {reg & 0x7F, value};  /* Clear RW bit for write */
    uint8_t rxBuf[2];
    
    _ncsLow(dev);
    int ret = _spiTransfer(dev, txBuf, rxBuf, 2);
    _ncsHigh(dev);
    
    return ret;
}

/**
 * @brief Read a single register
 */
int lsm6dsvReadReg(lsm6dsv_t *dev, uint8_t reg, uint8_t *value) {
    uint8_t txBuf[2] = {reg | 0x80, 0x00};  /* Set RW bit for read */
    uint8_t rxBuf[2];
    
    _ncsLow(dev);
    int ret = _spiTransfer(dev, txBuf, rxBuf, 2);
    _ncsHigh(dev);
    
    if (ret == HAL_OK) {
        *value = rxBuf[1];
    }
    
    return ret;
}

/**
 * @brief Read multiple consecutive registers
 */
int lsm6dsvReadRegs(lsm6dsv_t *dev, uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t txBuf[len + 1];
    uint8_t rxBuf[len + 1];
    
    txBuf[0] = reg | 0x80;  /* Set RW bit for read, auto-increment */
    for (int i = 1; i <= len; i++) {
        txBuf[i] = 0x00;
    }
    
    _ncsLow(dev);
    int ret = _spiTransfer(dev, txBuf, rxBuf, len + 1);
    _ncsHigh(dev);
    
    if (ret == HAL_OK) {
        for (int i = 0; i < len; i++) {
            data[i] = rxBuf[i + 1];
        }
    }
    
    return ret;
}

/**
 * @brief Initialize LSM6DSV sensor
 * Configuration: Accel ±4g @ 208Hz, Gyro ±2000dps @ 208Hz, SFLP enabled
 */
int lsm6dsvInit(lsm6dsv_t *dev) {
    uint8_t whoami;
    
    /* Verify WHO_AM_I */
    if (lsm6dsvReadReg(dev, LSM6DSV_WHO_AM_I, &whoami) != HAL_OK) {
        return -1;  /* SPI communication error */
    }
    
    if (whoami != LSM6DSV_WHO_AM_I_VAL) {
        return -2;  /* Wrong device */
    }
    
    /* Soft reset via CTRL3_C */
    lsm6dsvWriteReg(dev, LSM6DSV_CTRL3_C, 0x01);  /* SW_RESET */
    HAL_Delay(100);
    
    /* Configure Accelerometer: ±4g @ 208Hz
     * CTRL1_XL: ODR_XL[3:0] = 0x6 (208Hz), FS_XL[1:0] = 0x0 (±2g -> use 0x2 for ±4g), LPF1_BW_SEL = 0
     * 0x60 = 0110 0000 = 208Hz, ±4g, no filter
     */
    if (lsm6dsvWriteReg(dev, LSM6DSV_CTRL1_XL, 0x62) != HAL_OK) {
        return -3;
    }
    
    /* Configure Gyroscope: ±2000dps @ 208Hz
     * CTRL2_G: ODR_G[3:0] = 0x6 (208Hz), FS_G[2:0] = 0x0 (±2000dps)
     * 0x60 = 0110 0000 = 208Hz, ±2000dps
     */
    if (lsm6dsvWriteReg(dev, LSM6DSV_CTRL2_G, 0x60) != HAL_OK) {
        return -4;
    }
    
    /* Enable BDU (Block Data Update) for atomic reads
     * CTRL3_C: BDU = 1 (bit 6)
     * 0x40 = 0100 0000
     */
    if (lsm6dsvWriteReg(dev, LSM6DSV_CTRL3_C, 0x40) != HAL_OK) {
        return -5;
    }
    
    /* Enable SFLP (Sensor Fusion Low Power) for game rotation vector
     * Bank 1 access required for EMB_FUNC registers
     */
    
    /* Switch to bank 1 via FUNC_CFG_ACCESS */
    if (lsm6dsvWriteReg(dev, LSM6DSV_FUNC_CFG_ACCESS, 0x40) != HAL_OK) {
        return -6;  /* Bank switch failed */
    }
    
    /* Enable SFLP game rotation vector in EMB_FUNC_EN_B (bank 1, register 0x05)
     * Bit 2 (0x04) = SFLP_GAME_RV enable
     */
    uint8_t emb_func_en_b = 0;
    if (lsm6dsvReadReg(dev, 0x05, &emb_func_en_b) != HAL_OK) {
        return -7;
    }
    if (lsm6dsvWriteReg(dev, 0x05, emb_func_en_b | 0x04) != HAL_OK) {
        return -8;
    }
    
    /* Switch back to bank 0 */
    if (lsm6dsvWriteReg(dev, LSM6DSV_FUNC_CFG_ACCESS, 0x00) != HAL_OK) {
        return -9;
    }
    
    /* Configure SFLP ODR.
     * Per datasheet format: [7]=0, [6]=1, [5:3]=SFLP_GAME_ODR, [2]=0, [1]=1, [0]=1
     * For 240 Hz, ODR bits are 100 -> 0b01100011 = 0x63.
     */
    if (lsm6dsvWriteReg(dev, LSM6DSV_SFLP_ODR, 0x63) != HAL_OK) {
        return -10;
    }
    
    /* Small delay to let sensor stabilize */
    HAL_Delay(50);
    
    return 0;  /* Success */
}

/**
 * @brief Convert raw 16-bit gyro to dps (degrees per second)
 * Full scale: ±2000dps, 16-bit range: ±32768
 * Sensitivity: 70 mdps/LSB for ±2000dps
 */
static float _rawGyroDps(int16_t raw) {
    return raw * 0.070f;  /* 70 mdps/LSB = 0.070 dps/LSB */
}

/**
 * @brief Convert raw 16-bit accel to mg (milligravities)
 * Full scale: ±4g, 16-bit range: ±32768
 * Sensitivity: 122 ug/LSB for ±4g
 * 1g = 1000 mg, so: 0.122 mg/LSB
 */
static float _rawAccelMg(int16_t raw) {
    return raw * 0.122f;  /* 122 ug/LSB = 0.122 mg/LSB */
}

/**
 * @brief Read sensor data: accel, gyro, and quaternion
 */
int lsm6dsvRead(lsm6dsv_t *dev, lsm6dsv_data_t *data) {
    uint8_t regData[12];  /* 6 bytes for gyro, 6 for accel */
    int16_t raw_gx, raw_gy, raw_gz;
    int16_t raw_ax, raw_ay, raw_az;
    
    /* Read gyro data (registers 0x22-0x27) */
    if (lsm6dsvReadRegs(dev, LSM6DSV_OUTX_L_G, regData, 6) != HAL_OK) {
        return -1;
    }
    
    raw_gx = (int16_t)((regData[1] << 8) | regData[0]);
    raw_gy = (int16_t)((regData[3] << 8) | regData[2]);
    raw_gz = (int16_t)((regData[5] << 8) | regData[4]);
    
    /* Read accel data (registers 0x28-0x2D) */
    if (lsm6dsvReadRegs(dev, LSM6DSV_OUTX_L_A, regData, 6) != HAL_OK) {
        return -2;
    }
    
    raw_ax = (int16_t)((regData[1] << 8) | regData[0]);
    raw_ay = (int16_t)((regData[3] << 8) | regData[2]);
    raw_az = (int16_t)((regData[5] << 8) | regData[4]);
    
    /* Convert to physical units */
    data->gyro.x = _rawGyroDps(raw_gx);
    data->gyro.y = _rawGyroDps(raw_gy);
    data->gyro.z = _rawGyroDps(raw_gz);
    
    data->accel.x = _rawAccelMg(raw_ax);
    data->accel.y = _rawAccelMg(raw_ay);
    data->accel.z = _rawAccelMg(raw_az);
    
    /* Read SFLP game rotation vector (quaternion as Q16 fixed-point: 16-bit signed) */
    uint8_t quat_regs[8];  /* 4 registers * 2 bytes each = 8 bytes for Q, I, J, K */
    if (lsm6dsvReadRegs(dev, LSM6DSV_SFLP_GAME_RV_QI, quat_regs, 8) == HAL_OK) {
        /* Q16 format: value / 32768.0 = normalized quaternion component */
        int16_t raw_qi = (int16_t)((quat_regs[1] << 8) | quat_regs[0]);
        int16_t raw_qj = (int16_t)((quat_regs[3] << 8) | quat_regs[2]);
        int16_t raw_qk = (int16_t)((quat_regs[5] << 8) | quat_regs[4]);
        int16_t raw_qr = (int16_t)((quat_regs[7] << 8) | quat_regs[6]);
        
        data->quat.q1 = raw_qi / 32768.0f;  /* i (imaginary X) */
        data->quat.q2 = raw_qj / 32768.0f;  /* j (imaginary Y) */
        data->quat.q3 = raw_qk / 32768.0f;  /* k (imaginary Z) */
        data->quat.q0 = raw_qr / 32768.0f;  /* r (real/scalar) */
    } else {
        /* Fallback to identity if read fails */
        data->quat.q0 = 1.0f;
        data->quat.q1 = 0.0f;
        data->quat.q2 = 0.0f;
        data->quat.q3 = 0.0f;
    }
    
    data->timestamp_us = micros();
    
    return 0;  /* Success */
}
