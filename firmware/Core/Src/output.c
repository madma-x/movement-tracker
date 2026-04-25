#include "output.h"
#include <string.h>
#include <math.h>

static I2C_HandleTypeDef *_hi2c = NULL;
static FDCAN_HandleTypeDef *_hfdcan = NULL;

#define I2C_TARGET_ADDR  0x42  /* 7-bit address */
#define I2C_TIMEOUT_MS   100

/**
 * @brief Convert float32 to float16
 * Simplified: just truncate mantissa, preserve exponent and sign
 */
uint16_t float32_to_float16(float f) {
    uint32_t f32 = *(uint32_t*)&f;
    uint16_t sign = (f32 >> 31) & 0x1;
    int32_t exp = ((f32 >> 23) & 0xFF) - 127;
    uint32_t mant = f32 & 0x7FFFFF;
    
    /* Clamp exponent to float16 range [-14, 15] */
    if (exp < -14) return 0;  /* Underflow to zero */
    if (exp > 15) exp = 15;   /* Overflow to max */
    
    uint16_t exp_adj = (exp + 15) & 0x1F;
    uint16_t mant_reduced = (mant >> 13) & 0x3FF;
    
    uint16_t f16 = (sign << 15) | (exp_adj << 10) | mant_reduced;
    return f16;
}

/**
 * @brief Convert float16 to float32
 */
float float16_to_float32(uint16_t h) {
    uint32_t sign = (h >> 15) & 0x1;
    uint32_t exp = (h >> 10) & 0x1F;
    uint32_t mant = h & 0x3FF;
    
    uint32_t exp_adj = (exp - 15 + 127) & 0xFF;
    uint32_t mant_expanded = mant << 13;
    
    uint32_t f32_bits = (sign << 31) | (exp_adj << 23) | mant_expanded;
    float f = *(float*)&f32_bits;
    return f;
}

/**
 * @brief Initialize output module
 */
int output_init(I2C_HandleTypeDef *hi2c, FDCAN_HandleTypeDef *hfdcan) {
    _hi2c = hi2c;
    _hfdcan = hfdcan;
    
    if (_hfdcan != NULL) {
        /* Start CAN reception if needed */
        HAL_FDCAN_Start(_hfdcan);
    }
    
    return 0;
}

/**
 * @brief Send via I2C as master
 * Transmits a 20-byte frame to address 0x42
 */
static int _send_i2c(const output_frame_t *frame) {
    if (_hi2c == NULL) return -1;
    
    uint8_t data[20];
    uint32_t offset = 0;
    
    /* Serialize frame: x, y, vx, vy (4 floats = 16 bytes) */
    memcpy(&data[offset], &frame->x, 4);
    offset += 4;
    memcpy(&data[offset], &frame->y, 4);
    offset += 4;
    memcpy(&data[offset], &frame->vx, 4);
    offset += 4;
    memcpy(&data[offset], &frame->vy, 4);
    offset += 4;
    
    /* Quaternion as 4x float16 (8 bytes) */
    memcpy(&data[offset], &frame->q0, 2);
    offset += 2;
    memcpy(&data[offset], &frame->q1, 2);
    offset += 2;
    memcpy(&data[offset], &frame->q2, 2);
    offset += 2;
    memcpy(&data[offset], &frame->q3, 2);
    offset += 2;
    
    /* Send via I2C master mode */
    return HAL_I2C_Master_Transmit(_hi2c, I2C_TARGET_ADDR << 1, data, 20, I2C_TIMEOUT_MS);
}

/**
 * @brief Send via CAN (split into 2 frames)
 * Frame 1 (ID 0x100): x, y, vx, vy (as floats fit 2 per 8-byte frame)
 * Frame 2 (ID 0x101): q0, q1, q2, q3 (as float16, 2 per 4-byte CAN word)
 */
static int _send_can(const output_frame_t *frame) {
    if (_hfdcan == NULL) return -1;
    
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];
    
    /* Frame 1: Position and velocity (2 floats = 8 bytes) */
    txHeader.Identifier = OUTPUT_CAN_ID_POSVEL;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENT_FIFO;
    
    memcpy(&txData[0], &frame->x, 4);
    memcpy(&txData[4], &frame->y, 4);
    
    uint32_t txFifoIndex;
    if (HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &txHeader, txData, &txFifoIndex) != HAL_OK) {
        return -1;
    }
    
    /* Frame 2: Velocity X/Y as floats (8 bytes) */
    txHeader.Identifier = OUTPUT_CAN_ID_POSVEL + 1;  /* Next frame for vx, vy */
    memcpy(&txData[0], &frame->vx, 4);
    memcpy(&txData[4], &frame->vy, 4);
    
    if (HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &txHeader, txData, &txFifoIndex) != HAL_OK) {
        return -1;
    }
    
    /* Frame 3: Quaternion (4x float16 = 8 bytes) */
    txHeader.Identifier = OUTPUT_CAN_ID_ORIENT;
    memcpy(&txData[0], &frame->q0, 2);
    memcpy(&txData[2], &frame->q1, 2);
    memcpy(&txData[4], &frame->q2, 2);
    memcpy(&txData[6], &frame->q3, 2);
    
    if (HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &txHeader, txData, &txFifoIndex) != HAL_OK) {
        return -1;
    }
    
    return 0;
}

/**
 * @brief Send output (I2C + CAN)
 */
int output_send(float x_mm, float y_mm, float vx_mms, float vy_mms,
                float q0, float q1, float q2, float q3) {
    output_frame_t frame;
    
    frame.x = x_mm;
    frame.y = y_mm;
    frame.vx = vx_mms;
    frame.vy = vy_mms;
    frame.q0 = float32_to_float16(q0);
    frame.q1 = float32_to_float16(q1);
    frame.q2 = float32_to_float16(q2);
    frame.q3 = float32_to_float16(q3);
    
    /* Try both, don't fail if one fails */
    int i2c_ret = _send_i2c(&frame);
    int can_ret = _send_can(&frame);
    
    if (i2c_ret != HAL_OK && can_ret != HAL_OK) {
        return -1;  /* Both failed */
    }
    
    return 0;
}
