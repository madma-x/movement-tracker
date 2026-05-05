#include "output.h"
#include <string.h>

static I2C_HandleTypeDef *_hi2c = NULL;

static uint8_t _tx_buf[sizeof(output_frame_t)];
static uint8_t _rx_dummy[1];

/* -------------------------------------------------------------------------
 * float16 helpers (used by runtime.c for SFLP quaternion decoding)
 * ------------------------------------------------------------------------- */

uint16_t float32_to_float16(float f) {
    uint32_t f32;
    memcpy(&f32, &f, 4);
    uint16_t sign = (f32 >> 31) & 0x1;
    int32_t  exp  = ((f32 >> 23) & 0xFF) - 127;
    uint32_t mant = f32 & 0x7FFFFF;
    if (exp < -14) return (uint16_t)(sign << 15);
    if (exp > 15)  exp = 15;
    uint16_t exp_adj  = (uint16_t)((exp + 15) & 0x1F);
    uint16_t mant_red = (uint16_t)((mant >> 13) & 0x3FF);
    return (uint16_t)((sign << 15) | (exp_adj << 10) | mant_red);
}

float float16_to_float32(uint16_t h) {
    uint32_t sign = (h >> 15) & 0x1;
    uint32_t exp  = (h >> 10) & 0x1F;
    uint32_t mant = h & 0x3FF;
    uint32_t exp_adj = (exp - 15 + 127) & 0xFF;
    uint32_t f32 = (sign << 31) | (exp_adj << 23) | (mant << 13);
    float f;
    memcpy(&f, &f32, 4);
    return f;
}

/* -------------------------------------------------------------------------
 * I2C slave
 * ------------------------------------------------------------------------- */

void output_init(I2C_HandleTypeDef *hi2c) {
    _hi2c = hi2c;
    memset(_tx_buf, 0, sizeof(_tx_buf));
    HAL_I2C_EnableListen_IT(_hi2c);
}

/**
 * @brief Call every main loop iteration.
 * Re-arms the slave listen if the peripheral drifted back to READY.
 */
void output_process(void) {
    if (_hi2c == NULL) return;
    if (HAL_I2C_GetState(_hi2c) == HAL_I2C_STATE_READY) {
        HAL_I2C_EnableListen_IT(_hi2c);
    }
}

/* -------------------------------------------------------------------------
 * HAL I2C slave callbacks
 * ------------------------------------------------------------------------- */

/**
 * @brief Address-match callback: arm TX or RX depending on master direction.
 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    (void)AddrMatchCode;
    if (hi2c != _hi2c) return;
    if (TransferDirection == I2C_DIRECTION_RECEIVE) {
        /* Master wants to read from us — send the motion frame */
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, _tx_buf, sizeof(_tx_buf), I2C_LAST_FRAME);
    } else {
        /* Master is writing to us — absorb one byte then re-listen */
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, _rx_dummy, 1, I2C_NEXT_FRAME);
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != _hi2c) return;
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != _hi2c) return;
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != _hi2c) return;
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != _hi2c) return;
    /* Clear any error and re-arm — AF (NACK at end of master read) is normal */
    HAL_I2C_EnableListen_IT(hi2c);
}

/**
 * @brief Update the transmit buffer. Safe to call from main loop.
 */
void output_send(float x_m, float y_m, float vx_ms, float vy_ms, float yaw_rad) {
    output_frame_t frame;
    frame.x       = x_m;
    frame.y       = y_m;
    frame.vx      = vx_ms;
    frame.vy      = vy_ms;
    frame.yaw_rad = yaw_rad;
    /* Disable IRQ briefly so the ISR never reads a half-written frame */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(_tx_buf, &frame, sizeof(_tx_buf));
    __set_PRIMASK(primask);
}
