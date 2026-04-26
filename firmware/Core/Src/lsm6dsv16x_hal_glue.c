#include "lsm6dsv16x_reg.h"
#include "main.h"

// HAL SPI glue for ST API
static int32_t lsm6dsv16x_spi_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)handle;
    uint8_t tx = reg | 0x80;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // NCS low (adjust as needed)
    int32_t ret = HAL_SPI_Transmit(hspi, &tx, 1, 1000);
    if (ret == HAL_OK) ret = HAL_SPI_Receive(hspi, data, len, 1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // NCS high
    return (ret == HAL_OK) ? 0 : -1;
}

static int32_t lsm6dsv16x_spi_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len) {
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)handle;
    uint8_t tx[len + 1];
    tx[0] = reg & 0x7F;
    for (uint16_t i = 0; i < len; i++) tx[i + 1] = data[i];
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // NCS low
    int32_t ret = HAL_SPI_Transmit(hspi, tx, len + 1, 1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // NCS high
    return (ret == HAL_OK) ? 0 : -1;
}

void lsm6dsv16x_delay(uint32_t ms) {
    HAL_Delay(ms);
}

void lsm6dsv16x_ctx_init(lsm6dsv16x_ctx_t *ctx, SPI_HandleTypeDef *hspi) {
    ctx->read_reg = lsm6dsv16x_spi_read;
    ctx->write_reg = lsm6dsv16x_spi_write;
    ctx->mdelay = lsm6dsv16x_delay;
    ctx->handle = hspi;
}
