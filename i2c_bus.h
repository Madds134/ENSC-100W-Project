#pragma once
#include "esp_err.h"
#include "driver/gpio.h"
#include <stddef.h>
#include <stdint.h>

#define I2C_MASTER_PORT  I2C_NUM_0
#define I2C_MASTER_SDA_IO GPIO_NUM_8 // SDA
#define I2C_MASTER_SCL_IO GPIO_NUM_9 // SCL
#define I2C_MASTER_FREQ_HZ 400000

esp_err_t i2c_bus_init(void);
esp_err_t i2c_bus_write(uint8_t addr, uint8_t *data, size_t len);
esp_err_t i2c_bus_write_read(uint8_t addr, uint8_t *wdata, size_t wlen,
                             uint8_t *rdata, size_t rlen);