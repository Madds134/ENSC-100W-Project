#include "i2c_bus.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_TIMEOUT_MS 1000
static const char *TAG = "I2C_BUS";

esp_err_t i2c_bus_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &conf));
    return i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

esp_err_t i2c_bus_write(uint8_t addr, uint8_t *data, size_t len)
{
    return i2c_master_write_to_device(
        I2C_MASTER_PORT, addr, data, len,
        I2C_TIMEOUT_MS / portTICK_PERIOD_MS
    );
}

esp_err_t i2c_bus_write_read(uint8_t addr, uint8_t *wdata, size_t wlen,
                             uint8_t *rdata, size_t rlen)
{
    return i2c_master_write_read_device(
        I2C_MASTER_PORT, addr,
        wdata, wlen,
        rdata, rlen,
        I2C_TIMEOUT_MS / portTICK_PERIOD_MS
    );
}
