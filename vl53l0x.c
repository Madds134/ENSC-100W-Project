#include "vl53l0x.h"
#include "i2c_bus.h"
#include "esp_log.h"


#include "driver/gpio.h"
#include "stdbool.h"
#include "freertos/FreeRTOS.h"

#define VL_ADDR 0x29
static const char *TAG = "VL53L0X";

bool in_beam_prev = false;
bool cat_inside = false;
// -----------------------------------------------------------------------------
// Low-level I2C helpers
// -----------------------------------------------------------------------------

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_bus_write(VL_ADDR, buf, 2);
}

static esp_err_t read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_bus_write_read(VL_ADDR, &reg, 1, value, 1);
}

static esp_err_t read_reg16(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    esp_err_t err = i2c_bus_write_read(VL_ADDR, &reg, 1, data, 2);
    if (err != ESP_OK) return err;

    *value = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Sensor Initialization
// -----------------------------------------------------------------------------

esp_err_t vl53_init(void)
{
    uint8_t id = 0;
    ESP_ERROR_CHECK(read_reg(0xC0, &id));
    ESP_LOGI(TAG, "VL53L0X Model ID: 0x%02X", id);

    // Minimal init sequence (from ST app note)
    write_reg(0x88, 0x00);
    write_reg(0x80, 0x01);
    write_reg(0xFF, 0x01);
    write_reg(0x00, 0x00);
    write_reg(0x91, 0x3C);
    write_reg(0x00, 0x01);
    write_reg(0xFF, 0x00);
    write_reg(0x80, 0x00);

    // ---- FIXED ----
    // Continuous ranging mode = 0x02 (NOT 0x04)
    return write_reg(0x00, 0x02);
}

// -----------------------------------------------------------------------------
// Read distance (mm)
// -----------------------------------------------------------------------------

esp_err_t vl53_read_mm(uint16_t *out_mm)
{
    uint8_t reg = 0x1E;   // Distance MSB
    uint8_t buf[2];

    // Read two bytes from distance registers (0x1E + 0x1F)
    esp_err_t err = i2c_bus_write_read(VL_ADDR, &reg, 1, buf, 2);
    if (err != ESP_OK) return err;

    uint16_t dist = ((uint16_t)buf[0] << 8) | buf[1];

    // ---- Common filtering ----
    // Distances <= 20 mm or >= 8190 are typically "no object" or saturation
    if (dist <= 20 || dist >= 8190) {
        *out_mm = 0;
    } else {
        *out_mm = dist;
    }

    return ESP_OK;
}

void cat_led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << CAT_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t io_cong = {
        .pin_bit_mask = 1ULL << CAT_LED_GPIO_OUTSIDE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);
    gpio_config(&io_cong);

    // Start with LED OFF (assume cat starts inside)
    gpio_set_level(CAT_LED_GPIO, 1);
    gpio_set_level(CAT_LED_GPIO_OUTSIDE, 0);
}

void cat_sensor_step(void)
{
    uint16_t dist_mm = 0;

    // Read current distance
    if (vl53_read_mm(&dist_mm) != ESP_OK) {
        // If read fails, do nothing this cycle
        return;
    }

    // In-beam = something closer than threshold (and non-zero reading)
    bool in_beam = (dist_mm > 0 && dist_mm < CAT_THRESHOLD_MM);

    // A "pass" is defined as:
    // was in beam last sample, now out of beam -> object just finished passing
    if (!in_beam && in_beam_prev) {
        // Toggle inside/outside state
        cat_inside = !cat_inside;

        // Update LED: ON = inside, OFF = outside
        gpio_set_level(CAT_LED_GPIO, cat_inside ? 1 : 0);
        gpio_set_level(CAT_LED_GPIO_OUTSIDE, cat_inside ? 0 : 1);

        ESP_LOGI("CAT", "Pass detected. cat_inside = %s",
                 cat_inside ? "INSIDE" : "OUTSIDE");
    }

    // Save state for next iteration
    in_beam_prev = in_beam;
}

void blink_led(void)
{
    gpio_set_level(CAT_LED_GPIO, 1);
    gpio_set_level(CAT_LED_GPIO_OUTSIDE, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(CAT_LED_GPIO, 0);
    gpio_set_level(CAT_LED_GPIO_OUTSIDE, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(CAT_LED_GPIO, cat_inside ? 1 : 0);
    gpio_set_level(CAT_LED_GPIO_OUTSIDE, cat_inside ? 0 : 1);

}

