#include "led_control.h" // made by me
#include "led_strip.h" 
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "SCHRODINGER_SYSTEMS";
static led_strip_handle_t led_strip;

// === Initialization ===
void led_init(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_BLINK_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

// === Helper ===
// Set LED to specific color
static void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

// Blink LED with specific color, count, and delay
static void led_blink(uint8_t r, uint8_t g, uint8_t b, int count, int delay_ms) {
    for (int i = 0; i < count; i++) {
        led_set_color(r, g, b);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
        led_strip_clear(led_strip);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    }
}

// === State Functions ===
void led_open(void) {
    ESP_LOGI(TAG, "LED: Opening (Red blink)");
    led_blink(255, 0, 0, 6, 100); // (255, 0, 0) = Red ; 6 blinks ; 100 ms delay
}

void led_close(void) {
    ESP_LOGI(TAG, "LED: Closing (Red blink)");
    led_blink(255, 0, 0, 6, 100); // (255, 0, 0) = Red ; 6 blinks ; 100 ms delay
}

void led_inside(void) {
    ESP_LOGI(TAG, "LED: Inside (Green)");
    led_set_color(0, 255, 0); // (0, 255, 0) = Green
}

void led_outside(void) {
    ESP_LOGI(TAG, "LED: Outside (Blue)");
    led_set_color(0, 0, 255); // (0, 0, 255) = Blue
}

void led_idle(void) {
    ESP_LOGI(TAG, "LED: Door open, idle (White)");
    led_set_color(64, 64, 64); // (64, 64, 64) = Dim White
}