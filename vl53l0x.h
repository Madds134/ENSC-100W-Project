#pragma once
#include "esp_err.h"
#include "driver/gpio.h"
#include <stdint.h>

#define CAT_THRESHOLD_MM 800
#define CAT_LED_GPIO GPIO_NUM_10
#define CAT_LED_GPIO_OUTSIDE GPIO_NUM_11 // red
extern bool cat_inside; // False = outside, true = inside
extern bool in_beam_prev; // previous in beam state for edge dection
esp_err_t vl53_init(void);
esp_err_t vl53_read_mm(uint16_t *out_mm);
void cat_led_init(void);
void cat_sensor_step(void);
void blink_led(void);