#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

typedef struct {
    gpio_num_t in1, in2, in3, in4;   // H-bridge input pins
    uint32_t   us_per_step;          // Delay per step (us)
    int        idx;                  // Current sequence index
    bool       initialized;
} stepper_t;

// Initialize pins and default speed
void stepper_init(stepper_t *s,
                  gpio_num_t in1, gpio_num_t in2,
                  gpio_num_t in3, gpio_num_t in4);

// Preferred API
void stepper_set_speed_rpm(stepper_t *s, float rpm);

// Back-compat alias for your current call sites
static inline void stepper_set_rpm(stepper_t *s, float rpm) {
    stepper_set_speed_rpm(s, rpm);
}

void stepper_one_rev_cw(stepper_t *s);
void stepper_one_rev_ccw(stepper_t *s);
void stepper_release(stepper_t *s);
void stepper_step_once(stepper_t *s, int dir);

#endif // STEPPER_H
