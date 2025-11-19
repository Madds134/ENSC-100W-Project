#include "stepper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const int STEP_SEQ[4][4] = {
    {1,0,1,0}, // A+ B+
    {0,1,1,0}, // A- B+
    {0,1,0,1}, // A- B-
    {1,0,0,1}  // A+ B-
};

static inline void delay_us_yield(uint32_t us)
{
    const uint32_t tick_us = portTICK_PERIOD_MS * 1000;   // e.g., 10_000 us at 100 Hz
    if (us >= tick_us) {
        TickType_t ticks = us / tick_us;
        vTaskDelay(ticks);                                 // feeds WDT
        us -= ticks * tick_us;
    }
    if (us) {
        esp_rom_delay_us(us);                              // fine remainder
        // feed WDT occasionally if we only did busy-waits
        static uint32_t cnt = 0;
        if ((++cnt & 31) == 0) vTaskDelay(1);              // every ~32 calls
    }
    
}

static inline void set_coils(const stepper_t *s, int a1, int a2, int b1, int b2)
{
    gpio_set_level(s->in1, a1);
    gpio_set_level(s->in2, a2);
    gpio_set_level(s->in3, b1);
    gpio_set_level(s->in4, b2);
}

void stepper_init(stepper_t *s, gpio_num_t in1, gpio_num_t in2, gpio_num_t in3, gpio_num_t in4)
{
    s->in1 = in1; s->in2 = in2; s->in3 = in3; s->in4 = in4;
    s->us_per_step = 3000; // default ~3 ms/step
    s->idx = 0;
    s->initialized = true;

    const gpio_num_t pins[4] = { in1, in2, in3, in4 };
    for (int i = 0; i < 4; ++i) {
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << pins[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = 0,
            .pull_down_en = 0,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&cfg);
        gpio_set_level(pins[i], 0);
    }
}

void stepper_set_speed_rpm(stepper_t *s, float rpm)
{
    if (rpm <= 0.0f) rpm = 1.0f;
    const float steps_per_rev = 200.0f; // 1.8° motor
    s->us_per_step = (uint32_t)((60.0f * 1e6f) / (rpm * steps_per_rev));
}

void stepper_release(stepper_t *s)
{
    set_coils(s, 0, 0, 0, 0);
}

// dir: +1 = CW, -1 = CCW
void stepper_step_once(stepper_t *s, int dir)
{
    if (!s->initialized) return;

    s->idx = (s->idx + (dir > 0 ? 1 : -1));
    if (s->idx < 0) s->idx += 4;
    s->idx %= 4;

    set_coils(s,
              STEP_SEQ[s->idx][0],
              STEP_SEQ[s->idx][1],
              STEP_SEQ[s->idx][2],
              STEP_SEQ[s->idx][3]);

    delay_us_yield(s->us_per_step);
}

void stepper_one_rev_cw(stepper_t *s)
{
    for (int i = 0; i < 200; ++i) stepper_step_once(s, +1);
    stepper_release(s);
}

void stepper_one_rev_ccw(stepper_t *s)
{
    for (int i = 0; i < 200; ++i) stepper_step_once(s, -1);
    stepper_release(s);
}
