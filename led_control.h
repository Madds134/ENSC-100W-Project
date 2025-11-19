#pragma once

#include <stdint.h>

// Initializes the WS2812 LED strip
void led_init(void);

// LED state functions
void led__open(void);       // Red blink
void led_close(void);       // Red blink
void led_inside(void);        // Solid green
void led_outside(void);       // Solid blue
void led_idle(void);     // Solid white


// This header file defines the interface for controlling an LED strip, including initialization and various LED state functions.