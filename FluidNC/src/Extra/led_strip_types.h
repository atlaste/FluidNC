#ifndef LED_STRIP_TYPES_H
#define LED_STRIP_TYPES_H

#include <stdint.h>

// Supported LED strip types
typedef enum {
    LED_TYPE_WS2812,       // 5V, most common
    LED_TYPE_WS2812B,      // 5V, improved WS2812
    LED_TYPE_WS2811_FAST,  // 5V/12V/24V, WS2812-compatible timing
    LED_TYPE_WS2811_SLOW,  // 5V/12V/24V, original WS2811 timing
    LED_TYPE_SK6812,       // 5V, RGB variant
    LED_TYPE_SK6812_RGBW,  // 5V, with white channel
    LED_TYPE_WS2813,       // 5V, data backup (same timing as WS2812)
    LED_TYPE_WS2815,       // 12V, data backup (same timing as WS2812)
} led_strip_type_t;

// Color channel order
typedef enum {
    COLOR_ORDER_RGB,       // Red, Green, Blue
    COLOR_ORDER_GRB,       // Green, Red, Blue (most common)
    COLOR_ORDER_BGR,       // Blue, Green, Red
    COLOR_ORDER_GRBW,      // Green, Red, Blue, White (SK6812 RGBW)
} led_color_order_t;

// Timing configuration (in 0.1µs units at 10MHz resolution)
typedef struct {
    uint32_t t0h;         // Time for 0 bit high
    uint32_t t0l;         // Time for 0 bit low
    uint32_t t1h;         // Time for 1 bit high
    uint32_t t1l;         // Time for 1 bit low
    uint32_t reset;       // Reset time (low)
} led_timing_config_t;

// LED strip configuration
typedef struct {
    led_strip_type_t type;
    led_color_order_t color_order;
    uint16_t led_count;
    uint8_t gpio_num;
    uint8_t bytes_per_led;     // 3 for RGB, 4 for RGBW
    led_timing_config_t timing;
} led_strip_config_t;

// Predefined timing configurations
static const led_timing_config_t LED_TIMING_WS2812 = {
    .t0h = 4,   // 0.4µs
    .t0l = 9,   // 0.9µs
    .t1h = 8,   // 0.8µs
    .t1l = 6,   // 0.6µs
    .reset = 500  // 50µs
};

static const led_timing_config_t LED_TIMING_WS2811_SLOW = {
    .t0h = 5,   // 0.5µs
    .t0l = 20,  // 2.0µs
    .t1h = 12,  // 1.2µs
    .t1l = 13,  // 1.3µs
    .reset = 500  // 50µs
};

static const led_timing_config_t LED_TIMING_SK6812 = {
    .t0h = 3,   // 0.3µs
    .t0l = 9,   // 0.9µs
    .t1h = 6,   // 0.6µs
    .t1l = 6,   // 0.6µs
    .reset = 800  // 80µs
};

// Helper function to get default config for a LED type
static inline led_strip_config_t led_strip_get_default_config(led_strip_type_t type, 
                                                               uint16_t led_count, 
                                                               uint8_t gpio_num)
{
    led_strip_config_t config = {
        .type = type,
        .led_count = led_count,
        .gpio_num = gpio_num,
    };
    
    switch (type) {
        case LED_TYPE_WS2812:
        case LED_TYPE_WS2812B:
        case LED_TYPE_WS2813:
        case LED_TYPE_WS2815:
        case LED_TYPE_WS2811_FAST:
            config.color_order = COLOR_ORDER_GRB;
            config.bytes_per_led = 3;
            config.timing = LED_TIMING_WS2812;
            break;
            
        case LED_TYPE_WS2811_SLOW:
            config.color_order = COLOR_ORDER_GRB;  // Can be RGB, check your strip!
            config.bytes_per_led = 3;
            config.timing = LED_TIMING_WS2811_SLOW;
            break;
            
        case LED_TYPE_SK6812:
            config.color_order = COLOR_ORDER_GRB;
            config.bytes_per_led = 3;
            config.timing = LED_TIMING_SK6812;
            break;
            
        case LED_TYPE_SK6812_RGBW:
            config.color_order = COLOR_ORDER_GRBW;
            config.bytes_per_led = 4;
            config.timing = LED_TIMING_SK6812;
            break;
    }
    
    return config;
}

// Quick preset configurations
#define LED_STRIP_CONFIG_WS2812(led_count, gpio) \
    led_strip_get_default_config(LED_TYPE_WS2812, led_count, gpio)

#define LED_STRIP_CONFIG_WS2811_12V(led_count, gpio) \
    led_strip_get_default_config(LED_TYPE_WS2811_FAST, led_count, gpio)

#define LED_STRIP_CONFIG_SK6812_RGBW(led_count, gpio) \
    led_strip_get_default_config(LED_TYPE_SK6812_RGBW, led_count, gpio)

#endif // LED_STRIP_TYPES_H

