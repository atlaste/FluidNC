# Generic LED Strip Driver for ESP-IDF

A flexible, timing-based LED strip driver supporting WS2812, WS2811, SK6812, WS2813, and WS2815.

## 🎯 Supported LED Strips

### Common Timing-Based Strips
| IC Type | Voltage | Color Order | Common Use |
|---------|---------|-------------|------------|
| WS2812/B | 5V | GRB | Most popular, individual LEDs |
| WS2811 | 5V/12V/24V | GRB/RGB | Groups of 3-6 LEDs, long runs |
| SK6812 | 5V | GRB | Better color quality |
| SK6812 RGBW | 5V | GRBW | With white channel |
| WS2813 | 5V | GRB | With data backup line |
| WS2815 | 12V | GRB | With data backup line |

## 🚀 Quick Start

### 1. Basic Usage (WS2812)

```c
#include "led_strip_rmt.h"

// Configure strip
led_strip_config_t config = LED_STRIP_CONFIG_WS2812(60, 45);  // 60 LEDs, GPIO 45

// Create strip
led_strip_t *strip = NULL;
ESP_ERROR_CHECK(led_strip_new(&config, &strip));

// Set pixel colors
led_strip_set_pixel(strip, 0, 255, 0, 0);   // Red
led_strip_set_pixel(strip, 1, 0, 255, 0);   // Green
led_strip_set_pixel(strip, 2, 0, 0, 255);   // Blue

// Update strip
led_strip_refresh(strip);

// Clear all LEDs
led_strip_clear(strip);

// Cleanup
led_strip_del(strip);
```

### 2. WS2811 (12V/24V strips)

```c
// 12V strip with 240 addressable units (720 physical LEDs)
led_strip_config_t config = LED_STRIP_CONFIG_WS2811_12V(240, 45);

led_strip_t *strip = NULL;
ESP_ERROR_CHECK(led_strip_new(&config, &strip));
```

**Important**: For WS2811:
- **12V strips**: 3 LEDs per controller → divide LED count by 3
- **24V strips**: 6 LEDs per controller → divide LED count by 6
- Example: 720 physical LEDs on 12V = 240 addressable units

### 3. SK6812 RGBW (with white channel)

```c
led_strip_config_t config = LED_STRIP_CONFIG_SK6812_RGBW(60, 45);

led_strip_t *strip = NULL;
ESP_ERROR_CHECK(led_strip_new(&config, &strip));

// Set RGBW pixel
led_strip_set_pixel_rgbw(strip, 0, 255, 0, 0, 100);  // Red + White
led_strip_refresh(strip);
```

## 🔧 Custom Configuration

If the presets don't work, create a custom config:

```c
led_strip_config_t config = {
    .type = LED_TYPE_WS2811_SLOW,
    .color_order = COLOR_ORDER_RGB,      // Try RGB if GRB doesn't work
    .led_count = 720,
    .gpio_num = 45,
    .bytes_per_led = 3,                  // 3 for RGB, 4 for RGBW
    .timing = LED_TIMING_WS2811_SLOW     // Try different timings
};

led_strip_t *strip = NULL;
ESP_ERROR_CHECK(led_strip_new(&config, &strip));
```

## 🎨 Color Order Issues?

If colors appear wrong, try different color orders:

```c
config.color_order = COLOR_ORDER_GRB;  // Most common (WS2812, WS2811)
config.color_order = COLOR_ORDER_RGB;  // Some WS2811
config.color_order = COLOR_ORDER_BGR;  // Rare
```

**Diagnostic test**: Set pixel to red (255, 0, 0):
- If you see **green** → use `COLOR_ORDER_GRB`
- If you see **blue** → use `COLOR_ORDER_BGR`
- If you see **red** → use `COLOR_ORDER_RGB`

## ⏱️ Timing Issues?

If strip shows white or random colors, try different timing:

```c
// Fast timing (WS2812-compatible) - try this first
config.timing = LED_TIMING_WS2812;

// Slow timing (original WS2811)
config.timing = LED_TIMING_WS2811_SLOW;

// SK6812 timing
config.timing = LED_TIMING_SK6812;
```

## 📖 API Reference

### Strip Management

```c
// Create strip
esp_err_t led_strip_new(const led_strip_config_t *config, led_strip_t **ret_strip);

// Delete strip
esp_err_t led_strip_del(led_strip_t *strip);
```

### Setting Colors

```c
// Set RGB color for a pixel
esp_err_t led_strip_set_pixel(led_strip_t *strip, uint16_t index, 
                               uint8_t r, uint8_t g, uint8_t b);

// Set RGBW color (SK6812 RGBW only)
esp_err_t led_strip_set_pixel_rgbw(led_strip_t *strip, uint16_t index,
                                    uint8_t r, uint8_t g, uint8_t b, uint8_t w);
```

### Updating Display

```c
// Refresh strip (send data to LEDs)
esp_err_t led_strip_refresh(led_strip_t *strip);

// Clear all LEDs (set to black and refresh)
esp_err_t led_strip_clear(led_strip_t *strip);
```

## 🔍 Troubleshooting

### Strip shows white or random colors
1. Try different timing: `LED_TIMING_WS2812` or `LED_TIMING_WS2811_SLOW`
2. Check voltage compatibility (5V ESP32 → 5V strip)
3. Use level shifter for 5V strips (74AHCT125 or similar)
4. Verify reset time is long enough (increase `.reset` value)

### Wrong colors
1. Try different color orders: `COLOR_ORDER_GRB`, `COLOR_ORDER_RGB`, `COLOR_ORDER_BGR`
2. Run diagnostic: set first pixel to red, observe actual color
3. Check IC datasheet for correct color order

### Only some LEDs work
1. **WS2811 12V/24V**: Divide LED count by 3 (12V) or 6 (24V)
2. Check power supply capacity
3. Verify data signal quality with oscilloscope
4. Add 330Ω resistor in series with data line

### Flickering or unstable
1. Add 1000µF capacitor across power supply
2. Keep data wire short and away from power wires
3. Reduce refresh rate (increase delay between updates)
4. Lower brightness

## 🎨 Example: Moving Rainbow

```c
void rainbow_effect(led_strip_t *strip, float offset, uint16_t led_count)
{
    const float rainbow_length = 120.0f;
    
    for (int i = 0; i < led_count; i++) {
        float hue = fmodf(offset + (i * 360.0f / rainbow_length), 360.0f);
        rgb_color_t color;
        hsv_to_rgb(hue, 1.0f, 0.3f, &color);
        led_strip_set_pixel(strip, i, color.r, color.g, color.b);
    }
    led_strip_refresh(strip);
}

// In main loop
float offset = 0.0f;
while (1) {
    rainbow_effect(strip, offset, LED_COUNT);
    offset += 2.0f;
    if (offset >= 360.0f) offset -= 360.0f;
    vTaskDelay(pdMS_TO_TICKS(30));
}
```

## 🔌 Hardware Setup

### Wiring
```
ESP32 GPIO → [74AHCT125] → LED Strip Data
ESP32 GND  → LED Strip GND
5V Supply  → LED Strip 5V
           → 74AHCT125 VCC
```

### Components
- **Level Shifter**: 74AHCT125 or 74HCT245 (3.3V → 5V)
- **Capacitor**: 1000µF across power supply
- **Resistor**: 330Ω in series with data line (optional, but recommended)

### Power Calculation
- RGB LED at full white: ~60mA
- RGB LED at 30% brightness: ~20mA
- 100 LEDs at 30%: ~2A
- **Always use external power supply for >10 LEDs**

## 📝 Quick Reference Table

| Strip Type | LED Count Calculation | Example |
|------------|----------------------|---------|
| WS2812 5V | Physical LED count | 60 LEDs = 60 |
| WS2811 12V | Physical LEDs ÷ 3 | 720 LEDs = 240 |
| WS2811 24V | Physical LEDs ÷ 6 | 720 LEDs = 120 |
| SK6812 5V | Physical LED count | 60 LEDs = 60 |

## 🎯 Common Configurations

```c
// Adafruit NeoPixel strip (WS2812B)
led_strip_config_t config = LED_STRIP_CONFIG_WS2812(60, 45);

// BTF-Lighting WS2811 12V strip
led_strip_config_t config = LED_STRIP_CONFIG_WS2811_12V(240, 45);  // 720 LEDs

// SK6812 RGBW strip
led_strip_config_t config = LED_STRIP_CONFIG_SK6812_RGBW(60, 45);

// WS2815 12V with data backup
led_strip_config_t config = led_strip_get_default_config(LED_TYPE_WS2815, 240, 45);
```

## 📚 More Information

- See `ledstrip_example.c` for complete working example
- Check `led_strip_types.h` for all configuration options
- Timing specifications in `led_strip_types.h`

