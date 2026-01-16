#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip_rmt.h"

#define LED_STRIP_GPIO 45
#define LED_COUNT 720

static const char *TAG = "LED_EXAMPLE";

// HSV to RGB conversion
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_color_t;

void hsv_to_rgb(float h, float s, float v, rgb_color_t *rgb)
{
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float r, g, b;
    
    if (h < 60) {
        r = c; g = x; b = 0;
    } else if (h < 120) {
        r = x; g = c; b = 0;
    } else if (h < 180) {
        r = 0; g = c; b = x;
    } else if (h < 240) {
        r = 0; g = x; b = c;
    } else if (h < 300) {
        r = x; g = 0; b = c;
    } else {
        r = c; g = 0; b = x;
    }
    
    rgb->r = (uint8_t)((r + m) * 255);
    rgb->g = (uint8_t)((g + m) * 255);
    rgb->b = (uint8_t)((b + m) * 255);
}

// Moving rainbow effect
void rainbow_effect(led_strip_t *strip, float offset, float brightness)
{
    const float rainbow_length = 120.0f; // Rainbow repeats every 120 LEDs
    
    for (int i = 0; i < LED_COUNT; i++) {
        float hue = fmodf(offset + (i * 360.0f / rainbow_length), 360.0f);
        rgb_color_t color;
        hsv_to_rgb(hue, 1.0f, brightness, &color);
        led_strip_set_pixel(strip, i, color.r, color.g, color.b);
    }
    led_strip_refresh(strip);
}

// Simple diagnostic tests
void run_diagnostics(led_strip_t *strip)
{
    ESP_LOGI(TAG, "Running diagnostic tests...");
    
    // Test 1: All off
    ESP_LOGI(TAG, "Test 1: All LEDs OFF");
    led_strip_clear(strip);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test 2: Single LED red
    ESP_LOGI(TAG, "Test 2: First LED RED");
    led_strip_set_pixel(strip, 0, 255, 0, 0);
    led_strip_refresh(strip);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test 3: Single LED green
    ESP_LOGI(TAG, "Test 3: First LED GREEN");
    led_strip_clear(strip);
    led_strip_set_pixel(strip, 0, 0, 255, 0);
    led_strip_refresh(strip);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test 4: Single LED blue
    ESP_LOGI(TAG, "Test 4: First LED BLUE");
    led_strip_clear(strip);
    led_strip_set_pixel(strip, 0, 0, 0, 255);
    led_strip_refresh(strip);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test 5: Color blocks
    ESP_LOGI(TAG, "Test 5: Color blocks (Red, Green, Blue)");
    led_strip_clear(strip);
    for (int i = 0; i < 10; i++) {
        led_strip_set_pixel(strip, i, 50, 0, 0);      // Red
        led_strip_set_pixel(strip, i + 10, 0, 50, 0); // Green
        led_strip_set_pixel(strip, i + 20, 0, 0, 50); // Blue
    }
    led_strip_refresh(strip);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI(TAG, "Diagnostics complete!");
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting LED Strip Example");
    
    // ============================================================================
    // Configure your LED strip here - choose one of these presets:
    // ============================================================================
    
    // Option 1: WS2812/WS2812B (5V, most common, GRB)
    // led_strip_config_t config = LED_STRIP_CONFIG_WS2812(LED_COUNT, LED_STRIP_GPIO);
    
    // Option 2: WS2811 (12V/24V, fast timing, GRB) - YOUR STRIP
    led_strip_config_t config = LED_STRIP_CONFIG_WS2811_12V(LED_COUNT, LED_STRIP_GPIO);
    
    // Option 3: SK6812 RGBW (5V with white channel, GRBW)
    // led_strip_config_t config = LED_STRIP_CONFIG_SK6812_RGBW(LED_COUNT, LED_STRIP_GPIO);
    
    // Option 4: Custom configuration
    // led_strip_config_t config = {
    //     .type = LED_TYPE_WS2811_SLOW,
    //     .color_order = COLOR_ORDER_RGB,  // Change if needed!
    //     .led_count = LED_COUNT,
    //     .gpio_num = LED_STRIP_GPIO,
    //     .bytes_per_led = 3,
    //     .timing = LED_TIMING_WS2811_SLOW
    // };
    
    // ============================================================================
    
    // Create LED strip
    led_strip_t *strip = NULL;
    ESP_ERROR_CHECK(led_strip_new(&config, &strip));
    
    ESP_LOGI(TAG, "LED strip initialized: %d LEDs on GPIO %d", LED_COUNT, LED_STRIP_GPIO);
    
    // Run diagnostics
    run_diagnostics(strip);
    
    // Rainbow animation
    ESP_LOGI(TAG, "Starting rainbow animation...");
    float hue_offset = 0.0f;
    
    while (1) {
        rainbow_effect(strip, hue_offset, 0.3f);  // 30% brightness
        
        hue_offset += 2.0f;
        if (hue_offset >= 360.0f) {
            hue_offset -= 360.0f;
        }
        
        vTaskDelay(pdMS_TO_TICKS(30));  // ~33 FPS
    }
    
    // Cleanup (never reached in this example)
    led_strip_del(strip);
}

