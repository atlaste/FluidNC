#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

#define LED_STRIP_GPIO 45
#define LED_COUNT 720
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution

static const char *TAG = "LED_STRIP";

// WS2811 timing configuration (in 0.1us units at 10MHz)
typedef struct {
    uint32_t t0h;  // Time for 0 bit high
    uint32_t t0l;  // Time for 0 bit low
    uint32_t t1h;  // Time for 1 bit high
    uint32_t t1l;  // Time for 1 bit low
} ws2811_timing_t;

// WS2812-compatible timing (many WS2811F strips use this)
// T0H=0.4us, T0L=0.85us, T1H=0.8us, T1L=0.45us
static const ws2811_timing_t ws2811_timing = {
    .t0h = 4,   // 0.4us
    .t0l = 9,   // 0.9us (slightly longer for safety)
    .t1h = 8,   // 0.8us
    .t1l = 6    // 0.6us (slightly longer for safety)
};

// RGB color structure
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_t;

// GRB color structure (WS2811 expects GRB order!)
typedef struct {
    uint8_t g;
    uint8_t r;
    uint8_t b;
} grb_t;

// LED strip buffer in GRB format
static grb_t led_buffer[LED_COUNT];

// RMT encoder for WS2811
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    rmt_symbol_word_t ws2811_bit0;
    rmt_symbol_word_t ws2811_bit1;
    rmt_symbol_word_t reset;
} ws2811_encoder_t;

static size_t ws2811_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                           const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    ws2811_encoder_t *ws2811_encoder = __containerof(encoder, ws2811_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    
    rmt_encoder_handle_t bytes_encoder = ws2811_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = ws2811_encoder->copy_encoder;
    
    switch (session_state) {
    case RMT_ENCODING_RESET:
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &state);
        if (state & RMT_ENCODING_COMPLETE) {
            session_state = RMT_ENCODING_COMPLETE;
        }
        if (state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    // fallthrough
    case RMT_ENCODING_COMPLETE:
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &ws2811_encoder->reset,
                                               sizeof(ws2811_encoder->reset), &state);
        if (state & RMT_ENCODING_COMPLETE) {
            session_state = RMT_ENCODING_RESET;
        }
        if (state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        default:
        break;
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t ws2811_encoder_del(rmt_encoder_t *encoder)
{
    ws2811_encoder_t *ws2811_encoder = __containerof(encoder, ws2811_encoder_t, base);
    rmt_del_encoder(ws2811_encoder->bytes_encoder);
    rmt_del_encoder(ws2811_encoder->copy_encoder);
    free(ws2811_encoder);
    return ESP_OK;
}

static esp_err_t ws2811_encoder_reset(rmt_encoder_t *encoder)
{
    ws2811_encoder_t *ws2811_encoder = __containerof(encoder, ws2811_encoder_t, base);
    rmt_encoder_reset(ws2811_encoder->bytes_encoder);
    rmt_encoder_reset(ws2811_encoder->copy_encoder);
    return ESP_OK;
}

static esp_err_t create_ws2811_encoder(rmt_encoder_handle_t *ret_encoder)
{
    ws2811_encoder_t *encoder = calloc(1, sizeof(ws2811_encoder_t));
    if (!encoder) {
        return ESP_ERR_NO_MEM;
    }
    
    encoder->base.encode = ws2811_encode;
    encoder->base.del = ws2811_encoder_del;
    encoder->base.reset = ws2811_encoder_reset;
    
    // WS2811 bit encoding
    encoder->ws2811_bit0.level0 = 1;
    encoder->ws2811_bit0.duration0 = ws2811_timing.t0h;
    encoder->ws2811_bit0.level1 = 0;
    encoder->ws2811_bit0.duration1 = ws2811_timing.t0l;
    
    encoder->ws2811_bit1.level0 = 1;
    encoder->ws2811_bit1.duration0 = ws2811_timing.t1h;
    encoder->ws2811_bit1.level1 = 0;
    encoder->ws2811_bit1.duration1 = ws2811_timing.t1l;
    
    // Reset code (>50us low, using 80us to be safe)
    encoder->reset.level0 = 0;
    encoder->reset.duration0 = 800; // 80us
    encoder->reset.level1 = 0;
    encoder->reset.duration1 = 0;
    
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = encoder->ws2811_bit0,
        .bit1 = encoder->ws2811_bit1,
        .flags.msb_first = 1
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_encoder_config, &encoder->bytes_encoder));
    
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &encoder->copy_encoder));
    
    *ret_encoder = &encoder->base;
    return ESP_OK;
}

// HSV to RGB conversion
void hsv_to_rgb(float h, float s, float v, rgb_t *rgb)
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

// Create moving rainbow effect
// The rainbow repeats every 'rainbow_length' LEDs and moves along the strip
void rainbow_effect(float offset)
{
    const float rainbow_length = 120.0f; // Rainbow repeats every 120 LEDs (~16.6cm at 720/m)
    
    for (int i = 0; i < LED_COUNT; i++) {
        // Calculate hue based on position and offset
        // This creates a rainbow that spans 'rainbow_length' LEDs and moves with offset
        float hue = fmodf(offset + (i * 360.0f / rainbow_length), 360.0f);
        rgb_t color;
        hsv_to_rgb(hue, 1.0f, 0.3f, &color); // 30% brightness
        
        // Convert RGB to GRB for WS2811
        led_buffer[i].g = color.g;
        led_buffer[i].r = color.r;
        led_buffer[i].b = color.b;
    }
}

// Simple color test pattern
void test_colors(void)
{
    // Clear all LEDs first
    memset(led_buffer, 0, sizeof(led_buffer));
    
    // First 10 LEDs: Red (in GRB format: g=0, r=50, b=0)
    for (int i = 0; i < 10 && i < LED_COUNT; i++) {
        led_buffer[i].g = 0;
        led_buffer[i].r = 50;
        led_buffer[i].b = 0;
    }
    
    // Next 10 LEDs: Green (in GRB format: g=50, r=0, b=0)
    for (int i = 10; i < 20 && i < LED_COUNT; i++) {
        led_buffer[i].g = 50;
        led_buffer[i].r = 0;
        led_buffer[i].b = 0;
    }
    
    // Next 10 LEDs: Blue (in GRB format: g=0, r=0, b=50)
    for (int i = 20; i < 30 && i < LED_COUNT; i++) {
        led_buffer[i].g = 0;
        led_buffer[i].r = 0;
        led_buffer[i].b = 50;
    }
    
    ESP_LOGI(TAG, "Test pattern: LED[0]={G:%d,R:%d,B:%d}, LED[10]={G:%d,R:%d,B:%d}, LED[20]={G:%d,R:%d,B:%d}",
             led_buffer[0].g, led_buffer[0].r, led_buffer[0].b,
             led_buffer[10].g, led_buffer[10].r, led_buffer[10].b,
             led_buffer[20].g, led_buffer[20].r, led_buffer[20].b);
}

// Turn all LEDs off
void all_off(void)
{
    memset(led_buffer, 0, sizeof(led_buffer));
    ESP_LOGI(TAG, "All LEDs off");
}

// Turn first LED to specific color for testing
void test_single_led(uint8_t g, uint8_t r, uint8_t b)
{
    memset(led_buffer, 0, sizeof(led_buffer));
    led_buffer[0].g = g;
    led_buffer[0].r = r;
    led_buffer[0].b = b;
    ESP_LOGI(TAG, "Single LED test: G=%d, R=%d, B=%d", g, r, b);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting WS2811 LED strip demo");
    ESP_LOGI(TAG, "LED count: %d, GPIO: %d", LED_COUNT, LED_STRIP_GPIO);
    
    // Configure RMT TX channel
    rmt_channel_handle_t led_channel = NULL;
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = LED_STRIP_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_config, &led_channel));
    
    // Create WS2811 encoder
    rmt_encoder_handle_t led_encoder = NULL;
    ESP_ERROR_CHECK(create_ws2811_encoder(&led_encoder));
    
    // Enable RMT channel
    ESP_ERROR_CHECK(rmt_enable(led_channel));
    
    ESP_LOGI(TAG, "LED strip initialized, starting diagnostic sequence");
    
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };
    
    // Test 1: Turn all LEDs off
    ESP_LOGI(TAG, "Test 1: All LEDs OFF");
    all_off();
    ESP_ERROR_CHECK(rmt_transmit(led_channel, led_encoder, led_buffer, 
                                sizeof(led_buffer), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_channel, portMAX_DELAY));
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test 2: Single LED red
    ESP_LOGI(TAG, "Test 2: First LED RED only");
    test_single_led(0, 255, 0); // GRB: G=0, R=255, B=0
    ESP_ERROR_CHECK(rmt_transmit(led_channel, led_encoder, led_buffer, 
                                sizeof(led_buffer), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_channel, portMAX_DELAY));
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test 3: Single LED green
    ESP_LOGI(TAG, "Test 3: First LED GREEN only");
    test_single_led(255, 0, 0); // GRB: G=255, R=0, B=0
    ESP_ERROR_CHECK(rmt_transmit(led_channel, led_encoder, led_buffer, 
                                sizeof(led_buffer), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_channel, portMAX_DELAY));
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test 4: Single LED blue
    ESP_LOGI(TAG, "Test 4: First LED BLUE only");
    test_single_led(0, 0, 255); // GRB: G=0, R=0, B=255
    ESP_ERROR_CHECK(rmt_transmit(led_channel, led_encoder, led_buffer, 
                                sizeof(led_buffer), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_channel, portMAX_DELAY));
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test 5: Color pattern on first 30 LEDs
    ESP_LOGI(TAG, "Test 5: Color pattern (Red, Green, Blue blocks)");
    test_colors();
    ESP_ERROR_CHECK(rmt_transmit(led_channel, led_encoder, led_buffer, 
                                sizeof(led_buffer), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_channel, portMAX_DELAY));
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    ESP_LOGI(TAG, "Diagnostic complete. Starting rainbow animation...");
    
    // Rainbow animation loop
    float hue_offset = 0.0f;
    
    while (1) {
        // Generate rainbow pattern
        rainbow_effect(hue_offset);
        
        // Transmit to LED strip
        ESP_ERROR_CHECK(rmt_transmit(led_channel, led_encoder, led_buffer, 
                                    sizeof(led_buffer), &tx_cfg));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_channel, portMAX_DELAY));
        
        // Update animation
        hue_offset += 2.0f;
        if (hue_offset >= 360.0f) {
            hue_offset -= 360.0f;
        }
        
        vTaskDelay(pdMS_TO_TICKS(30)); // ~33 FPS
    }
}
