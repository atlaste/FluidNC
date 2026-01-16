#include "led_strip_rmt.h"
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz

static const char *TAG = "led_strip_rmt";

// RMT encoder for LED strips
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    rmt_symbol_word_t led_bit0;
    rmt_symbol_word_t led_bit1;
    rmt_symbol_word_t reset;
} led_strip_encoder_t;

// LED strip structure
struct led_strip_t {
    rmt_channel_handle_t rmt_channel;
    rmt_encoder_handle_t rmt_encoder;
    led_strip_config_t config;
    uint8_t *buffer;
    size_t buffer_size;
};

// RMT encoder callbacks
static size_t led_strip_rmt_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                    const void *primary_data, size_t data_size, 
                                    rmt_encode_state_t *ret_state)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    
    rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
    
    switch (session_state) {
    case RMT_ENCODING_RESET:
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, 
                                                 data_size, &state);
        if (state & RMT_ENCODING_COMPLETE) {
            session_state = RMT_ENCODING_COMPLETE;
        }
        if (state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    // fallthrough
    case RMT_ENCODING_COMPLETE:
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset,
                                               sizeof(led_encoder->reset), &state);
        if (state & RMT_ENCODING_COMPLETE) {
            session_state = RMT_ENCODING_RESET;
        }
        if (state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        // fallthrough
    default:
        break;
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t led_strip_rmt_encoder_del(rmt_encoder_t *encoder)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

static esp_err_t led_strip_rmt_encoder_reset(rmt_encoder_t *encoder)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    return ESP_OK;
}

// Create RMT encoder with specific timing
static esp_err_t create_led_strip_encoder(const led_timing_config_t *timing, 
                                          rmt_encoder_handle_t *ret_encoder)
{
    led_strip_encoder_t *encoder = calloc(1, sizeof(led_strip_encoder_t));
    ESP_RETURN_ON_FALSE(encoder, ESP_ERR_NO_MEM, TAG, "no mem for led strip encoder");
    
    encoder->base.encode = led_strip_rmt_encode;
    encoder->base.del = led_strip_rmt_encoder_del;
    encoder->base.reset = led_strip_rmt_encoder_reset;
    
    // Configure bit encoding based on timing
    encoder->led_bit0.level0 = 1;
    encoder->led_bit0.duration0 = timing->t0h;
    encoder->led_bit0.level1 = 0;
    encoder->led_bit0.duration1 = timing->t0l;
    
    encoder->led_bit1.level0 = 1;
    encoder->led_bit1.duration0 = timing->t1h;
    encoder->led_bit1.level1 = 0;
    encoder->led_bit1.duration1 = timing->t1l;
    
    // Reset code
    encoder->reset.level0 = 0;
    encoder->reset.duration0 = timing->reset;
    encoder->reset.level1 = 0;
    encoder->reset.duration1 = 0;
    
    // Create bytes encoder
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = encoder->led_bit0,
        .bit1 = encoder->led_bit1,
        .flags.msb_first = 1
    };
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &encoder->bytes_encoder),
                        TAG, "create bytes encoder failed");
    
    // Create copy encoder
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &encoder->copy_encoder),
                        TAG, "create copy encoder failed");
    
    *ret_encoder = &encoder->base;
    return ESP_OK;
}

esp_err_t led_strip_new(const led_strip_config_t *config, led_strip_t **ret_strip)
{
    ESP_RETURN_ON_FALSE(config && ret_strip, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    
    led_strip_t *strip = calloc(1, sizeof(led_strip_t));
    ESP_RETURN_ON_FALSE(strip, ESP_ERR_NO_MEM, TAG, "no mem for led strip");
    
    // Copy configuration
    memcpy(&strip->config, config, sizeof(led_strip_config_t));
    
    // Allocate buffer
    strip->buffer_size = config->led_count * config->bytes_per_led;
    strip->buffer = calloc(1, strip->buffer_size);
    if (!strip->buffer) {
        free(strip);
        ESP_RETURN_ON_FALSE(false, ESP_ERR_NO_MEM, TAG, "no mem for led buffer");
    }
    
    // Configure RMT TX channel
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = config->gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    ESP_GOTO_ON_ERROR(rmt_new_tx_channel(&tx_config, &strip->rmt_channel),
                      err_free_buffer, TAG, "create RMT TX channel failed");
    
    // Create encoder
    ESP_GOTO_ON_ERROR(create_led_strip_encoder(&config->timing, &strip->rmt_encoder),
                      err_del_channel, TAG, "create led strip encoder failed");
    
    // Enable RMT channel
    ESP_GOTO_ON_ERROR(rmt_enable(strip->rmt_channel),
                      err_del_encoder, TAG, "enable RMT channel failed");
    
    *ret_strip = strip;
    ESP_LOGI(TAG, "Created LED strip: type=%d, LEDs=%d, GPIO=%d, order=%d",
             config->type, config->led_count, config->gpio_num, config->color_order);
    return ESP_OK;

err_del_encoder:
    rmt_del_encoder(strip->rmt_encoder);
err_del_channel:
    rmt_del_channel(strip->rmt_channel);
err_free_buffer:
    free(strip->buffer);
    free(strip);
    return ESP_FAIL;
}

esp_err_t led_strip_del(led_strip_t *strip)
{
    ESP_RETURN_ON_FALSE(strip, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    
    rmt_disable(strip->rmt_channel);
    rmt_del_encoder(strip->rmt_encoder);
    rmt_del_channel(strip->rmt_channel);
    free(strip->buffer);
    free(strip);
    return ESP_OK;
}

esp_err_t led_strip_set_pixel(led_strip_t *strip, uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    ESP_RETURN_ON_FALSE(strip, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(index < strip->config.led_count, ESP_ERR_INVALID_ARG, TAG, "index out of range");
    
    uint8_t *pixel = strip->buffer + (index * strip->config.bytes_per_led);
    
    // Set color based on color order
    switch (strip->config.color_order) {
        case COLOR_ORDER_RGB:
            pixel[0] = r;
            pixel[1] = g;
            pixel[2] = b;
            break;
        case COLOR_ORDER_GRB:
            pixel[0] = g;
            pixel[1] = r;
            pixel[2] = b;
            break;
        case COLOR_ORDER_BGR:
            pixel[0] = b;
            pixel[1] = g;
            pixel[2] = r;
            break;
        case COLOR_ORDER_GRBW:
            pixel[0] = g;
            pixel[1] = r;
            pixel[2] = b;
            pixel[3] = 0;  // White channel = 0 for RGB mode
            break;
    }
    
    return ESP_OK;
}

esp_err_t led_strip_set_pixel_rgbw(led_strip_t *strip, uint16_t index, 
                                    uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    ESP_RETURN_ON_FALSE(strip, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(index < strip->config.led_count, ESP_ERR_INVALID_ARG, TAG, "index out of range");
    ESP_RETURN_ON_FALSE(strip->config.color_order == COLOR_ORDER_GRBW, 
                        ESP_ERR_NOT_SUPPORTED, TAG, "RGBW not supported by this strip type");
    
    uint8_t *pixel = strip->buffer + (index * 4);
    pixel[0] = g;
    pixel[1] = r;
    pixel[2] = b;
    pixel[3] = w;
    
    return ESP_OK;
}

esp_err_t led_strip_refresh(led_strip_t *strip)
{
    ESP_RETURN_ON_FALSE(strip, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };
    
    ESP_RETURN_ON_ERROR(rmt_transmit(strip->rmt_channel, strip->rmt_encoder, 
                                     strip->buffer, strip->buffer_size, &tx_cfg),
                        TAG, "transmit failed");
    
    ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(strip->rmt_channel, portMAX_DELAY),
                        TAG, "wait transmit done failed");
    
    return ESP_OK;
}

esp_err_t led_strip_clear(led_strip_t *strip)
{
    ESP_RETURN_ON_FALSE(strip, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    
    memset(strip->buffer, 0, strip->buffer_size);
    return led_strip_refresh(strip);
}

