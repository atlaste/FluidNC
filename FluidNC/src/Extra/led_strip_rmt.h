#ifndef LED_STRIP_RMT_H
#define LED_STRIP_RMT_H

#include "led_strip_types.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"

// LED strip handle
typedef struct led_strip_t led_strip_t;

/**
 * @brief Create a new LED strip instance
 * 
 * @param config LED strip configuration
 * @param ret_strip Pointer to store the created strip handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t led_strip_new(const led_strip_config_t *config, led_strip_t **ret_strip);

/**
 * @brief Delete LED strip instance
 * 
 * @param strip LED strip handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t led_strip_del(led_strip_t *strip);

/**
 * @brief Set RGB color for a specific LED
 * 
 * @param strip LED strip handle
 * @param index LED index (0-based)
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t led_strip_set_pixel(led_strip_t *strip, uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set RGBW color for a specific LED (SK6812 RGBW only)
 * 
 * @param strip LED strip handle
 * @param index LED index (0-based)
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 * @param w White value (0-255)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t led_strip_set_pixel_rgbw(led_strip_t *strip, uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w);

/**
 * @brief Refresh/update the LED strip (send data)
 * 
 * @param strip LED strip handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t led_strip_refresh(led_strip_t *strip);

/**
 * @brief Clear all LEDs (set to black)
 * 
 * @param strip LED strip handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t led_strip_clear(led_strip_t *strip);

#endif // LED_STRIP_RMT_H

