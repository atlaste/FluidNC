#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char* TAG = "PRESSURE_SENSOR";

// ADC Class - Handles ADC initialization and reading
class ADC {
private:
    adc_oneshot_unit_handle_t adc_handle = nullptr;
    adc_cali_handle_t cali_handle = nullptr;
    adc_channel_t channel;
    adc_atten_t attenuation;
    bool calibration_enabled = false;

public:
    ADC(adc_unit_t unit, adc_channel_t chan, adc_atten_t atten = ADC_ATTEN_DB_12) 
        : channel(chan), attenuation(atten) {
        
        // Configure ADC
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = unit,
            .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

        // Configure channel
        adc_oneshot_chan_cfg_t config = {
            .atten = attenuation,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, channel, &config));

        // Initialize calibration
        initCalibration(unit, chan, atten);
        
        ESP_LOGI(TAG, "ADC initialized on channel %d", channel);
    }

    ~ADC() {
        if (calibration_enabled && cali_handle) {
            adc_cali_delete_scheme_curve_fitting(cali_handle);
        }
        if (adc_handle) {
            adc_oneshot_del_unit(adc_handle);
        }
    }

    // Read raw ADC value
    int readRaw() {
        int raw_value = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, channel, &raw_value));
        return raw_value;
    }

    // Read voltage in millivolts
    int readMillivolts() {
        int raw_value = readRaw();
        int voltage_mv = 0;
        
        if (calibration_enabled && cali_handle) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw_value, &voltage_mv));
        } else {
            // Fallback: approximate conversion (not recommended for production)
            ESP_LOGW(TAG, "Not using calibrated values. Raw is %d", int(raw_value) );
            voltage_mv = raw_value * 3300 / 4095;
        }
        
        return voltage_mv;
    }

    bool isCalibrated() const {
        return calibration_enabled;
    }

private:
    void initCalibration(adc_unit_t unit, adc_channel_t chan, adc_atten_t atten) {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = chan,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        
        esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
        if (ret == ESP_OK) {
            calibration_enabled = true;
            ESP_LOGI(TAG, "ADC calibration initialized");
        } else {
            calibration_enabled = false;
            ESP_LOGW(TAG, "ADC calibration failed, using approximate values");
        }
    }
};

// Pressure Sensor Class - Converts ADC readings to pressure values
class PressureSensor {
private:
    ADC& adc;
    float min_voltage_mv;      // Minimum voltage in mV (after voltage divider)
    float max_voltage_mv;      // Maximum voltage in mV (after voltage divider)
    float min_pressure_mpa;    // Minimum pressure in MPa
    float max_pressure_mpa;    // Maximum pressure in MPa
    int num_samples;           // Number of samples to average

public:
    PressureSensor(ADC& adc_instance, 
                   float sensor_min_v = 0.5f, 
                   float sensor_max_v = 4.5f,
                   float min_press_mpa = 0.0f,
                   float max_press_mpa = 1.6f,
                   float voltage_divider_ratio = 0.5f,
                   int samples = 10)
        : adc(adc_instance), 
          min_pressure_mpa(min_press_mpa),
          max_pressure_mpa(max_press_mpa),
          num_samples(samples) {
        
        // Apply voltage divider ratio to sensor voltage range
        min_voltage_mv = sensor_min_v * voltage_divider_ratio * 1000.0f;
        max_voltage_mv = sensor_max_v * voltage_divider_ratio * 1000.0f;
        
        ESP_LOGI(TAG, "Pressure sensor initialized: %.2f-%.2f mV -> %.2f-%.2f MPa",
                 min_voltage_mv, max_voltage_mv, min_pressure_mpa, max_pressure_mpa);
    }

    // Read pressure in MPa
    float readMPa() {
        float voltage_mv = readAverageVoltage();
        
        // Clamp voltage to valid range
        if (voltage_mv < min_voltage_mv) {
            voltage_mv = min_voltage_mv;
        }
        if (voltage_mv > max_voltage_mv) {
            voltage_mv = max_voltage_mv;
        }
        
        // Linear interpolation: pressure = min + (voltage - min_v) * (max - min) / (max_v - min_v)
        float pressure = min_pressure_mpa + 
                        (voltage_mv - min_voltage_mv) * (max_pressure_mpa - min_pressure_mpa) / 
                        (max_voltage_mv - min_voltage_mv);
        
        return pressure;
    }

    // Read pressure in Bar
    float readBar() {
        return readMPa() * 10.0f;  // 1 MPa = 10 Bar
    }

    // Read pressure in PSI
    float readPSI() {
        return readMPa() * 145.038f;  // 1 MPa = 145.038 PSI
    }

    // Get raw voltage reading in mV
    float getVoltage() {
        return readAverageVoltage();
    }

private:
    float readAverageVoltage() {
        float sum = 0.0f;
        for (int i = 0; i < num_samples; i++) {
            sum += adc.readMillivolts();
            vTaskDelay(pdMS_TO_TICKS(2));  // Small delay between samples
        }
        return sum / num_samples;
    }
};