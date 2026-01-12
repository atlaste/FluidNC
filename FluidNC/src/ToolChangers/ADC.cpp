#include "ADC.h"
#include "Logging.h"

namespace ATCs {
    ADC::ADC(adc_unit_t unit, adc_channel_t chan, adc_atten_t atten = ADC_ATTEN_DB_12) : channel(chan), attenuation(atten) {
        // Configure ADC
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id  = unit,
            .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

        // Configure channel
        adc_oneshot_chan_cfg_t config = {
            .atten    = attenuation,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, channel, &config));

        // Initialize calibration
        initCalibration(unit, chan, atten);

        log_info("ADC initialized on channel " << int(channel));
    }

    // Read raw ADC value
    int ADC::readRaw() {
        int raw_value = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, channel, &raw_value));
        return raw_value;
    }

    // Read voltage in millivolts
    int ADC::readMillivolts() {
        int raw_value  = readRaw();
        int voltage_mv = 0;

        if (calibration_enabled && cali_handle) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw_value, &voltage_mv));
        } else {
            // Fallback: approximate conversion (not recommended for production)
            log_warn("Not using calibrated values. Raw ADC value is " << int(raw_value));
            voltage_mv = raw_value * 3300 / 4095;
        }

        return voltage_mv;
    }

    bool ADC::isCalibrated() const {
        return calibration_enabled;
    }

    void ADC::initCalibration(adc_unit_t unit, adc_channel_t chan, adc_atten_t atten) {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id  = unit,
            .chan     = chan,
            .atten    = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };

        esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
        if (ret == ESP_OK) {
            calibration_enabled = true;
            log_info("ADC calibration initialized");
        } else {
            calibration_enabled = false;
            log_warn("ADC calibration failed, using approximate values");
        }
    }

    ADC::~ADC() {
        if (calibration_enabled && cali_handle) {
            adc_cali_delete_scheme_curve_fitting(cali_handle);
        }
        if (adc_handle) {
            adc_oneshot_del_unit(adc_handle);
        }
    }
}
