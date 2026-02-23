#pragma once

#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

namespace ATCs {
    // ADC Class - Handles ADC initialization and reading
    class ADC {
    private:
        adc_oneshot_unit_handle_t adc_handle  = nullptr;
        adc_cali_handle_t         cali_handle = nullptr;
        adc_channel_t             channel;
        adc_atten_t               attenuation;
        bool                      calibration_enabled = false;

        void initCalibration(adc_unit_t unit, adc_channel_t chan, adc_atten_t atten);

    public:
        ADC(adc_unit_t unit, adc_channel_t chan, adc_atten_t atten = ADC_ATTEN_DB_12);

        ~ADC();

        // Read raw ADC value
        int readRaw();

        // Read voltage in millivolts
        int readMillivolts();

        bool isCalibrated() const;
    };
}
