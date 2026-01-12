#include "ADC.h"

namespace ATCs {
    // Pressure Sensor Class - Converts ADC readings to pressure values
    class PressureSensor {
    private:
        ADC   adc;
        float min_voltage_mv;    // Minimum voltage in mV (after voltage divider)
        float max_voltage_mv;    // Maximum voltage in mV (after voltage divider)
        float min_pressure_mpa;  // Minimum pressure in MPa
        float max_pressure_mpa;  // Maximum pressure in MPa
        int   num_samples;       // Number of samples to average

        float readAverageVoltage();

    public:
        PressureSensor(float sensor_min_v          = 0.5f,
                       float sensor_max_v          = 4.5f,
                       float min_press_mpa         = 0.0f,
                       float max_press_mpa         = 1.6f,
                       float voltage_divider_ratio = 0.5f,
                       int   samples               = 10);

        // Read pressure in MPa
        float readMPa();

        // Read pressure in Bar
        float readBar();

        // Read pressure in PSI
        float readPSI();

        // Get raw voltage reading in mV
        float getVoltage();

        // TODO FIXME: Group and init methods.
    };
}
