#include "PressureSensor.h"
#include "NutsBolts.h"

namespace ATCs {
    PressureSensor::PressureSensor( float sensor_min_v,
                                   float sensor_max_v,
                                   float min_press_mpa,
                                   float max_press_mpa,
                                   float voltage_divider_ratio,
                                   int   samples) :
        adc(ADC_UNIT_1, ADC_CHANNEL_1, ADC_ATTEN_DB_12),
        min_pressure_mpa(min_press_mpa), max_pressure_mpa(max_press_mpa), num_samples(samples) {
        // Apply voltage divider ratio to sensor voltage range
        min_voltage_mv = sensor_min_v * voltage_divider_ratio * 1000.0f;
        max_voltage_mv = sensor_max_v * voltage_divider_ratio * 1000.0f;

        log_info("Pressure sensor initialized: " << min_voltage_mv << "-" << max_voltage_mv << " -> " << min_pressure_mpa << "-"
                                                 << max_press_mpa);
    }

    float PressureSensor::readAverageVoltage() {
        float sum = 0.0f;
        for (int i = 0; i < num_samples; i++) {
            sum += adc.readMillivolts();
            delay_ms(2);  // Small delay between samples
        }
        return sum / num_samples;
    }

    // Read pressure in MPa
    float PressureSensor::readMPa() {
        float voltage_mv = readAverageVoltage();

        // Clamp voltage to valid range
        if (voltage_mv < min_voltage_mv) {
            voltage_mv = min_voltage_mv;
        }
        if (voltage_mv > max_voltage_mv) {
            voltage_mv = max_voltage_mv;
        }

        // Linear interpolation: pressure = min + (voltage - min_v) * (max - min) / (max_v - min_v)
        float pressure =
            min_pressure_mpa + (voltage_mv - min_voltage_mv) * (max_pressure_mpa - min_pressure_mpa) / (max_voltage_mv - min_voltage_mv);

        return pressure;
    }

    // Read pressure in Bar
    float PressureSensor::readBar() {
        return readMPa() * 10.0f;  // 1 MPa = 10 Bar
    }

    // Read pressure in PSI
    float PressureSensor::readPSI() {
        return readMPa() * 145.038f;  // 1 MPa = 145.038 PSI
    }

    // Get raw voltage reading in mV
    float PressureSensor::getVoltage() {
        return readAverageVoltage();
    }
}
