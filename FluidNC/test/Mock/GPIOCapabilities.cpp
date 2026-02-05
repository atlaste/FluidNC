// GPIOCapabilities.cpp - Mock GPIO capabilities for Windows unit testing
// Provides GetDefaultCapabilities for GPIOPinDetail on Windows
// Based on the capture platform version but simplified for testing

#include "Pins/GPIOPinDetail.h"

namespace Pins {
    PinCapabilities GPIOPinDetail::GetDefaultCapabilities(pinnum_t index) {
        // For testing purposes, most pins have full capabilities
        // This matches the ESP32 capture platform behavior for common test pins (16, 17)
        switch (index) {
            case 6:  // Reserved pins (SPI flash on real ESP32)
            case 7:
            case 8:
            case 11:
                return PinCapabilities::Reserved;

            case 34:  // Input only pins (on real ESP32)
            case 35:
            case 36:
            case 37:
            case 38:
            case 39:
                return PinCapabilities::Native | PinCapabilities::Input | PinCapabilities::ADC | 
                       PinCapabilities::ISR | PinCapabilities::UART;

            default:
                // Most pins have full capabilities for testing
                return PinCapabilities::Native | PinCapabilities::Input | PinCapabilities::Output | 
                       PinCapabilities::PullUp | PinCapabilities::PullDown | PinCapabilities::ADC | 
                       PinCapabilities::PWM | PinCapabilities::ISR | PinCapabilities::UART;
        }
    }
}
