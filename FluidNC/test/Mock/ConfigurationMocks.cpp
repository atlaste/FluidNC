// ConfigurationMocks.cpp - Mock implementations for Configuration testing
// These provide stub implementations for functions used by Parser.cpp that
// are not needed for basic YAML parsing tests.

#include "Pin.h"
#include "UartTypes.h"
#include <string_view>

// Mock decodeUartMode - returns empty string (no error) for testing
const char* decodeUartMode(std::string_view s, UartData& wordLength, UartParity& parity, UartStop& stopBits) {
    wordLength = UartData::Bits8;
    parity = UartParity::None;
    stopBits = UartStop::Bits1;
    return "";  // No error
}

// Note: Platform-specific PinDetail mocks (GPIOPinDetail, I2SOPinDetail, 
// ChannelPinDetail, ExtPinDetail) are in PinsMocks.cpp
