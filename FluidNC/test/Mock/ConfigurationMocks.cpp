// ConfigurationMocks.cpp - Mock implementations for Configuration testing
// These provide stub implementations for functions used by Parser.cpp that
// are not needed for basic YAML parsing tests.

#include "Pin.h"
#include "UartTypes.h"
#include <string_view>

// Static member for Pin - required by Pin class
namespace Pins {
    class PinDetail;
}
Pins::PinDetail* Pin::undefinedPin = nullptr;

// Mock Pin::create - returns an undefined pin for testing
Pin Pin::create(std::string_view str) {
    return Pin();  // Return undefined pin
}

// Mock Pin destructor
Pin::~Pin() {
    // Mock - don't delete _detail since we're not allocating it
}

// Mock decodeUartMode - returns empty string (no error) for testing
const char* decodeUartMode(std::string_view s, UartData& wordLength, UartParity& parity, UartStop& stopBits) {
    wordLength = UartData::Bits8;
    parity = UartParity::None;
    stopBits = UartStop::Bits1;
    return "";  // No error
}
