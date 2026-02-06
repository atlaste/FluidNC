// ConfigurationMocks.cpp - Mock implementations for Configuration testing
// These provide stub implementations for functions used by Parser.cpp that
// are not needed for basic YAML parsing tests.

#include "Pin.h"
#include "UartTypes.h"
#include <string_view>

// decodeUartMode is now provided by the real Uart.cpp (which is compiled for VFD spindle tests)

// Note: Platform-specific PinDetail mocks (GPIOPinDetail, I2SOPinDetail, 
// ChannelPinDetail, ExtPinDetail) are in PinsMocks.cpp
