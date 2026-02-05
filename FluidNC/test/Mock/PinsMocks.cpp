// PinsMocks.cpp - Mock implementations for Pin testing on Windows
// These provide stub implementations for platform-specific PinDetail types
// that are not available on Windows (I2SO, UART Channel, Extender pins)
//
// NOTE: GPIOPinDetail uses the REAL implementation from FluidNC/src/Pins/GPIOPinDetail.cpp
// which calls the mocked fluidnc_gpio functions that forward to SoftwareGPIO.
// This gives us real code coverage while still allowing testability.

#include "Pins/ChannelPinDetail.h"
#include "Pins/ExtPinDetail.h"

// Check if I2SO is enabled
#include "Config.h"
#if MAX_N_I2SO
#include "Pins/I2SOPinDetail.h"
#endif

namespace Pins {

// ============================================================================
// I2SOPinDetail mock implementation (if enabled)
// ============================================================================
#if MAX_N_I2SO

std::vector<bool> I2SOPinDetail::_claimed(32, false);

I2SOPinDetail::I2SOPinDetail(pinnum_t index, const PinOptionsParser& options) :
    PinDetail(index),
    _capabilities(PinCapabilities::Output),
    _attributes(PinAttributes::None) {
    // Mock - don't actually initialize I2S on Windows
}

PinCapabilities I2SOPinDetail::capabilities() const {
    return _capabilities;
}

void I2SOPinDetail::write(bool high) {
    _lastWrittenValue = high;
}

void I2SOPinDetail::synchronousWrite(bool high) {
    _lastWrittenValue = high;
}

bool I2SOPinDetail::read() {
    return _lastWrittenValue;
}

void I2SOPinDetail::setAttr(PinAttributes value, uint32_t frequency) {
    _attributes = value;
}

PinAttributes I2SOPinDetail::getAttr() const {
    return _attributes;
}

std::string I2SOPinDetail::toString() {
    return "i2so." + std::to_string(_index);
}

#endif // MAX_N_I2SO

// ============================================================================
// ChannelPinDetail mock implementation
// ============================================================================

ChannelPinDetail::ChannelPinDetail(UartChannel* channel, pinnum_t index, const PinOptionsParser& options) :
    PinDetail(index),
    _channel(channel),
    _attributes(PinAttributes::None) {
    // Mock - don't actually initialize UART channel on Windows
}

PinCapabilities ChannelPinDetail::capabilities() const {
    return PinCapabilities::Input | PinCapabilities::Output;
}

void ChannelPinDetail::write(bool high) {
    _value = high;
}

bool ChannelPinDetail::read() {
    return _value;
}

void ChannelPinDetail::setAttr(PinAttributes value, uint32_t frequency) {
    _attributes = value;
}

PinAttributes ChannelPinDetail::getAttr() const {
    return _attributes;
}

void ChannelPinDetail::setDuty(uint32_t duty) {
    // Mock - no PWM on UART channel
}

uint32_t ChannelPinDetail::maxDuty() {
    return 0;
}

void ChannelPinDetail::registerEvent(InputPin* obj) {
    // Mock - no ISR on UART channel
}

std::string ChannelPinDetail::toString() {
    return "uart_channel." + std::to_string(_index);
}

// ============================================================================
// ExtPinDetail mock implementation
// ============================================================================

ExtPinDetail::ExtPinDetail(uint32_t device, pinnum_t index, const PinOptionsParser& options) :
    PinDetail(index),
    _device(device),
    _capabilities(PinCapabilities::Input | PinCapabilities::Output),
    _attributes(PinAttributes::None) {
    // Mock - don't actually initialize pin extender on Windows
}

PinCapabilities ExtPinDetail::capabilities() const {
    return _capabilities;
}

void ExtPinDetail::write(bool high) {
    // Mock - no actual pin extender
}

void ExtPinDetail::synchronousWrite(bool high) {
    // Mock - no actual pin extender
}

bool ExtPinDetail::read() {
    return false;
}

void ExtPinDetail::setAttr(PinAttributes value, uint32_t frequency) {
    _attributes = value;
}

PinAttributes ExtPinDetail::getAttr() const {
    return _attributes;
}

std::string ExtPinDetail::toString() {
    return "pinext" + std::to_string(_device) + "." + std::to_string(_index);
}

ExtPinDetail::~ExtPinDetail() {
    // Mock - nothing to clean up
}

} // namespace Pins
