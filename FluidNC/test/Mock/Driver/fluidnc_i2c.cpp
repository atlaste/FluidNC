// Mock implementation of fluidnc_i2c for Windows unit testing
// Bridges to the Wire mock from X86TestSupport

#include "Driver/fluidnc_i2c.h"
#include <Wire.h>

// Map bus numbers to Wire instances
static TwoWire* getWire(objnum_t bus_number) {
    switch (bus_number) {
        case 0: return &Wire;
        case 1: return &Wire1;
        default: return nullptr;
    }
}

bool i2c_master_init(objnum_t bus_number, pinnum_t sda_pin, pinnum_t scl_pin, uint32_t frequency) {
    TwoWire* wire = getWire(bus_number);
    if (!wire) {
        return true;  // Error
    }
    return !wire->begin(sda_pin, scl_pin, frequency);  // Wire::begin returns true on success, we return false on success
}

int i2c_write(objnum_t bus_number, uint8_t address, const uint8_t* data, size_t count) {
    TwoWire* wire = getWire(bus_number);
    if (!wire) {
        return -1;
    }
    
    wire->beginTransmission(address);
    size_t written = wire->write(data, count);
    uint8_t error = wire->endTransmission();
    
    if (error != 0) {
        return -error;
    }
    return static_cast<int>(written);
}

int i2c_read(objnum_t bus_number, uint8_t address, uint8_t* data, size_t count) {
    TwoWire* wire = getWire(bus_number);
    if (!wire) {
        return -1;
    }
    
    size_t received = wire->requestFrom(address, static_cast<uint8_t>(count));
    size_t i = 0;
    while (wire->available() && i < count) {
        data[i++] = static_cast<uint8_t>(wire->read());
    }
    return static_cast<int>(i);
}
