// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cstdint>

// A byte-addressable non-volatile memory that can be written without an erase cycle and
// without a wear budget, which is what lets StatePersistence rewrite the machine state
// several times a second.  SPI and I2C parts differ only in transport, so the persistence
// layer talks to them through this.
class FramDevice {
public:
    // Brings up the device and proves it can hold a byte.  False means the caller should
    // carry on without persistence rather than write into the void.
    virtual bool initialize() = 0;
    virtual bool initialized() const = 0;

    // Usable capacity in bytes, which decides how much state fits.
    virtual uint32_t size() const = 0;

    virtual bool read(uint32_t address, uint8_t* data, uint32_t length)        = 0;
    virtual bool write(uint32_t address, const uint8_t* data, uint32_t length) = 0;

    bool readByte(uint32_t address, uint8_t* value) { return read(address, value, 1); }
    bool writeByte(uint32_t address, uint8_t value) { return write(address, &value, 1); }

    // For the startup log, so it is obvious which part was actually found.
    virtual const char* description() const = 0;

    virtual ~FramDevice() = default;
};
