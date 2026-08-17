// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "FramDevice.h"
#include "Machine/I2CBus.h"

#include <cstdint>

// Fujitsu MB85RCxx I2C FRAM.
//
// The family splits into two addressing schemes and the difference is not cosmetic:
//
//  - Parts up to 4 Kbit (MB85RC04V and friends) take a single address byte and carry the
//    high address bits in the slave address, so a 512 byte part answers to two consecutive
//    I2C addresses of 256 bytes each.  A transfer that crosses that seam has to be split.
//  - Parts from 16 Kbit up take a two byte address and occupy one I2C address.
//
// Which scheme applies is derived from the configured capacity rather than probed, because
// the density register is not present on the small parts.
class MB85RC : public FramDevice {
    Machine::I2CBus* _bus         = nullptr;
    uint8_t          _baseAddress = 0x50;
    uint32_t         _size        = 512;

    bool    _twoByteAddress = false;
    uint8_t _pageMask       = 0;
    bool    _initialized    = false;

    // Kept well inside the driver's 10 ms transaction timeout at 100 kHz.
    static const uint32_t MaxChunk = 32;

    uint8_t slaveAddress(uint32_t address) const {
        return _twoByteAddress ? _baseAddress : uint8_t(_baseAddress | ((address >> 8) & _pageMask));
    }

    // Largest run starting at address that stays within one chunk and, for the small parts,
    // within one page.
    uint32_t runLength(uint32_t address, uint32_t remaining) const;

    bool readChunk(uint32_t address, uint8_t* data, uint32_t length);
    bool writeChunk(uint32_t address, const uint8_t* data, uint32_t length);

public:
    MB85RC(Machine::I2CBus* bus, uint8_t baseAddress, uint32_t sizeBytes);

    bool        initialize() override;
    bool        initialized() const override { return _initialized; }
    uint32_t    size() const override { return _size; }
    bool        read(uint32_t address, uint8_t* data, uint32_t length) override;
    bool        write(uint32_t address, const uint8_t* data, uint32_t length) override;
    const char* description() const override { return "MB85RC I2C FRAM"; }

    ~MB85RC() = default;
};
