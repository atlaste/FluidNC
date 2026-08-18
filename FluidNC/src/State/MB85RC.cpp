// Copyright (c) 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"
#if MAX_N_I2C

#    include "MB85RC.h"
#    include "Logging.h"
#    include "NutsBolts.h"

#    include <cstring>

MB85RC::MB85RC(Machine::I2CBus* bus, uint8_t baseAddress, uint32_t sizeBytes) :
    _bus(bus), _baseAddress(baseAddress), _size(sizeBytes) {
    _twoByteAddress = _size > 512;

    // The small parts spill their high address bits into the slave address, so work out how
    // many of those bits this capacity actually uses.
    _pageMask = _twoByteAddress ? 0 : uint8_t((_size - 1) >> 8);
}

uint32_t MB85RC::runLength(uint32_t address, uint32_t remaining) const {
    uint32_t length = remaining < MaxChunk ? remaining : MaxChunk;

    if (!_twoByteAddress) {
        // Stop at the page seam; past it the slave address changes and the device's own
        // address counter would wrap to the start of the same page instead.
        uint32_t toPageEnd = 256 - (address & 0xFF);
        if (length > toPageEnd) {
            length = toPageEnd;
        }
    }

    return length;
}

bool MB85RC::readChunk(uint32_t address, uint8_t* data, uint32_t length) {
    uint8_t addressBytes[2];
    uint8_t addressLength = 0;

    if (_twoByteAddress) {
        addressBytes[addressLength++] = uint8_t(address >> 8);
    }
    addressBytes[addressLength++] = uint8_t(address & 0xFF);

    uint8_t slave = slaveAddress(address);

    // A dummy write of the address followed by a separate read.  The driver puts a stop between the
    // two, which the address latch survives; this is the same sequence the I2C pin extenders use and
    // it avoids needing a combined-transaction primitive in the platform layer.  It does mean no other
    // task may use the bus in between, or this read comes back from wherever that task left the
    // address counter, hence the lock.
    Machine::I2CBus::Lock lock(*_bus);

    if (_bus->write(slave, addressBytes, addressLength) != addressLength) {
        return false;
    }

    return _bus->read(slave, data, length) == int(length);
}

bool MB85RC::writeChunk(uint32_t address, const uint8_t* data, uint32_t length) {
    uint8_t buffer[2 + MaxChunk];
    uint8_t index = 0;

    if (_twoByteAddress) {
        buffer[index++] = uint8_t(address >> 8);
    }
    buffer[index++] = uint8_t(address & 0xFF);

    memcpy(buffer + index, data, length);
    index += uint8_t(length);

    return _bus->write(slaveAddress(address), buffer, index) == index;
}

bool MB85RC::read(uint32_t address, uint8_t* data, uint32_t length) {
    if (_bus == nullptr || address + length > _size) {
        return false;
    }

    while (length) {
        uint32_t run = runLength(address, length);
        if (!readChunk(address, data, run)) {
            return false;
        }
        address += run;
        data += run;
        length -= run;
    }

    return true;
}

bool MB85RC::write(uint32_t address, const uint8_t* data, uint32_t length) {
    if (_bus == nullptr || address + length > _size) {
        return false;
    }

    while (length) {
        uint32_t run = runLength(address, length);
        if (!writeChunk(address, data, run)) {
            return false;
        }
        address += run;
        data += run;
        length -= run;
    }

    return true;
}

bool MB85RC::initialize() {
    _initialized = false;

    if (_bus == nullptr) {
        log_error("I2C FRAM has no I2C bus");
        return false;
    }
    if (_size < 256 || (_size & 0xFF) != 0) {
        log_error("I2C FRAM size_bytes must be a whole number of 256 byte pages");
        return false;
    }

    log_info("I2C FRAM at address " << to_hex(_baseAddress) << ", " << _size << " bytes, "
                                    << (_twoByteAddress ? "2 byte" : "1 byte") << " addressing");

    // Prove the part is both present and writable before anything relies on it.  A floating
    // write-protect pin or a wrong address both surface here rather than as state that
    // silently fails to come back after a reboot.  Address 0 is reserved by the layout, so
    // the test cannot land on live state.
    const uint32_t testAddress = 0;

    uint8_t original;
    if (!read(testAddress, &original, 1)) {
        log_error("I2C FRAM did not respond");
        return false;
    }

    for (uint8_t pattern : { uint8_t(0xAA), uint8_t(0x55) }) {
        uint8_t readBack = uint8_t(~pattern);
        if (!write(testAddress, &pattern, 1) || !read(testAddress, &readBack, 1)) {
            log_error("I2C FRAM access failed");
            return false;
        }
        if (readBack != pattern) {
            log_error("I2C FRAM write test failed: wrote " << to_hex(pattern) << " read " << to_hex(readBack)
                                                           << " (check the WP pin is tied low)");
            return false;
        }
    }

    write(testAddress, &original, 1);

    _initialized = true;
    return true;
}

#endif
