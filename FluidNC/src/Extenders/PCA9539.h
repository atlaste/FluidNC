// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "PinExtenderDriver.h"
#include "../Configuration/Configurable.h"
#include "../Machine/MachineConfig.h"
#include "../Machine/I2CBus.h"
#include "../Platform.h"

#include <bitset>

namespace Pins {
    class PCA9539PinDetail;
}

namespace Extenders {
    // NOTE: The PCA9539 is identical to the PCA9555 in terms of API
    class PCA9539 : public PinExtenderDriver {
        friend class Pins::PCA9539PinDetail;

        // Address can be set for up to 4 devices. Each device supports 16 pins.

        static const int        numberPins = 16 * 4;
        std::bitset<numberPins> _claimed;

        Machine::I2CBus* _i2cBus;

        uint8_t I2CGetValue(uint8_t address, uint8_t reg);
        void    I2CSetValue(uint8_t address, uint8_t reg, uint8_t value);

        // Registers:
        // 4x16 = 64 bits. Fits perfectly into an uint64.
        uint64_t _configuration = 0;
        uint64_t _invert        = 0;
        uint64_t _value         = 0;

        // 4 devices, 2 registers per device:
        uint8_t _dirtyRegisters = 0;

        // State:
        int _error;  // error code from I2C

    public:
        PCA9539() = default;

        void claim(pinnum_t index) override;
        void free(pinnum_t index) override;

        void validate() const override;
        void group(Configuration::HandlerBase& handler) override;

        void init();

        void IRAM_ATTR setupPin(pinnum_t index, Pins::PinAttributes attr) override;
        void IRAM_ATTR writePin(pinnum_t index, bool high) override;
        bool IRAM_ATTR readPin(pinnum_t index) override;
        void IRAM_ATTR flushWrites() override;

        const char* name() const override;

        ~PCA9539() = default;
    };
}
