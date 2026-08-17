// Copyright (c) 2024 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "TMC2240Stepper.h"
#include "TrinamicSpiDriver.h"
#include "Pin.h"
#include "PinMapper.h"

#include <cstdint>

// The TMC2240 has no sense resistor.  Full scale current is set by the resistor
// from IREF to ground, which is 12k on most breakout boards.
const float TMC2240_RREF_DEFAULT = 12.0f;

namespace MotorDrivers {

    class TMC2240Driver : public TrinamicSpiDriver {
    public:
        TMC2240Driver(const char* name) : TrinamicSpiDriver(name) {}

        // Overrides for inherited methods
        void init() override;
        void set_disable(bool disable);
        void config_motor() override;
        void debug_message() override;
        void validate() override { StandardStepper::validate(); }

        void group(Configuration::HandlerBase& handler) override {
            TrinamicSpiDriver::group(handler);
            handler.item("tpfd", _tpfd, 0, 15);
            handler.item("r_ref_kohms", _r_ref, 12.0f, 60.0f);
            handler.item("slope_control", _slope_control, 0, 3);
        }

    private:
        TMC2240Stepper* tmc2240 = nullptr;

        uint8_t _tpfd          = 4;
        uint8_t _slope_control = 0;
        float   _r_ref         = TMC2240_RREF_DEFAULT;

        bool test();
        void set_registers(bool isHoming) override;
        void config_message() override;
    };
}
