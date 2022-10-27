// Copyright (c) 2020 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cstdint>

#include <WString.h>

#include "Spindle.h"

#include "../Configuration/Configurable.h"
#include "../Configuration/GenericFactory.h"
#include "../Machine/Motor.h"

namespace Spindles {
    // This is the base class. Do not use this as your spindle
    class MotorSpindle : public Spindle {
        String axis_;
        Machine::Motor* motor_ = nullptr;
        
    public:
        void init() override;

        // Used by Protocol.cpp to restore the state during a restart
        void setState(SpindleState state, uint32_t speed) override;
        void config_message() override;
        void setSpeedfromISR(uint32_t dev_speed) override;
        
        // Name is required for the configuration factory to work.
        const char* name() const override { return "motor_spindle"; }

        void afterParse() override;

        void group(Configuration::HandlerBase& handler) override {
            handler.item("axis", axis_);
            Spindle::group(handler);
        }

        // Virtual base classes require a virtual destructor.
        virtual ~MotorSpindle();
    };
}
