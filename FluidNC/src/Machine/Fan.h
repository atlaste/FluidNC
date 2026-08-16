// Copyright (c) 2026 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Module.h"
#include "Pin.h"

namespace Machine {
    // Speed control for a three wire 12V/GND/PWM fan.  The fan runs at one
    // speed while the machine is idle and another while a job is in progress,
    // so it can be quiet at rest without having to stop altogether.
    class Fan : public ConfigurableModule {
        Pin _pwmPin;

        // 25 kHz is the frequency that fan speed controllers conventionally
        // use.  It is above the audible range, so the fan does not whine at
        // partial speed.
        uint32_t _pwmHz = 25000;

        float _idlePercent = 0.0f;
        float _jobPercent  = 100.0f;

        // False until init() has established a working PWM pin
        bool _active = false;

        // Percentage last written, or -1 if nothing has been written yet
        float _currentPercent = -1.0f;

        void setPercent(float percent);

    public:
        Fan(const char* name) : ConfigurableModule(name) {}

        void init() override;

        // Applies the idle or the job speed according to the machine state
        void poll() override;

        void group(Configuration::HandlerBase& handler) override;

        ~Fan() = default;
    };
}
