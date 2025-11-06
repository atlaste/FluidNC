#pragma once

// Copyright (c) 2020 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include <esp_timer.h>
#include "can_simple_messages.hpp"
#include <cstdlib>

#include <cstdint>

namespace Spindles::ODrive {
    class CanESP32 {
    public:
        bool init(int txPin, int rxPin);

        bool send(uint32_t id, uint8_t length, const uint8_t* data);
        int  tryReceive(uint8_t* data, uint32_t* identifier);

        ~CanESP32();
    };
}
