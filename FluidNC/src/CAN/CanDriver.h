// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cstdint>

namespace CAN {
    // Thin platform wrapper around the ESP32 TWAI peripheral.  The peripheral is a chip-wide
    // singleton so this is a namespace of free functions rather than an instantiable class;
    // CanBus owns it and is the only thing that should call into here.
    namespace Driver {
        // baud_kbit must be one of 25, 50, 100, 125, 250, 500, 800 or 1000.
        bool init(int tx_pin, int rx_pin, int baud_kbit);
        void deinit();

        // Identifiers with bit 31 set are transmitted as 29 bit extended frames.
        // Passing data == nullptr sends a remote transmission request.
        bool send(uint32_t id, uint8_t length, const uint8_t* data, uint32_t timeout_ms);

        // Returns the payload length, or -1 if nothing arrived within the timeout.
        int receive(uint8_t* data, uint32_t* identifier, uint32_t timeout_ms);

        // Bus health.  Recovery from bus-off is a two step dance: initiateRecovery() starts
        // it, and once the peripheral reports stopped() the caller must restart() it.
        bool busOff();
        bool stopped();
        void initiateRecovery();
        bool restart();
    }
}
