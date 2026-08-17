// Mock implementation of the CAN platform driver for unit tests.
// Delegates to CanMock callbacks or default buffer behavior.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "CAN/CanDriver.h"
#include "ODrive/CanMock.h"

#include <chrono>
#include <thread>

namespace CAN {
    namespace Driver {
        bool init(int tx_pin, int rx_pin, int baud_kbit) {
            auto& m = CanMock::instance();
            if (m.onInit) {
                return m.onInit(tx_pin, rx_pin);
            }
            return true;  // success by default
        }

        void deinit() {}

        bool send(uint32_t id, uint8_t length, const uint8_t* data, uint32_t timeout_ms) {
            auto& m = CanMock::instance();
            if (m.onSend) {
                return m.onSend(id, length, data);
            }
            return m.defaultSend(id, length, data);
        }

        int receive(uint8_t* data, uint32_t* identifier, uint32_t timeout_ms) {
            auto& m   = CanMock::instance();
            int   len = m.onTryReceive ? m.onTryReceive(data, identifier) : m.defaultTryReceive(data, identifier);
            if (len < 0 && timeout_ms > 0) {
                // The real driver blocks for the timeout.  Without a matching wait here the
                // RX task would spin the host CPU flat out during tests.
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            return len;
        }

        bool busOff() {
            return false;
        }
        bool stopped() {
            return false;
        }
        void initiateRecovery() {}
        bool restart() {
            return true;
        }
    }
}
