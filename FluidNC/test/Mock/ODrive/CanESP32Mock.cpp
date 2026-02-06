// Mock implementation of CanESP32 for unit tests.
// Delegates to CanMock callbacks or default buffer behavior.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Spindles/ODrive/CanESP32.h"
#include "ODrive/CanMock.h"

namespace Spindles::ODrive {

    bool CanESP32::init(int txPin, int rxPin) {
        auto& m = CanMock::instance();
        if (m.onInit) { return m.onInit(txPin, rxPin); }
        return true;  // success by default
    }

    bool CanESP32::send(uint32_t id, uint8_t length, const uint8_t* data) {
        auto& m = CanMock::instance();
        if (m.onSend) { return m.onSend(id, length, data); }
        return m.defaultSend(id, length, data);
    }

    int CanESP32::tryReceive(uint8_t* data, uint32_t* identifier) {
        auto& m = CanMock::instance();
        if (m.onTryReceive) { return m.onTryReceive(data, identifier); }
        return m.defaultTryReceive(data, identifier);
    }

    CanESP32::~CanESP32() {
        // Nothing to clean up in mock
    }
}
