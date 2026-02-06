// Mock for ODrive CanESP32 -- callback-based CAN mock for ODrive spindle tests.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <vector>

class CanMock {
public:
    // --- Callbacks (tests override these to interact with CAN communication) ---

    std::function<bool(int txPin, int rxPin)>                                     onInit;
    std::function<bool(uint32_t id, uint8_t length, const uint8_t* data)>         onSend;
    std::function<int(uint8_t* data, uint32_t* identifier)>                       onTryReceive;

    // --- Default buffer behavior ---

    struct CanMessage {
        uint32_t identifier;
        uint8_t  data[8];
        uint8_t  length;
    };

    // RX queue: test pre-loads messages, tryReceive consumes them
    // TX queue: send appends messages, test inspects them
    std::vector<CanMessage> rxQueue;
    std::vector<CanMessage> txQueue;
    std::mutex              mtx;

    // Queue a response message for the ODrive to receive
    void queueResponse(uint32_t identifier, const uint8_t* data, uint8_t length) {
        std::lock_guard<std::mutex> lock(mtx);
        CanMessage msg;
        msg.identifier = identifier;
        msg.length     = length;
        if (data && length > 0) {
            memcpy(msg.data, data, length > 8 ? 8 : length);
        } else {
            memset(msg.data, 0, 8);
        }
        rxQueue.push_back(msg);
    }

    // Get all messages sent by ODrive (test inspects these)
    std::vector<CanMessage> getSent() {
        std::lock_guard<std::mutex> lock(mtx);
        return txQueue;
    }

    // Clear sent messages after inspection
    void clearSent() {
        std::lock_guard<std::mutex> lock(mtx);
        txQueue.clear();
    }

    // Reset everything
    void reset() {
        onInit       = nullptr;
        onSend       = nullptr;
        onTryReceive = nullptr;
        std::lock_guard<std::mutex> lock(mtx);
        rxQueue.clear();
        txQueue.clear();
    }

    // --- Default send: store in TX queue ---
    bool defaultSend(uint32_t id, uint8_t length, const uint8_t* data) {
        std::lock_guard<std::mutex> lock(mtx);
        CanMessage msg;
        msg.identifier = id;
        msg.length     = length;
        if (data && length > 0) {
            memcpy(msg.data, data, length > 8 ? 8 : length);
        } else {
            memset(msg.data, 0, 8);
        }
        txQueue.push_back(msg);
        return true;
    }

    // --- Default tryReceive: consume from RX queue ---
    int defaultTryReceive(uint8_t* data, uint32_t* identifier) {
        std::lock_guard<std::mutex> lock(mtx);
        if (rxQueue.empty()) {
            return -1;  // nothing to receive
        }
        auto& msg   = rxQueue.front();
        *identifier = msg.identifier;
        memcpy(data, msg.data, msg.length);
        int len = msg.length;
        rxQueue.erase(rxQueue.begin());
        return len;
    }

    static CanMock& instance() {
        static CanMock inst;
        return inst;
    }

private:
    CanMock() = default;
};
