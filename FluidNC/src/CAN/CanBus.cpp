// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "CanBus.h"

#include "CanDriver.h"
#include "../Config.h"
#include "../Logging.h"

#include <esp_timer.h>
#include <cstring>

namespace CAN {
    // The RX task must outrank the protocol and polling tasks (priority 1) so that a button
    // press or jog tick is dispatched promptly rather than waiting behind segment preparation.
    // The TX task outranks the RX task so aborts and scheduled motion frames leave without
    // being held up by inbound traffic.
    static constexpr UBaseType_t RX_TASK_PRIORITY = 5;
    static constexpr UBaseType_t TX_TASK_PRIORITY = 6;
    static constexpr int         TX_QUEUE_DEPTH   = 64;

    void CanBus::group(Configuration::HandlerBase& handler) {
        handler.item("tx_pin", _txPin);
        handler.item("rx_pin", _rxPin);
        handler.item("baud_kbit", _baudKbit);
    }

    void CanBus::afterParse() {
        Assert(_txPin.defined() && _rxPin.defined(), "CAN bus requires both tx_pin and rx_pin");
        switch (_baudKbit) {
            case 25:
            case 50:
            case 100:
            case 125:
            case 250:
            case 500:
            case 800:
            case 1000:
                break;
            default:
                Assert(false, "CAN baud_kbit must be one of 25, 50, 100, 125, 250, 500, 800, 1000");
        }
    }

    void CanBus::init() {
        if (_started) {
            return;
        }
        startTasks(_txPin.getNative(Pin::Capabilities::Output), _rxPin.getNative(Pin::Capabilities::Input), _baudKbit);
    }

    void CanBus::initLegacy(int tx_pin, int rx_pin, int32_t baud_kbit) {
        if (_started) {
            return;
        }
        startTasks(tx_pin, rx_pin, baud_kbit);
    }

    void CanBus::startTasks(int tx_pin, int rx_pin, int32_t baud_kbit) {
        if (!Driver::init(tx_pin, rx_pin, baud_kbit)) {
            return;
        }

        _txQueue = xQueueCreate(TX_QUEUE_DEPTH, sizeof(TxFrame));
        if (_txQueue == nullptr) {
            log_error("CAN: cannot allocate transmit queue");
            Driver::deinit();
            return;
        }

        xTaskCreatePinnedToCore(txTask, "can_tx", 3072, this, TX_TASK_PRIORITY, &_txTaskH, SUPPORT_TASK_CORE);
        xTaskCreatePinnedToCore(rxTask, "can_rx", 4096, this, RX_TASK_PRIORITY, &_rxTaskH, SUPPORT_TASK_CORE);

        _started = true;
    }

    void CanBus::subscribe(CanListener* listener, uint32_t base, uint32_t mask) {
        Assert(listener != nullptr, "CAN subscription requires a listener");
        _subscriptions.push_back({ listener, base & mask, mask });
    }

    bool CanBus::send(uint32_t id, uint8_t len, const uint8_t* data) {
        return sendStamped(id, len, data, nullptr);
    }

    bool CanBus::sendStamped(uint32_t id, uint8_t len, const uint8_t* data, volatile int64_t* stamp) {
        if (!_started) {
            return false;
        }
        TxFrame frame;
        frame.id    = id;
        frame.len   = len;
        frame.stamp = stamp;
        memset(frame.data, 0, sizeof(frame.data));
        if (data != nullptr && len != 0) {
            memcpy(frame.data, data, len > 8 ? 8 : len);
        }
        if (xQueueSend(_txQueue, &frame, 0) != pdTRUE) {
            ++_txDropped;
            return false;
        }
        return true;
    }

    bool IRAM_ATTR CanBus::sendFromISR(uint32_t id, uint8_t len, const uint8_t* data) {
        if (!_started) {
            return false;
        }
        TxFrame frame;
        frame.id    = id;
        frame.len   = len;
        frame.stamp = nullptr;
        memset(frame.data, 0, sizeof(frame.data));
        if (data != nullptr && len != 0) {
            memcpy(frame.data, data, len > 8 ? 8 : len);
        }

        BaseType_t higherPriorityWoken = pdFALSE;
        if (xQueueSendFromISR(_txQueue, &frame, &higherPriorityWoken) != pdTRUE) {
            ++_txDropped;
            return false;
        }
        if (higherPriorityWoken) {
            portYIELD_FROM_ISR();
        }
        return true;
    }

    void CanBus::abortAll() {
        if (!_started) {
            return;
        }
        TxFrame frame;
        frame.id    = IdAbort;
        frame.len   = 0;
        frame.stamp = nullptr;
        memset(frame.data, 0, sizeof(frame.data));

        // Jump the queue.  On the wire IdAbort also wins arbitration against everything else,
        // so this reaches the nodes ahead of any motion frame already in flight elsewhere.
        xQueueSendToFront(_txQueue, &frame, 0);
    }

    void CanBus::txTask(void* arg) {
        auto self = static_cast<CanBus*>(arg);
        TxFrame frame;
        while (true) {
            if (xQueueReceive(self->_txQueue, &frame, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (frame.stamp != nullptr) {
                    *frame.stamp = esp_timer_get_time();
                }
                if (Driver::send(frame.id, frame.len, frame.len ? frame.data : nullptr, 10)) {
                    ++self->_txCount;
                } else {
                    ++self->_txDropped;
                }
            } else {
                self->checkBusHealth();
            }
        }
    }

    void CanBus::rxTask(void* arg) {
        auto     self = static_cast<CanBus*>(arg);
        uint8_t  data[8];
        uint32_t id;
        while (true) {
            int len = Driver::receive(data, &id, 100);
            if (len >= 0) {
                ++self->_rxCount;
                self->dispatch(id, uint8_t(len), data, esp_timer_get_time());
            }
        }
    }

    void CanBus::dispatch(uint32_t id, uint8_t len, const uint8_t* data, int64_t rx_time_us) {
        for (auto& sub : _subscriptions) {
            if ((id & sub.mask) == sub.base) {
                sub.listener->onCanFrame(id, len, data, rx_time_us);
            }
        }
    }

    void CanBus::checkBusHealth() {
        if (!_started) {
            return;
        }
        if (!_recovering && Driver::busOff()) {
            log_error("CAN bus off; attempting recovery");
            Driver::initiateRecovery();
            _recovering = true;
            return;
        }
        if (_recovering && Driver::stopped()) {
            if (Driver::restart()) {
                log_info("CAN bus recovered");
                _recovering = false;
            }
        }
    }

    std::string CanBus::statusString() const {
        std::string s = "CAN rx:";
        s += std::to_string(_rxCount);
        s += " tx:";
        s += std::to_string(_txCount);
        s += " dropped:";
        s += std::to_string(_txDropped);
        if (_recovering) {
            s += " RECOVERING";
        }
        return s;
    }

    CanBus::~CanBus() {
        if (_rxTaskH) {
            vTaskDelete(_rxTaskH);
        }
        if (_txTaskH) {
            vTaskDelete(_txTaskH);
        }
        if (_txQueue) {
            vQueueDelete(_txQueue);
        }
        Driver::deinit();
    }
}
