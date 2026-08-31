// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../Configuration/Configurable.h"
#include "../Pin.h"
#include "CanIds.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <cstdint>
#include <vector>

namespace CAN {
    // Implemented by anything that wants to see received frames.  onCanFrame() runs on the
    // shared RX task, which sits above the protocol task in priority precisely so that button
    // and jog latency stays low.  Handlers must therefore be short and must never block.
    class CanListener {
    public:
        virtual void onCanFrame(uint32_t id, uint8_t len, const uint8_t* data, int64_t rx_time_us) = 0;
        virtual ~CanListener() {}
    };

    class CanBus : public Configuration::Configurable {
        struct Subscription {
            CanListener* listener;
            uint32_t     base;
            uint32_t     mask;
        };

        struct TxFrame {
            uint32_t id;
            uint8_t  len;
            uint8_t  data[8];
            // When set, the TX task stores the moment it handed the frame to the peripheral.
            // Clock synchronisation needs that instant rather than the enqueue time, since
            // the difference between the two is queueing delay that would bias the fit.
            volatile int64_t* stamp;
        };

    Pin     _txPin;
    Pin     _rxPin;
    int32_t _baudKbit = 500;

    // Optional MOSFET controlling the bus 12V rail, so the whole bus (and any head-mounted
    // module) can be powered down for a tool change and back up afterwards.
    Pin     _powerPin;
    int32_t _powerSettleMs = 250;
    bool    _powerOnBoot   = true;
    bool    _powered       = true;

    std::vector<Subscription> _subscriptions;

    // Guards _subscriptions against the RX task while a CanModule splices listeners in or
    // out.  Null until the tasks start, which is after all boot-time subscribe() calls.
    SemaphoreHandle_t _subMutex = nullptr;

    QueueHandle_t _txQueue     = nullptr;
    TaskHandle_t  _rxTaskH     = nullptr;
    TaskHandle_t  _txTaskH     = nullptr;
    bool          _started     = false;
    bool          _recovering  = false;

        uint32_t _rxCount   = 0;
        uint32_t _txCount   = 0;
        uint32_t _txDropped = 0;

        static void rxTask(void* arg);
        static void txTask(void* arg);

        void startTasks(int tx_pin, int rx_pin, int32_t baud_kbit);

        void dispatch(uint32_t id, uint8_t len, const uint8_t* data, int64_t rx_time_us);
        void checkBusHealth();

    public:
        CanBus() = default;

        // Subscribes to every identifier for which (id & mask) == base.
        void subscribe(CanListener* listener, uint32_t base, uint32_t mask);

        // Removes every subscription belonging to a listener.  Used by CanModule::unload()
        // so the RX task stops dispatching into an unloaded module.
        void unsubscribe(CanListener* listener);

        // Queues a frame on the TX task.  Safe from any task context; never blocks on the bus.
        bool send(uint32_t id, uint8_t len, const uint8_t* data);

        // As send(), but the transmit instant is written to *stamp by the TX task.
        bool sendStamped(uint32_t id, uint8_t len, const uint8_t* data, volatile int64_t* stamp);

        // Same, from interrupt context.  ESP-IDF has no ISR-safe TWAI transmit, so this only
        // enqueues; the TX task performs the actual transmission.
        bool IRAM_ATTR sendFromISR(uint32_t id, uint8_t len, const uint8_t* data);

        // Broadcasts the abort frame ahead of anything already queued.
        void abortAll();

        // Switches the bus 12V rail via _powerPin and waits power_settle_ms.  While unpowered,
        // TX is quiesced and bus-off recovery is suppressed, because the transceiver is dead.
        // A no-op (always powered) when no power_pin is configured.
        void setPower(bool on);
        bool powered() const { return _powered; }
        bool hasPowerControl() const { return _powerPin.defined(); }

        bool started() const { return _started; }

        void init();

        // Brings the bus up from raw pin numbers rather than from the "can:" section.
        // Only for configs that predate the shared bus and declare CAN pins on the ODrive
        // spindle; new configuration should use the "can:" section.
        void initLegacy(int tx_pin, int rx_pin, int32_t baud_kbit);

        void group(Configuration::HandlerBase& handler) override;
        void afterParse() override;

        std::string statusString() const;

        ~CanBus();
    };
}
