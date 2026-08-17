// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "PinExtenderDriver.h"
#include "../CAN/CanBus.h"
#include "../CAN/CanNode.h"
#include "../Configuration/Configurable.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdint>

class InputPin;

namespace Extenders {
    /*
        A pin extender living on a remote CAN node.

        The node owns the debouncing and sends the whole input bitmap whenever anything
        changes, plus unconditionally at a fixed rate.  That periodic frame doubles as the
        heartbeat, so a node that has stopped reporting is indistinguishable from a node
        whose inputs never change only for as long as the heartbeat interval.

        Fail-safe behaviour
        -------------------
        These pins are explicitly allowed to carry e-stop, fault and safety door inputs.
        Latency is not the issue for those: a human takes tens of milliseconds to hit a
        button and the machine has to react in milliseconds, which a CAN node manages
        comfortably.  What matters is that losing the node is never mistaken for "all clear".

        So when the node stops reporting, the cached input bitmap is forced to
        failsafe_state (all zeros by default) and the resulting transitions are dispatched
        normally.  Wire safety inputs normally closed and declare them active low, and the
        loss of a node then asserts them exactly as cutting the wire would.  Inputs declared
        active high -- ordinary buttons -- go inactive instead, which is what you want: a
        vanished node must not press buttons.

        Independently of that, losing a node raises an ExpanderReset alarm, because a
        configuration that puts safety inputs on a node has no business continuing to cut
        while that node is unaccounted for.
    */
    class CanPinExtender : public PinExtenderDriver, public CAN::CanListener {
        static const int numberPins = 64;

        int32_t _nodeId = 0;

        // Bitmap semantics: bit N is pin N, in the node's own electrical sense.  Inversion
        // from the :low pin option happens above us, in Pin.
        volatile uint64_t _inputs        = 0;
        uint64_t          _outputs       = 0;
        uint64_t          _claimed       = 0;
        uint64_t          _outputMask    = 0;
        bool              _outputsDirty  = false;

        // Value the inputs are forced to when the node stops reporting.
        uint32_t _failsafeStateLow  = 0;
        uint32_t _failsafeStateHigh = 0;
        bool     _failsafeInputs    = true;
        bool     _alarmOnLoss       = true;

        bool _nodeLost   = false;
        bool _everOnline = false;

        InputPin* _eventPins[numberPins] = { nullptr };

        CAN::CanNode* _node = nullptr;

        TaskHandle_t _supervisor = nullptr;

        const char* _name;

        static void supervisorTask(void* arg);

        // Applies a new bitmap and dispatches an event for every pin that changed and has a
        // registered listener.  Runs on the CAN RX task, so it only enqueues events.
        void applyInputs(uint64_t next);

        uint64_t failsafeState() const { return (uint64_t(_failsafeStateHigh) << 32) | uint64_t(_failsafeStateLow); }

    public:
        // The configuration factory constructs drivers with their registered name.
        explicit CanPinExtender(const char* name) : _name(name) {}

        const char* name() const override { return _name; }

        void claim(pinnum_t index) override;
        void free(pinnum_t index) override;

        void init() override;

        void IRAM_ATTR setupPin(pinnum_t index, Pins::PinAttributes attr) override;
        void IRAM_ATTR writePin(pinnum_t index, bool high) override;
        bool IRAM_ATTR readPin(pinnum_t index) override;
        void IRAM_ATTR flushWrites() override;

        Pins::PinCapabilities capabilities() const override;
        void                  registerEvent(pinnum_t index, InputPin* obj) override;

        void onCanFrame(uint32_t id, uint8_t len, const uint8_t* data, int64_t rx_time_us) override;

        void group(Configuration::HandlerBase& handler) override;
        void afterParse() override;

        ~CanPinExtender();
    };
}
