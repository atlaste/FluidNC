// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../Config.h"
#include "../Module.h"
#include "CanBus.h"
#include "CanNode.h"

#include <cstdint>

namespace CAN {
    /*
        A jog wheel / pendant on a CAN node.

        Latency is the whole point of this module, so the work is split to put as little as
        possible between the operator turning the wheel and the machine moving.

        The node accumulates encoder counts and sends one frame per report interval carrying
        the delta since the last frame, the selected axis and the selected step scale.  It
        does not send a frame per detent: on a shared bus that would be both wasteful and,
        under load, slower than batching.

        The master converts a delta straight into a short incremental jog.  Two of those are
        allowed to be in flight at once.  One is not enough -- the machine would stop between
        detents and the wheel would feel notchy -- and more than two makes the machine keep
        moving noticeably after the operator stops turning, which is the thing people
        actually complain about.

        A reversal of direction, or the wheel stopping, cancels whatever is queued rather
        than letting it play out.
    */
    class CanPendant : public ConfigurableModule, public CanListener {
    public:
        CanPendant(const char* name) : ConfigurableModule(name) {}

        void init() override;
        void poll() override;

        void onCanFrame(uint32_t id, uint8_t len, const uint8_t* data, int64_t rx_time_us) override;

        void group(Configuration::HandlerBase& handler) override;
        void afterParse();

    private:
        // Report payload: int16 delta, uint8 axis, uint8 scale index, uint8 buttons.
        struct Report {
            int16_t delta;
            uint8_t axis;
            uint8_t scaleIndex;
            uint8_t buttons;
        };

        static constexpr int MaxJogsInFlight = 2;
        static constexpr int NumScales       = 4;

        int32_t _nodeId = 0;

        // Millimetres of travel per encoder detent, selected by the pendant's own switch.
        float _scales[NumScales] = { 0.001f, 0.01f, 0.1f, 1.0f };

        // Feed rate used for pendant jogs, mm/min.
        float _feedRate = 1000.0f;

        // Below this many detents per report the wheel counts as stopped.
        int32_t _idleReports = 3;

        // Written by the CAN RX task, read by the protocol task.  Kept as separate scalars
        // rather than a volatile struct so that each store is a single word.
        volatile int16_t _rxDelta      = 0;
        volatile uint8_t _rxAxis       = 0;
        volatile uint8_t _rxScaleIndex = 0;
        volatile uint8_t _rxButtons    = 0;
        volatile bool    _haveNew      = false;

        int32_t _lastSign   = 0;
        int32_t _quietPolls = 0;
        bool    _jogging    = false;

        CanNode* _node = nullptr;

        void   handleReport(const Report& report);
        bool   jogsInFlight() const;
        void   cancelJog();
    };
}
