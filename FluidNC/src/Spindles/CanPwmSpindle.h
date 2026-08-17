// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Spindle.h"

#include <cstdint>
#include <vector>

namespace Spindles {
    /*
        A PWM tool -- typically a laser -- whose output transistor is on a CAN node.

        Laser power has to change in step with the feed rate, so it cannot be sent when the
        master happens to notice a new speed: by the time a frame arrives, the head has moved.
        Instead the duty changes are scheduled the same way CAN axis steps are, with the
        absolute time at which they take effect, and the node applies them from its own timer.
        The result is that the burn tracks the corners as tightly as a locally driven laser.

        Outside motion -- an M3 with no move after it, or a jog -- the duty is sent as an
        immediate change, because there is no timeline to hang it on.
    */
    class CanPwm : public Spindle {
    public:
        CanPwm(const char* name);
        ~CanPwm() override;

        CanPwm(const CanPwm&)            = delete;
        CanPwm& operator=(const CanPwm&) = delete;

        void init() override;
        void setState(SpindleState state, SpindleSpeed speed) override;
        void config_message() override;
        bool isRateAdjusted() override { return _laser; }

        // Speed changes during motion arrive at the node ahead of time, from CanScheduler,
        // so there is nothing to do when the ISR reaches the segment.
        void setSpeedfromISR(uint32_t dev_speed) override {}

        void validate() override;

        void group(Configuration::HandlerBase& handler) override {
            handler.item("node", _nodeId);
            handler.item("channel", _channel);
            handler.item("max_duty", _maxDuty, 1, 65535);
            handler.item("enable_output", _enableOutput, -1, 63);
            handler.item("laser", _laser);

            Spindle::group(handler);
        }

        // Called when the active spindle changes, so that the scheduler drives whichever CAN
        // PWM spindle is now in charge and none when the active spindle is a local one.
        static void onSpindleChanged(Spindle* active);

    private:
        int32_t _nodeId       = 0;
        int32_t _channel      = 0;
        int32_t _maxDuty      = 1023;
        int32_t _enableOutput = -1;
        bool    _laser        = false;

        void setEnable(bool enable);
    };
}
