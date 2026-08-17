// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "MotorDriver.h"

#include <cstdint>

namespace CAN {
    class CanNode;
}

namespace MotorDrivers {
    /*
        A stepper whose step and direction pins live on a CAN node instead of on this board.

        The motor is a full participant in coordinated motion.  It is not stepped by the local
        stepper ISR; instead CanScheduler watches the segment stream, runs a shadow Bresenham
        for this axis, and sends the node a cumulative step target plus the absolute time at
        which that target must be reached.  The node interpolates between the targets it is
        given, so the traffic is one frame per segment rather than one frame per step.

        The consequence is that the axis is exactly as accurate as the clock synchronisation
        with the node, which is why motion refuses to start until the node is synchronised.

        Limit switches for this axis still belong on the master.  Homing drives the axis
        through the ordinary motion path, so a CAN-side limit input would put the bus latency
        inside the stopping distance.
    */
    class CanStepper : public MotorDriver {
    public:
        CanStepper(const char* name) : MotorDriver(name) {}

        void init() override;

        // Homing works because the motion runs through the normal segment stream and the
        // limit switch is local.
        bool set_homing_mode(bool isHoming) override { return true; }
        void set_disable(bool disable) override;

    protected:
        void config_message() override;
        void validate() override;

        void group(Configuration::HandlerBase& handler) override {
            handler.item("node", _nodeId);
            handler.item("motor", _motorIndex);
        }

    private:
        int32_t _nodeId      = 0;
        int32_t _motorIndex  = 0;

        CAN::CanNode* _node = nullptr;
    };
}
