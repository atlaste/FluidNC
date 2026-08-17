// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "CanStepper.h"

#include "../CAN/CanIds.h"
#include "../CAN/CanNode.h"
#include "../CAN/CanScheduler.h"
#include "../Machine/MachineConfig.h"

namespace MotorDrivers {
    void CanStepper::validate() {
        Assert(config->_can != nullptr, "CAN stepper: no CAN bus configured; add a top level 'can:' section");
        Assert(_nodeId >= 1 && _nodeId <= int32_t(CAN::MaxNodeId), "CAN stepper: node must be between 1 and %d", int(CAN::MaxNodeId));
        Assert(_motorIndex >= 0 && _motorIndex <= 7, "CAN stepper: motor must be between 0 and 7");
    }

    void CanStepper::init() {
        if (config->_can == nullptr) {
            return;
        }
        _node = CAN::CanNodes::instance().node(uint8_t(_nodeId));
        CAN::CanScheduler::bindAxis(axis_index(), uint8_t(_nodeId), uint8_t(_motorIndex));
        config_message();
    }

    void CanStepper::config_message() {
        log_info("    " << name() << " Node:" << _nodeId << " Motor:" << _motorIndex);
    }

    void CanStepper::set_disable(bool disable) {
        if (_node) {
            _node->sendEnable(uint8_t(_motorIndex), !disable);
        }
    }

    namespace {
        MotorFactory::InstanceBuilder<CanStepper> registration("can_stepper");
    }
}
