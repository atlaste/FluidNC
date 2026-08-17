// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "CanPwmSpindle.h"

#include "../CAN/CanIds.h"
#include "../CAN/CanNode.h"
#include "../CAN/CanScheduler.h"
#include "../Machine/MachineConfig.h"
#include "../System.h"  // sys.abort

#include <esp_timer.h>

namespace Spindles {
    namespace {
        std::vector<CanPwm*>& registry() {
            static std::vector<CanPwm*> instances;
            return instances;
        }
    }

    CanPwm::CanPwm(const char* name) : Spindle(name) {
        registry().push_back(this);
    }

    CanPwm::~CanPwm() {
        auto& instances = registry();
        for (auto it = instances.begin(); it != instances.end(); ++it) {
            if (*it == this) {
                instances.erase(it);
                break;
            }
        }
    }

    void CanPwm::validate() {
        Spindle::validate();
        Assert(config->_can != nullptr, "CAN PWM spindle: no CAN bus configured; add a top level 'can:' section");
        Assert(_nodeId >= 1 && _nodeId <= int32_t(CAN::MaxNodeId), "CAN PWM spindle: node must be between 1 and %d", int(CAN::MaxNodeId));
        Assert(_channel >= 0 && _channel <= 7, "CAN PWM spindle: channel must be between 0 and 7");
    }

    void CanPwm::init() {
        if (config->_can == nullptr) {
            return;
        }

        CAN::CanNodes::instance().node(uint8_t(_nodeId));

        is_reversable  = false;
        _current_state = SpindleState::Disable;

        if (_speeds.size() == 0) {
            linearSpeeds(_laser ? 255 : 10000, 100.0f);
        }
        setupSpeeds(uint32_t(_maxDuty));
        init_atc();
        config_message();

        // If this spindle is already the active one, e.g. because it is the only spindle,
        // claim the scheduler now; otherwise onSpindleChanged() will do it later.
        onSpindleChanged(spindle);
    }

    void CanPwm::onSpindleChanged(Spindle* active) {
        for (auto candidate : registry()) {
            if (static_cast<Spindle*>(candidate) == active) {
                CAN::CanScheduler::bindPwm(uint8_t(candidate->_nodeId), uint8_t(candidate->_channel), candidate->_laser);
                return;
            }
        }
        CAN::CanScheduler::unbindPwm();
    }

    void CanPwm::setEnable(bool enable) {
        if (_enableOutput < 0) {
            return;
        }
        auto node = CAN::CanNodes::instance().find(uint8_t(_nodeId));
        if (node) {
            node->scheduleDigital(uint8_t(_enableOutput), enable, esp_timer_get_time());
        }
    }

    void CanPwm::setState(SpindleState state, SpindleSpeed speed) {
        if (sys.abort()) {
            return;
        }

        uint32_t dev_speed = mapSpeed(state, speed);

        // In laser mode M4 hands power over to the motion planner, so the duty set here is
        // only the resting value; the first segment supplies the real one.
        if (isRateAdjusted() && state == SpindleState::Ccw) {
            dev_speed = offSpeed();
        }

        CAN::CanScheduler::setPwmImmediate(dev_speed);
        setEnable(state != SpindleState::Disable);

        _current_state = state;
        _current_speed = dev_speed;

        spindleDelay(state, speed);
    }

    void CanPwm::config_message() {
        log_info(name() << " Spindle Node:" << _nodeId << " Channel:" << _channel << " MaxDuty:" << _maxDuty
                        << (_laser ? " Laser" : "") << atc_info());
    }

    namespace {
        SpindleFactory::InstanceBuilder<CanPwm> registration("can_pwm");
    }
}
