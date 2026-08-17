// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "CanPinExtender.h"

#include "Extenders.h"
#include "../CAN/CanIds.h"
#include "../Config.h"
#include "../Logging.h"
#include "../Machine/EventPin.h"
#include "../Machine/MachineConfig.h"
#include "../MotionControl.h"  // mc_critical
#include "../Protocol.h"       // protocol_send_event, pinActiveEvent

#include <esp_timer.h>

namespace Extenders {
    void CanPinExtender::group(Configuration::HandlerBase& handler) {
        handler.item("node", _nodeId);
        handler.item("failsafe_inputs", _failsafeInputs);
        handler.item("alarm_on_loss", _alarmOnLoss);

        // The failsafe bitmap is split because the configuration system has no 64 bit item.
        handler.item("failsafe_state_low", _failsafeStateLow);
        handler.item("failsafe_state_high", _failsafeStateHigh);
    }

    void CanPinExtender::afterParse() {
        Assert(_nodeId >= 1 && _nodeId <= int32_t(CAN::MaxNodeId), "can_extender: node must be between 1 and %d", int(CAN::MaxNodeId));
        Assert(config->_can != nullptr, "can_extender: no CAN bus configured; add a top level 'can:' section");
    }

    void CanPinExtender::init() {
        Assert(config->_can != nullptr, "can_extender: no CAN bus configured");

        _node   = CAN::CanNodes::instance().node(uint8_t(_nodeId));
        _inputs = failsafeState();

        config->_can->subscribe(this, CAN::IdExtenderInputBase + uint32_t(_nodeId), 0x7FF);

        xTaskCreatePinnedToCore(supervisorTask, "can_ext", 3072, this, 4, &_supervisor, SUPPORT_TASK_CORE);

        log_info("CAN pin extender on node " << _nodeId);
    }

    void CanPinExtender::claim(pinnum_t index) {
        Assert(index >= 0 && index < numberPins, "CAN pin extender pin %d out of range", int(index));
        Assert((_claimed & (uint64_t(1) << index)) == 0, "CAN pin extender pin %d is already used", int(index));
        _claimed |= (uint64_t(1) << index);
    }

    void CanPinExtender::free(pinnum_t index) {
        _claimed &= ~(uint64_t(1) << index);
        _outputMask &= ~(uint64_t(1) << index);
    }

    void CanPinExtender::setupPin(pinnum_t index, Pins::PinAttributes attr) {
        uint64_t bit = uint64_t(1) << index;
        if (attr.has(Pins::PinAttributes::Output)) {
            _outputMask |= bit;
        } else {
            _outputMask &= ~bit;
        }
    }

    void CanPinExtender::writePin(pinnum_t index, bool high) {
        uint64_t bit  = uint64_t(1) << index;
        uint64_t next = high ? (_outputs | bit) : (_outputs & ~bit);
        if (next != _outputs) {
            _outputs      = next;
            _outputsDirty = true;
        }
    }

    bool CanPinExtender::readPin(pinnum_t index) {
        return (_inputs & (uint64_t(1) << index)) != 0;
    }

    void CanPinExtender::flushWrites() {
        if (!_outputsDirty || config->_can == nullptr) {
            return;
        }

        uint8_t  payload[8];
        uint64_t value = _outputs;
        for (int i = 0; i < 8; ++i) {
            payload[i] = uint8_t((value >> (8 * i)) & 0xFF);
        }

        if (config->_can->send(CAN::IdExtenderOutputBase + uint32_t(_nodeId), 8, payload)) {
            _outputsDirty = false;
        }
    }

    Pins::PinCapabilities CanPinExtender::capabilities() const {
        return Pins::PinCapabilities::Input | Pins::PinCapabilities::Output | Pins::PinCapabilities::CAN;
    }

    void CanPinExtender::registerEvent(pinnum_t index, InputPin* obj) {
        Assert(index >= 0 && index < numberPins, "CAN pin extender pin %d out of range", int(index));
        _eventPins[index] = obj;
    }

    void CanPinExtender::applyInputs(uint64_t next) {
        uint64_t previous = _inputs;
        _inputs           = next;

        uint64_t changed = previous ^ next;
        if (changed == 0) {
            return;
        }

        for (int index = 0; index < numberPins; ++index) {
            uint64_t bit = uint64_t(1) << index;
            if ((changed & bit) == 0 || _eventPins[index] == nullptr) {
                continue;
            }
            // Hand the transition to the protocol task rather than acting on it here; this
            // runs on the CAN RX task, which must stay short.
            bool active = (next & bit) != 0;
            protocol_send_event(active ? &pinActiveEvent : &pinInactiveEvent, _eventPins[index]);
        }
    }

    void CanPinExtender::onCanFrame(uint32_t id, uint8_t len, const uint8_t* data, int64_t rx_time_us) {
        if (len < 8) {
            return;
        }

        uint64_t value = 0;
        for (int i = 0; i < 8; ++i) {
            value |= uint64_t(data[i]) << (8 * i);
        }
        applyInputs(value);
    }

    void CanPinExtender::supervisorTask(void* arg) {
        auto self = static_cast<CanPinExtender*>(arg);

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(50));

            bool online = self->_node != nullptr && self->_node->online();

            if (online) {
                if (self->_nodeLost) {
                    log_info("CAN pin extender node " << self->_nodeId << " is back");
                    self->_nodeLost = false;
                }
                self->_everOnline = true;

                // A node that rebooted has forgotten its outputs, so push them again.
                if (self->_node->consumeResetFlag()) {
                    self->_outputsDirty = true;
                }
            } else if (!self->_nodeLost && self->_everOnline) {
                self->_nodeLost = true;
                log_error("CAN pin extender node " << self->_nodeId << " lost");

                if (self->_failsafeInputs) {
                    self->applyInputs(self->failsafeState());
                }
                if (self->_alarmOnLoss) {
                    mc_critical(ExecAlarm::ExpanderReset);
                }
            }

            // Retry any output update that could not be queued earlier.
            self->flushWrites();
        }
    }

    CanPinExtender::~CanPinExtender() {
        if (_supervisor) {
            vTaskDelete(_supervisor);
        }
    }

    namespace {
        PinExtenderFactory::InstanceBuilder<CanPinExtender> registration("can_extender");
    }
}
