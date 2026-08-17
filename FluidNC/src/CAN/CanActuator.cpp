// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "CanActuator.h"

#include "CanIds.h"
#include "../Logging.h"
#include "../Machine/MachineConfig.h"

#include <cstring>
#include <esp_timer.h>

namespace CAN {
    namespace {
        std::vector<CanActuator*>& registry() {
            static std::vector<CanActuator*> instances;
            return instances;
        }
    }

    CanActuator::CanActuator(const char* name) : ConfigurableModule(name) {
        registry().push_back(this);
    }

    CanActuator::~CanActuator() {
        auto& instances = registry();
        for (auto it = instances.begin(); it != instances.end(); ++it) {
            if (*it == this) {
                instances.erase(it);
                break;
            }
        }
    }

    void CanActuator::group(Configuration::HandlerBase& handler) {
        handler.item("label", _label);
        handler.item("node", _nodeId);
        handler.item("index", _actuatorIndex);
        handler.item("timeout_ms", _defaultTimeoutMs);
    }

    void CanActuator::afterParse() {
        Assert(!_label.empty(), "actuator: label is required so that macros and tool changers can refer to it");
        Assert(_nodeId >= 1 && _nodeId <= int32_t(MaxNodeId), "actuator '%s': node must be between 1 and %d", _label.c_str(), int(MaxNodeId));
        Assert(_actuatorIndex >= 0 && _actuatorIndex <= 15, "actuator '%s': index must be between 0 and 15", _label.c_str());
        Assert(config->_can != nullptr, "actuator '%s': no CAN bus configured; add a top level 'can:' section", _label.c_str());
    }

    void CanActuator::init() {
        if (config->_can == nullptr) {
            return;
        }
        _node      = CanNodes::instance().node(uint8_t(_nodeId));
        _doneQueue = xQueueCreate(4, sizeof(uint8_t));

        config->_can->subscribe(&_listener, IdActuatorDoneBase + uint32_t(_nodeId), 0x7FF);

        log_info("CAN actuator '" << _label << "' on node " << _nodeId << " index " << _actuatorIndex);
    }

    void CanActuator::DoneListener::onCanFrame(uint32_t id, uint8_t len, const uint8_t* data, int64_t rx_time_us) {
        if (len < 2) {
            return;
        }
        // Payload: uint8 actuator index, uint8 status (0 = ok).
        if (int32_t(data[0]) != _owner->_actuatorIndex) {
            return;
        }
        _owner->onDone(data[1]);
    }

    void CanActuator::onDone(uint8_t status) {
        if (_doneQueue) {
            xQueueSend(_doneQueue, &status, 0);
        }
    }

    bool CanActuator::run(Command command, int32_t argument, uint32_t timeout_ms) {
        if (config->_can == nullptr || _node == nullptr) {
            log_error("CAN actuator '" << _label << "' has no bus");
            return false;
        }
        if (!_node->online()) {
            log_error("CAN actuator '" << _label << "' node " << _nodeId << " is " << _node->healthName());
            return false;
        }

        // Discard any completion left over from a command that timed out, so that this one
        // does not immediately see a stale success.
        if (_doneQueue) {
            xQueueReset(_doneQueue);
        }

        uint8_t payload[6];
        payload[0] = uint8_t(command);
        payload[1] = uint8_t(_actuatorIndex);
        payload[2] = uint8_t(argument & 0xFF);
        payload[3] = uint8_t((argument >> 8) & 0xFF);
        payload[4] = uint8_t((argument >> 16) & 0xFF);
        payload[5] = uint8_t((argument >> 24) & 0xFF);

        if (!config->_can->send(IdActuatorCmdBase + uint32_t(_nodeId), 6, payload)) {
            log_error("CAN actuator '" << _label << "' could not queue command");
            return false;
        }

        if (timeout_ms == 0) {
            return true;
        }

        uint8_t status = 0xFF;
        if (_doneQueue == nullptr || xQueueReceive(_doneQueue, &status, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            log_error("CAN actuator '" << _label << "' timed out after " << timeout_ms << " ms");
            return false;
        }
        if (status != 0) {
            log_error("CAN actuator '" << _label << "' reported status " << int(status));
            return false;
        }
        return true;
    }

    const std::vector<CanActuator*>& CanActuator::all() {
        return registry();
    }

    CanActuator* CanActuator::byLabel(const std::string& label) {
        for (auto actuator : all()) {
            if (actuator->label() == label) {
                return actuator;
            }
        }
        return nullptr;
    }

    namespace {
        ConfigurableModuleFactory::InstanceBuilder<CanActuator> registration("can_actuator");
    }
}
