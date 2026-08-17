// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "CanNode.h"

#include "../Config.h"
#include "../Logging.h"
#include "../Machine/MachineConfig.h"

#include <cstring>
#include <esp_timer.h>

namespace CAN {
    // ---------------------------------------------------------------- CanNode

    const char* CanNode::healthName() const {
        switch (_health) {
            case Health::Offline:
                return "offline";
            case Health::Syncing:
                return "syncing";
            case Health::Online:
                return "online";
            case Health::Faulted:
                return "faulted";
        }
        return "?";
    }

    bool CanNode::consumeResetFlag() {
        bool was  = _resetFlag;
        _resetFlag = false;
        return was;
    }

    void CanNode::onStatus(uint8_t len, const uint8_t* data, int64_t rx_time_us) {
        _lastStatusUs = rx_time_us;

        // Status payload: uint16 boot_id, uint32 fault_flags, uint8 flags.
        if (len < 6) {
            return;
        }

        uint16_t bootId = uint16_t(data[0]) | (uint16_t(data[1]) << 8);
        uint32_t faults = uint32_t(data[2]) | (uint32_t(data[3]) << 8) | (uint32_t(data[4]) << 16) | (uint32_t(data[5]) << 24);

        if (_bootIdKnown && bootId != _bootId) {
            // The node rebooted.  Everything we scheduled on it is gone, and its clock
            // restarted, so the fit is meaningless.
            log_warn("CAN node " << int(_id) << " restarted");
            _clock.reset();
            _resetFlag = true;
        }
        _bootId      = bootId;
        _bootIdKnown = true;
        _faultFlags  = faults;
    }

    void CanNode::onClockReply(uint8_t len, const uint8_t* data, int64_t rx_time_us) {
        if (len < 4) {
            return;
        }
        uint32_t ticks = uint32_t(data[0]) | (uint32_t(data[1]) << 8) | (uint32_t(data[2]) << 16) | (uint32_t(data[3]) << 24);
        _clock.onReply(ticks, CanNodes::instance().clockRequestSentUs(), rx_time_us);
    }

    void CanNode::tick(int64_t now_us) {
        Health previous = _health;

        if (_lastStatusUs == 0 || (now_us - _lastStatusUs) > HeartbeatTimeoutUs) {
            _health = Health::Offline;
        } else if (_faultFlags != 0) {
            _health = Health::Faulted;
        } else if (!_clock.healthy()) {
            // Either the fit has not converged yet or it has stopped tracking.  Both mean we
            // cannot schedule anything on this node.
            _health = Health::Syncing;
        } else {
            _health = Health::Online;
        }

        if (_health != previous) {
            if (_health == Health::Online) {
                log_info("CAN node " << int(_id) << " online, clock " << _clock.frequency() / 1e6 << " MHz");
            } else {
                log_warn("CAN node " << int(_id) << " is " << healthName());
            }
        }
    }

    bool CanNode::sendScheduled(const uint8_t* payload, uint8_t len) {
        if (config->_can == nullptr) {
            return false;
        }
        return config->_can->send(IdScheduleBase + _id, len, payload);
    }

    bool CanNode::scheduleMoveTo(uint8_t motor_index, int32_t step_target, int64_t at_master_us) {
        if (!_clock.synced()) {
            return false;
        }
        uint32_t when = _clock.masterToNode(at_master_us);

        // opcode | motor | int32 target | uint24 time.  The node extends the truncated
        // timestamp against its own 32 bit counter.
        uint8_t payload[8];
        payload[0] = uint8_t(Op::MoveTo) | uint8_t(motor_index << 4);
        payload[1] = uint8_t(step_target & 0xFF);
        payload[2] = uint8_t((step_target >> 8) & 0xFF);
        payload[3] = uint8_t((step_target >> 16) & 0xFF);
        payload[4] = uint8_t((step_target >> 24) & 0xFF);
        payload[5] = uint8_t(when & 0xFF);
        payload[6] = uint8_t((when >> 8) & 0xFF);
        payload[7] = uint8_t((when >> 16) & 0xFF);
        return sendScheduled(payload, 8);
    }

    bool CanNode::schedulePwm(uint8_t channel, uint16_t duty, int64_t at_master_us) {
        if (!_clock.synced()) {
            return false;
        }
        uint32_t when = _clock.masterToNode(at_master_us);

        uint8_t payload[8];
        payload[0] = uint8_t(Op::SetPwm) | uint8_t(channel << 4);
        payload[1] = uint8_t(duty & 0xFF);
        payload[2] = uint8_t((duty >> 8) & 0xFF);
        payload[3] = uint8_t(when & 0xFF);
        payload[4] = uint8_t((when >> 8) & 0xFF);
        payload[5] = uint8_t((when >> 16) & 0xFF);
        payload[6] = uint8_t((when >> 24) & 0xFF);
        return sendScheduled(payload, 7);
    }

    bool CanNode::scheduleDigital(uint8_t pin, bool value, int64_t at_master_us) {
        if (!_clock.synced()) {
            return false;
        }
        uint32_t when = _clock.masterToNode(at_master_us);

        uint8_t payload[8];
        payload[0] = uint8_t(Op::SetDigital);
        payload[1] = pin;
        payload[2] = value ? 1 : 0;
        payload[3] = uint8_t(when & 0xFF);
        payload[4] = uint8_t((when >> 8) & 0xFF);
        payload[5] = uint8_t((when >> 16) & 0xFF);
        payload[6] = uint8_t((when >> 24) & 0xFF);
        return sendScheduled(payload, 7);
    }

    bool CanNode::sendReset(uint8_t motor_index, int32_t step_position) {
        uint8_t payload[8];
        payload[0] = uint8_t(Op::Reset) | uint8_t(motor_index << 4);
        payload[1] = uint8_t(step_position & 0xFF);
        payload[2] = uint8_t((step_position >> 8) & 0xFF);
        payload[3] = uint8_t((step_position >> 16) & 0xFF);
        payload[4] = uint8_t((step_position >> 24) & 0xFF);
        return sendScheduled(payload, 5);
    }

    bool CanNode::sendEnable(uint8_t motor_index, bool enable) {
        uint8_t payload[2];
        payload[0] = uint8_t(Op::Enable) | uint8_t(motor_index << 4);
        payload[1] = enable ? 1 : 0;
        return sendScheduled(payload, 2);
    }

    // --------------------------------------------------------------- CanNodes

    CanNodes& CanNodes::instance() {
        static CanNodes inst;
        return inst;
    }

    CanNode* CanNodes::find(uint8_t id) {
        for (auto n : _nodes) {
            if (n->id() == id) {
                return n;
            }
        }
        return nullptr;
    }

    CanNode* CanNodes::node(uint8_t id) {
        auto existing = find(id);
        if (existing) {
            return existing;
        }
        // The supervisor task walks _nodes without a lock, which is safe only because every
        // node is registered during single-threaded startup.
        Assert(!_started, "CAN nodes cannot be added after the supervisor has started");
        Assert(id >= 1 && id <= (IdClockReplyLast - IdClockReplyBase),
               "CAN node id %d is out of range; must be 1..%d",
               int(id),
               int(IdClockReplyLast - IdClockReplyBase));
        auto created = new CanNode(id);
        _nodes.push_back(created);
        return created;
    }

    void CanNodes::init() {
        if (_started || config->_can == nullptr || _nodes.empty()) {
            return;
        }

        // Clock replies and status frames.  Both ranges are contiguous and node-indexed.
        config->_can->subscribe(this, IdClockReplyBase, 0x7F0);
        config->_can->subscribe(this, IdNodeStatusBase, 0x7C0);

        xTaskCreatePinnedToCore(supervisorTask, "can_nodes", 3072, this, 4, nullptr, SUPPORT_TASK_CORE);
        _started = true;
    }

    void CanNodes::onCanFrame(uint32_t id, uint8_t len, const uint8_t* data, int64_t rx_time_us) {
        if (id >= IdClockReplyBase && id <= IdClockReplyLast) {
            auto n = find(uint8_t(id - IdClockReplyBase));
            if (n) {
                n->onClockReply(len, data, rx_time_us);
            }
            return;
        }
        if (id >= IdNodeStatusBase && id <= IdNodeStatusBase + MaxNodeId) {
            auto n = find(uint8_t(id - IdNodeStatusBase));
            if (n) {
                n->onStatus(len, data, rx_time_us);
            }
        }
    }

    void CanNodes::supervisorTask(void* arg) {
        auto self = static_cast<CanNodes*>(arg);

        while (true) {
            int64_t now = esp_timer_get_time();

            // Every node answers the same broadcast, on its own identifier, so one frame
            // costs one slot on the bus and refreshes every clock fit.
            if (config->_can != nullptr) {
                config->_can->sendStamped(IdClockRequest, 0, nullptr, &self->_clockRequestSentUs);
            }

            for (auto n : self->_nodes) {
                n->tick(now);
            }

            vTaskDelay(pdMS_TO_TICKS(SyncPeriodMs));
        }
    }

    bool CanNodes::allOnline() const {
        for (auto n : _nodes) {
            if (!n->online()) {
                return false;
            }
        }
        return true;
    }

    std::string CanNodes::statusString() const {
        std::string s;
        for (auto n : _nodes) {
            if (!s.empty()) {
                s += " ";
            }
            s += "node";
            s += std::to_string(n->id());
            s += ":";
            s += n->healthName();
        }
        return s;
    }
}
