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
        Assert(id >= 1 && id <= (IdClockReplyLast - IdClockReplyBase),
               "CAN node id %d is out of range; must be 1..%d",
               int(id),
               int(IdClockReplyLast - IdClockReplyBase));

        if (_nodesMutex) {
            xSemaphoreTake(_nodesMutex, portMAX_DELAY);
        }
        // Re-check under the lock: a concurrent path may have created it between find() and
        // here.
        CanNode* created = find(id);
        if (created == nullptr) {
            created = new CanNode(id);
            _nodes.push_back(created);
        }
        if (_nodesMutex) {
            xSemaphoreGive(_nodesMutex);
        }
        return created;
    }

    void CanNodes::remove(uint8_t id) {
        if (_nodesMutex) {
            xSemaphoreTake(_nodesMutex, portMAX_DELAY);
        }
        for (auto it = _nodes.begin(); it != _nodes.end(); ++it) {
            if ((*it)->id() == id) {
                _nodes.erase(it);
                break;
            }
        }
        if (_nodesMutex) {
            xSemaphoreGive(_nodesMutex);
        }
    }

    void CanNodes::init() {
        if (_nodesMutex == nullptr) {
            _nodesMutex = xSemaphoreCreateMutex();
        }
        if (_started || config->_can == nullptr) {
            return;
        }

        // Clock replies, status frames and admin replies.
        config->_can->subscribe(this, IdClockReplyBase, 0x7F0);
        config->_can->subscribe(this, IdNodeStatusBase, 0x7C0);
        config->_can->subscribe(this, IdAdminReply, 0x7FF);

        xTaskCreatePinnedToCore(supervisorTask, "can_nodes", 3072, this, 4, nullptr, SUPPORT_TASK_CORE);
        _started = true;
    }

    void CanNodes::onCanFrame(uint32_t id, uint8_t len, const uint8_t* data, int64_t rx_time_us) {
        if (id == IdAdminReply) {
            if (len >= 7) {
                uint64_t uuid = 0;
                for (int i = 0; i < 6; ++i) {
                    uuid |= uint64_t(data[1 + i]) << (8 * i);
                }
                _adminReplyUuid  = uuid;
                _adminReplyId    = data[7 < len ? 7 : 0];
                _adminReplyValid = true;
            }
            return;
        }
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

    bool CanNodes::adminExchange(AdminOp op, uint64_t uuid, uint8_t node_id, uint8_t& reply_id) {
        if (config->_can == nullptr) {
            return false;
        }

        uint8_t payload[8] = { 0 };
        payload[0]         = uint8_t(op);
        for (int i = 0; i < 6; ++i) {
            payload[1 + i] = uint8_t((uuid >> (8 * i)) & 0xFF);
        }
        payload[7] = node_id;

        _adminReplyValid = false;
        if (!config->_can->send(IdAdminCommand, 8, payload)) {
            return false;
        }

        // A node answers within a couple of report intervals if it is present at all.  Poll
        // rather than block on a queue, because this only runs from the console/refresh path.
        for (int waited = 0; waited < 60; ++waited) {
            if (_adminReplyValid && _adminReplyUuid == uuid) {
                reply_id = _adminReplyId;
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        return false;
    }

    bool CanNodes::probe(uint64_t uuid, uint8_t& current_id) {
        return adminExchange(AdminOp::Probe, uuid, 0, current_id);
    }

    bool CanNodes::assign(uint64_t uuid, uint8_t node_id) {
        uint8_t reply_id = 0;
        if (!adminExchange(AdminOp::Assign, uuid, node_id, reply_id)) {
            return false;
        }
        return reply_id == node_id;
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

            if (self->_nodesMutex) {
                xSemaphoreTake(self->_nodesMutex, portMAX_DELAY);
            }
            for (auto n : self->_nodes) {
                n->tick(now);
            }
            if (self->_nodesMutex) {
                xSemaphoreGive(self->_nodesMutex);
            }

            vTaskDelay(pdMS_TO_TICKS(SyncPeriodMs));
        }
    }

    bool CanNodes::allOnline() const {
        bool result = true;
        if (_nodesMutex) {
            xSemaphoreTake(_nodesMutex, portMAX_DELAY);
        }
        for (auto n : _nodes) {
            if (!n->online()) {
                result = false;
                break;
            }
        }
        if (_nodesMutex) {
            xSemaphoreGive(_nodesMutex);
        }
        return result;
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
