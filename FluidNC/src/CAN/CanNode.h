// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "CanBus.h"
#include "CanClockSync.h"
#include "CanIds.h"

#include <cstdint>
#include <vector>

namespace CAN {
    /*
        One remote node.

        A node owns its clock fit and its health state.  Health is deliberately conservative:
        anything that means "I can no longer vouch for what this node is doing" -- no
        heartbeat, a clock fit that will not converge, an unexpected reset -- puts the node in
        an unhealthy state, and consumers that carry safety inputs treat unhealthy as
        asserted rather than as unknown.
    */
    class CanNode {
    public:
        enum class Health {
            Offline,   // never heard from, or heartbeat lost
            Syncing,   // heard from, clock fit not yet converged
            Online,    // fully usable
            Faulted,   // node reported an internal fault
        };

        // A node must report in at least this often.  Nodes send status frames at 10 Hz, so
        // this tolerates two consecutive losses.
        static constexpr int64_t HeartbeatTimeoutUs = 300000;

        explicit CanNode(uint8_t id) : _id(id), _clock(id) {}

        uint8_t id() const { return _id; }

        Health health() const { return _health; }
        bool   online() const { return _health == Health::Online; }

        CanClockSync&       clock() { return _clock; }
        const CanClockSync& clock() const { return _clock; }

        // Bumped by the node every time it starts up.  A change means the node rebooted and
        // has lost everything we told it.
        uint16_t bootId() const { return _bootId; }
        bool     consumeResetFlag();

        uint32_t faultFlags() const { return _faultFlags; }

        // Frame handling, called from the CAN RX task.
        void onStatus(uint8_t len, const uint8_t* data, int64_t rx_time_us);
        void onClockReply(uint8_t len, const uint8_t* data, int64_t rx_time_us);

        // Re-evaluates health against the clock and the heartbeat deadline.  Called from the
        // node supervisor task.
        void tick(int64_t now_us);

        // Scheduled event encoders.  All of them convert a master timestamp into this node's
        // clock domain, and all of them fail if the node is not synced.
        bool scheduleMoveTo(uint8_t motor_index, int32_t step_target, int64_t at_master_us);
        bool schedulePwm(uint8_t channel, uint16_t duty, int64_t at_master_us);
        bool scheduleDigital(uint8_t pin, bool value, int64_t at_master_us);

        // Immediate, unscheduled commands.
        bool sendReset(uint8_t motor_index, int32_t step_position);
        bool sendEnable(uint8_t motor_index, bool enable);

        const char* healthName() const;

    private:
        uint8_t      _id;
        CanClockSync _clock;
        Health       _health = Health::Offline;

        int64_t  _lastStatusUs = 0;
        uint16_t _bootId       = 0;
        bool     _bootIdKnown  = false;
        bool     _resetFlag    = false;
        uint32_t _faultFlags   = 0;

        bool sendScheduled(const uint8_t* payload, uint8_t len);
    };

    /*
        Registry and supervisor for all nodes.

        Owns the periodic clock request broadcast and the health sweep.  Nodes are created
        on demand by whichever consumer references them, so a node id that appears in an
        extender, a pendant and an axis binding is one CanNode with one clock fit.
    */
    class CanNodes : public CanListener {
    public:
        // How often the master asks the nodes for their clocks.  Fast enough to converge in
        // well under a second, slow enough to be negligible bus load.
        static constexpr uint32_t SyncPeriodMs = 30;

        static CanNodes& instance();

        // The instant the most recent clock request left the CAN peripheral.  Shared by all
        // nodes because a single broadcast solicits every reply.
        int64_t clockRequestSentUs() const { return _clockRequestSentUs; }

        // Creates the node if it does not exist yet.
        CanNode* node(uint8_t id);

        // Returns nullptr rather than creating.
        CanNode* find(uint8_t id);

        void init();

        void onCanFrame(uint32_t id, uint8_t len, const uint8_t* data, int64_t rx_time_us) override;

        // True when every registered node is online.  Used to gate motion at startup.
        bool allOnline() const;

        std::string statusString() const;

    private:
        CanNodes() = default;

        static void supervisorTask(void* arg);

        std::vector<CanNode*> _nodes;
        bool                  _started            = false;
        volatile int64_t      _clockRequestSentUs = 0;
    };
}
