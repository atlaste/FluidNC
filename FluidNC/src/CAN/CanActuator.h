// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../Config.h"
#include "../Module.h"
#include "CanBus.h"
#include "CanNode.h"

#include <cstdint>
#include <string>
#include <vector>

namespace CAN {
    /*
        Auxiliary actuators on CAN nodes: the small NEMA-17s and solenoids that a tool
        changer or a dust shoe needs to move, where the motion is a self-contained gesture
        rather than part of the tool path.

        These are deliberately not coordinated.  The master issues a command, the node runs
        its own profile, and the node reports back when it is done.  That is the right model
        for "push the tool out of the pocket": the master has no useful opinion about the
        velocity profile, and making it part of the planner would only add constraints.

        A command is either fire-and-forget or blocking.  Blocking is the normal case for a
        tool change, since carrying on before the pocket has opened is how tools get broken.
    */
    class CanActuator : public ConfigurableModule {
    public:
        // What the master asks a node to do.  The node owns the motion profile.
        enum class Command : uint8_t {
            MoveAbsolute = 0,  // int32 steps
            MoveRelative = 1,  // int32 steps
            Home         = 2,
            SetOutput    = 3,  // uint8 value
            Stop         = 4,
        };

        CanActuator(const char* name);
        ~CanActuator();

        void init() override;

        void group(Configuration::HandlerBase& handler) override;
        void afterParse();

        // Runs a command and, when timeout_ms is non-zero, waits for the node to report
        // completion.  Returns false on timeout or if the node is not usable.
        bool run(Command command, int32_t argument, uint32_t timeout_ms);

        // Convenience wrappers used by tool changers and by the M-code bindings.
        bool moveAbsolute(int32_t steps, uint32_t timeout_ms) { return run(Command::MoveAbsolute, steps, timeout_ms); }
        bool moveRelative(int32_t steps, uint32_t timeout_ms) { return run(Command::MoveRelative, steps, timeout_ms); }
        bool home(uint32_t timeout_ms) { return run(Command::Home, 0, timeout_ms); }
        bool stop() { return run(Command::Stop, 0, 0); }

        const std::string& label() const { return _label; }
        int32_t            defaultTimeoutMs() const { return _defaultTimeoutMs; }

        // Looks up a configured actuator by its label, for use from tool changer code and
        // from the actuator M-codes.
        static CanActuator* byLabel(const std::string& label);

        // Every configured actuator, in configuration order.  RTTI is off in this build, so
        // instances add themselves to this registry rather than being filtered out of the
        // module list.
        static const std::vector<CanActuator*>& all();

    private:
        // Node -> master completion report.
        class DoneListener : public CanListener {
            CanActuator* _owner;

        public:
            explicit DoneListener(CanActuator* owner) : _owner(owner) {}
            void onCanFrame(uint32_t id, uint8_t len, const uint8_t* data, int64_t rx_time_us) override;
        };

        std::string _label        = "";
        int32_t     _nodeId       = 0;
        int32_t     _actuatorIndex = 0;

        // Default wait for a blocking command, in milliseconds.
        int32_t _defaultTimeoutMs = 10000;

        CanNode*      _node = nullptr;
        DoneListener  _listener { this };
        QueueHandle_t _doneQueue = nullptr;

        void onDone(uint8_t status);
    };
}
