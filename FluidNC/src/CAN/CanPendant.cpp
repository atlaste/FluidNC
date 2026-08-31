// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "CanPendant.h"

#include "CanIds.h"
#include "CanModule.h"
#include "../GCode.h"
#include "../Logging.h"
#include "../Machine/Axes.h"
#include "../Machine/MachineConfig.h"
#include "../Planner.h"
#include "../Protocol.h"
#include "../System.h"

#include <cstdio>

namespace CAN {
    void CanPendant::group(Configuration::HandlerBase& handler) {
        handler.item("node", _nodeId);
        handler.item("module", _moduleLabel);
        handler.item("feed_rate_mm_per_min", _feedRate);
        handler.item("idle_reports", _idleReports);
        handler.item("scale0_mm", _scales[0]);
        handler.item("scale1_mm", _scales[1]);
        handler.item("scale2_mm", _scales[2]);
        handler.item("scale3_mm", _scales[3]);
    }

    void CanPendant::afterParse() {
        if (_moduleLabel.empty()) {
            Assert(_nodeId >= 1 && _nodeId <= int32_t(MaxNodeId), "pendant: node must be between 1 and %d", int(MaxNodeId));
        }
        Assert(config->_can != nullptr, "pendant: no CAN bus configured; add a top level 'can:' section");
        Assert(_feedRate > 0.0f, "pendant: feed_rate_mm_per_min must be positive");
    }

    void CanPendant::init() {
        if (config->_can == nullptr) {
            return;
        }
        if (!_moduleLabel.empty()) {
            int32_t n = CanModules::nodeForLabel(_moduleLabel);
            Assert(n >= 1, "pendant: unknown module '%s'", _moduleLabel.c_str());
            _nodeId = n;
        }
        _node = CanNodes::instance().node(uint8_t(_nodeId));
        config->_can->subscribe(this, IdPendantBase + uint32_t(_nodeId), 0x7FF);
        log_info("CAN pendant on node " << _nodeId);
    }

    void CanPendant::deinit() {
        if (config->_can != nullptr) {
            config->_can->unsubscribe(this);
        }
        _node = nullptr;
    }

    void CanPendant::onCanFrame(uint32_t id, uint8_t len, const uint8_t* data, int64_t rx_time_us) {
        if (len < 5) {
            return;
        }

        // Overwrite rather than queue.  If the protocol task has not caught up, the newest
        // wheel position is the only one worth acting on; replaying stale deltas is exactly
        // how a pendant ends up feeling laggy.
        _rxDelta      = int16_t(uint16_t(data[0]) | (uint16_t(data[1]) << 8));
        _rxAxis       = data[2];
        _rxScaleIndex = data[3];
        _rxButtons    = data[4];
        _haveNew      = true;
    }

    bool CanPendant::jogsInFlight() const {
        int32_t total = config->_planner_blocks;
        return (total - int32_t(plan_get_block_buffer_available())) >= MaxJogsInFlight;
    }

    void CanPendant::cancelJog() {
        if (state_is(State::Jog)) {
            protocol_send_event(&motionCancelEvent);
        }
        _jogging = false;
    }

    void CanPendant::poll() {
        if (!_haveNew) {
            // The wheel has gone quiet.  Let whatever is queued finish rather than cancelling
            // it, so a single detent produces a single clean move.
            if (_jogging && ++_quietPolls > _idleReports) {
                _jogging  = false;
                _lastSign = 0;
            }
            return;
        }

        Report report;
        report.delta      = _rxDelta;
        report.axis       = _rxAxis;
        report.scaleIndex = _rxScaleIndex;
        report.buttons    = _rxButtons;
        _haveNew          = false;
        _quietPolls       = 0;

        handleReport(report);
    }

    void CanPendant::handleReport(const Report& report) {
        if (report.delta == 0) {
            return;
        }

        if (_node != nullptr && !_node->online()) {
            // Acting on a delta from a node we cannot vouch for risks a move nobody asked
            // for, so ignore it; the extender layer raises the alarm.
            return;
        }

        if (axis_t(report.axis) >= Machine::Axes::_numberAxis) {
            log_warn("CAN pendant selected axis " << int(report.axis) << " which this machine does not have");
            return;
        }

        int scaleIndex = report.scaleIndex < NumScales ? report.scaleIndex : 0;
        float distance = float(report.delta) * _scales[scaleIndex];

        int sign = report.delta > 0 ? 1 : -1;
        if (_jogging && _lastSign != 0 && sign != _lastSign) {
            // The operator reversed.  Queued motion in the old direction is now wrong.
            cancelJog();
            return;
        }
        _lastSign = sign;

        // Only states that can accept a jog.
        if (!state_is(State::Idle) && !state_is(State::Jog)) {
            return;
        }

        if (jogsInFlight()) {
            return;
        }

        char line[64];
        snprintf(line,
                 sizeof(line),
                 "$J=G91 G21 %s%.4f F%.1f",
                 Machine::Axes::axisName(axis_t(report.axis)),
                 double(distance),
                 double(_feedRate));

        // gc_execute_line overwrites the global parser block; nothing else is using it here
        // because poll() runs on the protocol task between g-code lines.
        Error err = gc_execute_line(line);
        if (err != Error::Ok && err != Error::JogCancelled) {
            log_warn("CAN pendant jog rejected: " << int(err));
            return;
        }

        _jogging = true;
    }

    namespace {
        ConfigurableModuleFactory::InstanceBuilder<CanPendant> registration("can_pendant");
    }
}
