// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "CanScheduler.h"

#include "CanBus.h"
#include "CanNode.h"
#include "../Logging.h"
#include "../Machine/MachineConfig.h"
#include "../Protocol.h"
#include "../Stepping.h"

#include <esp_timer.h>

namespace CAN {
    CanScheduler::PendingEvent CanScheduler::_pending[CanScheduler::PendingCapacity];
    volatile int               CanScheduler::_pendingHead = 0;
    volatile int               CanScheduler::_pendingTail = 0;

    CanScheduler::AxisBinding CanScheduler::_axes[MAX_N_AXIS];
    uint32_t                  CanScheduler::_boundAxisMask = 0;
    CanScheduler::PwmBinding  CanScheduler::_pwm;

    int64_t CanScheduler::_prepTicks      = 0;
    int     CanScheduler::_prepBlockIndex = -1;

    volatile uint32_t CanScheduler::_anchorSeq       = 0;
    volatile int64_t  CanScheduler::_anchorUs        = 0;
    volatile int64_t  CanScheduler::_anchorTicks     = 0;
    volatile int64_t  CanScheduler::_isrCumTicks     = 0;
    volatile uint32_t CanScheduler::_isrPendingTicks = 0;
    volatile bool     CanScheduler::_anchorValid     = false;

    bool CanScheduler::_underrun = false;

    // The stepper timer runs at Machine::Stepping::fStepperTimer, so one microsecond is
    // exactly fStepperTimer / 1e6 ticks.
    static constexpr int64_t TicksPerUs = Machine::Stepping::fStepperTimer / 1000000;

    void CanScheduler::bindAxis(axis_t axis, uint8_t node_id, uint8_t motor_index) {
        Assert(axis < MAX_N_AXIS, "CAN axis index out of range");
        _axes[axis].bound      = true;
        _axes[axis].nodeId     = node_id;
        _axes[axis].motorIndex = motor_index;
        _boundAxisMask |= (1u << axis);

        // Make sure the node exists so the supervisor starts synchronising its clock.
        CanNodes::instance().node(node_id);
    }

    void CanScheduler::bindPwm(uint8_t node_id, uint8_t channel, bool off_on_idle) {
        _pwm.bound     = true;
        _pwm.offOnIdle = off_on_idle;
        _pwm.nodeId    = node_id;
        _pwm.channel   = channel;
        _pwm.dutyKnown = false;

        CanNodes::instance().node(node_id);
    }

    void CanScheduler::unbindPwm() {
        _pwm.bound     = false;
        _pwm.dutyKnown = false;
    }

    void CanScheduler::setPwmImmediate(uint32_t duty) {
        if (!_pwm.bound) {
            return;
        }
        _pwm.lastDuty  = duty;
        _pwm.dutyKnown = true;

        auto node = CanNodes::instance().find(_pwm.nodeId);
        if (node) {
            // Scheduled at "now", which the node treats as due immediately.
            node->schedulePwm(_pwm.channel, uint16_t(duty), esp_timer_get_time());
        }
    }

    void CanScheduler::onPositionSet(axis_t axis, int32_t steps) {
        if (axis >= MAX_N_AXIS) {
            return;
        }
        auto& binding = _axes[axis];
        if (!binding.bound) {
            return;
        }

        binding.cumSteps = steps;

        auto node = CanNodes::instance().find(binding.nodeId);
        if (node) {
            node->sendReset(binding.motorIndex, steps);
        }
    }

    bool CanScheduler::nodesReady() {
        auto ready = [](uint8_t node_id) {
            auto node = CanNodes::instance().find(node_id);
            return node != nullptr && node->online() && node->clock().synced();
        };

        for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; ++axis) {
            const auto& binding = _axes[axis];
            if (binding.bound && !ready(binding.nodeId)) {
                return false;
            }
        }
        return !_pwm.bound || ready(_pwm.nodeId);
    }

    bool CanScheduler::readAnchor(int64_t& us, int64_t& ticks) {
        // Seqlock: the ISR bumps _anchorSeq to odd while writing.  Retry until we read a
        // stable, even-numbered generation.
        for (int attempt = 0; attempt < 8; ++attempt) {
            uint32_t before = _anchorSeq;
            if (before & 1) {
                continue;
            }
            us    = _anchorUs;
            ticks = _anchorTicks;
            if (_anchorSeq == before) {
                return _anchorValid;
            }
        }
        return false;
    }

    int64_t CanScheduler::ticksToMasterUs(int64_t ticks, int64_t anchor_us, int64_t anchor_ticks) {
        return anchor_us + (ticks - anchor_ticks) / TicksPerUs;
    }

    void IRAM_ATTR CanScheduler::onSegmentStarted(uint32_t segment_ticks) {
        if (!active()) {
            return;
        }

        // The segment that was running has completed, so the timeline has advanced by its
        // duration.  The new segment starts now.
        int64_t cum = _isrCumTicks + int64_t(_isrPendingTicks);
        _isrCumTicks     = cum;
        _isrPendingTicks = segment_ticks;

        _anchorSeq   = _anchorSeq + 1;
        _anchorUs    = esp_timer_get_time();
        _anchorTicks = cum;
        _anchorValid = true;
        _anchorSeq   = _anchorSeq + 1;
    }

    int64_t CanScheduler::onWakeUp() {
        if (!active()) {
            return 0;
        }

        if (!nodesReady()) {
            log_error("Refusing to move: a CAN axis node is not synchronised");
            send_alarm(ExecAlarm::CanNodeLost);
            return 0;
        }

        // A node that rebooted has forgotten its step counter.  Hand it the master's idea of
        // where the axis is before anything is scheduled against it.
        for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; ++axis) {
            auto& binding = _axes[axis];
            if (!binding.bound) {
                continue;
            }
            auto node = CanNodes::instance().find(binding.nodeId);
            if (node && node->consumeResetFlag()) {
                binding.cumSteps = Machine::Stepping::getSteps(axis);
                node->sendReset(binding.motorIndex, int32_t(binding.cumSteps));
                log_warn("CAN node " << int(binding.nodeId) << " rebooted; axis " << Machine::Axes::axisName(axis)
                                     << " position re-sent");
            }
        }

        // Restart the timeline.  Step positions stay where they are: they are absolute and
        // the nodes are holding them.  Only the time base is new.
        _prepTicks       = 0;
        _isrCumTicks     = 0;
        _isrPendingTicks = 0;

        int64_t start = esp_timer_get_time() + DefaultLookaheadUs;

        _anchorSeq   = _anchorSeq + 1;
        _anchorUs    = start;
        _anchorTicks = 0;
        _anchorValid = true;
        _anchorSeq   = _anchorSeq + 1;

        // Anything prep already queued can now be given a real time.
        flush();

        return start;
    }

    void CanScheduler::waitForStart(int64_t start_us) {
        if (start_us == 0) {
            return;
        }

        // Give the scheduler back to other tasks for the bulk of the wait, then close the
        // last couple of milliseconds by spinning so the timer starts on time rather than on
        // the next RTOS tick boundary.
        int64_t remaining = start_us - esp_timer_get_time();
        if (remaining > 3000) {
            vTaskDelay(pdMS_TO_TICKS((remaining - 2000) / 1000));
        }

        while (esp_timer_get_time() < start_us) {
            flush();
        }
    }

    void CanScheduler::reportUnderrun(const char* why) {
        if (_underrun) {
            return;  // Already alarming; do not bury the first message under repeats.
        }
        _underrun = true;
        log_error(why);

        // An event that arrives late means an axis is somewhere other than where the master
        // believes.  There is no way to recover that within the running job.
        send_alarm(ExecAlarm::CanNodeLost);
    }

    void CanScheduler::push(const PendingEvent& event) {
        int next = (_pendingHead + 1) % PendingCapacity;
        if (next == _pendingTail) {
            // The ring is full, which means flush() is not keeping up.  Dropping a move_to
            // would silently corrupt the axis position, so treat it as an underrun.
            reportUnderrun("CAN scheduler queue overflow");
            return;
        }
        _pending[_pendingHead] = event;
        _pendingHead           = next;
    }

    void CanScheduler::onSegmentPrepared(const uint32_t* block_steps,
                                         uint32_t        step_event_count,
                                         uint32_t        direction_bits,
                                         uint8_t         block_index,
                                         uint16_t        n_step,
                                         uint8_t         amass_level,
                                         uint16_t        isr_period,
                                         uint32_t        spindle_dev_speed) {
        if (!active() || step_event_count == 0 || n_step == 0) {
            return;
        }

        bool new_block = _prepBlockIndex != int(block_index);
        _prepBlockIndex = int(block_index);

        // The segment covers [segStart, _prepTicks) on the timeline.  Step targets are due at
        // the end of the segment; a speed change belongs at the start of it.
        int64_t segStart = _prepTicks;
        _prepTicks += int64_t(n_step) * int64_t(isr_period);

        if (_pwm.bound && (!_pwm.dutyKnown || spindle_dev_speed != _pwm.lastDuty)) {
            _pwm.lastDuty  = spindle_dev_speed;
            _pwm.dutyKnown = true;

            PendingEvent event;
            event.ticks      = segStart;
            event.nodeId     = _pwm.nodeId;
            event.motorIndex = _pwm.channel;
            event.op         = Op::SetPwm;
            event.value      = int32_t(spindle_dev_speed);
            push(event);
        }

        for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; ++axis) {
            auto& binding = _axes[axis];
            if (!binding.bound) {
                continue;
            }

            if (new_block) {
                // The ISR seeds every Bresenham counter with half the event count when it
                // moves to a new block; mirror that exactly or the shadow drifts.
                binding.bresenham = step_event_count >> 1;
            }

            uint32_t steps_per_event = block_steps[axis] >> amass_level;
            if (steps_per_event == 0) {
                continue;
            }

            // Bulk form of the ISR's per-event Bresenham.  Each event can emit at most one
            // step because steps_per_event never exceeds step_event_count, so accumulating
            // n_step events at once gives an identical result.
            uint64_t accumulated = uint64_t(binding.bresenham) + uint64_t(steps_per_event) * uint64_t(n_step);
            uint64_t emitted     = (accumulated - 1) / step_event_count;
            binding.bresenham    = uint32_t(accumulated - emitted * step_event_count);

            if (emitted == 0) {
                continue;
            }

            binding.cumSteps += bitnum_is_true(direction_bits, axis) ? -int64_t(emitted) : int64_t(emitted);

            PendingEvent event;
            event.ticks      = _prepTicks;
            event.nodeId     = binding.nodeId;
            event.motorIndex = binding.motorIndex;
            event.op         = Op::MoveTo;
            event.value      = int32_t(binding.cumSteps);
            push(event);
        }
    }

    void CanScheduler::flush() {
        if (_pendingHead == _pendingTail) {
            return;
        }

        int64_t anchor_us, anchor_ticks;
        if (!readAnchor(anchor_us, anchor_ticks)) {
            return;  // No timeline yet; the events stay queued until wake_up establishes one.
        }

        int64_t now = esp_timer_get_time();

        while (_pendingTail != _pendingHead) {
            const PendingEvent& event = _pending[_pendingTail];

            int64_t when = ticksToMasterUs(event.ticks, anchor_us, anchor_ticks);

            // Only hand over events that are within the lookahead window.  Sending earlier
            // than that would fill the node's queue with events whose timing may still be
            // revised by a feed hold or an override.
            if (when - now > DefaultLookaheadUs) {
                break;
            }

            auto node = CanNodes::instance().find(event.nodeId);
            if (node == nullptr) {
                _pendingTail = (_pendingTail + 1) % PendingCapacity;
                continue;
            }

            bool sent = false;
            switch (event.op) {
                case Op::MoveTo:
                    sent = node->scheduleMoveTo(event.motorIndex, event.value, when);
                    break;
                case Op::SetPwm:
                    sent = node->schedulePwm(event.motorIndex, uint16_t(event.value), when);
                    break;
                case Op::SetDigital:
                    sent = node->scheduleDigital(event.motorIndex, event.value != 0, when);
                    break;
                default:
                    sent = true;
                    break;
            }

            if (!sent) {
                // The bus or the node is not ready.  Leave the event queued and try again on
                // the next pass, unless it is already too late to be useful.
                if (when <= now) {
                    reportUnderrun("A CAN node missed a scheduled event");
                    _pendingTail = (_pendingTail + 1) % PendingCapacity;
                    continue;
                }
                break;
            }

            _pendingTail = (_pendingTail + 1) % PendingCapacity;
        }
    }

    void CanScheduler::onGoIdle() {
        if (!active()) {
            return;
        }

        _anchorValid = false;
        _pendingHead = _pendingTail;

        // A laser must not keep burning through a feed hold or the end of a job, and any
        // scheduled duty change was just discarded along with the rest of the queue.
        if (_pwm.bound && _pwm.offOnIdle) {
            setPwmImmediate(0);
        }

        // Nodes hold their last commanded position.  They are not told to abort here: a
        // normal end of motion simply runs out of scheduled events.
    }

    void CanScheduler::onReset() {
        _pendingHead = _pendingTail = 0;
        _prepTicks                  = 0;
        _prepBlockIndex             = -1;
        _isrCumTicks                = 0;
        _isrPendingTicks            = 0;
        _anchorValid                = false;
        _underrun                   = false;
        _pwm.dutyKnown              = false;

        if (config->_can != nullptr) {
            config->_can->abortAll();
        }

        for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; ++axis) {
            _axes[axis].bresenham = 0;
        }
    }
}
