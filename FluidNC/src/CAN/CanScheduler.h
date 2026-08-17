// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../Config.h"
#include "CanIds.h"

#include <cstdint>

namespace CAN {
    class CanNode;

    /*
        Timestamped motion scheduling for CAN nodes.

        FluidNC executes motion as a stream of segments.  A timer-mode segment runs for
        exactly n_step * isr_period ticks of the 20 MHz stepper timer, so the segment stream
        is also a precise timeline.  This class tracks two positions on that timeline:

          - the prepare cursor, advanced by prep_buffer() as segments are committed, which
            runs roughly 40 ms ahead of execution;
          - the execution anchor, refreshed by the stepper ISR each time a segment is loaded,
            which ties a known point on the timeline to a real master timestamp.

        A segment boundary at cumulative tick T therefore happens at master time
        anchor_us + (T - anchor_ticks) / 20.  Re-reading the anchor from the ISR on every
        segment keeps that extrapolation short, so scheduling error stays at the level of ISR
        entry jitter instead of accumulating over the length of a job.

        Because the nodes need advance notice, the local step timer start is delayed by the
        lookahead window at the beginning of each motion burst.  That is the one place where
        CAN motion costs anything: a few tens of milliseconds before the first move.
    */
    class CanScheduler {
    public:
        // How far ahead of execution events are handed to the nodes.  Must be comfortably
        // less than the segment buffer depth (~40 ms) and comfortably more than the worst
        // case bus latency.
        static constexpr int64_t DefaultLookaheadUs = 20000;

        struct AxisBinding {
            bool     bound       = false;
            uint8_t  nodeId      = 0;
            uint8_t  motorIndex  = 0;
            int64_t  cumSteps    = 0;  // absolute commanded step position
            uint32_t bresenham   = 0;  // shadow of the ISR Bresenham accumulator
        };

        static void bindAxis(axis_t axis, uint8_t node_id, uint8_t motor_index);
        static bool hasAxes() { return _boundAxisMask != 0; }

        // Binds the motion-synchronised PWM output, used by a laser or any other tool whose
        // power has to track feed rate.  Only one can be active, because only one spindle is
        // active at a time.  off_on_idle asks for a zero the moment motion stops, which is
        // what a laser needs and what a spindle would find merely annoying.
        static void bindPwm(uint8_t node_id, uint8_t channel, bool off_on_idle);
        static void unbindPwm();

        // Sets the duty outside of motion, e.g. from an M3 that is not followed by a move.
        static void setPwmImmediate(uint32_t duty);

        // True when any consumer needs the timeline maintained.
        static bool active() { return _boundAxisMask != 0 || _pwm.bound; }

        // Called whenever the master sets an axis position outside of motion: after homing,
        // after G92-style resets, and when the saved position is restored at boot.  The node
        // counts in the same absolute step space as the master, so it has to be told.
        static void onPositionSet(axis_t axis, int32_t steps);

        // True when every node that carries an axis is online and clock-synced.  Motion is
        // refused otherwise, because a CAN axis that silently does not move is worse than a
        // job that will not start.
        static bool nodesReady();

        // Called by Stepper::wake_up() before the step timer starts.  Establishes the
        // execution anchor and returns the absolute master time at which the step timer
        // should start, or 0 when no CAN axis is bound and the caller need not wait.
        static int64_t onWakeUp();

        // Blocks until the time returned by onWakeUp(), pushing scheduled events to the
        // nodes while it waits.  The last part of the wait is a busy loop, because the
        // master's own steppers have to start in phase with what the nodes were promised.
        static void waitForStart(int64_t start_us);

        // Called by Stepper::prep_buffer() once a timer-mode segment has been committed.
        // The block arguments describe the stepper block the segment belongs to; a change of
        // block_index reseeds the Bresenham shadows the same way the ISR does.
        static void onSegmentPrepared(const uint32_t* block_steps,
                                      uint32_t        step_event_count,
                                      uint32_t        direction_bits,
                                      uint8_t         block_index,
                                      uint16_t        n_step,
                                      uint8_t         amass_level,
                                      uint16_t        isr_period,
                                      uint32_t        spindle_dev_speed);

        // Called by the stepper ISR each time a segment begins executing.  segment_ticks is
        // n_step * isr_period for the segment being loaded.
        static void IRAM_ATTR onSegmentStarted(uint32_t segment_ticks);

        // Pushes everything that is due out onto the bus.  Called at the end of prep_buffer
        // and from onWakeUp(); safe to call at any time.
        static void flush();

        // Motion has stopped.  Nodes are told to abort, and the timeline is invalidated.
        static void onGoIdle();

        // Full reset, e.g. after mc_reset().
        static void onReset();

        // True when a scheduled event could not be delivered far enough in advance.  The
        // motion is no longer trustworthy and the caller should alarm.
        static bool underrun() { return _underrun; }
        static void clearUnderrun() { _underrun = false; }

        static const AxisBinding& binding(axis_t axis) { return _axes[axis]; }

    private:
        // A pending event, tagged with its position on the timeline rather than an absolute
        // time, because the anchor may not exist yet when prep produces it.
        struct PendingEvent {
            int64_t ticks;   // cumulative timeline position at which it takes effect
            uint8_t nodeId;
            uint8_t motorIndex;
            Op      op;
            int32_t value;  // step target for MoveTo, duty for SetPwm
        };

        static constexpr int PendingCapacity = 128;

        static PendingEvent _pending[PendingCapacity];
        static volatile int _pendingHead;
        static volatile int _pendingTail;

        struct PwmBinding {
            bool     bound     = false;
            bool     offOnIdle = false;
            uint8_t  nodeId    = 0;
            uint8_t  channel   = 0;
            uint32_t lastDuty  = 0;
            bool     dutyKnown = false;
        };

        static AxisBinding _axes[MAX_N_AXIS];
        static uint32_t    _boundAxisMask;
        static PwmBinding  _pwm;

        // Prepare cursor: cumulative stepper-timer ticks of all committed segments.
        static int64_t _prepTicks;

        // Tracks block changes so the Bresenham shadows are reseeded in step with the ISR.
        static int _prepBlockIndex;

        // Execution anchor, written by the ISR and read by prep.  Guarded by a sequence
        // counter because a 64 bit value cannot be read atomically on this target.
        static volatile uint32_t _anchorSeq;
        static volatile int64_t  _anchorUs;
        static volatile int64_t  _anchorTicks;
        static volatile int64_t  _isrCumTicks;
        static volatile uint32_t _isrPendingTicks;
        static volatile bool     _anchorValid;

        static bool _underrun;

        static void    reportUnderrun(const char* why);
        static bool    readAnchor(int64_t& us, int64_t& ticks);
        static int64_t ticksToMasterUs(int64_t ticks, int64_t anchor_us, int64_t anchor_ticks);
        static void    push(const PendingEvent& event);
    };
}
