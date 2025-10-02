// Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC
// Copyright (c) 2018 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
  System.cpp - Header for system level commands and real-time processes
*/

#include "System.h"
#include "Report.h"                 // report_ovr_counter
#include "Config.h"                 // MAX_N_AXIS
#include "Machine/MachineConfig.h"  // config
#include "Stepping.h"               // config

#include <cstring>  // memset
#include <cmath>    // roundf

#include "driver/pulse_cnt.h"  // pctr

// Declare system global variable structure
system_t sys;
int32_t  probe_steps[MAX_N_AXIS];  // Last probe position in steps.

void system_reset() {
    // Reset system variables.
    State prior_state = sys.state();
    bool  prior_abort = sys.abort();
    sys.reset();  // Clear system struct variable.
    set_state(prior_state);
    sys.set_abort(prior_abort);
    sys.set_f_override(FeedOverride::Default);                 // Set to 100%
    sys.set_r_override(RapidOverride::Default);                // Set to 100%
    sys.set_spindle_speed_ovr(SpindleSpeedOverride::Default);  // Set to 100%
    memset(probe_steps, 0, sizeof(probe_steps));               // Clear probe position.
    report_ovr_counter = 0;
    report_wco_counter = 0;
}

float steps_to_mpos(int32_t steps, size_t axis) {
    return float(steps / Axes::_axis[axis]->_stepsPerMm);
}
int32_t mpos_to_steps(float mpos, size_t axis) {
    return lroundf(mpos * Axes::_axis[axis]->_stepsPerMm);
}

void motor_steps_to_mpos(float* position, int32_t* steps) {
    float motor_mpos[MAX_N_AXIS];
    auto  a      = config->_axes;
    auto  n_axis = a ? a->_numberAxis : 0;
    for (size_t idx = 0; idx < n_axis; idx++) {
        motor_mpos[idx] = steps_to_mpos(steps[idx], idx);
    }
    config->_kinematics->motors_to_cartesian(position, motor_mpos, n_axis);
}

void set_motor_steps(size_t axis, int32_t steps) {
    Stepping::setSteps(axis, steps);
}

void set_motor_steps_from_mpos(float* mpos) {
    auto  n_axis = Axes::_numberAxis;
    float motor_steps[n_axis];
    config->_kinematics->transform_cartesian_to_motors(motor_steps, mpos);
    for (size_t axis = 0; axis < n_axis; axis++) {
        set_motor_steps(axis, mpos_to_steps(motor_steps[axis], axis));
    }
}

int32_t get_axis_motor_steps(size_t axis) {
    return Stepping::getSteps(axis);
}

void get_motor_steps(int32_t* motor_steps) {
    auto axes   = config->_axes;
    auto n_axis = axes->_numberAxis;
    for (size_t axis = 0; axis < n_axis; axis++) {
        motor_steps[axis] = Stepping::getSteps(axis);
    }
}
int32_t* get_motor_steps() {
    static int32_t motor_steps[MAX_N_AXIS];

    get_motor_steps(motor_steps);
    return motor_steps;
}

float* get_mpos() {
    static float position[MAX_N_AXIS];

    motor_steps_to_mpos(position, get_motor_steps());
    return position;
};

float* get_wco() {
    static float wco[MAX_N_AXIS];
    auto         n_axis = Axes::_numberAxis;
    for (int idx = 0; idx < n_axis; idx++) {
        // Apply work coordinate offsets and tool length offset to current position.
        wco[idx] = gc_state.coord_system[idx] + gc_state.coord_offset[idx];
        if (idx == TOOL_LENGTH_OFFSET_AXIS) {
            wco[idx] += gc_state.tool_length_offset;
        }
    }
    return wco;
}

const std::map<State, const char*> StateName = {
    { State::Idle, "Idle" },
    { State::Alarm, "Alarm" },
    { State::CheckMode, "CheckMode" },
    { State::Homing, "Homing" },
    { State::Cycle, "Cycle" },
    { State::Hold, "Hold" },
    { State::Jog, "Jog" },
    { State::SafetyDoor, "SafetyDoor" },
    { State::Sleep, "Sleep" },
    { State::ConfigAlarm, "ConfigAlarm" },
    { State::Critical, "Critical" },
};

void set_state(State s) {
    sys.set_state(s);
}
bool state_is(State s) {
    return sys.state() == s;
}

bool inMotionState() {
    return state_is(State::Cycle) || state_is(State::Homing) || state_is(State::Jog) ||
           (state_is(State::Hold) && !sys.suspend().bit.holdComplete);
}

// TODO FIXME: Put the following in some class in Machine. And fix the implementation; this is the general idea, but not good enough.
pcnt_unit_handle_t pcnt_unit = NULL;
int                watch_point = 32767;

// Interrupt handler for encoder pulses
bool IRAM_ATTR encoder_pulse_isr(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx) {
    // Call stepper pulse function if in sync mode
    if (Stepper::spindle_sync_active) {
        Stepper::pulse_func();
    }
    return true;
}

// Configure PCNT for the encoder
void setupEncoderInterrupt() {
    // Configure PCNT unit for quadrature mode
    log_info("Configuring encoder");

    pcnt_unit_config_t unit_config = { .low_limit = -32767, .high_limit = 32767, .intr_priority = 12, .flags { .accum_count = 1 } };
    pcnt_new_unit(&unit_config, &pcnt_unit);

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 100,  // TODO: Calculate this based on max spindle speed and ppr.
    };
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

    pcnt_chan_config_t chan_a_config = {};
    chan_a_config.edge_gpio_num      = 1;
    chan_a_config.level_gpio_num     = 2;
    pcnt_chan_config_t chan_b_config = {};
    chan_b_config.edge_gpio_num      = chan_a_config.level_gpio_num;
    chan_b_config.level_gpio_num     = chan_a_config.edge_gpio_num;

    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_channel_handle_t pcnt_chan_b = NULL;

    pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b);

    pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    watch_point = 32767;
    pcnt_unit_add_watch_point(pcnt_unit, watch_point);
    pcnt_unit_add_watch_point(pcnt_unit, -watch_point);

    pcnt_event_callbacks_t cbs = {};
    cbs.on_reach               = encoder_pulse_isr;
    pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, nullptr);

    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_start(pcnt_unit);
}
void IRAM_ATTR enableSpindleSync(bool enabled) {
    // TODO FIXME: Seems wrong.
    if (enabled) {
        pcnt_unit_enable(pcnt_unit);
    } else {
        pcnt_unit_disable(pcnt_unit);
    }
}

// This function sets the PCNT threshold when a sync segment is loaded
void IRAM_ATTR configureEncoderThreshold(uint16_t pulse_count) {
    pcnt_unit_remove_watch_point(pcnt_unit, watch_point);
    pcnt_unit_remove_watch_point(pcnt_unit, -watch_point);
    pcnt_unit_add_watch_point(pcnt_unit, pulse_count);
    pcnt_unit_add_watch_point(pcnt_unit, -pulse_count);
    watch_point = pulse_count;
}

// set spindle sync mode
void setSpindleSyncMode(bool enable) {
    if (enable && !Stepper::spindle_sync_active) {
        // Disable timer interrupts
        config->_stepping->stopTimer();

        // Configure initial PCNT behavior
        // We'll set the specific threshold when a segment is loaded

        Stepper::spindle_sync_active = true;
        pcnt_unit_enable(pcnt_unit);

    } else if (!enable && Stepper::spindle_sync_active) {
        // Disable encoder interrupts
        pcnt_unit_disable(pcnt_unit);

        // Re-enable timer interrupts
        config->_stepping->startTimer();

        Stepper::spindle_sync_active = false;
    }
}
