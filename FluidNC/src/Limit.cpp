// Copyright (c) 2012 - 2016 Sungeun K. Jeon for Gnea Research LLC
// Copyright (c) 2018 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Limit.h"

#include "Machine/MachineConfig.h"
#include "MotionControl.h"  // mc_critical
#include "System.h"         // sys.*
#include "Protocol.h"       // protocol_execute_realtime
#include "Platform.h"       // WEAK_LINK
#include "Machine/Axis.h"

#include <freertos/task.h>
#include <freertos/queue.h>
#include <atomic>  // fence
#include <cmath>   // roundf

// Debouncing is done by the input sampler; see Machine::Debounce and
// esp32/gpio.cpp.
void limits_init() {}

// Returns limit state as a bitmask. Each bit indicates one motor's limit, where triggered is 1
// and not triggered is 0. Invert mask is applied. Each motor gets a 16 bit lane of axis bits,
// so Z motor0 is bit 2 and Z motor1 is bit 18; see Machine::Axes::motor_bit().
MotorMask limits_get_state() {
    return Machine::Axes::posLimitMask | Machine::Axes::negLimitMask;
}

bool limits_startup_check() {  // return true if there is a hard limit error.
    MotorMask lim_pin_state = limits_get_state();
    if (lim_pin_state) {
        auto n_axis = Axes::_numberAxis;
        for (axis_t axis = X_AXIS; axis < n_axis; axis++) {
            for (motor_t motor = 0; motor < Machine::Axis::MAX_MOTORS_PER_AXIS; motor++) {
                if (Machine::Axes::motor_is_set(lim_pin_state, axis, motor)) {
                    log_warn("Active limit switch on " << Axes::axisName(axis) << " axis motor " << int(motor));
                }
            }
        }
    }
    return (config->_start->_checkLimits && (Axes::hardLimitMask() & lim_pin_state));
}

// Called only from Kinematics canHome() methods, hence from states allowing homing
bool ambiguousLimit() {
    if (Machine::Axes::posLimitMask & Machine::Axes::negLimitMask) {
        mc_critical(ExecAlarm::HomingAmbiguousSwitch);
        return true;
    }
    return false;
}

bool soft_limit = false;

// Performs a soft limit check. Called from mcline() only. Assumes the machine has been homed,
// the workspace volume is in all negative space, and the system is in normal operation.
// NOTE: Used by jogging to limit travel within soft-limit volume.
void limit_error(axis_t axis, float coordinate) {
    log_info("Soft limit on " << Machine::Axes::axisName(axis) << " target:" << coordinate);

    limit_error();
}

void limit_error() {
    soft_limit = true;
    // Force feed hold if cycle is active. All buffered blocks are guaranteed to be within
    // workspace volume so just come to a controlled stop so position is not lost. When complete
    // enter alarm mode.
    protocol_buffer_synchronize();
    if (state_is(State::Cycle)) {
        protocol_send_event(&feedHoldEvent);
        do {
            protocol_execute_realtime();
            if (sys.abort()) {
                return;
            }
        } while (!state_is(State::Idle));
    }

    mc_critical(ExecAlarm::SoftLimit);
}

// Snap mpos to the nearest step boundary so soft limits match the actual
// quantized home position.  Without this, the configured mpos (e.g. 88.867)
// can be slightly below the step-rounded home position (e.g. 88.867188),
// causing an immediate soft-limit alarm at the home position.
static float quantize_mpos(float mpos, float stepsPerMm, int extra_steps) {
    if (stepsPerMm > 0) {
        return (roundf(mpos * stepsPerMm) + extra_steps) / stepsPerMm;
    }
    return mpos;
}

float limitsMaxPosition(axis_t axis) {
    auto  axisConfig = Axes::_axis[axis];
    auto  homing     = axisConfig->_homing;
    float mpos       = homing ? quantize_mpos(homing->_mpos, axisConfig->_stepsPerMm, +1) : 0;
    auto  maxtravel  = axisConfig->_maxTravel;

    return (!homing || homing->_positiveDirection) ? mpos : mpos + maxtravel;
}

float limitsMinPosition(axis_t axis) {
    auto  axisConfig = Axes::_axis[axis];
    auto  homing     = axisConfig->_homing;
    float mpos       = homing ? quantize_mpos(homing->_mpos, axisConfig->_stepsPerMm, -1) : 0;
    auto  maxtravel  = axisConfig->_maxTravel;

    return (!homing || homing->_positiveDirection) ? mpos - maxtravel : mpos;
}
