// Stubs for motor/spindle unit tests. Provides limitEvent and Stepping/Homing/Limit stubs so motor code links.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Protocol.h"
#include "Limit.h"
#include "Report.h"
#include "Stepping.h"
#include "Stepper.h"
#include "Machine/Homing.h"
#include "GCode.h"
#include "Macro.h"
#include "NutsBolts.h"
#include "Types.h"

static void protocol_do_limit(void*) {}
const ArgEvent limitEvent { protocol_do_limit };

void protocol_send_event(const Event*, void*) {}

void report_recompute_pin_string() {}

float limitsMinPosition(axis_t) { return 0.0f; }
float limitsMaxPosition(axis_t) { return 100.0f; }

namespace Machine {
    uint32_t Stepping::_engine           = Stepping::TIMED;
    uint32_t Stepping::_disableDelayUsecs = 0;
    AxisMask Homing::direction_mask     = 0;
    Homing::Phase Homing::_phase        = Homing::None;

    void   Stepping::unblock(axis_t, motor_t) {}
    void   Stepping::assignMotor(axis_t, motor_t, pinnum_t, bool, pinnum_t, bool) {}
    uint32_t Stepping::maxPulsesPerSec() { return 100000; }
    static bool s_limitVars[MAX_N_AXIS][2] = {};
    bool* Stepping::limit_var(axis_t axis, motor_t motor) {
        if (axis < MAX_N_AXIS && motor < 2) return &s_limitVars[axis][motor];
        return nullptr;
    }
}

// Spindle.cpp / Stepper / GCode stubs for spindle tests
void Stepper::updateSpindleCallback() {}

parser_state_t gc_state = {};

bool dwell_ms(uint32_t, DwellMode) {
    return true;
}

bool Macro::run(Channel*) {
    return true;
}
