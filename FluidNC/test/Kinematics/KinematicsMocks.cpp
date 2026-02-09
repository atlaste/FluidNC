// Stubs for Kinematics unit tests.
// Provides mock implementations for symbols referenced by Cartesian.cpp,
// CoreXY.cpp, Midtbot.cpp, WallPlotter.cpp and ParallelDelta.cpp that are
// not already supplied by PlannerMocks.cpp or MotorTestMocks.cpp.
//
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "MotionControl.h"
#include "NutsBolts.h"
#include "Limit.h"
#include "System.h"
#include "Stepping.h"
#include "Protocol.h"
#include "DynamicLimits.h"
#include "Machine/Homing.h"

#include <cmath>
#include <cstring>
#include <vector>
#include <string>

// ============================================================================
// NutsBolts pure-math implementations
// ============================================================================

float hypot_f(float x, float y) {
    return sqrtf(x * x + y * y);
}

float vector_distance(float* v1, float* v2, size_t n) {
    float sum = 0.0f;
    for (size_t i = 0; i < n; i++) {
        float d = v1[i] - v2[i];
        sum += d * d;
    }
    return sqrtf(sum);
}

float vector_length(float* v, size_t n) {
    float sum = 0.0f;
    for (size_t i = 0; i < n; i++) {
        sum += v[i] * v[i];
    }
    return sqrtf(sum);
}

void scale_vector(float* v, float scale, size_t n) {
    for (size_t i = 0; i < n; i++) {
        v[i] *= scale;
    }
}

bool read_float(const char*, size_t&, float&) {
    return false;
}

bool multiple_bits_set(uint32_t val) {
    return val && (val & (val - 1));
}

// ============================================================================
// MotionControl stub
// ============================================================================

bool mc_move_motors(float* target, plan_line_data_t* pl_data) {
    return true;  // Always succeed for tests
}

// ============================================================================
// System stubs
// ============================================================================

static float  mock_motor_pos[MAX_N_AXIS] = { 0 };
static steps_t mock_axis_steps[MAX_N_AXIS] = { 0 };

void set_steps(axis_t axis, steps_t steps) {
    if (axis < MAX_N_AXIS) {
        mock_axis_steps[axis] = steps;
    }
}

steps_t get_axis_steps(axis_t axis) {
    return axis < MAX_N_AXIS ? mock_axis_steps[axis] : 0;
}

void set_motor_pos(size_t motor, float motor_pos) {
    if (motor < MAX_N_AXIS) {
        mock_motor_pos[motor] = motor_pos;
    }
}

void set_motor_pos(float* motor_pos, size_t n_motors) {
    for (size_t i = 0; i < n_motors && i < MAX_N_AXIS; i++) {
        mock_motor_pos[i] = motor_pos[i];
    }
}

float* get_motor_pos() {
    return mock_motor_pos;
}

// Note: motor_pos_to_steps(float, size_t) is already provided by PlannerMocks.cpp

float* get_mpos() {
    static float mpos[MAX_N_AXIS] = { 0 };
    return mpos;
}

// ============================================================================
// Stepping stubs
// ============================================================================

namespace Machine {
    void Stepping::unlimit(axis_t, motor_t) {}
    void Stepping::limit(axis_t, motor_t) {}
    void Stepping::block(axis_t, motor_t) {}
    void Homing::fail(ExecAlarm) {}
}

// ============================================================================
// Limit stubs
// ============================================================================

static bool soft_limit_flag = false;

void limit_error() {
    soft_limit_flag = true;
}

void limit_error(axis_t, float) {
    soft_limit_flag = true;
}

bool ambiguousLimit() {
    return false;
}

MotorMask limits_get_state() {
    return 0;
}

// ============================================================================
// DynamicLimits stubs
// ============================================================================

std::vector<DynamicLimitProvider*> DynamicLimits::_providers;

void DynamicLimits::registerProvider(DynamicLimitProvider*) {}
void DynamicLimits::unregisterProvider(DynamicLimitProvider*) {}

void DynamicLimits::getEffectiveLimits(const float*, const float*, float* axis_min, float* axis_max) {
    for (int i = 0; i < MAX_N_AXIS; i++) {
        axis_min[i] = std::nanf("");
        axis_max[i] = std::nanf("");
    }
}

bool DynamicLimits::checkMove(const float*, const float*, const float*, std::string&) {
    return true;
}

bool DynamicLimits::checkPosition(const float*, const float*, std::string&) {
    return true;
}

// ============================================================================
// Protocol event objects referenced by kinematics homing code
// ============================================================================

static void noop_event() {}
const NoArgEvent cycleStartEvent { noop_event };
const NoArgEvent cycleStopEvent  { noop_event };

void protocol_disable_steppers() {}
