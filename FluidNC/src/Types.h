#pragma once

#include <cstdint>

typedef uint_fast8_t  motor_t;   // Motor number
typedef uint_fast16_t AxisMask;  // Bits indexed by axis number
typedef uint64_t      MotorMask;  // Bits indexed by motor_num*16 + axis; see MAX_MOTORS_PER_AXIS
typedef uint_fast8_t  Percent;    // Integer percent
typedef uint_fast8_t  objnum_t;   // Index number for things like uarts, channels, ...
typedef uint_fast8_t  tool_t;     // Tool number

// Motors that a single axis can gang together, e.g. the four corner motors that lift a
// Voron-style gantry as one Z axis.
//
// MotorMask gives each motor a 16 bit lane, so this cannot exceed 4 without either widening
// MotorMask past 64 bits or narrowing the lane below MAX_N_AXIS.  Axis and Stepping both alias
// this rather than declaring their own copy, because the two must agree.
const int MAX_MOTORS_PER_AXIS = 4;

// Width of one motor's lane in a MotorMask.  Must be at least MAX_N_AXIS.
const int MOTOR_MASK_STRIDE = 16;

// Axis array index values. Must start with 0 and be continuous.
// Note: You set the number of axes used by changing MAX_N_AXIS.
// Be sure to define pins or servos in the machine definition file.
typedef enum {
    X_AXIS = 0,  // Axis indexing value.
    Y_AXIS = 1,
    Z_AXIS = 2,
    A_AXIS = 3,
    B_AXIS = 4,
    C_AXIS = 5,
    U_AXIS = 6,
    V_AXIS = 7,
    W_AXIS = 8,

    MAX_N_AXIS,  // Number of axes

    INVALID_AXIS = 255,
} axis_t;

axis_t& operator++(axis_t& axis);
axis_t  operator++(axis_t& axis, int);
axis_t& operator--(axis_t& axis);
axis_t  operator--(axis_t& axis, int);

inline bool is_linear(axis_t axis) {
    return axis < A_AXIS || axis > C_AXIS;
}

const motor_t MOTOR0        = 0;
const motor_t MOTOR1        = 1;
const motor_t MOTOR2        = 2;
const motor_t MOTOR3        = 3;
const motor_t INVALID_MOTOR = 255;

static_assert(MOTOR_MASK_STRIDE >= MAX_N_AXIS, "A motor lane must be wide enough to hold every axis bit");
static_assert(MAX_MOTORS_PER_AXIS * MOTOR_MASK_STRIDE <= 64, "MotorMask cannot hold that many motor lanes");
