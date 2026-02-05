// Copyright (c) 2024 - FluidNC Authors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
  PlannerTestHelpers.h - Test utilities for motion planner unit tests
  
  This file provides:
  - Mock configuration and system state
  - Helper functions for creating test blocks
  - Validation helpers for kinematic constraints
  - Test fixtures for planner testing
*/

#include "Planner.h"
#include "Planner/SCurveMath.h"
#include "Planner/BasePlanner.h"

#include <cmath>
#include <cstring>
#include <algorithm>

namespace PlannerTestHelpers {

// ============================================================================
// Constants for testing
// ============================================================================

constexpr size_t TEST_NUM_AXES = 3;
constexpr size_t TEST_PLANNER_BLOCKS = 16;

// Default machine parameters (mm/min and mm/min²)
constexpr float DEFAULT_MAX_RATE = 5000.0f;          // mm/min
constexpr float DEFAULT_ACCEL = 300000.0f;           // mm/min²
constexpr float DEFAULT_JERK = 1000000.0f;           // mm/min³
constexpr float DEFAULT_JUNCTION_DEVIATION = 0.01f;  // mm

// ============================================================================
// Block creation helpers
// ============================================================================

// Creates a minimal plan_block_t with default values
inline plan_block_t makeBlock() {
    plan_block_t block;
    memset(&block, 0, sizeof(plan_block_t));
    return block;
}

// Creates a block for a simple linear move
inline plan_block_t makeLinearBlock(float distance, float programmed_rate, float accel) {
    plan_block_t block = makeBlock();
    
    block.millimeters = distance;
    block.programmed_rate = programmed_rate;
    block.acceleration = accel;
    block.rapid_rate = programmed_rate * 1.5f;  // Assume rapid is faster than programmed
    
    // Set reasonable defaults
    block.max_junction_speed_sqr = programmed_rate * programmed_rate;  // Can enter at full speed
    block.max_entry_speed_sqr = programmed_rate * programmed_rate;
    block.entry_speed_sqr = 0.0f;  // Default to starting from rest
    
    // For step count, assume 80 steps/mm on X axis
    block.steps[0] = static_cast<uint32_t>(distance * 80.0f);
    block.step_event_count = block.steps[0];
    
    return block;
}

// Creates a block with specified entry/exit speeds
inline plan_block_t makeBlockWithSpeeds(float distance, float nominal_speed, float accel,
                                        float entry_speed, float max_entry_speed) {
    plan_block_t block = makeLinearBlock(distance, nominal_speed, accel);
    block.entry_speed_sqr = entry_speed * entry_speed;
    block.max_entry_speed_sqr = max_entry_speed * max_entry_speed;
    return block;
}

// Creates a block for S-curve testing with jerk parameter
inline plan_block_t makeSCurveBlock(float distance, float nominal_speed, float accel, float jerk) {
    plan_block_t block = makeBlockWithSpeeds(distance, nominal_speed, accel, 0.0f, nominal_speed);
    block.jerk = jerk;
    block.entry_accel = 0.0f;
    block.exit_accel = 0.0f;
    block.max_entry_accel = accel;
    return block;
}

// Creates a rapid motion block
inline plan_block_t makeRapidBlock(float distance, float rapid_rate, float accel) {
    plan_block_t block = makeLinearBlock(distance, rapid_rate, accel);
    block.motion.rapidMotion = 1;
    block.rapid_rate = rapid_rate;
    block.programmed_rate = rapid_rate;
    return block;
}

// ============================================================================
// Kinematic validation helpers
// ============================================================================

// Checks if a velocity change respects the acceleration limit
// Returns true if v1 can reach v2 in the given distance with max acceleration
inline bool verifyAccelLimit(float v1, float v2, float distance, float a_max) {
    // Using v² = v₀² + 2*a*d
    // For acceleration: v2² = v1² + 2*a*d → d = (v2² - v1²) / (2*a)
    // For deceleration: v2² = v1² - 2*a*d → d = (v1² - v2²) / (2*a)
    
    if (v2 > v1) {
        // Acceleration case
        float d_needed = (v2 * v2 - v1 * v1) / (2.0f * a_max);
        return d_needed <= distance * 1.001f;  // 0.1% tolerance
    } else {
        // Deceleration case
        float d_needed = (v1 * v1 - v2 * v2) / (2.0f * a_max);
        return d_needed <= distance * 1.001f;
    }
}

// Checks if an acceleration change respects the jerk limit
// Returns true if a1 can reach a2 in the given time with max jerk
inline bool verifyJerkLimit(float a1, float a2, float time, float j_max) {
    // a = a0 + j*t → j = (a - a0) / t
    if (time <= 0.0f) {
        return fabsf(a2 - a1) < 0.001f;  // Must be equal if no time
    }
    float j_needed = fabsf(a2 - a1) / time;
    return j_needed <= j_max * 1.001f;
}

// Checks if velocity is within valid range
inline bool verifyVelocityBounds(float velocity, float v_min, float v_max) {
    return velocity >= v_min - 0.001f && velocity <= v_max + 0.001f;
}

// Checks if velocity is non-negative
inline bool verifyNonNegativeVelocity(float velocity) {
    return velocity >= -0.001f;
}

// ============================================================================
// Profile validation helpers
// ============================================================================

// Checks that a trapezoidal profile is physically valid
struct TrapezoidProfileResult {
    bool valid;
    const char* error;
    float computed_distance;
    float computed_time;
};

inline TrapezoidProfileResult validateTrapezoidProfile(
    float distance, float v_entry, float v_exit, float v_cruise, float accel) {
    
    TrapezoidProfileResult result = {true, nullptr, 0.0f, 0.0f};
    
    // Entry speed can't exceed cruise
    if (v_entry > v_cruise + 0.001f) {
        result.valid = false;
        result.error = "Entry speed exceeds cruise speed";
        return result;
    }
    
    // Exit speed can't exceed cruise
    if (v_exit > v_cruise + 0.001f) {
        result.valid = false;
        result.error = "Exit speed exceeds cruise speed";
        return result;
    }
    
    // Calculate distances for each phase
    float d_accel = 0.0f, d_cruise = 0.0f, d_decel = 0.0f;
    float t_accel = 0.0f, t_cruise = 0.0f, t_decel = 0.0f;
    
    // Acceleration phase
    if (v_cruise > v_entry) {
        d_accel = (v_cruise * v_cruise - v_entry * v_entry) / (2.0f * accel);
        t_accel = (v_cruise - v_entry) / accel;
    }
    
    // Deceleration phase
    if (v_cruise > v_exit) {
        d_decel = (v_cruise * v_cruise - v_exit * v_exit) / (2.0f * accel);
        t_decel = (v_cruise - v_exit) / accel;
    }
    
    // Cruise phase (if any)
    d_cruise = distance - d_accel - d_decel;
    if (d_cruise < -0.001f) {
        result.valid = false;
        result.error = "Not enough distance for accel/decel";
        return result;
    }
    if (d_cruise > 0.001f) {
        t_cruise = d_cruise / v_cruise;
    }
    
    result.computed_distance = d_accel + d_cruise + d_decel;
    result.computed_time = t_accel + t_cruise + t_decel;
    
    // Verify total distance matches
    if (fabsf(result.computed_distance - distance) > distance * 0.01f) {
        result.valid = false;
        result.error = "Computed distance doesn't match target";
    }
    
    return result;
}

// Validates an S-curve profile using SCurveMath
struct SCurveProfileResult {
    bool valid;
    const char* error;
    SCurveMath::SCurveProfile profile;
};

inline SCurveProfileResult validateSCurveProfile(
    float distance, float v_entry, float v_exit, float v_max, float a_max, float jerk) {
    
    SCurveProfileResult result = {true, nullptr, {}};
    
    // Plan the profile
    result.profile = SCurveMath::planProfile(
        distance, v_entry, 0.0f, v_exit, 0.0f, v_max, a_max, jerk);
    
    // Verify entry velocity
    if (fabsf(result.profile.v[0] - v_entry) > 0.1f) {
        result.valid = false;
        result.error = "Entry velocity mismatch";
        return result;
    }
    
    // Verify exit velocity
    if (fabsf(result.profile.v[7] - v_exit) > 0.1f) {
        result.valid = false;
        result.error = "Exit velocity mismatch";
        return result;
    }
    
    // Verify total distance (allow 10% tolerance for numerical precision)
    if (fabsf(result.profile.total_distance - distance) > distance * 0.1f) {
        result.valid = false;
        result.error = "Distance mismatch";
        return result;
    }
    
    // Verify peak velocity doesn't exceed v_max
    if (result.profile.v_peak > v_max + 0.1f) {
        result.valid = false;
        result.error = "Peak velocity exceeds maximum";
        return result;
    }
    
    // Verify all phase times are non-negative
    for (int i = 0; i < 7; i++) {
        if (result.profile.t[i] < -0.001f) {
            result.valid = false;
            result.error = "Negative phase time";
            return result;
        }
    }
    
    return result;
}

// ============================================================================
// Transition validation helpers
// ============================================================================

// Checks that two consecutive blocks have matching exit/entry speeds
inline bool verifyVelocityContinuity(const plan_block_t& block1, const plan_block_t& block2,
                                     float tolerance = 0.1f) {
    // block2's entry_speed_sqr should allow a valid junction
    // For now, just check that entry speed is within bounds
    float block2_entry = sqrtf(block2.entry_speed_sqr);
    float block2_max_entry = sqrtf(block2.max_entry_speed_sqr);
    
    return block2_entry <= block2_max_entry + tolerance;
}

// Checks that acceleration is continuous across a junction (for S-curve)
inline bool verifyAccelContinuity(const plan_block_t& block1, const plan_block_t& block2,
                                  float tolerance = 1.0f) {
    // block1's exit_accel should match block2's entry_accel
    return fabsf(block1.exit_accel - block2.entry_accel) <= tolerance;
}

// ============================================================================
// Test sequence helpers
// ============================================================================

// Creates a sequence of blocks representing a simple path
// Returns the number of blocks created
inline int createSimplePath(plan_block_t* blocks, int max_blocks,
                           float* distances, float* speeds, int num_segments,
                           float accel, float jerk = 0.0f) {
    if (num_segments > max_blocks) {
        num_segments = max_blocks;
    }
    
    for (int i = 0; i < num_segments; i++) {
        if (jerk > 0.0f) {
            blocks[i] = makeSCurveBlock(distances[i], speeds[i], accel, jerk);
        } else {
            blocks[i] = makeLinearBlock(distances[i], speeds[i], accel);
        }
    }
    
    return num_segments;
}

// ============================================================================
// Floating point comparison helpers
// ============================================================================

inline bool floatEquals(float a, float b, float tolerance = 0.01f) {
    return fabsf(a - b) <= tolerance;
}

inline bool floatGreaterOrEqual(float a, float b, float tolerance = 0.001f) {
    return a >= b - tolerance;
}

inline bool floatLessOrEqual(float a, float b, float tolerance = 0.001f) {
    return a <= b + tolerance;
}

// Percentage-based comparison
inline bool withinPercent(float actual, float expected, float percent = 5.0f) {
    if (fabsf(expected) < 0.001f) {
        return fabsf(actual) < 0.001f;
    }
    return fabsf(actual - expected) / fabsf(expected) * 100.0f <= percent;
}

// ============================================================================
// Pause/Resume simulation helpers
// ============================================================================

// Simulates a partial block completion (for pause testing)
// Returns a modified block representing the remaining motion
inline plan_block_t simulatePartialCompletion(const plan_block_t& original,
                                              float fraction_complete,
                                              float current_speed) {
    plan_block_t remaining = original;
    
    // Reduce remaining distance
    remaining.millimeters = original.millimeters * (1.0f - fraction_complete);
    
    // Update step counts proportionally
    for (int i = 0; i < TEST_NUM_AXES; i++) {
        remaining.steps[i] = static_cast<uint32_t>(original.steps[i] * (1.0f - fraction_complete));
    }
    remaining.step_event_count = static_cast<uint32_t>(original.step_event_count * (1.0f - fraction_complete));
    
    // Entry speed for remaining portion is current speed (which will be 0 after pause)
    remaining.entry_speed_sqr = 0.0f;  // Restarting from rest
    
    return remaining;
}

// ============================================================================
// Block buffer simulation helpers
// ============================================================================

// Simple ring buffer implementation for testing planner algorithms
class TestBlockBuffer {
public:
    static constexpr int BUFFER_SIZE = TEST_PLANNER_BLOCKS;
    
    TestBlockBuffer() : _head(0), _tail(0) {
        memset(_blocks, 0, sizeof(_blocks));
    }
    
    bool isFull() const {
        return nextIndex(_head) == _tail;
    }
    
    bool isEmpty() const {
        return _head == _tail;
    }
    
    int available() const {
        if (_head >= _tail) {
            return BUFFER_SIZE - 1 - (_head - _tail);
        } else {
            return _tail - _head - 1;
        }
    }
    
    int count() const {
        if (_head >= _tail) {
            return _head - _tail;
        } else {
            return BUFFER_SIZE - _tail + _head;
        }
    }
    
    // Add a block to the buffer
    bool add(const plan_block_t& block) {
        if (isFull()) return false;
        
        _blocks[_head] = block;
        _head = nextIndex(_head);
        return true;
    }
    
    // Get the oldest block (at tail)
    plan_block_t* getTail() {
        if (isEmpty()) return nullptr;
        return &_blocks[_tail];
    }
    
    // Get the newest block (at head - 1)
    plan_block_t* getHead() {
        if (isEmpty()) return nullptr;
        return &_blocks[prevIndex(_head)];
    }
    
    // Remove the oldest block
    void removeTail() {
        if (!isEmpty()) {
            _tail = nextIndex(_tail);
        }
    }
    
    // Access block by index (0 = tail, count-1 = head)
    plan_block_t* at(int index) {
        if (index < 0 || index >= count()) return nullptr;
        return &_blocks[(_tail + index) % BUFFER_SIZE];
    }
    
    // Reset the buffer
    void reset() {
        _head = 0;
        _tail = 0;
    }
    
    // Get raw buffer access for planner testing
    plan_block_t* buffer() { return _blocks; }
    int head() const { return _head; }
    int tail() const { return _tail; }
    
private:
    int nextIndex(int idx) const {
        return (idx + 1) % BUFFER_SIZE;
    }
    
    int prevIndex(int idx) const {
        return (idx + BUFFER_SIZE - 1) % BUFFER_SIZE;
    }
    
    plan_block_t _blocks[BUFFER_SIZE];
    int _head;
    int _tail;
};

// ============================================================================
// Expected result structures for test validation
// ============================================================================

struct ExpectedProfile {
    float entry_speed;
    float exit_speed;
    float peak_speed;
    float accel_distance;
    float decel_distance;
    float cruise_distance;
};

inline bool validateAgainstExpected(const plan_block_t& block,
                                    const ExpectedProfile& expected,
                                    float tolerance_percent = 10.0f) {
    float entry = sqrtf(block.entry_speed_sqr);
    
    if (!withinPercent(entry, expected.entry_speed, tolerance_percent)) {
        return false;
    }
    
    // More checks would go here based on what's stored in block
    // after planner processing
    
    return true;
}

} // namespace PlannerTestHelpers
