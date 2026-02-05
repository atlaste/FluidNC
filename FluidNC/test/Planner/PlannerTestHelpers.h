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

// ============================================================================
// Comprehensive Block Validation
// ============================================================================

// Structure to hold computed kinematic properties of a processed block
struct BlockKinematics {
    // Speeds (mm/min)
    float entry_speed;          // Speed at start of block
    float exit_speed;           // Speed at end of block (computed from next block or 0 for last)
    float max_entry_speed;      // Maximum allowed entry speed
    float peak_speed;           // Maximum speed reached during block
    float nominal_speed;        // Target cruise speed
    
    // Accelerations (mm/min²)
    float entry_accel;          // Acceleration at start (for S-curve)
    float exit_accel;           // Acceleration at end (for S-curve)
    float max_accel;            // Maximum acceleration
    
    // Jerk (mm/min³)
    float jerk;                 // Jerk limit (for S-curve)
    
    // Distances (mm)
    float total_distance;       // Total block distance
    float accel_distance;       // Distance in acceleration phase
    float cruise_distance;      // Distance at cruise speed
    float decel_distance;       // Distance in deceleration phase
    
    // Times (minutes)
    float total_time;           // Total block time
    float accel_time;           // Time in acceleration
    float cruise_time;          // Time at cruise
    float decel_time;           // Time in deceleration
    
    // Validation flags
    bool is_valid;
    const char* error_message;
};

// Extract kinematics from a processed block (trapezoidal planner)
inline BlockKinematics extractTrapezoidKinematics(
    const plan_block_t& block, 
    float exit_speed_sqr = 0.0f)  // 0 for last block
{
    BlockKinematics k = {};
    k.is_valid = true;
    k.error_message = nullptr;
    
    k.entry_speed = sqrtf(block.entry_speed_sqr);
    k.exit_speed = sqrtf(exit_speed_sqr);
    k.max_entry_speed = sqrtf(block.max_entry_speed_sqr);
    k.max_accel = block.acceleration;
    k.total_distance = block.millimeters;
    k.nominal_speed = block.programmed_rate;
    
    // Check basic validity
    if (std::isnan(k.entry_speed) || std::isinf(k.entry_speed)) {
        k.is_valid = false;
        k.error_message = "Entry speed is NaN or infinite";
        return k;
    }
    
    if (k.entry_speed < 0.0f) {
        k.is_valid = false;
        k.error_message = "Negative entry speed";
        return k;
    }
    
    if (k.entry_speed > k.max_entry_speed + 0.1f) {
        k.is_valid = false;
        k.error_message = "Entry speed exceeds maximum";
        return k;
    }
    
    // Calculate peak speed (limited by what can be achieved)
    // v_peak² = v_entry² + 2*a*d_accel = v_exit² + 2*a*d_decel
    float v_entry_sq = block.entry_speed_sqr;
    float v_exit_sq = exit_speed_sqr;
    float a = block.acceleration;
    float d = block.millimeters;
    
    // Compute intersection of accel and decel curves
    // v_peak² = v_entry² + 2*a*d_accel
    // v_peak² = v_exit² + 2*a*(d - d_accel)
    // Solving: d_accel = (v_exit² - v_entry² + 2*a*d) / (4*a)
    if (a > 0.001f) {
        k.accel_distance = (v_exit_sq - v_entry_sq + 2.0f * a * d) / (4.0f * a);
        
        if (k.accel_distance < 0.0f) {
            k.accel_distance = 0.0f;  // Pure deceleration
            k.peak_speed = k.entry_speed;
        } else if (k.accel_distance > d) {
            k.accel_distance = d;  // Pure acceleration
            k.peak_speed = sqrtf(v_entry_sq + 2.0f * a * d);
        } else {
            // Triangular or trapezoidal
            float v_peak_sq = v_entry_sq + 2.0f * a * k.accel_distance;
            k.peak_speed = sqrtf(v_peak_sq);
        }
        
        // Limit peak speed to nominal
        if (k.peak_speed > k.nominal_speed) {
            k.peak_speed = k.nominal_speed;
            // Recalculate accel distance
            k.accel_distance = (k.peak_speed * k.peak_speed - v_entry_sq) / (2.0f * a);
            if (k.accel_distance < 0.0f) k.accel_distance = 0.0f;
        }
        
        k.decel_distance = (k.peak_speed * k.peak_speed - v_exit_sq) / (2.0f * a);
        if (k.decel_distance < 0.0f) k.decel_distance = 0.0f;
        
        k.cruise_distance = d - k.accel_distance - k.decel_distance;
        if (k.cruise_distance < 0.0f) k.cruise_distance = 0.0f;
        
        // Calculate times
        if (k.accel_distance > 0.001f && a > 0.001f) {
            k.accel_time = (k.peak_speed - k.entry_speed) / a;
        }
        if (k.cruise_distance > 0.001f && k.peak_speed > 0.001f) {
            k.cruise_time = k.cruise_distance / k.peak_speed;
        }
        if (k.decel_distance > 0.001f && a > 0.001f) {
            k.decel_time = (k.peak_speed - k.exit_speed) / a;
        }
        k.total_time = k.accel_time + k.cruise_time + k.decel_time;
    }
    
    return k;
}

// Validate that a block can stop within its distance (for last block)
inline bool validateCanStop(const plan_block_t& block) {
    float entry_speed = sqrtf(block.entry_speed_sqr);
    float a = block.acceleration;
    float d = block.millimeters;
    
    if (a <= 0.001f) return true;  // Can't accelerate anyway
    
    // Distance needed to stop: d = v²/(2a)
    float d_needed = (entry_speed * entry_speed) / (2.0f * a);
    return d_needed <= d * 1.01f;  // 1% tolerance
}

// Validate velocity continuity between consecutive blocks
inline bool validateVelocityContinuityBetween(
    const plan_block_t& block1, 
    const plan_block_t& block2,
    const char** error_msg = nullptr)
{
    // block1's exit speed must be achievable from its entry speed over its distance
    // block2's entry speed is what the planner computed
    
    float b1_entry = sqrtf(block1.entry_speed_sqr);
    float b2_entry = sqrtf(block2.entry_speed_sqr);
    float b2_max_entry = sqrtf(block2.max_entry_speed_sqr);
    
    // block2's entry speed should not exceed its max
    if (b2_entry > b2_max_entry + 0.1f) {
        if (error_msg) *error_msg = "Block entry speed exceeds max_entry_speed";
        return false;
    }
    
    // block2's entry speed should be achievable from block1
    // v2² = v1² + 2*a*d  (for accel) or v2² = v1² - 2*a*d (for decel)
    float max_achievable = sqrtf(b1_entry * b1_entry + 2.0f * block1.acceleration * block1.millimeters);
    if (b2_entry > max_achievable + 0.1f) {
        if (error_msg) *error_msg = "Entry speed not achievable by accelerating over previous block";
        return false;
    }
    
    return true;
}

// Validate acceleration continuity for S-curve planners
inline bool validateAccelContinuityBetween(
    const plan_block_t& block1,
    const plan_block_t& block2, 
    float jerk,
    const char** error_msg = nullptr)
{
    // For S-curve, exit accel of block1 should match entry accel of block2
    // or the transition should be achievable within jerk limits
    
    float a1_exit = block1.exit_accel;
    float a2_entry = block2.entry_accel;
    
    // Allow some tolerance for numerical precision
    if (fabsf(a1_exit - a2_entry) > block1.acceleration * 0.1f) {
        if (error_msg) *error_msg = "Acceleration discontinuity at junction";
        return false;
    }
    
    return true;
}

// Comprehensive validation of a planned block sequence
struct SequenceValidationResult {
    bool is_valid;
    int first_invalid_block;
    const char* error_message;
    
    // Summary statistics
    float total_distance;
    float total_time;
    float max_speed_achieved;
    float max_accel_achieved;
};

template<typename PlannerType>
inline SequenceValidationResult validatePlannedSequence(
    PlannerType& planner,
    int block_count,
    bool is_scurve = false)
{
    SequenceValidationResult result = {};
    result.is_valid = true;
    result.first_invalid_block = -1;
    
    for (int i = 0; i < block_count; i++) {
        plan_block_t* block = planner.getBlock(i);
        if (!block) {
            result.is_valid = false;
            result.first_invalid_block = i;
            result.error_message = "Null block pointer";
            return result;
        }
        
        // Get exit speed (from next block or 0 for last)
        float exit_speed_sqr = 0.0f;
        if (i < block_count - 1) {
            plan_block_t* next = planner.getBlock(i + 1);
            if (next) {
                exit_speed_sqr = next->entry_speed_sqr;
            }
        }
        
        // Extract and validate kinematics
        BlockKinematics k = extractTrapezoidKinematics(*block, exit_speed_sqr);
        if (!k.is_valid) {
            result.is_valid = false;
            result.first_invalid_block = i;
            result.error_message = k.error_message;
            return result;
        }
        
        // Accumulate statistics
        result.total_distance += k.total_distance;
        result.total_time += k.total_time;
        if (k.peak_speed > result.max_speed_achieved) {
            result.max_speed_achieved = k.peak_speed;
        }
        if (k.max_accel > result.max_accel_achieved) {
            result.max_accel_achieved = k.max_accel;
        }
        
        // Validate velocity continuity with next block
        if (i < block_count - 1) {
            plan_block_t* next = planner.getBlock(i + 1);
            const char* cont_error = nullptr;
            if (!validateVelocityContinuityBetween(*block, *next, &cont_error)) {
                result.is_valid = false;
                result.first_invalid_block = i;
                result.error_message = cont_error;
                return result;
            }
            
            // For S-curve, also check acceleration continuity
            if (is_scurve) {
                const char* accel_error = nullptr;
                if (!validateAccelContinuityBetween(*block, *next, block->jerk, &accel_error)) {
                    result.is_valid = false;
                    result.first_invalid_block = i;
                    result.error_message = accel_error;
                    return result;
                }
            }
        }
        
        // For last block, verify it can stop
        if (i == block_count - 1) {
            if (!validateCanStop(*block)) {
                result.is_valid = false;
                result.first_invalid_block = i;
                result.error_message = "Last block cannot decelerate to stop";
                return result;
            }
        }
    }
    
    return result;
}

// ============================================================================
// Kinematic simulation - step through a profile to verify actual motion
// ============================================================================

// State during profile simulation
struct SimulationState {
    float position;     // mm from block start
    float velocity;     // mm/min
    float acceleration; // mm/min²
    float time;         // minutes elapsed
};

// Simulate a single time step of trapezoidal motion
inline SimulationState simulateTrapezoidStep(
    const SimulationState& state,
    float dt,  // time step in minutes
    float target_accel,
    float max_speed)
{
    SimulationState next = state;
    
    // Apply acceleration
    next.velocity += target_accel * dt;
    
    // Clamp velocity
    if (next.velocity > max_speed) next.velocity = max_speed;
    if (next.velocity < 0.0f) next.velocity = 0.0f;
    
    // Average velocity for distance (trapezoidal integration)
    float avg_vel = (state.velocity + next.velocity) / 2.0f;
    next.position += avg_vel * dt;
    
    next.acceleration = target_accel;
    next.time += dt;
    
    return next;
}

// Simulate complete execution of a trapezoidal block and verify
struct SimulationResult {
    bool success;
    const char* error;
    float final_position;
    float final_velocity;
    float total_time;
    float max_velocity_achieved;
    float max_velocity_error;  // Max deviation from expected
};

inline SimulationResult simulateTrapezoidBlock(
    const plan_block_t& block,
    float exit_speed,
    float time_step = 0.0001f)  // 0.0001 min = 6ms
{
    SimulationResult result = {};
    result.success = true;
    
    float entry_speed = sqrtf(block.entry_speed_sqr);
    float a = block.acceleration;
    float d = block.millimeters;
    float v_nominal = block.programmed_rate;
    
    // Calculate profile phases
    BlockKinematics k = extractTrapezoidKinematics(block, exit_speed * exit_speed);
    if (!k.is_valid) {
        result.success = false;
        result.error = k.error_message;
        return result;
    }
    
    SimulationState state = {0.0f, entry_speed, 0.0f, 0.0f};
    
    // Safety limit: prevent infinite loops
    int max_iterations = 1000000;
    int iter = 0;
    
    while (state.position < d && iter++ < max_iterations) {
        float target_accel = 0.0f;
        float remaining = d - state.position;
        
        // Determine which phase we're in
        if (state.position < k.accel_distance && state.velocity < k.peak_speed - 0.1f) {
            // Acceleration phase
            target_accel = a;
        } else if (remaining < k.decel_distance + 0.1f || state.velocity > k.peak_speed + 0.1f) {
            // Deceleration phase
            target_accel = -a;
        } else {
            // Cruise phase
            target_accel = 0.0f;
        }
        
        state = simulateTrapezoidStep(state, time_step, target_accel, v_nominal);
        
        if (state.velocity > result.max_velocity_achieved) {
            result.max_velocity_achieved = state.velocity;
        }
        
        // Check velocity error
        float expected_vel = k.peak_speed;  // Simplified
        float vel_error = fabsf(state.velocity - expected_vel);
        if (vel_error > result.max_velocity_error) {
            result.max_velocity_error = vel_error;
        }
    }
    
    if (iter >= max_iterations) {
        result.success = false;
        result.error = "Simulation timeout";
        return result;
    }
    
    result.final_position = state.position;
    result.final_velocity = state.velocity;
    result.total_time = state.time;
    
    // Verify final state
    if (fabsf(result.final_position - d) > d * 0.01f) {
        result.success = false;
        result.error = "Final position error";
    }
    
    if (fabsf(result.final_velocity - exit_speed) > exit_speed * 0.1f + 0.1f) {
        result.success = false;
        result.error = "Final velocity error";
    }
    
    return result;
}

// ============================================================================
// Test assertions with detailed error messages
// ============================================================================

// Macro-like function for detailed kinematic assertions
inline void assertKinematicsValid(
    const BlockKinematics& k,
    const char* block_name,
    void (*assert_fn)(bool, const char*))
{
    char msg[256];
    
    snprintf(msg, sizeof(msg), "%s: entry_speed should be non-negative (got %.2f)", 
             block_name, k.entry_speed);
    assert_fn(k.entry_speed >= -0.001f, msg);
    
    snprintf(msg, sizeof(msg), "%s: entry_speed should not exceed max (%.2f > %.2f)", 
             block_name, k.entry_speed, k.max_entry_speed);
    assert_fn(k.entry_speed <= k.max_entry_speed + 0.1f, msg);
    
    snprintf(msg, sizeof(msg), "%s: peak_speed should be >= entry_speed (%.2f < %.2f)", 
             block_name, k.peak_speed, k.entry_speed);
    assert_fn(k.peak_speed >= k.entry_speed - 0.1f, msg);
    
    snprintf(msg, sizeof(msg), "%s: peak_speed should not exceed nominal (%.2f > %.2f)", 
             block_name, k.peak_speed, k.nominal_speed);
    assert_fn(k.peak_speed <= k.nominal_speed + 0.1f, msg);
    
    snprintf(msg, sizeof(msg), "%s: distances should sum to total (%.2f + %.2f + %.2f != %.2f)", 
             block_name, k.accel_distance, k.cruise_distance, k.decel_distance, k.total_distance);
    float d_sum = k.accel_distance + k.cruise_distance + k.decel_distance;
    assert_fn(fabsf(d_sum - k.total_distance) < k.total_distance * 0.05f, msg);
}

} // namespace PlannerTestHelpers
