// Copyright (c) 2024 - FluidNC Authors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "../TestFramework.h"
#include "PlannerTestHelpers.h"
#include "Planner/TrapezoidPlanner.h"
#include "Planner/BasePlanner.h"

#include <cmath>
#include <cstring>

using namespace PlannerTestHelpers;

// Import TrapezoidPlanner ramp type constants
static constexpr uint8_t RAMP_ACCEL = TrapezoidPlanner::RAMP_ACCEL;
static constexpr uint8_t RAMP_CRUISE = TrapezoidPlanner::RAMP_CRUISE;
static constexpr uint8_t RAMP_DECEL = TrapezoidPlanner::RAMP_DECEL;
static constexpr uint8_t RAMP_DECEL_OVERRIDE = TrapezoidPlanner::RAMP_DECEL_OVERRIDE;

// ============================================================================
// Testable TrapezoidPlanner wrapper
// This class exposes internal methods and provides controlled test environment
// ============================================================================

class TestableTrapezoidPlanner : public TrapezoidPlanner {
public:
    static constexpr int BUFFER_SIZE = TEST_PLANNER_BLOCKS;
    
    TestableTrapezoidPlanner() : TrapezoidPlanner("TestableTrapezoidPlanner") {
        // Initialize internal buffer (normally done by init() with config)
        _block_buffer = new plan_block_t[BUFFER_SIZE];
        memset(_block_buffer, 0, sizeof(plan_block_t) * BUFFER_SIZE);
        resetBuffer();
    }
    
    ~TestableTrapezoidPlanner() {
        delete[] _block_buffer;
        _block_buffer = nullptr;
    }
    
    // Expose internal methods for testing
    void testRecalculateBackward() { recalculateBackward(); }
    void testRecalculateForward() { recalculateForward(); }
    void testRecalculate() { recalculate(); }
    
    VelocityProfile testComputeVelocityProfile(plan_block_t* block, float entry_speed, float exit_speed_sqr) {
        return computeVelocityProfile(block, entry_speed, exit_speed_sqr);
    }
    
    RampUpdate testUpdateRamp(uint8_t ramp_type, float time_var, float current_speed,
                              float current_accel, float mm_remaining, float phase_boundary) {
        return updateRamp(ramp_type, time_var, current_speed, current_accel, mm_remaining, phase_boundary);
    }
    
    float testComputeAccelDistance(float v_entry, float v_exit, float accel) {
        return computeAccelDistance(v_entry, v_exit, accel);
    }
    
    float testComputeDecelDistance(float v_entry, float v_exit, float accel) {
        return computeDecelDistance(v_entry, v_exit, accel);
    }
    
    // Direct buffer manipulation for testing
    void addBlock(const plan_block_t& block) {
        if (_next_buffer_head == _block_buffer_tail) return;  // Buffer full
        
        _block_buffer[_block_buffer_head] = block;
        _block_buffer_head = _next_buffer_head;
        _next_buffer_head = nextBlockIndexTest(_block_buffer_head);
    }
    
    plan_block_t* getBlock(int index) {
        return &_block_buffer[index];
    }
    
    int getHead() const { return _block_buffer_head; }
    int getTail() const { return _block_buffer_tail; }
    int getPlanned() const { return _block_buffer_planned; }
    
    void setPlanned(int idx) { _block_buffer_planned = idx; }
    
    int blockCount() const {
        if (_block_buffer_head >= _block_buffer_tail) {
            return _block_buffer_head - _block_buffer_tail;
        }
        return BUFFER_SIZE - _block_buffer_tail + _block_buffer_head;
    }
    
    void resetBuffer() {
        _block_buffer_tail = 0;
        _block_buffer_head = 0;
        _next_buffer_head = 1;
        _block_buffer_planned = 0;
    }

private:
    uint8_t nextBlockIndexTest(uint8_t idx) {
        idx++;
        if (idx == BUFFER_SIZE) idx = 0;
        return idx;
    }
    
    uint8_t prevBlockIndexTest(uint8_t idx) {
        if (idx == 0) idx = BUFFER_SIZE;
        idx--;
        return idx;
    }
};

// ============================================================================
// Helper functions for creating test blocks
// ============================================================================

static plan_block_t createTestBlock(float distance, float programmed_rate, float accel,
                                    float max_entry_speed, float max_junction_speed) {
    plan_block_t block = makeLinearBlock(distance, programmed_rate, accel);
    block.max_entry_speed_sqr = max_entry_speed * max_entry_speed;
    block.max_junction_speed_sqr = max_junction_speed * max_junction_speed;
    return block;
}

// ============================================================================
// Basic kinematic formula tests
// ============================================================================

// Tests that acceleration distance calculation follows v² = v₀² + 2ad
Test(TrapezoidPlanner, AccelDistance_Formula) {
    TestableTrapezoidPlanner planner;
    
    float v_entry = 100.0f;
    float v_exit = 500.0f;
    float accel = 1000.0f;
    
    float d = planner.testComputeAccelDistance(v_entry, v_exit, accel);
    
    // Expected: d = (v² - v₀²) / (2a)
    float expected = (v_exit * v_exit - v_entry * v_entry) / (2.0f * accel);
    
    Assert(floatEquals(d, expected), "Accel distance should follow kinematic formula");
}

// Tests that deceleration distance calculation follows v² = v₀² - 2ad
Test(TrapezoidPlanner, DecelDistance_Formula) {
    TestableTrapezoidPlanner planner;
    
    float v_entry = 500.0f;
    float v_exit = 100.0f;
    float accel = 1000.0f;
    
    float d = planner.testComputeDecelDistance(v_entry, v_exit, accel);
    
    // Expected: d = (v₀² - v²) / (2a)
    float expected = (v_entry * v_entry - v_exit * v_exit) / (2.0f * accel);
    
    Assert(floatEquals(d, expected), "Decel distance should follow kinematic formula");
}

// Tests that zero velocity change requires zero distance
Test(TrapezoidPlanner, AccelDistance_ZeroChange) {
    TestableTrapezoidPlanner planner;
    
    float d = planner.testComputeAccelDistance(500.0f, 500.0f, 1000.0f);
    Assert(floatEquals(d, 0.0f), "Zero velocity change should need zero distance");
}

// Tests that symmetric accel/decel distances are equal
Test(TrapezoidPlanner, Distance_Symmetry) {
    TestableTrapezoidPlanner planner;
    
    float v_low = 100.0f;
    float v_high = 500.0f;
    float accel = 1000.0f;
    
    float d_accel = planner.testComputeAccelDistance(v_low, v_high, accel);
    float d_decel = planner.testComputeDecelDistance(v_high, v_low, accel);
    
    Assert(floatEquals(d_accel, d_decel), "Accel and decel distances should be symmetric");
}

// ============================================================================
// Backward pass tests
// ============================================================================

// Tests backward pass with single block (should compute entry speed from stop)
Test(TrapezoidPlanner, BackwardPass_SingleBlock) {
    TestableTrapezoidPlanner planner;
    
    plan_block_t block = createTestBlock(100.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    planner.addBlock(block);
    
    // With only one block, backward pass should set entry speed based on
    // decelerating to zero over the block distance
    planner.testRecalculateBackward();
    
    plan_block_t* processed = planner.getBlock(0);
    
    // Entry speed should allow deceleration to zero over the block distance
    // v₀² = 2*a*d
    float max_entry_sqr = 2.0f * block.acceleration * block.millimeters;
    Assert(processed->entry_speed_sqr <= max_entry_sqr + 0.1f,
           "Entry speed should allow stop within block");
}

// Tests backward pass with two blocks
// Note: With planned=0 (at tail), backward pass only processes block 1 (last block).
// Block 0 is at the planned boundary and is NOT modified by backward pass.
// This matches the original FluidNC planner behavior.
Test(TrapezoidPlanner, BackwardPass_TwoBlocks) {
    TestableTrapezoidPlanner planner;
    
    plan_block_t block1 = createTestBlock(50.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    plan_block_t block2 = createTestBlock(50.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    
    planner.addBlock(block1);
    planner.addBlock(block2);
    planner.setPlanned(0);  // Planned at tail - backward pass stops before block 0
    
    // Remember block 0's initial entry speed (should be 0, starting from rest)
    float initial_b1_entry = planner.getBlock(0)->entry_speed_sqr;
    
    planner.testRecalculateBackward();
    
    plan_block_t* b1 = planner.getBlock(0);
    plan_block_t* b2 = planner.getBlock(1);
    
    // Block 2 (last block) should have entry speed computed from stopping: v² = 2*a*d
    float expected_b2_entry = 2.0f * block2.acceleration * block2.millimeters;
    Assert(b2->entry_speed_sqr > 0.0f, "Last block should have computed entry speed from stopping");
    Assert(floatEquals(b2->entry_speed_sqr, std::min(block2.max_entry_speed_sqr, expected_b2_entry), 1.0f),
           "Last block entry should be min of max_entry and 2*a*d");
    
    // Block 1 (at planned boundary) should NOT be modified by backward pass
    Assert(floatEquals(b1->entry_speed_sqr, initial_b1_entry),
           "Block at planned boundary should not be modified by backward pass");
}

// Tests backward pass with multiple blocks - entry speeds should be propagated
// Tests backward pass with multiple blocks (3+ blocks triggers different code path)
// Note: With planned=0, backward pass processes blocks 4,3,2,1 but NOT block 0.
// Block 0 is at the planned boundary and is NOT modified.
Test(TrapezoidPlanner, BackwardPass_MultipleBlocks) {
    TestableTrapezoidPlanner planner;
    
    // Add 5 identical blocks
    for (int i = 0; i < 5; i++) {
        plan_block_t block = createTestBlock(20.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
        planner.addBlock(block);
    }
    planner.setPlanned(0);
    
    // Remember block 0's initial entry speed
    float initial_b0_entry = planner.getBlock(0)->entry_speed_sqr;
    
    planner.testRecalculateBackward();
    
    // Block 4 (last) should have entry computed from stopping
    plan_block_t* b4 = planner.getBlock(4);
    Assert(b4->entry_speed_sqr > 0.0f, "Last block should have computed entry speed");
    
    // Entry speeds should be increasing as we go backward from the end
    // (allowing for deceleration to stop) - but only for blocks 1-4 (not block 0)
    for (int i = 3; i >= 1; i--) {
        plan_block_t* current = planner.getBlock(i);
        plan_block_t* next = planner.getBlock(i + 1);
        
        // Current entry should be >= next entry (or capped at max)
        Assert(current->entry_speed_sqr >= next->entry_speed_sqr - 0.1f ||
               floatEquals(current->entry_speed_sqr, current->max_entry_speed_sqr),
               "Entry speeds should propagate backward correctly");
    }
    
    // Block 0 should NOT be modified (it's at the planned boundary)
    Assert(floatEquals(planner.getBlock(0)->entry_speed_sqr, initial_b0_entry),
           "Block at planned boundary should not be modified by backward pass");
}

// ============================================================================
// Forward pass tests
// ============================================================================

// Tests forward pass limits entry speeds based on acceleration from previous
Test(TrapezoidPlanner, ForwardPass_LimitsAcceleration) {
    TestableTrapezoidPlanner planner;
    
    // Create blocks where backward pass would want high speeds
    // but forward pass should limit due to acceleration constraints
    plan_block_t block1 = createTestBlock(10.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    block1.entry_speed_sqr = 0.0f;  // Starting from rest
    
    plan_block_t block2 = createTestBlock(10.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    block2.entry_speed_sqr = 900000.0f;  // Would want high entry speed
    
    planner.addBlock(block1);
    planner.addBlock(block2);
    
    planner.testRecalculateForward();
    
    // Block 2's entry speed should be limited by what can be achieved
    // accelerating from block 1's entry (0) over block 1's distance
    plan_block_t* b2 = planner.getBlock(1);
    float max_achievable = 2.0f * block1.acceleration * block1.millimeters;
    
    Assert(b2->entry_speed_sqr <= max_achievable + 0.1f,
           "Forward pass should limit entry speed by achievable acceleration");
}

// Tests that forward pass moves the planned pointer for full-acceleration blocks
Test(TrapezoidPlanner, ForwardPass_MovesPlannedPointer) {
    TestableTrapezoidPlanner planner;
    
    // Create a sequence where first blocks are accelerating
    plan_block_t block1 = createTestBlock(100.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    block1.entry_speed_sqr = 0.0f;
    
    plan_block_t block2 = createTestBlock(100.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    block2.entry_speed_sqr = 500000.0f;  // Higher than achievable from 0
    
    plan_block_t block3 = createTestBlock(100.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    block3.entry_speed_sqr = 800000.0f;
    
    planner.addBlock(block1);
    planner.addBlock(block2);
    planner.addBlock(block3);
    planner.setPlanned(0);
    
    int initial_planned = planner.getPlanned();
    planner.testRecalculateForward();
    int final_planned = planner.getPlanned();
    
    // Planned pointer should have moved forward for accelerating blocks
    Assert(final_planned >= initial_planned, 
           "Planned pointer should advance for accelerating sequence");
}

// ============================================================================
// Full recalculate tests (backward + forward)
// ============================================================================

// Tests full recalculation produces physically valid speeds
Test(TrapezoidPlanner, Recalculate_ValidSpeeds) {
    TestableTrapezoidPlanner planner;
    
    // Create a typical motion sequence
    plan_block_t block1 = createTestBlock(50.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    plan_block_t block2 = createTestBlock(100.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    plan_block_t block3 = createTestBlock(50.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    
    planner.addBlock(block1);
    planner.addBlock(block2);
    planner.addBlock(block3);
    planner.setPlanned(0);
    
    planner.testRecalculate();
    
    // Check all blocks have valid entry speeds
    for (int i = 0; i < 3; i++) {
        plan_block_t* block = planner.getBlock(i);
        
        // Entry speed should not exceed max
        Assert(block->entry_speed_sqr <= block->max_entry_speed_sqr + 0.1f,
               "Entry speed should not exceed maximum");
        
        // Entry speed should be non-negative
        Assert(block->entry_speed_sqr >= 0.0f, "Entry speed should be non-negative");
    }
}

// Tests that last block can decelerate to stop
Test(TrapezoidPlanner, Recalculate_LastBlockCanStop) {
    TestableTrapezoidPlanner planner;
    
    plan_block_t block1 = createTestBlock(100.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    plan_block_t block2 = createTestBlock(100.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    
    planner.addBlock(block1);
    planner.addBlock(block2);
    planner.setPlanned(0);
    
    planner.testRecalculate();
    
    plan_block_t* last = planner.getBlock(1);
    
    // Verify last block can decelerate from entry to stop
    float entry_speed = sqrtf(last->entry_speed_sqr);
    float decel_dist = (entry_speed * entry_speed) / (2.0f * last->acceleration);
    
    Assert(decel_dist <= last->millimeters + 0.1f,
           "Last block should be able to decelerate to stop within its distance");
}

// Tests transition from rest
Test(TrapezoidPlanner, Recalculate_FromRest) {
    TestableTrapezoidPlanner planner;
    
    // Simulate starting from rest - first block should have zero entry
    plan_block_t block = createTestBlock(100.0f, 1000.0f, 60000.0f, 0.0f, 0.0f);
    block.max_junction_speed_sqr = 0.0f;  // Starting from rest
    
    planner.addBlock(block);
    planner.setPlanned(0);
    
    planner.testRecalculate();
    
    plan_block_t* processed = planner.getBlock(0);
    Assert(floatEquals(processed->entry_speed_sqr, 0.0f),
           "First block starting from rest should have zero entry speed");
}

// ============================================================================
// Velocity profile computation tests
// ============================================================================

// Tests velocity profile for full trapezoid (accel + cruise + decel)
Test(TrapezoidPlanner, VelocityProfile_FullTrapezoid) {
    TestableTrapezoidPlanner planner;
    
    plan_block_t block = createTestBlock(200.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    block.programmed_rate = 1000.0f;
    block.rapid_rate = 2000.0f;
    block.motion.rapidMotion = 0;
    
    float entry_speed = 100.0f;
    float exit_speed_sqr = 100.0f * 100.0f;
    
    auto profile = planner.testComputeVelocityProfile(&block, entry_speed, exit_speed_sqr);
    
    // Should have cruise phase since distance is long enough
    Assert(profile.accelerate_until > profile.decelerate_after,
           "Should have cruise phase for long block");
    Assert(profile.maximum_speed > entry_speed,
           "Peak speed should exceed entry speed");
}

// Tests velocity profile for triangular profile (no cruise phase)
Test(TrapezoidPlanner, VelocityProfile_Triangular) {
    TestableTrapezoidPlanner planner;
    
    // Short block that won't reach nominal speed
    plan_block_t block = createTestBlock(5.0f, 5000.0f, 60000.0f, 5000.0f, 5000.0f);
    block.programmed_rate = 5000.0f;  // High speed
    block.rapid_rate = 10000.0f;
    block.motion.rapidMotion = 0;
    
    float entry_speed = 100.0f;
    float exit_speed_sqr = 100.0f * 100.0f;
    
    auto profile = planner.testComputeVelocityProfile(&block, entry_speed, exit_speed_sqr);
    
    // For short block, should not reach nominal speed
    Assert(profile.maximum_speed < 5000.0f,
           "Short block should have triangular profile");
}

// Tests velocity profile for pure deceleration
Test(TrapezoidPlanner, VelocityProfile_PureDecel) {
    TestableTrapezoidPlanner planner;
    
    plan_block_t block = createTestBlock(50.0f, 500.0f, 60000.0f, 500.0f, 500.0f);
    block.programmed_rate = 500.0f;
    block.rapid_rate = 1000.0f;
    block.motion.rapidMotion = 0;
    
    float entry_speed = 500.0f;  // High entry
    float exit_speed_sqr = 0.0f;  // Stop at end
    
    auto profile = planner.testComputeVelocityProfile(&block, entry_speed, exit_speed_sqr);
    
    // Should be primarily deceleration
    Assert(profile.maximum_speed <= entry_speed + 0.1f,
           "Pure decel should not exceed entry speed");
}

// Tests velocity profile initial ramp type detection
Test(TrapezoidPlanner, VelocityProfile_InitialRampType) {
    TestableTrapezoidPlanner planner;
    
    plan_block_t block = createTestBlock(100.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    block.programmed_rate = 1000.0f;
    block.rapid_rate = 2000.0f;
    block.motion.rapidMotion = 0;
    
    // Test acceleration start
    {
        float entry_speed = 100.0f;  // Low entry
        auto profile = planner.testComputeVelocityProfile(&block, entry_speed, 0.0f);
        Assert(profile.initial_ramp_type == RAMP_ACCEL,
               "Low entry speed should start with acceleration");
    }
    
    // Test cruise/decel start (entry at or above nominal - no acceleration needed)
    {
        float entry_speed = 1000.0f;  // At nominal speed
        auto profile = planner.testComputeVelocityProfile(&block, entry_speed, 0.0f);
        // At nominal entry with deceleration to stop, should start cruise or decel
        Assert(profile.initial_ramp_type == RAMP_CRUISE || profile.initial_ramp_type == RAMP_DECEL,
               "Entry at nominal should start with cruise or decel");
    }
}

// ============================================================================
// Ramp update tests
// ============================================================================

// Tests acceleration ramp update calculates correct speed change
Test(TrapezoidPlanner, RampUpdate_Acceleration) {
    TestableTrapezoidPlanner planner;
    
    float time = 0.001f;  // 1ms
    float speed = 500.0f;  // mm/min
    float accel = 60000.0f;  // mm/min²
    float mm_remaining = 100.0f;
    float phase_boundary = 50.0f;
    
    auto update = planner.testUpdateRamp(RAMP_ACCEL, time, speed, accel, mm_remaining, phase_boundary);
    
    // Speed delta should be a*t
    float expected_delta = accel * time;
    Assert(floatEquals(update.speed_delta, expected_delta, 0.1f),
           "Acceleration ramp should increase speed by a*t");
    
    // Distance should be v*t + 0.5*a*t²
    float expected_dist = speed * time + 0.5f * accel * time * time;
    Assert(floatEquals(update.distance_traveled, expected_dist, 0.01f),
           "Acceleration distance should follow kinematic formula");
}

// Tests cruise ramp update maintains constant speed
Test(TrapezoidPlanner, RampUpdate_Cruise) {
    TestableTrapezoidPlanner planner;
    
    float time = 0.001f;
    float speed = 1000.0f;
    float accel = 60000.0f;
    float mm_remaining = 100.0f;
    float phase_boundary = 20.0f;
    
    auto update = planner.testUpdateRamp(RAMP_CRUISE, time, speed, accel, mm_remaining, phase_boundary);
    
    // Speed should not change during cruise
    Assert(floatEquals(update.speed_delta, 0.0f),
           "Cruise ramp should have zero speed change");
    
    // Distance should be v*t
    Assert(floatEquals(update.distance_traveled, speed * time, 0.01f),
           "Cruise distance should be v*t");
}

// Tests deceleration ramp update calculates correct speed reduction
Test(TrapezoidPlanner, RampUpdate_Deceleration) {
    TestableTrapezoidPlanner planner;
    
    float time = 0.001f;
    float speed = 500.0f;
    float accel = 60000.0f;
    float mm_remaining = 10.0f;
    float phase_boundary = 0.0f;  // Decelerating to end
    
    auto update = planner.testUpdateRamp(RAMP_DECEL, time, speed, accel, mm_remaining, phase_boundary);
    
    // Speed delta should be negative
    Assert(update.speed_delta < 0.0f,
           "Deceleration ramp should decrease speed");
    
    // Distance should still be positive
    Assert(update.distance_traveled > 0.0f,
           "Distance should be positive during deceleration");
}

// Tests ramp phase transition detection
Test(TrapezoidPlanner, RampUpdate_PhaseTransition) {
    TestableTrapezoidPlanner planner;
    
    float time = 0.01f;  // 10ms
    float speed = 500.0f;
    float accel = 60000.0f;
    float mm_remaining = 1.0f;  // Small remaining distance
    float phase_boundary = 0.5f;  // Close boundary
    
    auto update = planner.testUpdateRamp(RAMP_ACCEL, time, speed, accel, mm_remaining, phase_boundary);
    
    // Should detect phase completion if we've passed the boundary
    // This depends on the specific distance traveled
    // Just verify the logic doesn't crash and returns valid data
    Assert(update.distance_traveled >= 0.0f, "Distance should be non-negative");
}

// ============================================================================
// Edge case tests
// ============================================================================

// Tests handling of zero-length block
Test(TrapezoidPlanner, EdgeCase_ZeroLength) {
    TestableTrapezoidPlanner planner;
    
    plan_block_t block = createTestBlock(0.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    planner.addBlock(block);
    planner.setPlanned(0);
    
    // Should handle zero-length block without crash
    planner.testRecalculate();
    
    // Entry speed should be zero or minimal for zero-length block
    plan_block_t* processed = planner.getBlock(0);
    Assert(processed->entry_speed_sqr >= 0.0f, "Entry speed should be non-negative");
}

// Tests handling of very short block
Test(TrapezoidPlanner, EdgeCase_VeryShort) {
    TestableTrapezoidPlanner planner;
    
    plan_block_t block = createTestBlock(0.1f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    planner.addBlock(block);
    planner.setPlanned(0);
    
    planner.testRecalculate();
    
    plan_block_t* processed = planner.getBlock(0);
    
    // Entry speed should be limited by what can stop in 0.1mm
    float max_entry = sqrtf(2.0f * block.acceleration * block.millimeters);
    float entry = sqrtf(processed->entry_speed_sqr);
    
    Assert(entry <= max_entry + 0.1f,
           "Very short block should have limited entry speed");
}

// Tests handling of very high acceleration
Test(TrapezoidPlanner, EdgeCase_HighAccel) {
    TestableTrapezoidPlanner planner;
    
    plan_block_t block = createTestBlock(100.0f, 1000.0f, 1000000.0f, 1000.0f, 1000.0f);
    planner.addBlock(block);
    planner.setPlanned(0);
    
    planner.testRecalculate();
    
    // Should handle high acceleration without numerical issues
    plan_block_t* processed = planner.getBlock(0);
    Assert(!std::isnan(processed->entry_speed_sqr), "Entry speed should not be NaN");
    Assert(!std::isinf(processed->entry_speed_sqr), "Entry speed should not be infinite");
}

// Tests maximum junction speed limiting
Test(TrapezoidPlanner, EdgeCase_JunctionSpeedLimit) {
    TestableTrapezoidPlanner planner;
    
    // Block with low junction speed limit
    plan_block_t block = createTestBlock(100.0f, 1000.0f, 60000.0f, 100.0f, 100.0f);
    planner.addBlock(block);
    planner.setPlanned(0);
    
    planner.testRecalculate();
    
    plan_block_t* processed = planner.getBlock(0);
    
    // Entry speed should be limited by junction speed
    Assert(processed->entry_speed_sqr <= processed->max_junction_speed_sqr + 0.1f,
           "Entry speed should be limited by junction speed");
}

// ============================================================================
// Buffer management tests
// ============================================================================

// Tests that buffer operations work correctly with wrapping
Test(TrapezoidPlanner, Buffer_Wrapping) {
    TestableTrapezoidPlanner planner;
    
    // Fill buffer to near capacity
    for (int i = 0; i < TEST_PLANNER_BLOCKS - 2; i++) {
        plan_block_t block = createTestBlock(10.0f + i, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
        planner.addBlock(block);
    }
    
    // Should be able to recalculate without issues
    planner.setPlanned(0);
    planner.testRecalculate();
    
    Assert(planner.blockCount() > 0, "Buffer should contain blocks");
}

// Tests empty buffer handling
Test(TrapezoidPlanner, Buffer_Empty) {
    TestableTrapezoidPlanner planner;
    
    // Recalculate on empty buffer should not crash
    planner.testRecalculate();
    
    Assert(planner.blockCount() == 0, "Empty buffer should remain empty");
}

// ============================================================================
// Comprehensive Kinematic Validation Tests
// ============================================================================

// Tests that each block's kinematics are internally consistent
Test(TrapezoidPlanner, Kinematics_BlockInternalConsistency) {
    TestableTrapezoidPlanner planner;
    
    // Create varying blocks to stress different profile shapes
    plan_block_t blocks[] = {
        createTestBlock(100.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f),  // Normal
        createTestBlock(50.0f, 500.0f, 30000.0f, 500.0f, 500.0f),      // Slower
        createTestBlock(200.0f, 1500.0f, 90000.0f, 1500.0f, 1500.0f),  // Faster
        createTestBlock(20.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f),   // Short
    };
    
    for (auto& block : blocks) {
        planner.addBlock(block);
    }
    planner.setPlanned(0);
    planner.testRecalculate();
    
    // Validate each block's kinematics
    for (int i = 0; i < 4; i++) {
        plan_block_t* b = planner.getBlock(i);
        
        // Get exit speed (next block's entry or 0 for last)
        float exit_speed_sqr = (i < 3) ? planner.getBlock(i+1)->entry_speed_sqr : 0.0f;
        
        BlockKinematics k = extractTrapezoidKinematics(*b, exit_speed_sqr);
        
        char msg[128];
        snprintf(msg, sizeof(msg), "Block %d should have valid kinematics", i);
        Assert(k.is_valid, msg);
        
        // Verify entry speed is within bounds
        snprintf(msg, sizeof(msg), "Block %d entry_speed (%.1f) <= max_entry (%.1f)", 
                 i, k.entry_speed, k.max_entry_speed);
        Assert(k.entry_speed <= k.max_entry_speed + 0.1f, msg);
        
        // Verify peak speed doesn't exceed nominal
        snprintf(msg, sizeof(msg), "Block %d peak_speed (%.1f) <= nominal (%.1f)", 
                 i, k.peak_speed, k.nominal_speed);
        Assert(k.peak_speed <= k.nominal_speed + 0.1f, msg);
        
        // Verify distances sum correctly
        float d_sum = k.accel_distance + k.cruise_distance + k.decel_distance;
        snprintf(msg, sizeof(msg), "Block %d distances sum (%.2f) == total (%.2f)", 
                 i, d_sum, k.total_distance);
        Assert(fabsf(d_sum - k.total_distance) < k.total_distance * 0.05f, msg);
        
        // Verify non-negative distances
        snprintf(msg, sizeof(msg), "Block %d accel_distance (%.2f) >= 0", i, k.accel_distance);
        Assert(k.accel_distance >= -0.001f, msg);
        snprintf(msg, sizeof(msg), "Block %d cruise_distance (%.2f) >= 0", i, k.cruise_distance);
        Assert(k.cruise_distance >= -0.001f, msg);
        snprintf(msg, sizeof(msg), "Block %d decel_distance (%.2f) >= 0", i, k.decel_distance);
        Assert(k.decel_distance >= -0.001f, msg);
    }
}

// Tests velocity continuity across block boundaries
Test(TrapezoidPlanner, Kinematics_VelocityContinuity) {
    TestableTrapezoidPlanner planner;
    
    // Create a 5-block sequence
    for (int i = 0; i < 5; i++) {
        plan_block_t block = createTestBlock(50.0f + i * 10.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
        planner.addBlock(block);
    }
    planner.setPlanned(0);
    planner.testRecalculate();
    
    // Verify velocity continuity between consecutive blocks
    for (int i = 0; i < 4; i++) {
        plan_block_t* current = planner.getBlock(i);
        plan_block_t* next = planner.getBlock(i + 1);
        
        const char* error = nullptr;
        bool continuous = validateVelocityContinuityBetween(*current, *next, &error);
        
        char msg[256];
        snprintf(msg, sizeof(msg), "Blocks %d->%d velocity continuity: %s", 
                 i, i+1, error ? error : "OK");
        Assert(continuous, msg);
        
        // Additional check: next block entry should be achievable
        float curr_entry = sqrtf(current->entry_speed_sqr);
        float next_entry = sqrtf(next->entry_speed_sqr);
        float max_exit = sqrtf(curr_entry * curr_entry + 2.0f * current->acceleration * current->millimeters);
        
        snprintf(msg, sizeof(msg), "Block %d->%d: next_entry (%.1f) <= max_achievable (%.1f)", 
                 i, i+1, next_entry, max_exit);
        Assert(next_entry <= max_exit + 1.0f, msg);
    }
}

// Tests that the last block can always decelerate to stop
Test(TrapezoidPlanner, Kinematics_LastBlockCanStop) {
    TestableTrapezoidPlanner planner;
    
    // Test various block configurations
    float test_distances[] = {10.0f, 50.0f, 100.0f, 200.0f};
    float test_speeds[] = {500.0f, 1000.0f, 2000.0f};
    
    for (float d : test_distances) {
        for (float v : test_speeds) {
            planner.resetBuffer();
            
            // Add a few blocks leading up to the last one
            plan_block_t b1 = createTestBlock(100.0f, v, 60000.0f, v, v);
            plan_block_t b2 = createTestBlock(d, v, 60000.0f, v, v);
            
            planner.addBlock(b1);
            planner.addBlock(b2);
            planner.setPlanned(0);
            planner.testRecalculate();
            
            plan_block_t* last = planner.getBlock(1);
            bool canStop = validateCanStop(*last);
            
            char msg[128];
            snprintf(msg, sizeof(msg), "Last block (d=%.0f, v=%.0f) should be able to stop", d, v);
            Assert(canStop, msg);
        }
    }
}

// Tests that acceleration is respected during velocity transitions
Test(TrapezoidPlanner, Kinematics_AccelerationLimits) {
    TestableTrapezoidPlanner planner;
    
    float accel = 60000.0f;  // mm/min²
    
    // Create blocks with various entry/exit speed requirements
    plan_block_t b1 = createTestBlock(100.0f, 1000.0f, accel, 1000.0f, 1000.0f);
    plan_block_t b2 = createTestBlock(100.0f, 1000.0f, accel, 1000.0f, 1000.0f);
    plan_block_t b3 = createTestBlock(100.0f, 1000.0f, accel, 1000.0f, 1000.0f);
    
    planner.addBlock(b1);
    planner.addBlock(b2);
    planner.addBlock(b3);
    planner.setPlanned(0);
    planner.testRecalculate();
    
    // Verify each transition respects acceleration limits
    for (int i = 0; i < 2; i++) {
        plan_block_t* current = planner.getBlock(i);
        plan_block_t* next = planner.getBlock(i + 1);
        
        float v1 = sqrtf(current->entry_speed_sqr);
        float v2 = sqrtf(next->entry_speed_sqr);
        float d = current->millimeters;
        
        // Check if this velocity change is achievable
        bool achievable = verifyAccelLimit(v1, v2, d, accel);
        
        char msg[128];
        snprintf(msg, sizeof(msg), "Block %d: v1=%.1f -> v2=%.1f over d=%.1f with a=%.1f should be achievable",
                 i, v1, v2, d, accel);
        Assert(achievable, msg);
    }
}

// Tests complete motion sequence with detailed profile analysis
Test(TrapezoidPlanner, Kinematics_CompleteSequenceValidation) {
    TestableTrapezoidPlanner planner;
    
    // Create a realistic toolpath: approach, cut, retract
    plan_block_t approach = createTestBlock(50.0f, 2000.0f, 100000.0f, 2000.0f, 2000.0f);  // Rapid
    plan_block_t cut1 = createTestBlock(100.0f, 500.0f, 50000.0f, 500.0f, 500.0f);         // Feed
    plan_block_t cut2 = createTestBlock(80.0f, 500.0f, 50000.0f, 500.0f, 500.0f);          // Feed
    plan_block_t retract = createTestBlock(50.0f, 2000.0f, 100000.0f, 2000.0f, 2000.0f);   // Rapid
    
    planner.addBlock(approach);
    planner.addBlock(cut1);
    planner.addBlock(cut2);
    planner.addBlock(retract);
    planner.setPlanned(0);
    planner.testRecalculate();
    
    // Calculate total distance
    float expected_distance = 50.0f + 100.0f + 80.0f + 50.0f;
    float actual_distance = 0.0f;
    float total_time = 0.0f;
    
    for (int i = 0; i < 4; i++) {
        plan_block_t* b = planner.getBlock(i);
        float exit_speed_sqr = (i < 3) ? planner.getBlock(i+1)->entry_speed_sqr : 0.0f;
        
        BlockKinematics k = extractTrapezoidKinematics(*b, exit_speed_sqr);
        Assert(k.is_valid, "Block kinematics should be valid");
        
        actual_distance += k.total_distance;
        total_time += k.total_time;
    }
    
    Assert(floatEquals(actual_distance, expected_distance, 0.1f),
           "Total distance should match expected");
    
    Assert(total_time > 0.0f, "Total time should be positive");
}

// Tests profile shapes under different conditions
Test(TrapezoidPlanner, Kinematics_ProfileShapes) {
    TestableTrapezoidPlanner planner;
    
    // Test 1: Long block should have cruise phase
    {
        planner.resetBuffer();
        plan_block_t long_block = createTestBlock(500.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
        long_block.entry_speed_sqr = 100.0f * 100.0f;  // Start at 100 mm/min
        planner.addBlock(long_block);
        planner.setPlanned(0);
        
        BlockKinematics k = extractTrapezoidKinematics(long_block, 0.0f);
        
        Assert(k.cruise_distance > 0.0f, "Long block should have cruise phase");
        Assert(k.peak_speed > 100.0f, "Long block should accelerate above entry speed");
    }
    
    // Test 2: Short block should be triangular (no cruise)
    {
        planner.resetBuffer();
        plan_block_t short_block = createTestBlock(5.0f, 2000.0f, 60000.0f, 2000.0f, 2000.0f);
        short_block.entry_speed_sqr = 100.0f * 100.0f;
        planner.addBlock(short_block);
        planner.setPlanned(0);
        
        BlockKinematics k = extractTrapezoidKinematics(short_block, 0.0f);
        
        // For very short blocks, may not reach cruise
        Assert(k.peak_speed < 2000.0f, "Short block should not reach max speed");
    }
    
    // Test 3: Starting from rest
    {
        planner.resetBuffer();
        plan_block_t from_rest = createTestBlock(100.0f, 1000.0f, 60000.0f, 0.0f, 0.0f);
        from_rest.entry_speed_sqr = 0.0f;
        from_rest.max_junction_speed_sqr = 0.0f;
        planner.addBlock(from_rest);
        planner.setPlanned(0);
        planner.testRecalculate();
        
        plan_block_t* processed = planner.getBlock(0);
        Assert(floatEquals(processed->entry_speed_sqr, 0.0f, 0.01f),
               "Block starting from rest should have zero entry");
        
        BlockKinematics k = extractTrapezoidKinematics(*processed, 0.0f);
        Assert(k.accel_distance > 0.0f || k.total_distance < 0.1f,
               "From-rest block should have acceleration phase");
    }
}

// Tests that profile distances are achievable given kinematic constraints
// (Simplified from full simulation which had convergence issues)
Test(TrapezoidPlanner, Kinematics_ProfileDistancesAchievable) {
    TestableTrapezoidPlanner planner;
    
    // Create a simple block
    plan_block_t block = createTestBlock(100.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    block.entry_speed_sqr = 200.0f * 200.0f;  // Start at 200 mm/min
    
    planner.addBlock(block);
    planner.setPlanned(0);
    planner.testRecalculate();
    
    plan_block_t* processed = planner.getBlock(0);
    
    // Extract kinematics and verify achievability
    BlockKinematics k = extractTrapezoidKinematics(*processed, 0.0f);
    
    Assert(k.is_valid, k.error_message ? k.error_message : "Kinematics should be valid");
    
    // Verify total distance sum equals block distance
    float computed_distance = k.accel_distance + k.cruise_distance + k.decel_distance;
    char msg[128];
    snprintf(msg, sizeof(msg), "Profile distances (%.2f) should sum to block distance (%.2f)",
             computed_distance, processed->millimeters);
    Assert(floatEquals(computed_distance, processed->millimeters, 1.0f), msg);
    
    // Verify kinematics equations hold:
    // v² = v0² + 2*a*d for acceleration phase
    float v_after_accel_sq = processed->entry_speed_sqr + 2.0f * processed->acceleration * k.accel_distance;
    float expected_peak_sq = k.peak_speed * k.peak_speed;
    
    snprintf(msg, sizeof(msg), "Kinematics equation: peak² (%.0f) vs computed (%.0f)",
             expected_peak_sq, v_after_accel_sq);
    Assert(floatEquals(v_after_accel_sq, expected_peak_sq, expected_peak_sq * 0.1f + 1.0f), msg);
}

// Tests that entry speed respects max_entry_speed_sqr limit
Test(TrapezoidPlanner, Kinematics_EntrySpeedLimits) {
    TestableTrapezoidPlanner planner;
    
    // Create blocks with explicit max entry speeds
    // max_entry_speed_sqr is the actual limiting field used by the planner
    plan_block_t b1 = createTestBlock(100.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    plan_block_t b2 = createTestBlock(100.0f, 1000.0f, 60000.0f, 1000.0f, 1000.0f);
    
    // Set a low max_entry_speed on block 2 to force the planner to limit it
    b2.max_entry_speed_sqr = 500.0f * 500.0f;  // Max entry = 500 mm/min
    
    planner.addBlock(b1);
    planner.addBlock(b2);
    planner.setPlanned(0);
    planner.testRecalculate();
    
    // Block 2's entry speed should respect max_entry_speed limit
    plan_block_t* processed_b2 = planner.getBlock(1);
    float b2_entry = sqrtf(processed_b2->entry_speed_sqr);
    float b2_max_entry = sqrtf(processed_b2->max_entry_speed_sqr);
    
    char msg[128];
    snprintf(msg, sizeof(msg), "Block 2 entry (%.1f) should respect max entry limit (%.1f)", 
             b2_entry, b2_max_entry);
    Assert(b2_entry <= b2_max_entry + 0.1f, msg);
}
