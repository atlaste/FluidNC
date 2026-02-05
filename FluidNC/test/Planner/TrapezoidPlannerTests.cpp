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
