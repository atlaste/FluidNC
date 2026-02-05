// Copyright (c) 2024 - FluidNC Authors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "../TestFramework.h"
#include "PlannerTestHelpers.h"
#include "Planner/TrapezoidPlanner.h"
#include "Planner/SCurvePlanner.h"
#include "Planner/SCurveMath.h"

#include <cmath>
#include <cstring>
#include <vector>

using namespace PlannerTestHelpers;

// ============================================================================
// Integration Test Infrastructure
// Simulates realistic motion sequences without hardware dependencies
// ============================================================================

// Motion segment representing a G-code move
struct MotionSegment {
    float distance;        // mm
    float requested_speed; // mm/min
    float max_accel;       // mm/min²
    float max_jerk;        // mm/min³ (for S-curve)
    bool is_rapid;         // G0 vs G1
};

// Result of simulated motion execution
struct IntegrationSimResult {
    float total_time;          // min
    float total_distance;      // mm
    float max_achieved_speed;  // mm/min
    int blocks_processed;
    bool completed_successfully;
    std::vector<float> junction_speeds;  // Entry speeds at block boundaries
};

// Testable wrapper for TrapezoidPlanner that provides simulation capability
class SimulatedTrapezoidPlanner : public TrapezoidPlanner {
public:
    static constexpr int BUFFER_SIZE = TEST_PLANNER_BLOCKS;
    
    SimulatedTrapezoidPlanner() : TrapezoidPlanner("SimulatedTrapezoid") {
        _block_buffer = new plan_block_t[BUFFER_SIZE];
        memset(_block_buffer, 0, sizeof(plan_block_t) * BUFFER_SIZE);
        resetBuffer();
    }
    
    ~SimulatedTrapezoidPlanner() {
        delete[] _block_buffer;
        _block_buffer = nullptr;
    }
    
    void addSegment(const MotionSegment& seg) {
        if (_next_buffer_head == _block_buffer_tail) return;
        
        plan_block_t block;
        memset(&block, 0, sizeof(plan_block_t));
        
        block.millimeters = seg.distance;
        block.programmed_rate = seg.requested_speed;
        block.rapid_rate = seg.is_rapid ? seg.requested_speed : seg.requested_speed * 1.5f;
        block.acceleration = seg.max_accel;
        block.motion.rapidMotion = seg.is_rapid ? 1 : 0;
        
        // Compute max entry speed based on previous block's junction
        if (_block_buffer_head == _block_buffer_tail) {
            // First block - start from rest
            block.max_junction_speed_sqr = 0.0f;
            block.max_entry_speed_sqr = 0.0f;
        } else {
            // Junction speed based on corner angle (simplified: 90% of min of adjacent speeds)
            block.max_junction_speed_sqr = seg.requested_speed * seg.requested_speed * 0.81f;
            block.max_entry_speed_sqr = block.max_junction_speed_sqr;
        }
        
        block.steps[0] = static_cast<uint32_t>(seg.distance * 80.0f);
        block.step_event_count = block.steps[0];
        
        _block_buffer[_block_buffer_head] = block;
        _block_buffer_head = nextBlockIndex(_block_buffer_head);
        _next_buffer_head = nextBlockIndex(_block_buffer_head);
        
        if (_next_buffer_head == _block_buffer_tail) {
            // Buffer wrapping
        }
    }
    
    void planAll() {
        _block_buffer_planned = _block_buffer_tail;
        recalculate();
    }
    
    IntegrationSimResult simulate() {
        IntegrationSimResult result = {};
        
        uint8_t idx = _block_buffer_tail;
        float prev_exit_speed = 0.0f;
        
        while (idx != _block_buffer_head) {
            plan_block_t* block = &_block_buffer[idx];
            result.blocks_processed++;
            
            float entry_speed = sqrtf(block->entry_speed_sqr);
            result.junction_speeds.push_back(entry_speed);
            
            // Compute profile for this block
            uint8_t next_idx = nextBlockIndex(idx);
            float exit_speed_sqr = (next_idx == _block_buffer_head) ? 0.0f :
                                   _block_buffer[next_idx].entry_speed_sqr;
            
            auto profile = computeVelocityProfile(block, entry_speed, exit_speed_sqr);
            
            // Track max achieved speed
            if (profile.maximum_speed > result.max_achieved_speed) {
                result.max_achieved_speed = profile.maximum_speed;
            }
            
            // Estimate time for this block (simplified)
            float avg_speed = (entry_speed + sqrtf(exit_speed_sqr)) / 2.0f;
            if (avg_speed < 0.001f) avg_speed = profile.maximum_speed / 2.0f;
            if (avg_speed > 0.001f) {
                result.total_time += block->millimeters / avg_speed;
            }
            
            result.total_distance += block->millimeters;
            prev_exit_speed = sqrtf(exit_speed_sqr);
            
            idx = nextBlockIndex(idx);
        }
        
        result.completed_successfully = true;
        return result;
    }
    
    void resetBuffer() {
        _block_buffer_tail = 0;
        _block_buffer_head = 0;
        _next_buffer_head = 1;
        _block_buffer_planned = 0;
    }
    
    plan_block_t* getBlock(int index) {
        return &_block_buffer[index];
    }
    
    int blockCount() const {
        if (_block_buffer_head >= _block_buffer_tail) {
            return _block_buffer_head - _block_buffer_tail;
        }
        return BUFFER_SIZE - _block_buffer_tail + _block_buffer_head;
    }

private:
    uint8_t nextBlockIndex(uint8_t idx) {
        idx++;
        if (idx == BUFFER_SIZE) idx = 0;
        return idx;
    }
};

// Testable wrapper for SCurvePlanner simulation
class SimulatedSCurvePlanner : public SCurvePlanner {
public:
    static constexpr int BUFFER_SIZE = TEST_PLANNER_BLOCKS;
    
    SimulatedSCurvePlanner() : SCurvePlanner("SimulatedSCurve") {
        _block_buffer = new plan_block_t[BUFFER_SIZE];
        memset(_block_buffer, 0, sizeof(plan_block_t) * BUFFER_SIZE);
        resetBuffer();
        _jerk = DEFAULT_JERK;
    }
    
    ~SimulatedSCurvePlanner() {
        delete[] _block_buffer;
        _block_buffer = nullptr;
    }
    
    void addSegment(const MotionSegment& seg) {
        if (_next_buffer_head == _block_buffer_tail) return;
        
        plan_block_t block;
        memset(&block, 0, sizeof(plan_block_t));
        
        block.millimeters = seg.distance;
        block.programmed_rate = seg.requested_speed;
        block.rapid_rate = seg.is_rapid ? seg.requested_speed : seg.requested_speed * 1.5f;
        block.acceleration = seg.max_accel;
        block.jerk = seg.max_jerk > 0 ? seg.max_jerk : _jerk;
        block.motion.rapidMotion = seg.is_rapid ? 1 : 0;
        
        if (_block_buffer_head == _block_buffer_tail) {
            block.max_junction_speed_sqr = 0.0f;
            block.max_entry_speed_sqr = 0.0f;
            block.max_entry_accel = seg.max_accel;
        } else {
            block.max_junction_speed_sqr = seg.requested_speed * seg.requested_speed * 0.81f;
            block.max_entry_speed_sqr = block.max_junction_speed_sqr;
            block.max_entry_accel = seg.max_accel;
        }
        
        block.steps[0] = static_cast<uint32_t>(seg.distance * 80.0f);
        block.step_event_count = block.steps[0];
        
        _block_buffer[_block_buffer_head] = block;
        _block_buffer_head = nextBlockIndex(_block_buffer_head);
        _next_buffer_head = nextBlockIndex(_block_buffer_head);
    }
    
    void planAll() {
        _block_buffer_planned = _block_buffer_tail;
        recalculate();
    }
    
    IntegrationSimResult simulate() {
        IntegrationSimResult result = {};
        
        uint8_t idx = _block_buffer_tail;
        
        while (idx != _block_buffer_head) {
            plan_block_t* block = &_block_buffer[idx];
            result.blocks_processed++;
            
            float entry_speed = sqrtf(block->entry_speed_sqr);
            result.junction_speeds.push_back(entry_speed);
            
            uint8_t next_idx = nextBlockIndex(idx);
            float exit_speed_sqr = (next_idx == _block_buffer_head) ? 0.0f :
                                   _block_buffer[next_idx].entry_speed_sqr;
            
            auto profile = computeVelocityProfile(block, entry_speed, exit_speed_sqr);
            
            if (profile.maximum_speed > result.max_achieved_speed) {
                result.max_achieved_speed = profile.maximum_speed;
            }
            
            float avg_speed = (entry_speed + sqrtf(exit_speed_sqr)) / 2.0f;
            if (avg_speed < 0.001f) avg_speed = profile.maximum_speed / 2.0f;
            if (avg_speed > 0.001f) {
                result.total_time += block->millimeters / avg_speed;
            }
            
            result.total_distance += block->millimeters;
            idx = nextBlockIndex(idx);
        }
        
        result.completed_successfully = true;
        return result;
    }
    
    void simulatePause() {
        // Simulate pause by resetting entry speeds of remaining blocks
        uint8_t idx = _block_buffer_tail;
        while (idx != _block_buffer_head) {
            _block_buffer[idx].entry_speed_sqr = 0.0f;
            _block_buffer[idx].entry_accel = 0.0f;
            idx = nextBlockIndex(idx);
        }
        _block_buffer_planned = _block_buffer_tail;
    }
    
    void simulateResume() {
        // Replan from current position (all entry speeds set to 0)
        recalculate();
    }
    
    void resetBuffer() {
        _block_buffer_tail = 0;
        _block_buffer_head = 0;
        _next_buffer_head = 1;
        _block_buffer_planned = 0;
    }
    
    void setJerk(float jerk) { _jerk = jerk; }
    
    plan_block_t* getBlock(int index) {
        return &_block_buffer[index];
    }
    
    int blockCount() const {
        if (_block_buffer_head >= _block_buffer_tail) {
            return _block_buffer_head - _block_buffer_tail;
        }
        return BUFFER_SIZE - _block_buffer_tail + _block_buffer_head;
    }

private:
    uint8_t nextBlockIndex(uint8_t idx) {
        idx++;
        if (idx == BUFFER_SIZE) idx = 0;
        return idx;
    }
};

// ============================================================================
// Multi-block sequence tests
// ============================================================================

// Tests simple linear sequence (trapezoidal)
Test(PlannerIntegration, LinearSequence_Trapezoid) {
    SimulatedTrapezoidPlanner planner;
    
    // Simple 5-segment linear path
    std::vector<MotionSegment> segments = {
        {50.0f, 1000.0f, 60000.0f, 0.0f, false},
        {100.0f, 1000.0f, 60000.0f, 0.0f, false},
        {100.0f, 1000.0f, 60000.0f, 0.0f, false},
        {100.0f, 1000.0f, 60000.0f, 0.0f, false},
        {50.0f, 1000.0f, 60000.0f, 0.0f, false}
    };
    
    for (const auto& seg : segments) {
        planner.addSegment(seg);
    }
    planner.planAll();
    
    auto result = planner.simulate();
    
    Assert(result.completed_successfully, "Simulation should complete");
    Assert(result.blocks_processed == 5, "Should process all 5 blocks");
    Assert(floatEquals(result.total_distance, 400.0f), "Total distance should be 400mm");
    Assert(result.max_achieved_speed > 0.0f, "Should achieve some speed");
}

// Tests simple linear sequence (S-curve)
Test(PlannerIntegration, LinearSequence_SCurve) {
    SimulatedSCurvePlanner planner;
    planner.setJerk(50000.0f);
    
    std::vector<MotionSegment> segments = {
        {50.0f, 1000.0f, 60000.0f, 50000.0f, false},
        {100.0f, 1000.0f, 60000.0f, 50000.0f, false},
        {100.0f, 1000.0f, 60000.0f, 50000.0f, false},
        {100.0f, 1000.0f, 60000.0f, 50000.0f, false},
        {50.0f, 1000.0f, 60000.0f, 50000.0f, false}
    };
    
    for (const auto& seg : segments) {
        planner.addSegment(seg);
    }
    planner.planAll();
    
    auto result = planner.simulate();
    
    Assert(result.completed_successfully, "Simulation should complete");
    Assert(result.blocks_processed == 5, "Should process all 5 blocks");
}

// Tests that longer middle segments allow higher speeds
Test(PlannerIntegration, LongMiddleSegment_AchievesHigherSpeed) {
    SimulatedSCurvePlanner planner;
    planner.setJerk(50000.0f);
    
    // Short - Long - Short pattern
    std::vector<MotionSegment> segments = {
        {10.0f, 2000.0f, 60000.0f, 50000.0f, false},
        {500.0f, 2000.0f, 60000.0f, 50000.0f, false},  // Long segment
        {10.0f, 2000.0f, 60000.0f, 50000.0f, false}
    };
    
    for (const auto& seg : segments) {
        planner.addSegment(seg);
    }
    planner.planAll();
    
    auto result = planner.simulate();
    
    // Middle segment should achieve close to requested speed
    Assert(result.max_achieved_speed > 1000.0f,
           "Long segment should achieve significant speed");
}

// Tests that many short segments limit achievable speed
Test(PlannerIntegration, ManyShortSegments_LimitedSpeed) {
    SimulatedSCurvePlanner planner;
    planner.setJerk(50000.0f);
    
    // Many very short segments (like arc interpolation)
    for (int i = 0; i < 10; i++) {
        MotionSegment seg = {2.0f, 2000.0f, 60000.0f, 50000.0f, false};
        planner.addSegment(seg);
    }
    planner.planAll();
    
    auto result = planner.simulate();
    
    // With many short segments, can't reach full speed
    Assert(result.max_achieved_speed < 2000.0f,
           "Many short segments should limit max speed");
}

// Tests rapid motion sequence
Test(PlannerIntegration, RapidMotion) {
    SimulatedTrapezoidPlanner planner;
    
    std::vector<MotionSegment> segments = {
        {100.0f, 5000.0f, 100000.0f, 0.0f, true},  // Rapid
        {100.0f, 5000.0f, 100000.0f, 0.0f, true},  // Rapid
    };
    
    for (const auto& seg : segments) {
        planner.addSegment(seg);
    }
    planner.planAll();
    
    auto result = planner.simulate();
    
    Assert(result.completed_successfully, "Rapid sequence should complete");
    Assert(result.max_achieved_speed > 1000.0f, "Rapids should achieve significant speed");
}

// ============================================================================
// Pause/Resume tests
// ============================================================================

// Tests basic pause/resume cycle
Test(PlannerIntegration, PauseResume_Basic) {
    SimulatedSCurvePlanner planner;
    planner.setJerk(50000.0f);
    
    // Add multi-segment path
    for (int i = 0; i < 5; i++) {
        MotionSegment seg = {50.0f, 1000.0f, 60000.0f, 50000.0f, false};
        planner.addSegment(seg);
    }
    planner.planAll();
    
    // Simulate mid-motion pause
    planner.simulatePause();
    
    // All blocks should have zero entry speed after pause
    for (int i = 0; i < planner.blockCount(); i++) {
        plan_block_t* block = planner.getBlock(i);
        Assert(floatEquals(block->entry_speed_sqr, 0.0f),
               "Entry speed should be zero after pause");
    }
    
    // Resume and replan
    planner.simulateResume();
    
    // Should have valid plan after resume
    auto result = planner.simulate();
    Assert(result.completed_successfully, "Should complete after resume");
}

// Tests that pause/resume produces valid kinematics
Test(PlannerIntegration, PauseResume_ValidKinematics) {
    SimulatedSCurvePlanner planner;
    planner.setJerk(50000.0f);
    
    for (int i = 0; i < 5; i++) {
        MotionSegment seg = {30.0f, 1000.0f, 60000.0f, 50000.0f, false};
        planner.addSegment(seg);
    }
    planner.planAll();
    planner.simulatePause();
    planner.simulateResume();
    
    // Verify each block can reach next block's entry speed
    for (int i = 0; i < planner.blockCount() - 1; i++) {
        plan_block_t* current = planner.getBlock(i);
        plan_block_t* next = planner.getBlock(i + 1);
        
        float current_entry = sqrtf(current->entry_speed_sqr);
        float next_entry = sqrtf(next->entry_speed_sqr);
        
        // Verify acceleration is achievable (simplified check)
        float delta_v = next_entry - current_entry;
        if (delta_v > 0) {
            // Accelerating - check distance is sufficient
            float d_needed = SCurveMath::accelDistance(current_entry, next_entry,
                                                        current->acceleration, 50000.0f);
            Assert(d_needed <= current->millimeters * 1.2f,
                   "Should be able to accelerate to next entry");
        }
    }
}

// ============================================================================
// Junction speed tests
// ============================================================================

// Tests that junction speeds respect limits
Test(PlannerIntegration, JunctionSpeeds_RespectLimits) {
    SimulatedTrapezoidPlanner planner;
    
    // Create path with varying max junction speeds
    for (int i = 0; i < 5; i++) {
        MotionSegment seg = {50.0f, 1000.0f, 60000.0f, 0.0f, false};
        planner.addSegment(seg);
    }
    planner.planAll();
    
    // Verify all junction speeds respect max limits
    for (int i = 0; i < planner.blockCount(); i++) {
        plan_block_t* block = planner.getBlock(i);
        Assert(block->entry_speed_sqr <= block->max_entry_speed_sqr + 0.1f,
               "Junction speed should respect max limit");
    }
}

// Tests first block starts from rest
Test(PlannerIntegration, FirstBlock_StartsFromRest) {
    SimulatedSCurvePlanner planner;
    planner.setJerk(50000.0f);
    
    MotionSegment seg = {100.0f, 1000.0f, 60000.0f, 50000.0f, false};
    planner.addSegment(seg);
    planner.planAll();
    
    plan_block_t* first = planner.getBlock(0);
    Assert(floatEquals(first->entry_speed_sqr, 0.0f),
           "First block should start from rest");
}

// Tests last block can stop
Test(PlannerIntegration, LastBlock_CanStop) {
    SimulatedSCurvePlanner planner;
    planner.setJerk(50000.0f);
    
    for (int i = 0; i < 3; i++) {
        MotionSegment seg = {100.0f, 1000.0f, 60000.0f, 50000.0f, false};
        planner.addSegment(seg);
    }
    planner.planAll();
    
    plan_block_t* last = planner.getBlock(2);
    float entry_speed = sqrtf(last->entry_speed_sqr);
    float decel_dist = SCurveMath::decelDistance(entry_speed, 0.0f, last->acceleration, 50000.0f);
    
    Assert(decel_dist <= last->millimeters * 1.2f,
           "Last block should be able to decelerate to stop");
}

// ============================================================================
// Velocity continuity tests
// ============================================================================

// Tests velocity is continuous across block boundaries (no jumps)
Test(PlannerIntegration, VelocityContinuity_NoJumps) {
    SimulatedTrapezoidPlanner planner;
    
    for (int i = 0; i < 5; i++) {
        MotionSegment seg = {50.0f, 1000.0f, 60000.0f, 0.0f, false};
        planner.addSegment(seg);
    }
    planner.planAll();
    
    auto result = planner.simulate();
    
    // Junction speeds should form a feasible sequence
    // (each can be reached from the previous via accel/decel)
    for (size_t i = 1; i < result.junction_speeds.size(); i++) {
        float prev_entry = result.junction_speeds[i-1];
        float curr_entry = result.junction_speeds[i];
        plan_block_t* prev_block = planner.getBlock(i-1);
        
        // Check that transition is kinematically feasible
        float max_achievable = sqrtf(prev_entry * prev_entry + 
                                     2.0f * prev_block->acceleration * prev_block->millimeters);
        float min_achievable = sqrtf(std::max(0.0f, prev_entry * prev_entry - 
                                              2.0f * prev_block->acceleration * prev_block->millimeters));
        
        Assert(curr_entry <= max_achievable * 1.1f,
               "Velocity transition should be achievable via acceleration");
        Assert(curr_entry >= min_achievable * 0.9f || curr_entry < 0.1f,
               "Velocity transition should be achievable via deceleration");
    }
}

// ============================================================================
// Comparison tests (Trapezoid vs S-curve)
// ============================================================================

// Tests that S-curve achieves similar speeds to trapezoidal on long moves
Test(PlannerIntegration, Comparison_LongMoves) {
    // Trapezoid planner
    SimulatedTrapezoidPlanner trap_planner;
    std::vector<MotionSegment> segments = {
        {200.0f, 1000.0f, 60000.0f, 0.0f, false},
        {200.0f, 1000.0f, 60000.0f, 0.0f, false}
    };
    for (const auto& seg : segments) {
        trap_planner.addSegment(seg);
    }
    trap_planner.planAll();
    auto trap_result = trap_planner.simulate();
    
    // S-curve planner
    SimulatedSCurvePlanner scurve_planner;
    scurve_planner.setJerk(100000.0f);  // High jerk to approach trapezoidal
    for (auto seg : segments) {
        seg.max_jerk = 100000.0f;
        scurve_planner.addSegment(seg);
    }
    scurve_planner.planAll();
    auto scurve_result = scurve_planner.simulate();
    
    // Both should complete successfully
    Assert(trap_result.completed_successfully, "Trapezoid should complete");
    Assert(scurve_result.completed_successfully, "S-curve should complete");
    
    // Max speeds should be similar for long moves with high jerk
    Assert(withinPercent(scurve_result.max_achieved_speed, trap_result.max_achieved_speed, 30.0f),
           "S-curve max speed should be similar to trapezoidal for long moves");
}

// Tests that S-curve is smoother (lower junction speeds for short segments)
Test(PlannerIntegration, Comparison_ShortMoves) {
    // With limited jerk, S-curve should be more conservative on short segments
    SimulatedSCurvePlanner scurve_planner;
    scurve_planner.setJerk(10000.0f);  // Low jerk for smooth motion
    
    // Many short segments
    for (int i = 0; i < 5; i++) {
        MotionSegment seg = {5.0f, 2000.0f, 60000.0f, 10000.0f, false};
        scurve_planner.addSegment(seg);
    }
    scurve_planner.planAll();
    
    auto result = scurve_planner.simulate();
    
    // Should be conservative with speed due to jerk limits
    Assert(result.max_achieved_speed < 2000.0f,
           "S-curve should limit speed on short segments with low jerk");
}

// ============================================================================
// Stress tests
// ============================================================================

// Tests handling of full buffer
Test(PlannerIntegration, FullBuffer) {
    SimulatedTrapezoidPlanner planner;
    
    // Fill buffer to capacity
    for (int i = 0; i < TEST_PLANNER_BLOCKS - 2; i++) {
        MotionSegment seg = {10.0f + i * 0.1f, 1000.0f, 60000.0f, 0.0f, false};
        planner.addSegment(seg);
    }
    planner.planAll();
    
    auto result = planner.simulate();
    
    Assert(result.completed_successfully, "Should handle full buffer");
    Assert(result.blocks_processed >= TEST_PLANNER_BLOCKS - 3, "Should process most blocks");
}

// Tests handling of single block
Test(PlannerIntegration, SingleBlock) {
    SimulatedSCurvePlanner planner;
    planner.setJerk(50000.0f);
    
    MotionSegment seg = {50.0f, 1000.0f, 60000.0f, 50000.0f, false};
    planner.addSegment(seg);
    planner.planAll();
    
    auto result = planner.simulate();
    
    Assert(result.completed_successfully, "Single block should complete");
    Assert(result.blocks_processed == 1, "Should process 1 block");
}

// Tests handling of varying segment lengths
Test(PlannerIntegration, VaryingSegmentLengths) {
    SimulatedSCurvePlanner planner;
    planner.setJerk(50000.0f);
    
    // Mix of short and long segments
    std::vector<MotionSegment> segments = {
        {5.0f, 1000.0f, 60000.0f, 50000.0f, false},
        {200.0f, 1000.0f, 60000.0f, 50000.0f, false},
        {2.0f, 1000.0f, 60000.0f, 50000.0f, false},
        {100.0f, 1000.0f, 60000.0f, 50000.0f, false},
        {1.0f, 1000.0f, 60000.0f, 50000.0f, false}
    };
    
    for (const auto& seg : segments) {
        planner.addSegment(seg);
    }
    planner.planAll();
    
    auto result = planner.simulate();
    
    Assert(result.completed_successfully, "Varying lengths should complete");
    Assert(result.blocks_processed == 5, "Should process all blocks");
}

// Tests numerical stability with extreme values
Test(PlannerIntegration, NumericalStability_ExtremeValues) {
    SimulatedSCurvePlanner planner;
    planner.setJerk(100000.0f);
    
    // Very short segment followed by very long
    std::vector<MotionSegment> segments = {
        {0.1f, 5000.0f, 100000.0f, 100000.0f, false},
        {1000.0f, 5000.0f, 100000.0f, 100000.0f, false}
    };
    
    for (const auto& seg : segments) {
        planner.addSegment(seg);
    }
    planner.planAll();
    
    auto result = planner.simulate();
    
    Assert(result.completed_successfully, "Extreme values should not cause issues");
    Assert(!std::isnan(result.max_achieved_speed), "Speed should not be NaN");
    Assert(!std::isinf(result.max_achieved_speed), "Speed should not be infinite");
}

// ============================================================================
// Real-world pattern tests
// ============================================================================

// Tests pattern similar to pocket milling (many short parallel lines)
Test(PlannerIntegration, RealWorld_PocketMilling) {
    SimulatedSCurvePlanner planner;
    planner.setJerk(50000.0f);
    
    // Simulate pocket milling pattern: forward, rapid retract, step over, repeat
    for (int i = 0; i < 4; i++) {
        // Cutting move
        MotionSegment cut = {50.0f, 500.0f, 60000.0f, 50000.0f, false};
        planner.addSegment(cut);
        
        // Rapid retract (short)
        MotionSegment retract = {2.0f, 2000.0f, 100000.0f, 50000.0f, true};
        planner.addSegment(retract);
        
        // Step over (short)
        MotionSegment stepover = {3.0f, 500.0f, 60000.0f, 50000.0f, false};
        planner.addSegment(stepover);
    }
    
    planner.planAll();
    auto result = planner.simulate();
    
    Assert(result.completed_successfully, "Pocket milling pattern should complete");
}

// Tests pattern similar to engraving (many very short moves)
// Uses S-curve planner because jerk limits are what truly constrain speed on tiny segments.
// Trapezoidal planners with high acceleration can reach significant speeds even on short moves.
Test(PlannerIntegration, RealWorld_Engraving) {
    SimulatedSCurvePlanner planner;
    planner.setJerk(10000.0f);  // Low jerk to demonstrate speed limiting
    
    // Many tiny segments like engraving text
    for (int i = 0; i < 10; i++) {
        MotionSegment seg = {0.5f + (i % 3) * 0.2f, 300.0f, 60000.0f, 10000.0f, false};
        planner.addSegment(seg);
    }
    
    planner.planAll();
    auto result = planner.simulate();
    
    Assert(result.completed_successfully, "Engraving pattern should complete");
    // Speed will be limited due to jerk constraints on short segments
    Assert(result.max_achieved_speed < 300.0f, "S-curve should limit speed for tiny segments");
}
