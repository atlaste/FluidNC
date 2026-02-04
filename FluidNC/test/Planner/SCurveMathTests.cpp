// Copyright (c) 2024 - FluidNC Authors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "../TestFramework.h"
#include "src/Planner/SCurveMath.h"

#include <cmath>

// Helper to check floating point equality with tolerance
inline bool floatEquals(float a, float b, float tolerance = 0.01f) {
    return fabsf(a - b) <= tolerance;
}

// ============================================================================
// Core kinematic function tests
// ============================================================================

Test(SCurveMath, JerkDistance_Zero) {
    // Zero time should give zero distance
    float d = SCurveMath::jerkDistance(100.0f, 0.0f, 1000.0f, 0.0f);
    Assert(floatEquals(d, 0.0f), "Zero time should give zero distance");
}

Test(SCurveMath, JerkDistance_ConstantVelocity) {
    // With zero acceleration and jerk, distance = v * t
    float v0 = 100.0f;  // mm/min
    float t = 0.1f;     // min
    float d = SCurveMath::jerkDistance(v0, 0.0f, 0.0f, t);
    Assert(floatEquals(d, v0 * t), "Constant velocity should give d = v*t");
}

Test(SCurveMath, JerkDistance_ConstantAccel) {
    // With zero jerk, should match constant acceleration formula
    // d = v0*t + 0.5*a*t^2
    float v0 = 100.0f;
    float a0 = 1000.0f;
    float t = 0.1f;
    float d = SCurveMath::jerkDistance(v0, a0, 0.0f, t);
    float expected = v0 * t + 0.5f * a0 * t * t;
    Assert(floatEquals(d, expected), "Zero jerk should match constant accel formula");
}

Test(SCurveMath, JerkVelocity_Zero) {
    // Zero time should give initial velocity
    float v = SCurveMath::jerkVelocity(100.0f, 50.0f, 1000.0f, 0.0f);
    Assert(floatEquals(v, 100.0f), "Zero time should give initial velocity");
}

Test(SCurveMath, JerkVelocity_ConstantAccel) {
    // With zero jerk: v = v0 + a*t
    float v0 = 100.0f;
    float a0 = 1000.0f;
    float t = 0.1f;
    float v = SCurveMath::jerkVelocity(v0, a0, 0.0f, t);
    float expected = v0 + a0 * t;
    Assert(floatEquals(v, expected), "Zero jerk should give v = v0 + a*t");
}

Test(SCurveMath, JerkAccel_Basic) {
    // a = a0 + j*t
    float a0 = 100.0f;
    float j = 1000.0f;
    float t = 0.1f;
    float a = SCurveMath::jerkAccel(a0, j, t);
    float expected = a0 + j * t;
    Assert(floatEquals(a, expected), "Acceleration should follow a = a0 + j*t");
}

// ============================================================================
// S-curve distance calculation tests
// ============================================================================

Test(SCurveMath, AccelDistance_NoAccelNeeded) {
    // If v2 <= v1, no acceleration needed
    float d = SCurveMath::accelDistance(100.0f, 50.0f, 1000.0f, 50000.0f);
    Assert(floatEquals(d, 0.0f), "No distance needed when v2 <= v1");
}

Test(SCurveMath, AccelDistance_Positive) {
    // Accelerating should require positive distance
    float d = SCurveMath::accelDistance(100.0f, 500.0f, 1000.0f, 50000.0f);
    Assert(d > 0.0f, "Acceleration should require positive distance");
}

Test(SCurveMath, AccelDistance_LargerThanTrapezoidal) {
    // S-curve distance should be >= trapezoidal distance
    float v1 = 100.0f;
    float v2 = 500.0f;
    float a_max = 1000.0f;
    float jerk = 50000.0f;
    
    float d_scurve = SCurveMath::accelDistance(v1, v2, a_max, jerk);
    float d_trap = (v2 * v2 - v1 * v1) / (2.0f * a_max);  // Trapezoidal
    
    Assert(d_scurve >= d_trap - 0.1f, "S-curve distance should be >= trapezoidal");
}

Test(SCurveMath, DecelDistance_NoDecelNeeded) {
    // If v1 <= v2, no deceleration needed
    float d = SCurveMath::decelDistance(50.0f, 100.0f, 1000.0f, 50000.0f);
    Assert(floatEquals(d, 0.0f), "No distance needed when v1 <= v2");
}

Test(SCurveMath, DecelDistance_Positive) {
    // Decelerating should require positive distance
    float d = SCurveMath::decelDistance(500.0f, 100.0f, 1000.0f, 50000.0f);
    Assert(d > 0.0f, "Deceleration should require positive distance");
}

Test(SCurveMath, DecelDistance_ToZero) {
    // Should be able to decelerate to zero
    float d = SCurveMath::decelDistance(1000.0f, 0.0f, 5000.0f, 50000.0f);
    Assert(d > 0.0f, "Should have positive distance to stop");
}

// ============================================================================
// Solver function tests
// ============================================================================

Test(SCurveMath, SolveMaxEntrySpeed_FromZero) {
    // With zero exit speed, should compute reasonable entry speed
    float distance = 100.0f;  // mm
    float a_max = 60000.0f;   // mm/min²
    float jerk = 50000.0f;    // mm/min³
    
    float v_max = SCurveMath::solveMaxEntrySpeed(distance, 0.0f, 0.0f, a_max, jerk);
    Assert(v_max > 0.0f, "Should find positive max entry speed");
    
    // Verify that this speed can actually decelerate in the given distance
    float d_check = SCurveMath::decelDistance(v_max, 0.0f, a_max, jerk);
    Assert(d_check <= distance * 1.1f, "Computed speed should decel within distance");
}

Test(SCurveMath, SolveMaxExitSpeed_FromZero) {
    // Starting from zero, should compute achievable exit speed
    float distance = 100.0f;
    float a_max = 60000.0f;
    float jerk = 50000.0f;
    
    float v_max = SCurveMath::solveMaxExitSpeed(distance, 0.0f, 0.0f, a_max, jerk);
    Assert(v_max > 0.0f, "Should find positive max exit speed");
    
    // Verify that this speed can be reached from zero
    float d_check = SCurveMath::accelDistance(0.0f, v_max, a_max, jerk);
    Assert(d_check <= distance * 1.1f, "Computed speed should be reachable");
}

Test(SCurveMath, ComputePeakVelocity_ShortMove) {
    // Short move should not reach max velocity
    float distance = 10.0f;   // mm (short)
    float v_entry = 0.0f;
    float v_exit = 0.0f;
    float v_max = 10000.0f;   // mm/min (high)
    float a_max = 60000.0f;
    float jerk = 50000.0f;
    
    float v_peak = SCurveMath::computePeakVelocity(distance, v_entry, v_exit, v_max, a_max, jerk);
    Assert(v_peak < v_max, "Short move should not reach max velocity");
    Assert(v_peak > 0.0f, "Should achieve some peak velocity");
}

Test(SCurveMath, ComputePeakVelocity_LongMove) {
    // Long move should reach max velocity
    float distance = 1000.0f; // mm (long)
    float v_entry = 0.0f;
    float v_exit = 0.0f;
    float v_max = 1000.0f;    // mm/min
    float a_max = 60000.0f;
    float jerk = 50000.0f;
    
    float v_peak = SCurveMath::computePeakVelocity(distance, v_entry, v_exit, v_max, a_max, jerk);
    Assert(floatEquals(v_peak, v_max, 1.0f), "Long move should reach max velocity");
}

// ============================================================================
// Profile planning tests
// ============================================================================

Test(SCurveMath, PlanProfile_Basic) {
    float distance = 100.0f;
    float v_entry = 0.0f;
    float a_entry = 0.0f;
    float v_exit = 0.0f;
    float a_exit = 0.0f;
    float v_max = 1000.0f;
    float a_max = 60000.0f;
    float jerk = 50000.0f;
    
    auto profile = SCurveMath::planProfile(distance, v_entry, a_entry, v_exit, a_exit, v_max, a_max, jerk);
    
    // Total distance should match input
    Assert(floatEquals(profile.total_distance, distance, 1.0f), "Total distance should match input");
    
    // Entry and exit velocities should match
    Assert(floatEquals(profile.v[0], v_entry, 1.0f), "Entry velocity should match");
    Assert(floatEquals(profile.v[7], v_exit, 1.0f), "Exit velocity should match");
    
    // Peak velocity should be positive
    Assert(profile.v_peak > 0.0f, "Peak velocity should be positive");
}

Test(SCurveMath, PlanProfile_ConstantVelocity) {
    // If entry == exit == max, should cruise entire distance
    float distance = 100.0f;
    float v_const = 500.0f;
    float a_max = 60000.0f;
    float jerk = 50000.0f;
    
    auto profile = SCurveMath::planProfile(distance, v_const, 0.0f, v_const, 0.0f, v_const, a_max, jerk);
    
    // Should be mostly cruise phase
    Assert(profile.t[SCurveMath::PHASE_CRUISE] > 0.0f, "Should have cruise phase");
    Assert(floatEquals(profile.v_peak, v_const, 1.0f), "Peak should equal constant velocity");
}

// ============================================================================
// Edge case tests
// ============================================================================

Test(SCurveMath, TriangularProfile) {
    // Small velocity change should create triangular (no constant accel) profile
    float v1 = 100.0f;
    float v2 = 110.0f;  // Small change
    float a_max = 60000.0f;
    float jerk = 10000.0f;
    
    float d = SCurveMath::accelDistance(v1, v2, a_max, jerk);
    Assert(d > 0.0f, "Triangular profile should still have distance");
    
    // Should be smaller than full 3-phase would suggest
    // (triangular profiles are shorter)
}

Test(SCurveMath, ZeroJerkFallback) {
    // With very high jerk, should approach trapezoidal behavior
    float v1 = 100.0f;
    float v2 = 500.0f;
    float a_max = 1000.0f;
    float jerk = 1000000.0f;  // Very high
    
    float d_scurve = SCurveMath::accelDistance(v1, v2, a_max, jerk);
    float d_trap = (v2 * v2 - v1 * v1) / (2.0f * a_max);
    
    // Should be close to trapezoidal
    Assert(floatEquals(d_scurve, d_trap, d_trap * 0.1f), "High jerk should approach trapezoidal");
}

Test(SCurveMath, Symmetry) {
    // Accel and decel should be symmetric for same velocity change
    float v_low = 100.0f;
    float v_high = 500.0f;
    float a_max = 1000.0f;
    float jerk = 50000.0f;
    
    float d_accel = SCurveMath::accelDistance(v_low, v_high, a_max, jerk);
    float d_decel = SCurveMath::decelDistance(v_high, v_low, a_max, jerk);
    
    Assert(floatEquals(d_accel, d_decel, 1.0f), "Accel and decel should be symmetric");
}
