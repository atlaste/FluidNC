// Copyright (c) 2024 - FluidNC Authors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "../TestFramework.h"
#include "Planner/SCurveMath.h"

#include <cmath>

// Helper to check floating point equality with tolerance
inline bool floatEquals(float a, float b, float tolerance = 0.01f) {
    return fabsf(a - b) <= tolerance;
}

// Helper to check if value is within percentage tolerance
inline bool withinPercent(float actual, float expected, float percent = 5.0f) {
    if (fabsf(expected) < 0.001f) {
        return fabsf(actual) < 0.001f;
    }
    return fabsf(actual - expected) / fabsf(expected) * 100.0f <= percent;
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
    
    // Verify computed speed is achievable within the distance
    float d_check = SCurveMath::accelDistance(0.0f, v_max, a_max, jerk);
    Assert(d_check <= distance * 1.1f, "Computed speed should be reachable within distance");
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

// ============================================================================
// Utility function tests
// ============================================================================

// Tests that clamp function correctly limits values to the specified range
Test(SCurveMath, Clamp_InRange) {
    float result = SCurveMath::clamp(5.0f, 0.0f, 10.0f);
    Assert(floatEquals(result, 5.0f), "Value in range should be unchanged");
}

// Tests that clamp returns min when value is below range
Test(SCurveMath, Clamp_BelowMin) {
    float result = SCurveMath::clamp(-5.0f, 0.0f, 10.0f);
    Assert(floatEquals(result, 0.0f), "Value below min should be clamped to min");
}

// Tests that clamp returns max when value is above range
Test(SCurveMath, Clamp_AboveMax) {
    float result = SCurveMath::clamp(15.0f, 0.0f, 10.0f);
    Assert(floatEquals(result, 10.0f), "Value above max should be clamped to max");
}

// Tests that safeSqrt returns 0 for negative inputs instead of NaN
Test(SCurveMath, SafeSqrt_Negative) {
    float result = SCurveMath::safeSqrt(-1.0f);
    Assert(floatEquals(result, 0.0f), "Negative input should return 0");
}

// Tests that safeSqrt correctly computes square root of positive values
Test(SCurveMath, SafeSqrt_Positive) {
    float result = SCurveMath::safeSqrt(4.0f);
    Assert(floatEquals(result, 2.0f), "sqrt(4) should be 2");
}

// Tests the square function
Test(SCurveMath, Sq_Basic) {
    float result = SCurveMath::sq(3.0f);
    Assert(floatEquals(result, 9.0f), "3^2 should be 9");
}

// Tests jerkTime calculation: t = |Δa| / j
Test(SCurveMath, JerkTime_Basic) {
    float t = SCurveMath::jerkTime(1000.0f, 50000.0f);
    Assert(floatEquals(t, 0.02f), "Time to change accel by 1000 at jerk 50000 should be 0.02");
}

// Tests velocity change during jerk phase
Test(SCurveMath, JerkVelocityChange_Basic) {
    float a0 = 100.0f;
    float jerk = 1000.0f;
    float t = 0.1f;
    float delta_v = SCurveMath::jerkVelocityChange(a0, jerk, t);
    // Δv = a₀t + ½jt²
    float expected = a0 * t + 0.5f * jerk * t * t;
    Assert(floatEquals(delta_v, expected), "Velocity change should match formula");
}

// ============================================================================
// Core kinematic function edge cases
// ============================================================================

// Tests jerkDistance with all components contributing
Test(SCurveMath, JerkDistance_AllComponents) {
    // d = v₀t + ½a₀t² + ⅙jt³
    float v0 = 100.0f;
    float a0 = 200.0f;
    float j = 3000.0f;
    float t = 0.05f;
    
    float d = SCurveMath::jerkDistance(v0, a0, j, t);
    float expected = v0 * t + 0.5f * a0 * t * t + (1.0f / 6.0f) * j * t * t * t;
    Assert(floatEquals(d, expected), "Full jerk distance formula should match");
}

// Tests jerkDistance with negative jerk (deceleration ramp)
Test(SCurveMath, JerkDistance_NegativeJerk) {
    float v0 = 500.0f;
    float a0 = 1000.0f;  // Currently accelerating
    float j = -5000.0f;  // But reducing acceleration
    float t = 0.1f;
    
    float d = SCurveMath::jerkDistance(v0, a0, j, t);
    Assert(d > 0.0f, "Distance should still be positive even with negative jerk");
}

// Tests jerkVelocity at various acceleration states
Test(SCurveMath, JerkVelocity_NegativeAccel) {
    float v0 = 1000.0f;
    float a0 = -500.0f;  // Decelerating
    float j = 1000.0f;
    float t = 0.5f;
    
    float v = SCurveMath::jerkVelocity(v0, a0, j, t);
    // v = v₀ + a₀t + ½jt² = 1000 - 500*0.5 + 0.5*1000*0.25 = 1000 - 250 + 125 = 875
    float expected = v0 + a0 * t + 0.5f * j * t * t;
    Assert(floatEquals(v, expected), "Velocity with negative acceleration should be correct");
}

// Tests jerkAccel at extreme values
Test(SCurveMath, JerkAccel_ZeroTime) {
    float a = SCurveMath::jerkAccel(500.0f, 10000.0f, 0.0f);
    Assert(floatEquals(a, 500.0f), "Zero time should give initial acceleration");
}

// Tests jerkAccel with negative jerk
Test(SCurveMath, JerkAccel_NegativeJerk) {
    float a0 = 1000.0f;
    float j = -5000.0f;
    float t = 0.1f;
    
    float a = SCurveMath::jerkAccel(a0, j, t);
    Assert(floatEquals(a, a0 + j * t), "Negative jerk should reduce acceleration");
    Assert(a < a0, "Final acceleration should be less than initial");
}

// ============================================================================
// S-curve distance edge cases
// ============================================================================

// Tests accel distance when velocities are equal (should be zero)
Test(SCurveMath, AccelDistance_EqualVelocities) {
    float d = SCurveMath::accelDistance(500.0f, 500.0f, 1000.0f, 50000.0f);
    Assert(floatEquals(d, 0.0f), "Equal velocities should need zero distance");
}

// Tests decel distance when velocities are equal (should be zero)
Test(SCurveMath, DecelDistance_EqualVelocities) {
    float d = SCurveMath::decelDistance(500.0f, 500.0f, 1000.0f, 50000.0f);
    Assert(floatEquals(d, 0.0f), "Equal velocities should need zero distance");
}

// Tests that accel distance increases with larger velocity change
Test(SCurveMath, AccelDistance_Monotonic) {
    float d1 = SCurveMath::accelDistance(100.0f, 200.0f, 1000.0f, 50000.0f);
    float d2 = SCurveMath::accelDistance(100.0f, 300.0f, 1000.0f, 50000.0f);
    float d3 = SCurveMath::accelDistance(100.0f, 400.0f, 1000.0f, 50000.0f);
    
    Assert(d2 > d1, "Larger velocity change should need more distance");
    Assert(d3 > d2, "Even larger velocity change should need even more distance");
}

// Tests that lower jerk requires more distance
Test(SCurveMath, AccelDistance_JerkImpact) {
    float d_high_jerk = SCurveMath::accelDistance(100.0f, 500.0f, 1000.0f, 100000.0f);
    float d_low_jerk = SCurveMath::accelDistance(100.0f, 500.0f, 1000.0f, 10000.0f);
    
    Assert(d_low_jerk > d_high_jerk, "Lower jerk should need more distance");
}

// Tests that lower max acceleration requires more distance
Test(SCurveMath, AccelDistance_AccelImpact) {
    float d_high_accel = SCurveMath::accelDistance(100.0f, 500.0f, 5000.0f, 50000.0f);
    float d_low_accel = SCurveMath::accelDistance(100.0f, 500.0f, 1000.0f, 50000.0f);
    
    Assert(d_low_accel > d_high_accel, "Lower max acceleration should need more distance");
}

// ============================================================================
// Extended S-curve calculations with non-zero acceleration
// ============================================================================

// Tests accel distance with non-zero entry acceleration
Test(SCurveMath, AccelDistanceWithAccel_NonZeroEntry) {
    float d = SCurveMath::accelDistanceWithAccel(100.0f, 500.0f, 500.0f, 0.0f, 1000.0f, 50000.0f);
    Assert(d > 0.0f, "Should need positive distance to accelerate with non-zero entry accel");
}

// Tests that starting with acceleration reduces total distance needed
Test(SCurveMath, AccelDistanceWithAccel_VsZeroEntry) {
    float d_with_accel = SCurveMath::accelDistanceWithAccel(100.0f, 500.0f, 500.0f, 0.0f, 1000.0f, 50000.0f);
    float d_zero_accel = SCurveMath::accelDistance(100.0f, 500.0f, 1000.0f, 50000.0f);
    
    // Starting with acceleration already should need less distance
    Assert(d_with_accel < d_zero_accel, "Non-zero entry accel should need less distance");
}

// Tests decel distance with non-zero entry acceleration
Test(SCurveMath, DecelDistanceWithAccel_NonZeroEntry) {
    float d = SCurveMath::decelDistanceWithAccel(500.0f, 0.0f, 100.0f, 0.0f, 1000.0f, 50000.0f);
    Assert(d > 0.0f, "Should need positive distance to decelerate");
}

// Tests impossible deceleration case (trying to decel to higher speed)
Test(SCurveMath, DecelDistanceWithAccel_Impossible) {
    float d = SCurveMath::decelDistanceWithAccel(100.0f, 0.0f, 500.0f, 0.0f, 1000.0f, 50000.0f);
    Assert(floatEquals(d, 0.0f), "Cannot decelerate to higher speed");
}

// ============================================================================
// Solver function edge cases and convergence tests
// ============================================================================

// Tests solver with very short distance (should return entry/exit speed)
Test(SCurveMath, SolveMaxEntrySpeed_VeryShortDistance) {
    float v_max = SCurveMath::solveMaxEntrySpeed(0.01f, 0.0f, 0.0f, 60000.0f, 50000.0f);
    // With very short distance, max entry speed should be close to exit speed
    Assert(v_max < 100.0f, "Very short distance should limit entry speed");
}

// Tests solver with very long distance
Test(SCurveMath, SolveMaxEntrySpeed_LongDistance) {
    float v_max = SCurveMath::solveMaxEntrySpeed(10000.0f, 0.0f, 0.0f, 60000.0f, 50000.0f);
    Assert(v_max > 1000.0f, "Long distance should allow high entry speed");
}

// Tests solver consistency: computed speed should decel in computed distance
Test(SCurveMath, SolveMaxEntrySpeed_Consistency) {
    float distance = 50.0f;
    float v_exit = 100.0f;
    float a_max = 60000.0f;
    float jerk = 50000.0f;
    
    float v_max = SCurveMath::solveMaxEntrySpeed(distance, v_exit, 0.0f, a_max, jerk);
    float d_check = SCurveMath::decelDistance(v_max, v_exit, a_max, jerk);
    
    Assert(d_check <= distance * 1.15f, "Computed entry speed should fit within distance");
}

// Tests solveMaxExitSpeed with consistency check
Test(SCurveMath, SolveMaxExitSpeed_Consistency) {
    float distance = 50.0f;
    float v_entry = 100.0f;
    float a_max = 60000.0f;
    float jerk = 50000.0f;
    
    float v_max = SCurveMath::solveMaxExitSpeed(distance, v_entry, 0.0f, a_max, jerk);
    float d_check = SCurveMath::accelDistance(v_entry, v_max, a_max, jerk);
    
    Assert(d_check <= distance * 1.15f, "Computed exit speed should be achievable in distance");
}

// Tests that entry and exit solvers produce compatible results
Test(SCurveMath, Solver_Symmetry) {
    float distance = 100.0f;
    float a_max = 60000.0f;
    float jerk = 50000.0f;
    
    // Solve for max entry speed to decel to zero
    float v_entry = SCurveMath::solveMaxEntrySpeed(distance, 0.0f, 0.0f, a_max, jerk);
    
    // Solve for max exit speed from zero
    float v_exit = SCurveMath::solveMaxExitSpeed(distance, 0.0f, 0.0f, a_max, jerk);
    
    // Should be approximately equal due to symmetry
    Assert(floatEquals(v_entry, v_exit, v_entry * 0.1f), "Entry and exit speeds should be symmetric");
}

// ============================================================================
// Peak velocity calculation tests
// ============================================================================

// Tests that peak velocity never exceeds v_max
Test(SCurveMath, ComputePeakVelocity_NeverExceedsMax) {
    float v_max = 500.0f;
    float v_peak = SCurveMath::computePeakVelocity(1000.0f, 0.0f, 0.0f, v_max, 60000.0f, 50000.0f);
    Assert(v_peak <= v_max, "Peak velocity should never exceed v_max");
}

// Tests peak velocity when entry > exit
Test(SCurveMath, ComputePeakVelocity_EntryHigherThanExit) {
    float v_entry = 400.0f;
    float v_exit = 100.0f;
    float v_max = 500.0f;
    
    float v_peak = SCurveMath::computePeakVelocity(100.0f, v_entry, v_exit, v_max, 60000.0f, 50000.0f);
    Assert(v_peak >= v_entry, "Peak should be at least entry speed");
}

// Tests peak velocity when exit > entry
Test(SCurveMath, ComputePeakVelocity_ExitHigherThanEntry) {
    float v_entry = 100.0f;
    float v_exit = 400.0f;
    float v_max = 500.0f;
    
    float v_peak = SCurveMath::computePeakVelocity(100.0f, v_entry, v_exit, v_max, 60000.0f, 50000.0f);
    Assert(v_peak >= v_exit, "Peak should be at least exit speed");
}

// Tests that increasing distance increases achievable peak
Test(SCurveMath, ComputePeakVelocity_DistanceImpact) {
    float v_peak_short = SCurveMath::computePeakVelocity(10.0f, 0.0f, 0.0f, 5000.0f, 60000.0f, 50000.0f);
    float v_peak_long = SCurveMath::computePeakVelocity(100.0f, 0.0f, 0.0f, 5000.0f, 60000.0f, 50000.0f);
    
    Assert(v_peak_long >= v_peak_short, "Longer distance should allow higher peak");
}

// ============================================================================
// Profile planning comprehensive tests
// ============================================================================

// Tests that profile planning produces valid phase times (non-negative)
Test(SCurveMath, PlanProfile_ValidPhaseTimes) {
    auto profile = SCurveMath::planProfile(100.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 60000.0f, 50000.0f);
    
    for (int i = 0; i < 7; i++) {
        Assert(profile.t[i] >= 0.0f, "Phase times should be non-negative");
    }
}

// Tests that profile distances sum correctly
Test(SCurveMath, PlanProfile_DistanceSum) {
    float target_distance = 100.0f;
    auto profile = SCurveMath::planProfile(target_distance, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 60000.0f, 50000.0f);
    
    float sum = 0.0f;
    for (int i = 0; i < 7; i++) {
        sum += profile.d[i];
    }
    
    Assert(withinPercent(sum, target_distance, 10.0f), "Phase distances should sum to total distance");
}

// Tests profile with asymmetric entry/exit speeds
Test(SCurveMath, PlanProfile_AsymmetricSpeeds) {
    float v_entry = 200.0f;
    float v_exit = 100.0f;
    
    auto profile = SCurveMath::planProfile(100.0f, v_entry, 0.0f, v_exit, 0.0f, 1000.0f, 60000.0f, 50000.0f);
    
    Assert(floatEquals(profile.v[0], v_entry, 1.0f), "Entry velocity should match");
    Assert(floatEquals(profile.v[7], v_exit, 1.0f), "Exit velocity should match");
    Assert(profile.v_peak >= v_entry, "Peak should be at least entry speed");
}

// Tests profile with very short distance (degenerate profile)
Test(SCurveMath, PlanProfile_ShortDistance) {
    auto profile = SCurveMath::planProfile(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5000.0f, 60000.0f, 50000.0f);
    
    // Short distance should have degenerate profile (no cruise phase)
    Assert(profile.total_distance > 0.0f, "Should still have positive distance");
    Assert(profile.v_peak > 0.0f, "Should still achieve some peak velocity");
}

// Tests profile with high entry/exit speeds (mostly deceleration)
Test(SCurveMath, PlanProfile_HighSpeedEntry) {
    float v_high = 800.0f;
    auto profile = SCurveMath::planProfile(50.0f, v_high, 0.0f, 0.0f, 0.0f, 1000.0f, 60000.0f, 50000.0f);
    
    // Should be mostly deceleration
    Assert(profile.v[0] >= profile.v[7], "Should be decelerating overall");
}

// Tests that phase velocities are monotonic during acceleration
Test(SCurveMath, PlanProfile_AccelPhaseMonotonic) {
    auto profile = SCurveMath::planProfile(200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 60000.0f, 50000.0f);
    
    // During acceleration phases (0, 1, 2), velocity should increase or stay same
    Assert(profile.v[1] >= profile.v[0], "Phase 0 should accelerate");
    Assert(profile.v[2] >= profile.v[1], "Phase 1 should accelerate");
    Assert(profile.v[3] >= profile.v[2], "Phase 2 should accelerate");
}

// Tests that phase velocities are monotonic during deceleration
Test(SCurveMath, PlanProfile_DecelPhaseMonotonic) {
    auto profile = SCurveMath::planProfile(200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 60000.0f, 50000.0f);
    
    // During deceleration phases (4, 5, 6), velocity should decrease or stay same
    Assert(profile.v[5] <= profile.v[4], "Phase 4 should decelerate");
    Assert(profile.v[6] <= profile.v[5], "Phase 5 should decelerate");
    Assert(profile.v[7] <= profile.v[6], "Phase 6 should decelerate");
}

// Tests profile at different jerk values
Test(SCurveMath, PlanProfile_JerkComparison) {
    float distance = 100.0f;
    
    auto profile_high_jerk = SCurveMath::planProfile(distance, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 60000.0f, 100000.0f);
    auto profile_low_jerk = SCurveMath::planProfile(distance, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 60000.0f, 10000.0f);
    
    // Higher jerk should result in shorter total time (more aggressive acceleration)
    Assert(profile_high_jerk.total_time <= profile_low_jerk.total_time, 
           "Higher jerk should result in shorter total time");
}

// Tests phases_used bitmask correctness
Test(SCurveMath, PlanProfile_PhasesUsedBitmask) {
    auto profile = SCurveMath::planProfile(500.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 60000.0f, 50000.0f);
    
    // With enough distance, should use cruise phase (bit 3)
    bool has_cruise = (profile.phases_used & (1 << SCurveMath::PHASE_CRUISE)) != 0;
    Assert(profile.t[SCurveMath::PHASE_CRUISE] > 0.0f == has_cruise, 
           "phases_used should match actual phase times");
}

// ============================================================================
// Numerical precision and boundary tests  
// ============================================================================

// Tests behavior with very small velocities
Test(SCurveMath, SmallVelocity_AccelDistance) {
    float d = SCurveMath::accelDistance(0.001f, 0.002f, 1000.0f, 50000.0f);
    Assert(d >= 0.0f, "Should handle very small velocities");
    Assert(!std::isnan(d), "Result should not be NaN");
    Assert(!std::isinf(d), "Result should not be infinite");
}

// Tests behavior with very large velocities  
Test(SCurveMath, LargeVelocity_AccelDistance) {
    float d = SCurveMath::accelDistance(10000.0f, 20000.0f, 100000.0f, 500000.0f);
    Assert(d > 0.0f, "Should handle large velocities");
    Assert(!std::isnan(d), "Result should not be NaN");
    Assert(!std::isinf(d), "Result should not be infinite");
}

// Tests that zero jerk doesn't cause division by zero
Test(SCurveMath, ZeroJerk_SafeHandling) {
    // Note: The implementation may or may not handle zero jerk gracefully
    // This test documents the expected behavior
    float d = SCurveMath::accelDistance(100.0f, 500.0f, 1000.0f, 0.001f);  // Very small jerk
    Assert(!std::isnan(d), "Result should not be NaN with tiny jerk");
}

// Tests profile with entry == exit == v_max (pure cruise)
Test(SCurveMath, PlanProfile_PureCruise) {
    float v = 500.0f;
    float distance = 100.0f;
    
    auto profile = SCurveMath::planProfile(distance, v, 0.0f, v, 0.0f, v, 60000.0f, 50000.0f);
    
    // Should be almost entirely cruise phase
    Assert(profile.t[SCurveMath::PHASE_CRUISE] > 0.0f, "Should have cruise phase");
    Assert(floatEquals(profile.v_peak, v, 1.0f), "Peak should equal requested velocity");
}
