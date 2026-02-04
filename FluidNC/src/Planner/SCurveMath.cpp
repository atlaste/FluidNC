// Copyright (c) 2024 - FluidNC Authors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "SCurveMath.h"

#include <algorithm>

namespace SCurveMath {

// ============================================================================
// S-curve distance calculations (zero entry/exit acceleration)
// ============================================================================

float accelDistance(float v1, float v2, float a_max, float jerk) {
    if (v2 <= v1) return 0.0f;  // No acceleration needed
    
    // Time for each jerk phase (to ramp accel from 0 to a_max or back)
    float t_jerk = a_max / jerk;
    
    // Velocity change during one jerk phase: Δv = ½*j*t²
    float v_jerk = 0.5f * jerk * t_jerk * t_jerk;
    
    float delta_v = v2 - v1;
    
    // Check if we have a triangular profile (not enough delta_v for full jerk phases)
    if (delta_v <= 2.0f * v_jerk) {
        // Triangular S-curve: jerk+ then jerk- with no constant accel phase
        // delta_v = j * t² (symmetric triangle)
        float t = safeSqrt(delta_v / jerk);
        
        // Distance = 2 * (v1*t + ⅙*j*t³)
        float d1 = jerkDistance(v1, 0.0f, jerk, t);
        float v_mid = v1 + 0.5f * jerk * t * t;
        float d2 = jerkDistance(v_mid, jerk * t, -jerk, t);
        
        return d1 + d2;
    }
    
    // Full 3-phase S-curve: jerk+ → const accel → jerk-
    // Phase 1: jerk+ from a=0 to a=a_max
    float d1 = jerkDistance(v1, 0.0f, jerk, t_jerk);
    float v_after_jerk1 = v1 + v_jerk;
    
    // Phase 3: jerk- from a=a_max to a=0
    float v_before_jerk2 = v2 - v_jerk;
    float d3 = jerkDistance(v_before_jerk2, a_max, -jerk, t_jerk);
    
    // Phase 2: constant acceleration at a_max
    // v² = v₀² + 2*a*d → d = (v² - v₀²) / (2*a)
    float d2 = (sq(v_before_jerk2) - sq(v_after_jerk1)) / (2.0f * a_max);
    
    return d1 + d2 + d3;
}

float decelDistance(float v1, float v2, float a_max, float jerk) {
    if (v1 <= v2) return 0.0f;  // No deceleration needed
    
    float t_jerk = a_max / jerk;
    float v_jerk = 0.5f * jerk * t_jerk * t_jerk;
    
    float delta_v = v1 - v2;
    
    // Triangular decel profile
    if (delta_v <= 2.0f * v_jerk) {
        float t = safeSqrt(delta_v / jerk);
        
        // Distance for symmetric jerk- then jerk+ triangle
        float d1 = jerkDistance(v1, 0.0f, -jerk, t);
        float v_mid = v1 - 0.5f * jerk * t * t;
        float d2 = jerkDistance(v_mid, -jerk * t, jerk, t);
        
        return d1 + d2;
    }
    
    // Full 3-phase: jerk- → const decel → jerk+
    // Phase 1: jerk- from a=0 to a=-a_max
    float d1 = jerkDistance(v1, 0.0f, -jerk, t_jerk);
    float v_after_jerk1 = v1 - v_jerk;
    
    // Phase 3: jerk+ from a=-a_max to a=0
    float v_before_jerk2 = v2 + v_jerk;
    float d3 = jerkDistance(v_before_jerk2, -a_max, jerk, t_jerk);
    
    // Phase 2: constant deceleration at -a_max
    float d2 = (sq(v_after_jerk1) - sq(v_before_jerk2)) / (2.0f * a_max);
    
    return d1 + d2 + d3;
}

// ============================================================================
// Extended S-curve calculations for continuous jerk
// ============================================================================

float accelDistanceWithAccel(float v_entry, float a_entry,
                             float v_exit, float a_exit,
                             float a_max, float jerk) {
    // Handle the case where we start with non-zero acceleration
    // and need to end with a specific acceleration
    
    if (v_exit <= v_entry && a_entry >= 0.0f && a_exit >= 0.0f) {
        return 0.0f;  // Can't accelerate to lower speed
    }
    
    // Time to ramp from a_entry to a_max
    float t1 = (a_max - a_entry) / jerk;
    if (t1 < 0.0f) t1 = 0.0f;
    
    // Time to ramp from a_max to a_exit
    float t3 = (a_max - a_exit) / jerk;
    if (t3 < 0.0f) t3 = 0.0f;
    
    // Velocity and distance during phase 1 (jerk+)
    float v1_end = jerkVelocity(v_entry, a_entry, jerk, t1);
    float d1 = jerkDistance(v_entry, a_entry, jerk, t1);
    
    // Velocity at start of phase 3 (before jerk-)
    // v3_start needs to be computed from v_exit working backward
    float v3_end = v_exit;
    float v3_start = jerkVelocity(v3_end, a_exit, jerk, t3);  // Working forward from what v3_start would give v_exit
    // Actually, working backward: v3_start = v_exit - (a_exit*t3 + 0.5*(-jerk)*t3²)
    v3_start = v_exit - a_exit * t3 + 0.5f * jerk * t3 * t3;
    
    // Distance during phase 3 (jerk-)
    float d3 = jerkDistance(v3_start, a_max, -jerk, t3);
    
    // Check if we need constant accel phase
    if (v1_end >= v3_start) {
        // Triangular - no constant accel phase needed
        // Need to solve for the intersection
        float delta_v = v_exit - v_entry;
        if (delta_v <= 0.0f) return 0.0f;
        
        // Simplified triangular calculation
        float t = safeSqrt(2.0f * delta_v / jerk);
        return v_entry * t + (1.0f / 6.0f) * jerk * t * t * t;
    }
    
    // Phase 2: constant acceleration at a_max
    float d2 = (sq(v3_start) - sq(v1_end)) / (2.0f * a_max);
    
    return d1 + d2 + d3;
}

float decelDistanceWithAccel(float v_entry, float a_entry,
                             float v_exit, float a_exit,
                             float a_max, float jerk) {
    if (v_entry <= v_exit && a_entry <= 0.0f && a_exit <= 0.0f) {
        return 0.0f;  // Can't decelerate to higher speed
    }
    
    // Time to ramp from a_entry to -a_max (negative acceleration for decel)
    float t1 = (a_entry + a_max) / jerk;
    if (t1 < 0.0f) t1 = 0.0f;
    
    // Time to ramp from -a_max to a_exit
    float t3 = (-a_max - a_exit) / (-jerk);  // = (a_max + a_exit) / jerk
    if (t3 < 0.0f) t3 = 0.0f;
    
    // Phase 1: jerk- (decreasing acceleration toward -a_max)
    float v1_end = jerkVelocity(v_entry, a_entry, -jerk, t1);
    float d1 = jerkDistance(v_entry, a_entry, -jerk, t1);
    
    // Phase 3: jerk+ (increasing acceleration from -a_max toward a_exit)
    float v3_start = v_exit + 0.5f * jerk * t3 * t3 + a_exit * t3;  // Working backward
    float d3 = jerkDistance(v3_start, -a_max, jerk, t3);
    
    // Check if triangular
    if (v1_end <= v3_start) {
        // Triangular profile
        float delta_v = v_entry - v_exit;
        if (delta_v <= 0.0f) return 0.0f;
        
        float t = safeSqrt(2.0f * delta_v / jerk);
        return v_entry * t - (1.0f / 6.0f) * jerk * t * t * t;
    }
    
    // Phase 2: constant deceleration at -a_max
    float d2 = (sq(v1_end) - sq(v3_start)) / (2.0f * a_max);
    
    return d1 + d2 + d3;
}

// ============================================================================
// Solver functions
// ============================================================================

float solveMaxEntrySpeed(float distance, float v_exit, float a_exit,
                         float a_max, float jerk) {
    // Binary search for max entry speed that can decelerate to v_exit in distance
    // This is conservative - we want the highest speed that definitely works
    
    float v_low = v_exit;
    float v_high = v_exit + safeSqrt(2.0f * a_max * distance) * 1.5f;  // Upper bound estimate
    
    const int MAX_ITER = 20;
    const float TOLERANCE = 0.1f;  // mm/min tolerance
    
    for (int i = 0; i < MAX_ITER; i++) {
        float v_mid = (v_low + v_high) / 2.0f;
        float d_needed = decelDistanceWithAccel(v_mid, 0.0f, v_exit, a_exit, a_max, jerk);
        
        if (d_needed > distance) {
            v_high = v_mid;  // Too fast, need to slow down
        } else {
            v_low = v_mid;   // Can go faster
        }
        
        if (v_high - v_low < TOLERANCE) break;
    }
    
    return v_low;  // Return conservative (lower) value
}

float solveMaxExitSpeed(float distance, float v_entry, float a_entry,
                        float a_max, float jerk) {
    // Binary search for max exit speed achievable from entry conditions
    
    float v_low = v_entry;
    float v_high = v_entry + safeSqrt(2.0f * a_max * distance) * 1.5f;
    
    const int MAX_ITER = 20;
    const float TOLERANCE = 0.1f;
    
    for (int i = 0; i < MAX_ITER; i++) {
        float v_mid = (v_low + v_high) / 2.0f;
        float d_needed = accelDistanceWithAccel(v_entry, a_entry, v_mid, 0.0f, a_max, jerk);
        
        if (d_needed > distance) {
            v_high = v_mid;
        } else {
            v_low = v_mid;
        }
        
        if (v_high - v_low < TOLERANCE) break;
    }
    
    return v_low;
}

float solveEntryAccel(float distance, float v_entry, float v_exit, float a_exit,
                      float a_max, float jerk) {
    // Find entry acceleration that produces the smoothest transition
    // For now, return 0 (simplified - assumes we want zero accel at junctions)
    // A more sophisticated implementation would solve for continuous jerk
    
    // The entry acceleration should be such that we can smoothly transition
    // to the required exit conditions. For continuous jerk, this typically
    // means the entry accel of the next block equals the exit accel of this block.
    
    // Simplified: return 0 for now
    return 0.0f;
}

float computePeakVelocity(float distance, float v_entry, float v_exit,
                          float v_max, float a_max, float jerk) {
    // Check if we can reach v_max given the distance constraints
    
    float d_accel = accelDistance(v_entry, v_max, a_max, jerk);
    float d_decel = decelDistance(v_max, v_exit, a_max, jerk);
    
    if (d_accel + d_decel <= distance) {
        return v_max;  // Can reach full speed
    }
    
    // Binary search for achievable peak velocity
    float v_low = std::max(v_entry, v_exit);
    float v_high = v_max;
    
    const int MAX_ITER = 20;
    const float TOLERANCE = 0.1f;
    
    for (int i = 0; i < MAX_ITER; i++) {
        float v_mid = (v_low + v_high) / 2.0f;
        float d_total = accelDistance(v_entry, v_mid, a_max, jerk) +
                        decelDistance(v_mid, v_exit, a_max, jerk);
        
        if (d_total > distance) {
            v_high = v_mid;
        } else {
            v_low = v_mid;
        }
        
        if (v_high - v_low < TOLERANCE) break;
    }
    
    return v_low;
}

// ============================================================================
// Profile planning
// ============================================================================

SCurveProfile planProfile(float distance, float v_entry, float a_entry,
                          float v_exit, float a_exit,
                          float v_max, float a_max, float jerk) {
    SCurveProfile profile = {};
    
    profile.v[0] = v_entry;
    profile.a[0] = a_entry;
    
    // Compute achievable peak velocity
    profile.v_peak = computePeakVelocity(distance, v_entry, v_exit, v_max, a_max, jerk);
    
    // Time for each jerk phase
    float t_jerk = a_max / jerk;
    
    // === Acceleration phases (0, 1, 2) ===
    
    // Phase 0: Jerk from a_entry to a_max
    float t0 = (a_max - a_entry) / jerk;
    if (t0 < 0.0f) t0 = 0.0f;
    profile.t[0] = t0;
    profile.v[1] = jerkVelocity(v_entry, a_entry, jerk, t0);
    profile.a[1] = a_max;
    profile.d[0] = jerkDistance(v_entry, a_entry, jerk, t0);
    
    // Check if we're already at or past peak velocity
    if (profile.v[1] >= profile.v_peak) {
        // Skip to deceleration
        profile.t[0] = 0.0f;
        profile.t[1] = 0.0f;
        profile.t[2] = 0.0f;
        profile.v[1] = v_entry;
        profile.v[2] = v_entry;
        profile.v[3] = v_entry;
        profile.a[1] = a_entry;
        profile.a[2] = a_entry;
        profile.a[3] = a_entry;
        profile.d[0] = 0.0f;
        profile.d[1] = 0.0f;
        profile.d[2] = 0.0f;
    } else {
        // Phase 2: Jerk from a_max to 0 (end of acceleration)
        float t2 = t_jerk;
        float v2_end_target = profile.v_peak;
        float v2_start = v2_end_target - 0.5f * jerk * t2 * t2;  // Working backward
        
        // Phase 1: Constant accel from v[1] to v2_start
        if (v2_start > profile.v[1]) {
            profile.t[1] = (v2_start - profile.v[1]) / a_max;
            profile.d[1] = profile.t[1] * (profile.v[1] + v2_start) / 2.0f;
            profile.v[2] = v2_start;
        } else {
            // No constant accel phase needed
            profile.t[1] = 0.0f;
            profile.d[1] = 0.0f;
            profile.v[2] = profile.v[1];
        }
        profile.a[2] = a_max;
        
        profile.t[2] = t2;
        profile.v[3] = profile.v_peak;
        profile.a[3] = 0.0f;
        profile.d[2] = jerkDistance(profile.v[2], a_max, -jerk, t2);
    }
    
    // === Cruise phase (3) ===
    
    // Calculate remaining distance for cruise and deceleration
    float d_accel = profile.d[0] + profile.d[1] + profile.d[2];
    float d_decel = decelDistance(profile.v_peak, v_exit, a_max, jerk);
    float d_cruise = distance - d_accel - d_decel;
    
    if (d_cruise > 0.0f) {
        profile.t[3] = d_cruise / profile.v_peak;
        profile.d[3] = d_cruise;
        profile.v[4] = profile.v_peak;
        profile.a[4] = 0.0f;
    } else {
        profile.t[3] = 0.0f;
        profile.d[3] = 0.0f;
        profile.v[4] = profile.v_peak;
        profile.a[4] = 0.0f;
        profile.degenerate = true;
    }
    
    // === Deceleration phases (4, 5, 6) ===
    
    // Phase 4: Jerk from 0 to -a_max
    float t4 = t_jerk;
    profile.t[4] = t4;
    profile.v[5] = jerkVelocity(profile.v[4], 0.0f, -jerk, t4);
    profile.a[5] = -a_max;
    profile.d[4] = jerkDistance(profile.v[4], 0.0f, -jerk, t4);
    
    // Phase 6: Jerk from -a_max to a_exit
    float t6 = (-a_max - a_exit) / (-jerk);
    if (t6 < 0.0f) t6 = 0.0f;
    profile.t[6] = t6;
    
    // Phase 5: Constant decel
    float v5_end = v_exit + 0.5f * jerk * t6 * t6;  // Working backward from v_exit
    if (v5_end < profile.v[5]) {
        profile.t[5] = (profile.v[5] - v5_end) / a_max;
        profile.d[5] = profile.t[5] * (profile.v[5] + v5_end) / 2.0f;
        profile.v[6] = v5_end;
    } else {
        profile.t[5] = 0.0f;
        profile.d[5] = 0.0f;
        profile.v[6] = profile.v[5];
    }
    profile.a[6] = -a_max;
    
    profile.v[7] = v_exit;
    profile.a[7] = a_exit;
    profile.d[6] = jerkDistance(profile.v[6], -a_max, jerk, t6);
    
    // Calculate totals
    profile.total_distance = 0.0f;
    profile.total_time = 0.0f;
    profile.phases_used = 0;
    
    for (int i = 0; i < 7; i++) {
        profile.total_distance += profile.d[i];
        profile.total_time += profile.t[i];
        if (profile.t[i] > 0.0001f) {
            profile.phases_used |= (1 << i);
        }
    }
    
    return profile;
}

} // namespace SCurveMath
