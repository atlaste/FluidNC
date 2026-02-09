// Copyright (c) 2026 - FluidNC Authors
// Unit tests for Kinematics transforms and interpolation algorithms.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "TestFramework.h"
#include "TestableKinematics.h"

#include <cmath>
#include <cstring>

namespace Kinematics {

// Tolerance for floating-point comparisons
static constexpr float TOLERANCE      = 1e-3f;
static constexpr float LOOSE_TOLERANCE = 0.05f;   // For trig-based round-trips

static bool near(float a, float b, float tol = TOLERANCE) {
    return std::fabs(a - b) <= tol;
}

// ============================================================================
//  CoreXY tests
// ============================================================================

Test(CoreXY, TransformCartesianToMotors_Origin) {
    CoreXY corexy("CoreXY");
    // _x_scaler defaults to 1.0

    float cartesian[MAX_N_AXIS] = { 0 };
    float motors[MAX_N_AXIS]    = { 0 };
    Machine::Axes::_numberAxis  = axis_t(3);

    bool ok = corexy.transform_cartesian_to_motors(motors, cartesian);
    Assert(ok, "transform should succeed");
    Assert(near(motors[X_AXIS], 0.0f), "M0 should be 0");
    Assert(near(motors[Y_AXIS], 0.0f), "M1 should be 0");
}

Test(CoreXY, TransformCartesianToMotors_XOnly) {
    CoreXY corexy("CoreXY");
    Machine::Axes::_numberAxis = axis_t(3);

    float cartesian[MAX_N_AXIS] = { 0 };
    float motors[MAX_N_AXIS]    = { 0 };
    cartesian[X_AXIS] = 10.0f;
    cartesian[Y_AXIS] = 0.0f;

    corexy.transform_cartesian_to_motors(motors, cartesian);
    // M0 = scaler*x + y = 1*10 + 0 = 10
    // M1 = scaler*x - y = 1*10 - 0 = 10
    Assert(near(motors[X_AXIS], 10.0f), "M0 should be 10");
    Assert(near(motors[Y_AXIS], 10.0f), "M1 should be 10");
}

Test(CoreXY, TransformCartesianToMotors_YOnly) {
    CoreXY corexy("CoreXY");
    Machine::Axes::_numberAxis = axis_t(3);

    float cartesian[MAX_N_AXIS] = { 0 };
    float motors[MAX_N_AXIS]    = { 0 };
    cartesian[X_AXIS] = 0.0f;
    cartesian[Y_AXIS] = 10.0f;

    corexy.transform_cartesian_to_motors(motors, cartesian);
    // M0 = scaler*0 + 10 = 10
    // M1 = scaler*0 - 10 = -10
    Assert(near(motors[X_AXIS],  10.0f), "M0 should be  10");
    Assert(near(motors[Y_AXIS], -10.0f), "M1 should be -10");
}

Test(CoreXY, TransformCartesianToMotors_BothAxes) {
    CoreXY corexy("CoreXY");
    Machine::Axes::_numberAxis = axis_t(3);

    float cartesian[MAX_N_AXIS] = { 0 };
    float motors[MAX_N_AXIS]    = { 0 };
    cartesian[X_AXIS] = 5.0f;
    cartesian[Y_AXIS] = 3.0f;

    corexy.transform_cartesian_to_motors(motors, cartesian);
    // M0 = 1*5 + 3 = 8
    // M1 = 1*5 - 3 = 2
    Assert(near(motors[X_AXIS], 8.0f), "M0 should be 8");
    Assert(near(motors[Y_AXIS], 2.0f), "M1 should be 2");
}

Test(CoreXY, MotorsToCartesian_Values) {
    CoreXY corexy("CoreXY");

    float cartesian[MAX_N_AXIS] = { 0 };
    float motors[MAX_N_AXIS]    = { 0 };
    motors[X_AXIS] = 8.0f;  // M0
    motors[Y_AXIS] = 2.0f;  // M1

    corexy.motors_to_cartesian(cartesian, motors, axis_t(3));
    // x = 0.5*(M0+M1)/scaler = 0.5*(8+2)/1 = 5
    // y = 0.5*(M0-M1)        = 0.5*(8-2)   = 3
    Assert(near(cartesian[X_AXIS], 5.0f), "X should be 5");
    Assert(near(cartesian[Y_AXIS], 3.0f), "Y should be 3");
}

Test(CoreXY, RoundTrip) {
    CoreXY corexy("CoreXY");
    Machine::Axes::_numberAxis = axis_t(3);

    float points[][2] = {
        { 0.0f, 0.0f }, { 10.0f, 0.0f }, { 0.0f, 10.0f },
        { 10.0f, 10.0f }, { -5.0f, 3.0f }, { 7.5f, -2.5f }
    };

    for (auto& pt : points) {
        float cartesian[MAX_N_AXIS] = { 0 };
        float motors[MAX_N_AXIS]    = { 0 };
        float result[MAX_N_AXIS]    = { 0 };

        cartesian[X_AXIS] = pt[0];
        cartesian[Y_AXIS] = pt[1];

        corexy.transform_cartesian_to_motors(motors, cartesian);
        corexy.motors_to_cartesian(result, motors, axis_t(3));

        Assert(near(result[X_AXIS], pt[0]), "X round-trip failed");
        Assert(near(result[Y_AXIS], pt[1]), "Y round-trip failed");
    }
}

Test(CoreXY, ZAxisPassthrough) {
    CoreXY corexy("CoreXY");
    Machine::Axes::_numberAxis = axis_t(3);

    float cartesian[MAX_N_AXIS] = { 0 };
    float motors[MAX_N_AXIS]    = { 0 };
    cartesian[Z_AXIS] = 42.0f;

    corexy.transform_cartesian_to_motors(motors, cartesian);
    Assert(near(motors[Z_AXIS], 42.0f), "Z motor should pass through");

    float result[MAX_N_AXIS] = { 0 };
    corexy.motors_to_cartesian(result, motors, axis_t(3));
    Assert(near(result[Z_AXIS], 42.0f), "Z cartesian should pass through");
}

// ============================================================================
//  Midtbot tests  (CoreXY with _x_scaler = 2.0)
// ============================================================================

Test(Midtbot, ScalerDoubled) {
    Midtbot midtbot("Midtbot");
    midtbot._x_scaler = 2.0f;   // normally set in init()
    Machine::Axes::_numberAxis = axis_t(3);

    float cartesian[MAX_N_AXIS] = { 0 };
    float motors[MAX_N_AXIS]    = { 0 };
    cartesian[X_AXIS] = 5.0f;
    cartesian[Y_AXIS] = 3.0f;

    midtbot.transform_cartesian_to_motors(motors, cartesian);
    // M0 = 2*5 + 3 = 13
    // M1 = 2*5 - 3 = 7
    Assert(near(motors[X_AXIS], 13.0f), "M0 should be 13");
    Assert(near(motors[Y_AXIS],  7.0f), "M1 should be 7");
}

Test(Midtbot, RoundTrip) {
    Midtbot midtbot("Midtbot");
    midtbot._x_scaler = 2.0f;
    Machine::Axes::_numberAxis = axis_t(3);

    float points[][2] = {
        { 0.0f, 0.0f }, { 10.0f, 0.0f }, { 0.0f, 10.0f },
        { 5.0f, 3.0f }, { -3.0f, 7.0f }
    };

    for (auto& pt : points) {
        float cartesian[MAX_N_AXIS] = { 0 };
        float motors[MAX_N_AXIS]    = { 0 };
        float result[MAX_N_AXIS]    = { 0 };

        cartesian[X_AXIS] = pt[0];
        cartesian[Y_AXIS] = pt[1];

        midtbot.transform_cartesian_to_motors(motors, cartesian);
        midtbot.motors_to_cartesian(result, motors, axis_t(3));

        Assert(near(result[X_AXIS], pt[0]), "X round-trip failed");
        Assert(near(result[Y_AXIS], pt[1]), "Y round-trip failed");
    }
}

// ============================================================================
//  ParallelDelta tests
// ============================================================================

Test(ParallelDelta, InverseKinematics_Center) {
    ParallelDelta delta("ParallelDelta");
    Machine::Axes::_numberAxis = axis_t(3);

    // Default geometry: rf=70, f=179.437, re=133.5, e=86.603
    // At cartesian (0, 0, -120) the robot should be reachable.
    float cartesian[MAX_N_AXIS] = { 0 };
    float motors[MAX_N_AXIS]    = { 0 };
    cartesian[Z_AXIS] = -120.0f;

    bool ok = delta.transform_cartesian_to_motors(motors, cartesian);
    Assert(ok, "Center point should be reachable");

    // Due to symmetry, all three arm angles should be equal
    Assert(near(motors[0], motors[1], LOOSE_TOLERANCE), "Arm 0 and 1 should be equal at center");
    Assert(near(motors[1], motors[2], LOOSE_TOLERANCE), "Arm 1 and 2 should be equal at center");
}

Test(ParallelDelta, ForwardKinematics_Center) {
    ParallelDelta delta("ParallelDelta");
    Machine::Axes::_numberAxis = axis_t(3);

    // First, get the motor angles for a known center point
    float cartesian_in[MAX_N_AXIS] = { 0 };
    float motors[MAX_N_AXIS]       = { 0 };
    cartesian_in[Z_AXIS] = -120.0f;

    bool ok = delta.transform_cartesian_to_motors(motors, cartesian_in);
    Assert(ok, "Inverse kinematics should succeed");

    // Now convert back
    float cartesian_out[MAX_N_AXIS] = { 0 };
    delta.motors_to_cartesian(cartesian_out, motors, axis_t(3));

    Assert(near(cartesian_out[X_AXIS], 0.0f, LOOSE_TOLERANCE), "X should be ~0");
    Assert(near(cartesian_out[Y_AXIS], 0.0f, LOOSE_TOLERANCE), "Y should be ~0");
    Assert(near(cartesian_out[Z_AXIS], -120.0f, LOOSE_TOLERANCE), "Z should be ~-120");
}

Test(ParallelDelta, RoundTrip_OffCenter) {
    ParallelDelta delta("ParallelDelta");
    Machine::Axes::_numberAxis = axis_t(3);

    // Test a few points within the work envelope
    float points[][3] = {
        {  0.0f,  0.0f, -120.0f },
        { 10.0f,  0.0f, -130.0f },
        {  0.0f, 10.0f, -130.0f },
        {  5.0f,  5.0f, -140.0f },
        { -5.0f,  3.0f, -125.0f },
    };

    for (auto& pt : points) {
        float cartesian[MAX_N_AXIS] = { 0 };
        float motors[MAX_N_AXIS]    = { 0 };
        float result[MAX_N_AXIS]    = { 0 };

        cartesian[X_AXIS] = pt[0];
        cartesian[Y_AXIS] = pt[1];
        cartesian[Z_AXIS] = pt[2];

        bool ok = delta.transform_cartesian_to_motors(motors, cartesian);
        Assert(ok, "Point should be reachable");

        delta.motors_to_cartesian(result, motors, axis_t(3));

        Assert(near(result[X_AXIS], pt[0], LOOSE_TOLERANCE), "X round-trip failed");
        Assert(near(result[Y_AXIS], pt[1], LOOSE_TOLERANCE), "Y round-trip failed");
        Assert(near(result[Z_AXIS], pt[2], LOOSE_TOLERANCE), "Z round-trip failed");
    }
}

Test(ParallelDelta, UnreachablePoint) {
    ParallelDelta delta("ParallelDelta");
    Machine::Axes::_numberAxis = axis_t(3);

    // A point far outside the work envelope should fail
    float cartesian[MAX_N_AXIS] = { 0 };
    float motors[MAX_N_AXIS]    = { 0 };
    cartesian[X_AXIS] = 500.0f;
    cartesian[Y_AXIS] = 500.0f;
    cartesian[Z_AXIS] = -10.0f;

    bool ok = delta.transform_cartesian_to_motors(motors, cartesian);
    Assert(!ok, "Unreachable point should fail");
}

// ============================================================================
//  WallPlotter tests  (using exposed private members)
// ============================================================================

Test(WallPlotter, XYToLengths_Origin) {
    WallPlotter wp("WallPlotter");
    // Default anchors: left=(-100,100), right=(100,100)

    float left_len = 0, right_len = 0;
    wp.xy_to_lengths(0.0f, 0.0f, left_len, right_len);

    // Distance from (-100,100) to (0,0) = sqrt(10000+10000) = sqrt(20000) ~ 141.42
    float expected = sqrtf(100.0f * 100.0f + 100.0f * 100.0f);
    Assert(near(left_len,  expected, 0.01f), "Left length at origin");
    Assert(near(right_len, expected, 0.01f), "Right length at origin (symmetric)");
}

Test(WallPlotter, XYToLengths_Offset) {
    WallPlotter wp("WallPlotter");

    float left_len = 0, right_len = 0;
    wp.xy_to_lengths(50.0f, 50.0f, left_len, right_len);

    // Distance from (-100,100) to (50,50) = sqrt(150^2 + 50^2) = sqrt(22500+2500) = sqrt(25000)
    float exp_left = sqrtf(150.0f * 150.0f + 50.0f * 50.0f);
    // Distance from (100,100) to (50,50) = sqrt(50^2 + 50^2) = sqrt(5000)
    float exp_right = sqrtf(50.0f * 50.0f + 50.0f * 50.0f);

    Assert(near(left_len,  exp_left,  0.01f), "Left length at (50,50)");
    Assert(near(right_len, exp_right, 0.01f), "Right length at (50,50)");
}

Test(WallPlotter, LengthsToXY_FromOrigin) {
    WallPlotter wp("WallPlotter");

    // Get lengths for origin, then reconstruct
    float left_len = 0, right_len = 0;
    wp.xy_to_lengths(0.0f, 0.0f, left_len, right_len);

    float x = 0, y = 0;
    wp.lengths_to_xy(left_len, right_len, x, y);

    Assert(near(x, 0.0f, 0.01f), "X should be 0");
    Assert(near(y, 0.0f, 0.01f), "Y should be 0");
}

Test(WallPlotter, RoundTrip) {
    WallPlotter wp("WallPlotter");

    float points[][2] = {
        { 0.0f, 0.0f }, { 30.0f, 20.0f }, { -30.0f, 20.0f },
        { 0.0f, 50.0f }, { 50.0f, 0.0f }, { -50.0f, 0.0f }
    };

    for (auto& pt : points) {
        float left_len = 0, right_len = 0;
        wp.xy_to_lengths(pt[0], pt[1], left_len, right_len);

        float x = 0, y = 0;
        wp.lengths_to_xy(left_len, right_len, x, y);

        Assert(near(x, pt[0], 0.01f), "X round-trip failed");
        Assert(near(y, pt[1], 0.01f), "Y round-trip failed");
    }
}

Test(WallPlotter, CustomAnchors) {
    WallPlotter wp("WallPlotter");
    wp._left_anchor_x  = -200.0f;
    wp._left_anchor_y  =  150.0f;
    wp._right_anchor_x =  200.0f;
    wp._right_anchor_y =  150.0f;

    float points[][2] = {
        { 0.0f, 0.0f }, { 50.0f, 50.0f }, { -50.0f, 50.0f }
    };

    for (auto& pt : points) {
        float left_len = 0, right_len = 0;
        wp.xy_to_lengths(pt[0], pt[1], left_len, right_len);

        float x = 0, y = 0;
        wp.lengths_to_xy(left_len, right_len, x, y);

        Assert(near(x, pt[0], 0.01f), "X round-trip failed (custom anchors)");
        Assert(near(y, pt[1], 0.01f), "Y round-trip failed (custom anchors)");
    }
}

// ============================================================================
//  Compensated1D interpolation tests
// ============================================================================

using namespace KinematicsTestHelpers;

Test(Compensated1D, InterpolateMiddle) {
    // Grid: z_min=0, z_max=10, granularity=1 => 11 points
    Comp1DGrid g;
    g.z_min       = 0.0f;
    g.z_max       = 10.0f;
    g.granularity = 1.0f;
    g.offsets.resize(11, 0.0f);

    // Set a ramp: offset = z * 0.1
    for (int i = 0; i <= 10; i++) {
        g.offsets[i] = i * 0.1f;
    }

    // At z=5 => offset=0.5
    Assert(near(comp1d_interpolate(g, 5.0f), 0.5f), "Interpolate at z=5");

    // At z=2.5 => between 0.2 and 0.3 => 0.25
    Assert(near(comp1d_interpolate(g, 2.5f), 0.25f), "Interpolate at z=2.5");

    // At z=7.3 => between 0.7 and 0.8 => 0.73
    Assert(near(comp1d_interpolate(g, 7.3f), 0.73f), "Interpolate at z=7.3");
}

Test(Compensated1D, EdgeClamping) {
    Comp1DGrid g;
    g.z_min       = 0.0f;
    g.z_max       = 10.0f;
    g.granularity = 1.0f;
    g.offsets.resize(11, 0.0f);
    g.offsets[0]  = 1.0f;
    g.offsets[10] = 5.0f;

    // Below range => clamp to first
    Assert(near(comp1d_interpolate(g, -5.0f), 1.0f), "Below range clamps to first");

    // Above range => clamp to last
    Assert(near(comp1d_interpolate(g, 20.0f), 5.0f), "Above range clamps to last");
}

Test(Compensated1D, ZeroOffsets) {
    Comp1DGrid g;
    g.z_min       = -100.0f;
    g.z_max       =  100.0f;
    g.granularity = 1.0f;
    g.offsets.resize(201, 0.0f);

    Assert(near(comp1d_interpolate(g, 0.0f),   0.0f), "Zero offsets at 0");
    Assert(near(comp1d_interpolate(g, 50.0f),  0.0f), "Zero offsets at 50");
    Assert(near(comp1d_interpolate(g, -50.0f), 0.0f), "Zero offsets at -50");
}

Test(Compensated1D, EmptyGrid) {
    Comp1DGrid g;
    g.z_min       = 0.0f;
    g.z_max       = 10.0f;
    g.granularity = 1.0f;
    // offsets is empty

    Assert(near(comp1d_interpolate(g, 5.0f), 0.0f), "Empty grid returns 0");
}

// ============================================================================
//  Compensated2D interpolation tests
// ============================================================================

Test(Compensated2D, CubicInterpolate_Endpoints) {
    // At t=0, result should be p1
    float r0 = comp2d_cubicInterpolate(0.0f, 1.0f, 2.0f, 3.0f, 4.0f);
    Assert(near(r0, 2.0f), "Cubic at t=0 should be p1");

    // At t=1, result should be p2
    float r1 = comp2d_cubicInterpolate(1.0f, 1.0f, 2.0f, 3.0f, 4.0f);
    Assert(near(r1, 3.0f), "Cubic at t=1 should be p2");
}

Test(Compensated2D, CubicInterpolate_Linear) {
    // When all points are on a line (1,2,3,4), the spline should give
    // the linear value at t=0.5 => 2.5
    float r = comp2d_cubicInterpolate(0.5f, 1.0f, 2.0f, 3.0f, 4.0f);
    Assert(near(r, 2.5f), "Cubic with linear data at t=0.5 should be 2.5");
}

Test(Compensated2D, FlatGrid) {
    // A flat grid (all offsets = 3.0) should interpolate to 3.0 everywhere
    Comp2DGrid g;
    g.x_min   = 0.0f;  g.x_max   = 100.0f;
    g.y_min   = 0.0f;  g.y_max   = 100.0f;
    g.x_count = 5;      g.y_count = 5;
    g.offsets.assign(25, 3.0f);

    Assert(near(comp2d_interpolate(g, 50.0f, 50.0f), 3.0f), "Flat grid center");
    Assert(near(comp2d_interpolate(g,  0.0f,  0.0f), 3.0f), "Flat grid corner (0,0)");
    Assert(near(comp2d_interpolate(g, 100.0f, 100.0f), 3.0f), "Flat grid corner (100,100)");
    Assert(near(comp2d_interpolate(g, 25.0f, 75.0f), 3.0f), "Flat grid arbitrary point");
}

Test(Compensated2D, LinearGradient) {
    // Grid with a linear gradient in X: offset = x_index
    Comp2DGrid g;
    g.x_min   = 0.0f;  g.x_max   = 40.0f;
    g.y_min   = 0.0f;  g.y_max   = 40.0f;
    g.x_count = 5;      g.y_count = 5;
    g.offsets.resize(25);

    for (int y = 0; y < 5; y++) {
        for (int x = 0; x < 5; x++) {
            g.offsets[y * 5 + x] = float(x);
        }
    }

    // At x=20 (grid center), the offset should be ~2.0
    float r = comp2d_interpolate(g, 20.0f, 20.0f);
    Assert(near(r, 2.0f, 0.01f), "Linear gradient at center");

    // At x=10 (between grid 0 and 1), offset should be ~1.0
    float r2 = comp2d_interpolate(g, 10.0f, 20.0f);
    Assert(near(r2, 1.0f, 0.1f), "Linear gradient at x=10");
}

Test(Compensated2D, ZeroOffsets) {
    Comp2DGrid g;
    g.x_min   = 0.0f;  g.x_max   = 200.0f;
    g.y_min   = 0.0f;  g.y_max   = 200.0f;
    g.x_count = 10;     g.y_count = 10;
    g.offsets.assign(100, 0.0f);

    Assert(near(comp2d_interpolate(g, 100.0f, 100.0f), 0.0f), "Zero grid center");
    Assert(near(comp2d_interpolate(g,   0.0f,   0.0f), 0.0f), "Zero grid origin");
}

Test(Compensated2D, EmptyGrid) {
    Comp2DGrid g;
    g.x_min   = 0.0f;  g.x_max   = 100.0f;
    g.y_min   = 0.0f;  g.y_max   = 100.0f;
    g.x_count = 0;      g.y_count = 0;

    Assert(near(comp2d_interpolate(g, 50.0f, 50.0f), 0.0f), "Empty grid returns 0");
}

Test(Compensated2D, SinglePeakGrid) {
    // Grid with a single peak at center
    Comp2DGrid g;
    g.x_min   = 0.0f;  g.x_max   = 40.0f;
    g.y_min   = 0.0f;  g.y_max   = 40.0f;
    g.x_count = 5;      g.y_count = 5;
    g.offsets.assign(25, 0.0f);

    // Set center point (2,2) to 1.0
    g.offsets[2 * 5 + 2] = 1.0f;

    // The center of the grid (x=20, y=20) should have the peak
    float center = comp2d_interpolate(g, 20.0f, 20.0f);
    Assert(center > 0.5f, "Peak should be visible at grid center");

    // Corners should be zero or near-zero
    float corner = comp2d_interpolate(g, 0.0f, 0.0f);
    Assert(near(corner, 0.0f, 0.1f), "Corner should be near zero");
}

Test(Compensated2D, OutOfRangeClamps) {
    Comp2DGrid g;
    g.x_min   = 0.0f;  g.x_max   = 100.0f;
    g.y_min   = 0.0f;  g.y_max   = 100.0f;
    g.x_count = 5;      g.y_count = 5;
    g.offsets.assign(25, 2.0f);

    // Way outside the grid should clamp and still return ~2.0
    Assert(near(comp2d_interpolate(g, -50.0f, -50.0f), 2.0f), "Below range clamps");
    Assert(near(comp2d_interpolate(g, 200.0f, 200.0f), 2.0f), "Above range clamps");
}

}  // namespace Kinematics
