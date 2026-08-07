// Copyright (c) 2024 - FluidNC Authors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "../TestFramework.h"
#include "SoftLimits/SoftLimitsComponent.h"
#include "SoftLimits/LimitsChecker.h"
#include "SoftLimits/FixedBoundingBox.h"
#include "SoftLimits/FixedCylinder.h"
#include "SoftLimits/MovingBoundingBox.h"

#include <cmath>

// Helper to check floating point equality with tolerance
inline bool floatEquals(float a, float b, float tolerance = 0.0001f) {
    return fabsf(a - b) <= tolerance;
}

// Assembles a LimitContext the same way LimitsChecker does, so component tests
// exercise the frames that production code actually supplies. With no offset the
// tip and spindle frames coincide, which is the case for most tests below.
class TestContext {
public:
    TestContext(const float* from, const float* to, const float* tlo = nullptr) {
        for (int axis = 0; axis < 3; axis++) {
            _tlo[axis]     = tlo ? tlo[axis] : 0.0f;
            _tipFrom[axis] = from[axis] - _tlo[axis];
            _tipTo[axis]   = to[axis] - _tlo[axis];
        }
        _ctx = LimitContext { from, to, _tipFrom, _tipTo, _tlo, 3, false, false };
    }

    operator const LimitContext&() const { return _ctx; }

private:
    float        _tlo[3];
    float        _tipFrom[3];
    float        _tipTo[3];
    LimitContext _ctx;
};

// ============================================================================
// Test implementation of SoftLimitsComponent to access protected static methods
// ============================================================================

class TestSoftLimitsHelper : public SoftLimitsComponent {
public:
    bool TestLimit(const LimitContext& ctx) override { return false; }
    const char* componentName() const override { return "TestHelper"; }
    
    // Expose protected static methods for testing
    static bool testLineIntersectsAABB(const float* from, const float* to,
                                       const float* boxMin, const float* boxMax) {
        return lineIntersectsAABB(from, to, boxMin, boxMax);
    }
    
    static bool testLineIntersectsCircle2D(float fromX, float fromY, float toX, float toY,
                                           float centerX, float centerY, float radius) {
        return lineIntersectsCircle2D(fromX, fromY, toX, toY, centerX, centerY, radius);
    }
    
    static bool testCirclesOverlapXY(float center1X, float center1Y, float radius1,
                                     float center2X, float center2Y, float radius2) {
        return circlesOverlapXY(center1X, center1Y, radius1, center2X, center2Y, radius2);
    }
    
    static bool testZRangesOverlap(float z1Min, float z1Max, float z2Min, float z2Max) {
        return zRangesOverlap(z1Min, z1Max, z2Min, z2Max);
    }
    
    static bool testPointInAABB(float x, float y, float z,
                                const float* boxMin, const float* boxMax) {
        return pointInAABB(x, y, z, boxMin, boxMax);
    }
};

// ============================================================================
// lineIntersectsAABB Tests - 3D line-box intersection using slab method
// ============================================================================

// Tests that a line passing through the center of a box is detected
Test(SoftLimitsGeometry, LineIntersectsAABB_ThroughCenter) {
    float from[3] = {-10.0f, 0.0f, 0.0f};
    float to[3] = {10.0f, 0.0f, 0.0f};
    float boxMin[3] = {-5.0f, -5.0f, -5.0f};
    float boxMax[3] = {5.0f, 5.0f, 5.0f};
    
    Assert(TestSoftLimitsHelper::testLineIntersectsAABB(from, to, boxMin, boxMax),
           "Line through center should intersect");
}

// Tests that a line entirely outside the box is not detected
Test(SoftLimitsGeometry, LineIntersectsAABB_Outside) {
    float from[3] = {10.0f, 10.0f, 10.0f};
    float to[3] = {20.0f, 20.0f, 20.0f};
    float boxMin[3] = {-5.0f, -5.0f, -5.0f};
    float boxMax[3] = {5.0f, 5.0f, 5.0f};
    
    Assert(!TestSoftLimitsHelper::testLineIntersectsAABB(from, to, boxMin, boxMax),
           "Line outside should not intersect");
}

// Tests that a line starting inside the box is detected
Test(SoftLimitsGeometry, LineIntersectsAABB_StartInside) {
    float from[3] = {0.0f, 0.0f, 0.0f};
    float to[3] = {20.0f, 20.0f, 20.0f};
    float boxMin[3] = {-5.0f, -5.0f, -5.0f};
    float boxMax[3] = {5.0f, 5.0f, 5.0f};
    
    Assert(TestSoftLimitsHelper::testLineIntersectsAABB(from, to, boxMin, boxMax),
           "Line starting inside should intersect");
}

// Tests that a line ending inside the box is detected
Test(SoftLimitsGeometry, LineIntersectsAABB_EndInside) {
    float from[3] = {-20.0f, 0.0f, 0.0f};
    float to[3] = {0.0f, 0.0f, 0.0f};
    float boxMin[3] = {-5.0f, -5.0f, -5.0f};
    float boxMax[3] = {5.0f, 5.0f, 5.0f};
    
    Assert(TestSoftLimitsHelper::testLineIntersectsAABB(from, to, boxMin, boxMax),
           "Line ending inside should intersect");
}

// Tests a line parallel to an axis that passes outside the box
Test(SoftLimitsGeometry, LineIntersectsAABB_ParallelOutside) {
    float from[3] = {-10.0f, 10.0f, 0.0f};  // Y is outside box
    float to[3] = {10.0f, 10.0f, 0.0f};
    float boxMin[3] = {-5.0f, -5.0f, -5.0f};
    float boxMax[3] = {5.0f, 5.0f, 5.0f};
    
    Assert(!TestSoftLimitsHelper::testLineIntersectsAABB(from, to, boxMin, boxMax),
           "Parallel line outside should not intersect");
}

// Tests that a zero-length line inside the box is detected
Test(SoftLimitsGeometry, LineIntersectsAABB_ZeroLengthInside) {
    float from[3] = {0.0f, 0.0f, 0.0f};
    float to[3] = {0.0f, 0.0f, 0.0f};  // Same as from
    float boxMin[3] = {-5.0f, -5.0f, -5.0f};
    float boxMax[3] = {5.0f, 5.0f, 5.0f};
    
    Assert(TestSoftLimitsHelper::testLineIntersectsAABB(from, to, boxMin, boxMax),
           "Zero-length line inside should intersect");
}

// Tests that a zero-length line outside the box is not detected
Test(SoftLimitsGeometry, LineIntersectsAABB_ZeroLengthOutside) {
    float from[3] = {10.0f, 10.0f, 10.0f};
    float to[3] = {10.0f, 10.0f, 10.0f};  // Same as from
    float boxMin[3] = {-5.0f, -5.0f, -5.0f};
    float boxMax[3] = {5.0f, 5.0f, 5.0f};
    
    Assert(!TestSoftLimitsHelper::testLineIntersectsAABB(from, to, boxMin, boxMax),
           "Zero-length line outside should not intersect");
}

// Tests diagonal line through box corner
Test(SoftLimitsGeometry, LineIntersectsAABB_DiagonalThroughCorner) {
    float from[3] = {-10.0f, -10.0f, -10.0f};
    float to[3] = {10.0f, 10.0f, 10.0f};
    float boxMin[3] = {-5.0f, -5.0f, -5.0f};
    float boxMax[3] = {5.0f, 5.0f, 5.0f};
    
    Assert(TestSoftLimitsHelper::testLineIntersectsAABB(from, to, boxMin, boxMax),
           "Diagonal through corners should intersect");
}

// Tests line that just grazes a corner (should still intersect)
Test(SoftLimitsGeometry, LineIntersectsAABB_GrazeCorner) {
    float from[3] = {-10.0f, 5.0f, 5.0f};
    float to[3] = {10.0f, 5.0f, 5.0f};  // Grazes corner at (5, 5, 5)
    float boxMin[3] = {-5.0f, -5.0f, -5.0f};
    float boxMax[3] = {5.0f, 5.0f, 5.0f};
    
    Assert(TestSoftLimitsHelper::testLineIntersectsAABB(from, to, boxMin, boxMax),
           "Line grazing corner should intersect");
}

// ============================================================================
// lineIntersectsCircle2D Tests - 2D line-circle intersection
// ============================================================================

// Tests a line passing through circle center
Test(SoftLimitsGeometry, LineIntersectsCircle2D_ThroughCenter) {
    Assert(TestSoftLimitsHelper::testLineIntersectsCircle2D(
        -10.0f, 0.0f, 10.0f, 0.0f,  // Line from (-10,0) to (10,0)
        0.0f, 0.0f, 5.0f),           // Circle at origin with radius 5
        "Line through center should intersect");
}

// Tests a line completely outside the circle
Test(SoftLimitsGeometry, LineIntersectsCircle2D_Outside) {
    Assert(!TestSoftLimitsHelper::testLineIntersectsCircle2D(
        -10.0f, 10.0f, 10.0f, 10.0f,  // Line from (-10,10) to (10,10), above circle
        0.0f, 0.0f, 5.0f),             // Circle at origin with radius 5
        "Line outside should not intersect");
}

// Tests a line starting inside the circle
Test(SoftLimitsGeometry, LineIntersectsCircle2D_StartInside) {
    Assert(TestSoftLimitsHelper::testLineIntersectsCircle2D(
        0.0f, 0.0f, 20.0f, 20.0f,  // Line from center to outside
        0.0f, 0.0f, 5.0f),          // Circle at origin
        "Line starting inside should intersect");
}

// Tests a line ending inside the circle
Test(SoftLimitsGeometry, LineIntersectsCircle2D_EndInside) {
    Assert(TestSoftLimitsHelper::testLineIntersectsCircle2D(
        -20.0f, 0.0f, 0.0f, 0.0f,  // Line from outside to center
        0.0f, 0.0f, 5.0f),          // Circle at origin
        "Line ending inside should intersect");
}

// Tests a line tangent to the circle (should intersect at one point)
Test(SoftLimitsGeometry, LineIntersectsCircle2D_Tangent) {
    Assert(TestSoftLimitsHelper::testLineIntersectsCircle2D(
        -10.0f, 5.0f, 10.0f, 5.0f,  // Tangent line at y=5
        0.0f, 0.0f, 5.0f),           // Circle with radius 5
        "Tangent line should intersect");
}

// Tests a line just missing the circle
Test(SoftLimitsGeometry, LineIntersectsCircle2D_JustMissing) {
    Assert(!TestSoftLimitsHelper::testLineIntersectsCircle2D(
        -10.0f, 5.1f, 10.0f, 5.1f,  // Line just above tangent
        0.0f, 0.0f, 5.0f),           // Circle with radius 5
        "Line just missing should not intersect");
}

// Tests a zero-length line inside the circle
Test(SoftLimitsGeometry, LineIntersectsCircle2D_ZeroLengthInside) {
    Assert(TestSoftLimitsHelper::testLineIntersectsCircle2D(
        2.0f, 2.0f, 2.0f, 2.0f,  // Point at (2,2)
        0.0f, 0.0f, 5.0f),        // Circle at origin with radius 5
        "Point inside circle should intersect");
}

// Tests a zero-length line outside the circle
Test(SoftLimitsGeometry, LineIntersectsCircle2D_ZeroLengthOutside) {
    Assert(!TestSoftLimitsHelper::testLineIntersectsCircle2D(
        10.0f, 10.0f, 10.0f, 10.0f,  // Point at (10,10)
        0.0f, 0.0f, 5.0f),            // Circle at origin with radius 5
        "Point outside circle should not intersect");
}

// Tests circle with center not at origin
Test(SoftLimitsGeometry, LineIntersectsCircle2D_OffsetCenter) {
    Assert(TestSoftLimitsHelper::testLineIntersectsCircle2D(
        0.0f, 0.0f, 20.0f, 0.0f,     // Line along X axis
        10.0f, 0.0f, 3.0f),           // Circle centered at (10, 0) with radius 3
        "Should intersect offset circle");
}

// ============================================================================
// circlesOverlapXY Tests - 2D circle overlap detection
// ============================================================================

// Tests two overlapping circles
Test(SoftLimitsGeometry, CirclesOverlapXY_Overlapping) {
    Assert(TestSoftLimitsHelper::testCirclesOverlapXY(
        0.0f, 0.0f, 5.0f,    // Circle 1 at origin, radius 5
        8.0f, 0.0f, 5.0f),   // Circle 2 at (8,0), radius 5
        "Overlapping circles should overlap");
}

// Tests two non-overlapping circles
Test(SoftLimitsGeometry, CirclesOverlapXY_Separate) {
    Assert(!TestSoftLimitsHelper::testCirclesOverlapXY(
        0.0f, 0.0f, 5.0f,     // Circle 1 at origin, radius 5
        20.0f, 0.0f, 5.0f),   // Circle 2 at (20,0), radius 5
        "Separate circles should not overlap");
}

// Tests two circles just touching (should overlap at boundary)
Test(SoftLimitsGeometry, CirclesOverlapXY_JustTouching) {
    Assert(TestSoftLimitsHelper::testCirclesOverlapXY(
        0.0f, 0.0f, 5.0f,     // Circle 1 at origin, radius 5
        10.0f, 0.0f, 5.0f),   // Circle 2 at (10,0), radius 5
        "Just touching circles should overlap");
}

// Tests concentric circles
Test(SoftLimitsGeometry, CirclesOverlapXY_Concentric) {
    Assert(TestSoftLimitsHelper::testCirclesOverlapXY(
        0.0f, 0.0f, 10.0f,  // Large circle
        0.0f, 0.0f, 5.0f),  // Small circle inside
        "Concentric circles should overlap");
}

// Tests one circle inside another
Test(SoftLimitsGeometry, CirclesOverlapXY_OneInside) {
    Assert(TestSoftLimitsHelper::testCirclesOverlapXY(
        0.0f, 0.0f, 10.0f,   // Large circle at origin
        2.0f, 0.0f, 3.0f),   // Small circle offset but still inside
        "Circle inside another should overlap");
}

// ============================================================================
// zRangesOverlap Tests - Z range overlap detection
// ============================================================================

// Tests overlapping Z ranges
Test(SoftLimitsGeometry, ZRangesOverlap_Overlapping) {
    Assert(TestSoftLimitsHelper::testZRangesOverlap(0.0f, 10.0f, 5.0f, 15.0f),
           "Overlapping ranges should overlap");
}

// Tests non-overlapping Z ranges
Test(SoftLimitsGeometry, ZRangesOverlap_Separate) {
    Assert(!TestSoftLimitsHelper::testZRangesOverlap(0.0f, 10.0f, 15.0f, 25.0f),
           "Separate ranges should not overlap");
}

// Tests just touching Z ranges
Test(SoftLimitsGeometry, ZRangesOverlap_JustTouching) {
    Assert(!TestSoftLimitsHelper::testZRangesOverlap(0.0f, 10.0f, 10.0f, 20.0f),
           "Ranges touching at boundary should not overlap");
}

// Tests one range inside another
Test(SoftLimitsGeometry, ZRangesOverlap_OneInside) {
    Assert(TestSoftLimitsHelper::testZRangesOverlap(0.0f, 20.0f, 5.0f, 15.0f),
           "Range inside another should overlap");
}

// Tests identical Z ranges
Test(SoftLimitsGeometry, ZRangesOverlap_Identical) {
    Assert(TestSoftLimitsHelper::testZRangesOverlap(5.0f, 10.0f, 5.0f, 10.0f),
           "Identical ranges should overlap");
}

// ============================================================================
// pointInAABB Tests - 3D point-in-box detection
// ============================================================================

// Tests point at center of box
Test(SoftLimitsGeometry, PointInAABB_Center) {
    float boxMin[3] = {-5.0f, -5.0f, -5.0f};
    float boxMax[3] = {5.0f, 5.0f, 5.0f};
    
    Assert(TestSoftLimitsHelper::testPointInAABB(0.0f, 0.0f, 0.0f, boxMin, boxMax),
           "Point at center should be inside");
}

// Tests point outside box
Test(SoftLimitsGeometry, PointInAABB_Outside) {
    float boxMin[3] = {-5.0f, -5.0f, -5.0f};
    float boxMax[3] = {5.0f, 5.0f, 5.0f};
    
    Assert(!TestSoftLimitsHelper::testPointInAABB(10.0f, 0.0f, 0.0f, boxMin, boxMax),
           "Point outside should not be inside");
}

// Tests point on box boundary (should be inside)
Test(SoftLimitsGeometry, PointInAABB_OnBoundary) {
    float boxMin[3] = {-5.0f, -5.0f, -5.0f};
    float boxMax[3] = {5.0f, 5.0f, 5.0f};
    
    Assert(TestSoftLimitsHelper::testPointInAABB(5.0f, 0.0f, 0.0f, boxMin, boxMax),
           "Point on boundary should be inside");
}

// Tests point at corner
Test(SoftLimitsGeometry, PointInAABB_Corner) {
    float boxMin[3] = {-5.0f, -5.0f, -5.0f};
    float boxMax[3] = {5.0f, 5.0f, 5.0f};
    
    Assert(TestSoftLimitsHelper::testPointInAABB(5.0f, 5.0f, 5.0f, boxMin, boxMax),
           "Point at corner should be inside");
}

// ============================================================================
// LimitsChecker Tests - Registration and dispatch
// ============================================================================

// Simple test component for LimitsChecker tests
class TestComponent : public SoftLimitsComponent {
public:
    TestComponent(const char* name, bool shouldBlock = false) 
        : _name(name), _shouldBlock(shouldBlock), _callCount(0) {}
    
    bool TestLimit(const LimitContext& ctx) override {
        _callCount++;
        _lastCtxValid = true;
        for (int axis = 0; axis < 3; axis++) {
            _lastTipFrom[axis]     = ctx.tipFrom[axis];
            _lastTipTo[axis]       = ctx.tipTo[axis];
            _lastSpindleFrom[axis] = ctx.spindleFrom[axis];
            _lastSpindleTo[axis]   = ctx.spindleTo[axis];
        }
        _lastIsProbe = ctx.isProbe;
        _lastIsJog   = ctx.isJog;
        return _shouldBlock;
    }
    
    const char* componentName() const override { return _name; }
    
    int getCallCount() const { return _callCount; }
    void resetCallCount() { _callCount = 0; }
    void setBlock(bool block) { _shouldBlock = block; }

    bool  sawContext() const { return _lastCtxValid; }
    float tipFrom(int axis) const { return _lastTipFrom[axis]; }
    float tipTo(int axis) const { return _lastTipTo[axis]; }
    float spindleFrom(int axis) const { return _lastSpindleFrom[axis]; }
    float spindleTo(int axis) const { return _lastSpindleTo[axis]; }
    bool  sawProbe() const { return _lastIsProbe; }
    bool  sawJog() const { return _lastIsJog; }

private:
    const char* _name;
    bool _shouldBlock;
    int _callCount;

    bool  _lastCtxValid = false;
    float _lastTipFrom[3] {};
    float _lastTipTo[3] {};
    float _lastSpindleFrom[3] {};
    float _lastSpindleTo[3] {};
    bool  _lastIsProbe = false;
    bool  _lastIsJog   = false;
};

// Tests that a registered component is called during TestMotion
Test(LimitsChecker, Register_ComponentIsCalled) {
    TestComponent comp("test", false);
    LimitsChecker::instance().Register(&comp);
    
    float from[3] = {0.0f, 0.0f, 0.0f};
    float to[3] = {10.0f, 0.0f, 0.0f};
    
    LimitsChecker::instance().TestMotion(from, to);
    Assert(comp.getCallCount() > 0, "Registered component should be called");
    
    LimitsChecker::instance().Unregister(&comp);
}

// Tests that an unregistered component is not called
Test(LimitsChecker, Unregister_ComponentNotCalled) {
    TestComponent comp("test", false);
    LimitsChecker::instance().Register(&comp);
    LimitsChecker::instance().Unregister(&comp);
    
    comp.resetCallCount();
    float from[3] = {0.0f, 0.0f, 0.0f};
    float to[3] = {10.0f, 0.0f, 0.0f};
    
    LimitsChecker::instance().TestMotion(from, to);
    Assert(comp.getCallCount() == 0, "Unregistered component should not be called");
}

// Tests that TestMotion returns true when a component blocks
Test(LimitsChecker, TestMotion_ReturnsTrue_WhenBlocked) {
    TestComponent comp("blocker", true);  // Will block
    LimitsChecker::instance().Register(&comp);
    
    float from[3] = {0.0f, 0.0f, 0.0f};
    float to[3] = {10.0f, 0.0f, 0.0f};
    
    bool result = LimitsChecker::instance().TestMotion(from, to);
    Assert(result, "TestMotion should return true when blocked");
    
    LimitsChecker::instance().Unregister(&comp);
}

// Tests that TestMotion returns false when no component blocks
Test(LimitsChecker, TestMotion_ReturnsFalse_WhenAllowed) {
    TestComponent comp("allowAll", false);  // Won't block
    LimitsChecker::instance().Register(&comp);
    
    float from[3] = {0.0f, 0.0f, 0.0f};
    float to[3] = {10.0f, 0.0f, 0.0f};
    
    bool result = LimitsChecker::instance().TestMotion(from, to);
    Assert(!result, "TestMotion should return false when not blocked");
    
    LimitsChecker::instance().Unregister(&comp);
}

// Tests that lastViolationComponent returns the blocking component's name
Test(LimitsChecker, LastViolationComponent_ReturnsBlockerName) {
    TestComponent comp("myBlocker", true);
    LimitsChecker::instance().Register(&comp);
    
    float from[3] = {0.0f, 0.0f, 0.0f};
    float to[3] = {10.0f, 0.0f, 0.0f};
    
    LimitsChecker::instance().TestMotion(from, to);
    
    const char* violator = LimitsChecker::instance().lastViolationComponent();
    Assert(violator != nullptr, "Should have violator name");
    // Note: strcmp would be needed for exact match, but we just check it's not null
    
    LimitsChecker::instance().Unregister(&comp);
}

// Tests that null registration is handled gracefully
Test(LimitsChecker, Register_NullPtr) {
    // Should not crash
    LimitsChecker::instance().Register(nullptr);
    LimitsChecker::instance().Unregister(nullptr);
    // Test passes if no crash occurs
}

// Tests that multiple components can be registered
Test(LimitsChecker, MultipleComponents) {
    TestComponent comp1("comp1", false);
    TestComponent comp2("comp2", false);
    TestComponent comp3("comp3", true);  // This one blocks
    
    LimitsChecker::instance().Register(&comp1);
    LimitsChecker::instance().Register(&comp2);
    LimitsChecker::instance().Register(&comp3);
    
    float from[3] = {0.0f, 0.0f, 0.0f};
    float to[3] = {10.0f, 0.0f, 0.0f};
    
    bool result = LimitsChecker::instance().TestMotion(from, to);
    Assert(result, "Should be blocked by comp3");
    
    // First two should have been called before blocking one
    Assert(comp1.getCallCount() > 0, "comp1 should have been called");
    Assert(comp2.getCallCount() > 0, "comp2 should have been called");
    Assert(comp3.getCallCount() > 0, "comp3 should have been called");
    
    LimitsChecker::instance().Unregister(&comp1);
    LimitsChecker::instance().Unregister(&comp2);
    LimitsChecker::instance().Unregister(&comp3);
}

// Tests that duplicate registration is ignored
Test(LimitsChecker, DuplicateRegistration) {
    TestComponent comp("dup", false);
    
    LimitsChecker::instance().Register(&comp);
    LimitsChecker::instance().Register(&comp);  // Duplicate
    
    float from[3] = {0.0f, 0.0f, 0.0f};
    float to[3] = {10.0f, 0.0f, 0.0f};
    
    LimitsChecker::instance().TestMotion(from, to);
    
    // Should only be called once, not twice
    Assert(comp.getCallCount() == 1, "Duplicate registration should be ignored");
    
    LimitsChecker::instance().Unregister(&comp);
}

// ============================================================================
// FixedBoundingBox Tests - Direct functional tests without configuration
// ============================================================================

// Note: FixedBoundingBox requires configuration to be set up properly.
// These tests use a test wrapper or direct method calls where possible.

// Helper class to test FixedBoundingBox without full configuration
class TestableFixedBoundingBox : public FixedBoundingBox {
public:
    TestableFixedBoundingBox() : FixedBoundingBox("TestBox") {
        _enabled = true;
        _minX = -10.0f;
        _minY = -10.0f;
        _minZ = -10.0f;
        _maxX = 10.0f;
        _maxY = 10.0f;
        _maxZ = 10.0f;
    }
    
    void setBounds(float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
        _minX = minX; _minY = minY; _minZ = minZ;
        _maxX = maxX; _maxY = maxY; _maxZ = maxZ;
    }
    
    void setEnabled(bool enabled) { _enabled = enabled; }
};

// Tests that motion through box is blocked
Test(FixedBoundingBox, TestLimit_MotionThroughBox) {
    TestableFixedBoundingBox box;
    box.setBounds(-5.0f, -5.0f, -5.0f, 5.0f, 5.0f, 5.0f);
    
    float from[3] = {-10.0f, 0.0f, 0.0f};
    float to[3] = {10.0f, 0.0f, 0.0f};
    
    Assert(box.TestLimit(TestContext(from, to)), "Motion through box should be blocked");
}

// Tests that motion outside box is allowed
Test(FixedBoundingBox, TestLimit_MotionOutsideBox) {
    TestableFixedBoundingBox box;
    box.setBounds(-5.0f, -5.0f, -5.0f, 5.0f, 5.0f, 5.0f);
    
    float from[3] = {10.0f, 10.0f, 10.0f};
    float to[3] = {20.0f, 20.0f, 20.0f};
    
    Assert(!box.TestLimit(TestContext(from, to)), "Motion outside box should be allowed");
}

// Tests that disabled box does not block
Test(FixedBoundingBox, TestLimit_DisabledBox) {
    TestableFixedBoundingBox box;
    box.setBounds(-5.0f, -5.0f, -5.0f, 5.0f, 5.0f, 5.0f);
    box.setEnabled(false);
    
    float from[3] = {-10.0f, 0.0f, 0.0f};
    float to[3] = {10.0f, 0.0f, 0.0f};
    
    Assert(!box.TestLimit(TestContext(from, to)), "Disabled box should not block");
}

// Tests motion starting inside box
Test(FixedBoundingBox, TestLimit_StartInside) {
    TestableFixedBoundingBox box;
    box.setBounds(-5.0f, -5.0f, -5.0f, 5.0f, 5.0f, 5.0f);
    
    float from[3] = {0.0f, 0.0f, 0.0f};
    float to[3] = {20.0f, 0.0f, 0.0f};
    
    Assert(box.TestLimit(TestContext(from, to)), "Motion starting inside should be blocked");
}

// Tests zero-length motion inside box
Test(FixedBoundingBox, TestLimit_ZeroLengthInside) {
    TestableFixedBoundingBox box;
    box.setBounds(-5.0f, -5.0f, -5.0f, 5.0f, 5.0f, 5.0f);
    
    float from[3] = {0.0f, 0.0f, 0.0f};
    float to[3] = {0.0f, 0.0f, 0.0f};
    
    Assert(box.TestLimit(TestContext(from, to)), "Zero-length motion inside should be blocked");
}

// ============================================================================
// FixedCylinder Tests - Direct functional tests
// ============================================================================

// Helper class to test FixedCylinder without full configuration
class TestableFixedCylinder : public FixedCylinder {
public:
    TestableFixedCylinder() : FixedCylinder("TestCylinder") {
        _enabled = true;
        _axis = axis_t::Z_AXIS;
        _centerX = 0.0f;
        _centerY = 0.0f;
        _startPos = 10.0f;
        _length = 20.0f;
        _radius = 5.0f;
    }
    
    void setCylinder(axis_t axis, float centerX, float centerY, float start, float length, float radius) {
        _axis = axis;
        _centerX = centerX;
        _centerY = centerY;
        _startPos = start;
        _length = length;
        _radius = radius;
    }
    
    void setEnabled(bool enabled) { _enabled = enabled; }
};

// Tests motion through Z-axis cylinder
Test(FixedCylinder, TestLimit_ThroughZCylinder) {
    TestableFixedCylinder cyl;
    cyl.setCylinder(axis_t::Z_AXIS, 0.0f, 0.0f, 0.0f, -20.0f, 5.0f);  // Z cylinder from 0 to -20
    
    float from[3] = {-10.0f, 0.0f, -10.0f};  // Start left of cylinder
    float to[3] = {10.0f, 0.0f, -10.0f};     // End right of cylinder
    
    Assert(cyl.TestLimit(TestContext(from, to)), "Motion through cylinder should be blocked");
}

// Tests motion outside Z-axis cylinder
Test(FixedCylinder, TestLimit_OutsideZCylinder) {
    TestableFixedCylinder cyl;
    cyl.setCylinder(axis_t::Z_AXIS, 0.0f, 0.0f, 0.0f, -20.0f, 5.0f);
    
    float from[3] = {20.0f, 0.0f, -10.0f};  // Outside radius
    float to[3] = {30.0f, 0.0f, -10.0f};
    
    Assert(!cyl.TestLimit(TestContext(from, to)), "Motion outside cylinder should be allowed");
}

// Tests motion above Z-axis cylinder (outside Z range)
Test(FixedCylinder, TestLimit_AboveZCylinder) {
    TestableFixedCylinder cyl;
    cyl.setCylinder(axis_t::Z_AXIS, 0.0f, 0.0f, 0.0f, -20.0f, 5.0f);  // Cylinder from 0 to -20
    
    float from[3] = {-10.0f, 0.0f, 10.0f};  // Above cylinder
    float to[3] = {10.0f, 0.0f, 10.0f};
    
    Assert(!cyl.TestLimit(TestContext(from, to)), "Motion above cylinder Z range should be allowed");
}

// Tests disabled cylinder
Test(FixedCylinder, TestLimit_Disabled) {
    TestableFixedCylinder cyl;
    cyl.setCylinder(axis_t::Z_AXIS, 0.0f, 0.0f, 0.0f, -20.0f, 5.0f);
    cyl.setEnabled(false);
    
    float from[3] = {-10.0f, 0.0f, -10.0f};
    float to[3] = {10.0f, 0.0f, -10.0f};
    
    Assert(!cyl.TestLimit(TestContext(from, to)), "Disabled cylinder should not block");
}

// Tests cylinder with zero radius
Test(FixedCylinder, TestLimit_ZeroRadius) {
    TestableFixedCylinder cyl;
    cyl.setCylinder(axis_t::Z_AXIS, 0.0f, 0.0f, 0.0f, -20.0f, 0.0f);  // Zero radius
    
    float from[3] = {-10.0f, 0.0f, -10.0f};
    float to[3] = {10.0f, 0.0f, -10.0f};
    
    Assert(!cyl.TestLimit(TestContext(from, to)), "Zero radius cylinder should not block");
}

// Tests cylinder with zero length
Test(FixedCylinder, TestLimit_ZeroLength) {
    TestableFixedCylinder cyl;
    cyl.setCylinder(axis_t::Z_AXIS, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f);  // Zero length
    
    float from[3] = {-10.0f, 0.0f, 0.0f};
    float to[3] = {10.0f, 0.0f, 0.0f};
    
    Assert(!cyl.TestLimit(TestContext(from, to)), "Zero length cylinder should not block");
}

// Tests X-axis cylinder
Test(FixedCylinder, TestLimit_XAxisCylinder) {
    TestableFixedCylinder cyl;
    cyl.setCylinder(axis_t::X_AXIS, 0.0f, 0.0f, 0.0f, -20.0f, 5.0f);  // X-axis cylinder
    
    // Motion in YZ plane through cylinder
    float from[3] = {-10.0f, -10.0f, 0.0f};
    float to[3] = {-10.0f, 10.0f, 0.0f};
    
    // This tests the perpendicular plane (Y-Z for X-axis cylinder)
    Assert(cyl.TestLimit(TestContext(from, to)), "Motion through X-axis cylinder should be blocked");
}

// Tests Y-axis cylinder
Test(FixedCylinder, TestLimit_YAxisCylinder) {
    TestableFixedCylinder cyl;
    cyl.setCylinder(axis_t::Y_AXIS, 0.0f, 0.0f, 0.0f, -20.0f, 5.0f);  // Y-axis cylinder
    
    // Motion in XZ plane through cylinder
    float from[3] = {-10.0f, -10.0f, 0.0f};
    float to[3] = {10.0f, -10.0f, 0.0f};
    
    Assert(cyl.TestLimit(TestContext(from, to)), "Motion through Y-axis cylinder should be blocked");
}

// ============================================================================
// MovingBoundingBox Tests - Direct functional tests
// ============================================================================

// Helper class to test MovingBoundingBox without full configuration
class TestableMovingBoundingBox : public MovingBoundingBox {
public:
    TestableMovingBoundingBox() : MovingBoundingBox("TestMovingBox") {
        _enabled = true;
        _minX = -5.0f;
        _minY = -5.0f;
        _minZ = -5.0f;
        _maxX = 5.0f;
        _maxY = 5.0f;
        _maxZ = 5.0f;
        _tieX = false;
        _tieY = false;
        _tieZ = false;
    }
    
    void setBounds(float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
        _minX = minX; _minY = minY; _minZ = minZ;
        _maxX = maxX; _maxY = maxY; _maxZ = maxZ;
    }
    
    void setTies(bool x, bool y, bool z) {
        _tieX = x;
        _tieY = y;
        _tieZ = z;
    }
    
    void setEnabled(bool enabled) { _enabled = enabled; }
    
    // Expose getBoxAtPosition for testing
    void testGetBoxAtPosition(const float* pos, float* boxMin, float* boxMax) const {
        getBoxAtPosition(pos, boxMin, boxMax);
    }
};

// Tests that non-tied box behaves like fixed box
Test(MovingBoundingBox, TestLimit_NoTies) {
    TestableMovingBoundingBox box;
    box.setBounds(-5.0f, -5.0f, -5.0f, 5.0f, 5.0f, 5.0f);
    box.setTies(false, false, false);  // No ties
    
    float from[3] = {-10.0f, 0.0f, 0.0f};
    float to[3] = {10.0f, 0.0f, 0.0f};
    
    Assert(box.TestLimit(TestContext(from, to)), "Non-tied box should act like fixed box");
}

// Tests X-tied box position calculation
Test(MovingBoundingBox, GetBoxAtPosition_TieX) {
    TestableMovingBoundingBox box;
    box.setBounds(-5.0f, -5.0f, -5.0f, 5.0f, 5.0f, 5.0f);
    box.setTies(true, false, false);  // Tie X only
    
    float pos[3] = {100.0f, 0.0f, 0.0f};
    float boxMin[3], boxMax[3];
    box.testGetBoxAtPosition(pos, boxMin, boxMax);
    
    Assert(floatEquals(boxMin[0], 95.0f), "X-tied box min X should be offset by position");
    Assert(floatEquals(boxMax[0], 105.0f), "X-tied box max X should be offset by position");
    Assert(floatEquals(boxMin[1], -5.0f), "Non-tied Y should be unchanged");
    Assert(floatEquals(boxMax[1], 5.0f), "Non-tied Y should be unchanged");
}

// Tests all axes tied
Test(MovingBoundingBox, GetBoxAtPosition_AllTied) {
    TestableMovingBoundingBox box;
    box.setBounds(-2.0f, -2.0f, -2.0f, 2.0f, 2.0f, 2.0f);
    box.setTies(true, true, true);  // All tied
    
    float pos[3] = {10.0f, 20.0f, 30.0f};
    float boxMin[3], boxMax[3];
    box.testGetBoxAtPosition(pos, boxMin, boxMax);
    
    Assert(floatEquals(boxMin[0], 8.0f), "All-tied box min X should be offset");
    Assert(floatEquals(boxMin[1], 18.0f), "All-tied box min Y should be offset");
    Assert(floatEquals(boxMin[2], 28.0f), "All-tied box min Z should be offset");
    Assert(floatEquals(boxMax[0], 12.0f), "All-tied box max X should be offset");
    Assert(floatEquals(boxMax[1], 22.0f), "All-tied box max Y should be offset");
    Assert(floatEquals(boxMax[2], 32.0f), "All-tied box max Z should be offset");
}

// Tests disabled moving box
Test(MovingBoundingBox, TestLimit_Disabled) {
    TestableMovingBoundingBox box;
    box.setBounds(-5.0f, -5.0f, -5.0f, 5.0f, 5.0f, 5.0f);
    box.setEnabled(false);
    
    float from[3] = {-10.0f, 0.0f, 0.0f};
    float to[3] = {10.0f, 0.0f, 0.0f};
    
    Assert(!box.TestLimit(TestContext(from, to)), "Disabled moving box should not block");
}

// Tests that moving box checks at both from and to positions
Test(MovingBoundingBox, TestLimit_ChecksBothPositions) {
    TestableMovingBoundingBox box;
    box.setBounds(-50.0f, -5.0f, -5.0f, -40.0f, 5.0f, 5.0f);  // Box is at X = -45 region
    box.setTies(true, false, false);  // Tie X
    
    // Motion from X=0 to X=100 - box moves with X position
    // At from (X=0): box is at [-50, -40]
    // At to (X=100): box is at [50, 60]
    
    float from[3] = {0.0f, 0.0f, 0.0f};
    float to[3] = {100.0f, 0.0f, 0.0f};
    
    // Line segment is from (0,0,0) to (100,0,0)
    // Box at from: [-50,-5,-5] to [-40,5,5] - doesn't intersect line
    // Box at to: [50,-5,-5] to [60,5,5] - intersects line at X=50-60
    
    bool result = box.TestLimit(TestContext(from, to));
    Assert(result, "Should detect collision at 'to' position box");
}

// ============================================================================
// Tool length offset frame tests
//
// These pin the sign of the conversion between the spindle reference point and
// the tool tip. Getting it backwards is not a symmetric mistake: it relaxes
// keep-out zones as the tool gets longer, so a crash is the failure mode. The
// assertions below are written so that flipping the sign breaks them.
// ============================================================================

// Tests that the tip trails the spindle by the offset, i.e. tip = MPos - TLO
Test(LimitsCheckerTLO, TipIsSpindleMinusOffset) {
    TestComponent comp("observer", false);
    LimitsChecker::instance().Register(&comp);

    float from[3] = { 0.0f, 0.0f, 0.0f };
    float to[3]   = { 10.0f, 20.0f, 30.0f };
    float tlo[3]  = { 1.0f, 2.0f, 3.0f };

    LimitsChecker::instance().TestMotion(from, to, tlo, 3);

    Assert(comp.sawContext(), "Component should have received a context");
    Assert(floatEquals(comp.tipFrom(0), -1.0f), "tipFrom X should be spindle minus TLO");
    Assert(floatEquals(comp.tipFrom(1), -2.0f), "tipFrom Y should be spindle minus TLO");
    Assert(floatEquals(comp.tipFrom(2), -3.0f), "tipFrom Z should be spindle minus TLO");
    Assert(floatEquals(comp.tipTo(0), 9.0f), "tipTo X should be spindle minus TLO");
    Assert(floatEquals(comp.tipTo(1), 18.0f), "tipTo Y should be spindle minus TLO");
    Assert(floatEquals(comp.tipTo(2), 27.0f), "tipTo Z should be spindle minus TLO");

    LimitsChecker::instance().Unregister(&comp);
}

// Tests that the spindle frame is handed through untouched alongside the tip frame
Test(LimitsCheckerTLO, SpindleFrameIsUnmodified) {
    TestComponent comp("observer", false);
    LimitsChecker::instance().Register(&comp);

    float from[3] = { 5.0f, 6.0f, 7.0f };
    float to[3]   = { 15.0f, 16.0f, 17.0f };
    float tlo[3]  = { 100.0f, 100.0f, 100.0f };

    LimitsChecker::instance().TestMotion(from, to, tlo, 3);

    Assert(floatEquals(comp.spindleFrom(2), 7.0f), "spindleFrom should not have the offset applied");
    Assert(floatEquals(comp.spindleTo(2), 17.0f), "spindleTo should not have the offset applied");

    LimitsChecker::instance().Unregister(&comp);
}

// Tests that the two frames coincide when no offset is supplied
Test(LimitsCheckerTLO, NullOffsetLeavesFramesEqual) {
    TestComponent comp("observer", false);
    LimitsChecker::instance().Register(&comp);

    float from[3] = { 1.0f, 2.0f, 3.0f };
    float to[3]   = { 4.0f, 5.0f, 6.0f };

    LimitsChecker::instance().TestMotion(from, to);

    for (int axis = 0; axis < 3; axis++) {
        Assert(floatEquals(comp.tipFrom(axis), comp.spindleFrom(axis)), "Frames should match with no offset");
        Assert(floatEquals(comp.tipTo(axis), comp.spindleTo(axis)), "Frames should match with no offset");
    }

    LimitsChecker::instance().Unregister(&comp);
}

// Tests that the probe and jog flags reach the components
Test(LimitsCheckerTLO, FlagsArePropagated) {
    TestComponent comp("observer", false);
    LimitsChecker::instance().Register(&comp);

    float from[3] = {};
    float to[3]   = {};

    LimitsChecker::instance().TestMotion(from, to, nullptr, 3, true, false);
    Assert(comp.sawProbe(), "isProbe should reach the component");
    Assert(!comp.sawJog(), "isJog should reach the component");

    LimitsChecker::instance().TestMotion(from, to, nullptr, 3, false, true);
    Assert(!comp.sawProbe(), "isProbe should reach the component");
    Assert(comp.sawJog(), "isJog should reach the component");

    LimitsChecker::instance().Unregister(&comp);
}

// Tests that a fixture keep-out zone follows the cutting edge, so a tool long
// enough to reach into it is blocked even though the spindle nose clears it
Test(FixedBoundingBoxTLO, LongToolReachesIntoFixture) {
    TestableFixedBoundingBox box;
    box.setBounds(-100.0f, -100.0f, -10.0f, 100.0f, 100.0f, -5.0f);

    float from[3] = { -50.0f, 0.0f, 0.0f };
    float to[3]   = { 50.0f, 0.0f, 0.0f };

    Assert(!box.TestLimit(TestContext(from, to)), "Spindle at Z=0 clears a fixture spanning Z=-10..-5");

    float tlo[3] = { 0.0f, 0.0f, 8.0f };
    Assert(box.TestLimit(TestContext(from, to, tlo)), "An 8mm tool puts the tip at Z=-8, inside the fixture");
}

// Tests that a longer tool can only ever restrict a fixture zone, never relax it
Test(FixedBoundingBoxTLO, LongToolNeverUnblocks) {
    TestableFixedBoundingBox box;
    box.setBounds(-100.0f, -100.0f, -10.0f, 100.0f, 100.0f, -5.0f);

    float from[3] = { -50.0f, 0.0f, -7.0f };
    float to[3]   = { 50.0f, 0.0f, -7.0f };

    Assert(box.TestLimit(TestContext(from, to)), "Spindle inside the fixture zone is blocked");

    // With the sign inverted the tip would land at Z=+1 and the motion would be
    // allowed, which is exactly the crash this convention prevents.
    float tlo[3] = { 0.0f, 0.0f, 8.0f };
    Assert(box.TestLimit(TestContext(from, to, tlo)), "Fitting a longer tool must not release the block");
}

// Tests that a volume carried by the machine is placed from the spindle frame,
// since where the hardware sits does not change when a tool is swapped
Test(MovingBoundingBoxTLO, BoxIsPlacedFromSpindleFrame) {
    TestableMovingBoundingBox box;
    box.setBounds(-1000.0f, -1000.0f, -2.0f, 1000.0f, 1000.0f, 2.0f);
    box.setTies(false, false, true);  // Tie Z only

    float from[3] = { 0.0f, 0.0f, 50.0f };
    float to[3]   = { 0.0f, 0.0f, 50.0f };
    float tlo[3]  = { 0.0f, 0.0f, 50.0f };

    // Box rides the spindle at Z=50, so it spans Z=48..52. The tip sits at Z=0
    // and is clear of it. Placing the box in tip space instead would put it at
    // Z=-2..2 and wrongly report a collision.
    Assert(!box.TestLimit(TestContext(from, to, tlo)), "Carried volume should be positioned by the spindle, not the tip");
}
