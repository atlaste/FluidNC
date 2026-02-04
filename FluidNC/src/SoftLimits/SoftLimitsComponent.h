// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../Types.h"
#include <algorithm>
#include <cmath>

// Base interface for components that can impose soft limits on motion.
// Components implementing this interface can test line segments (in machine position space)
// and report whether the motion would violate their limits.
//
// Examples:
// - Tool turret checking if tool path would collide with tailstock
// - Chuck/spindle defining a cylindrical exclusion zone
// - Fixed bounding box for vises or fixtures

class SoftLimitsComponent {
public:
    virtual ~SoftLimitsComponent() = default;

    // Test if line segment from->to violates this component's limits.
    // Coordinates are in machine position (MPos) space - physical Cartesian coordinates.
    // Returns true if limit violated (motion should be blocked).
    virtual bool TestLimit(const float* from, const float* to) = 0;

    // Name for error reporting
    virtual const char* componentName() const { return "unknown"; }

protected:
    // Geometry helpers - added incrementally as needed

    // Check if a line segment intersects an axis-aligned bounding box (3D)
    // Uses the slab method for efficient intersection testing
    static bool lineIntersectsAABB(const float* from, const float* to,
                                   const float* boxMin, const float* boxMax) {
        float tmin = 0.0f;
        float tmax = 1.0f;  // We only care about the segment [0, 1]

        for (int i = 0; i < 3; ++i) {
            float dir = to[i] - from[i];

            if (std::abs(dir) < 1e-8f) {
                // Ray is parallel to slab - check if origin is within slab
                if (from[i] < boxMin[i] || from[i] > boxMax[i]) {
                    return false;
                }
            } else {
                float invDir = 1.0f / dir;
                float t1 = (boxMin[i] - from[i]) * invDir;
                float t2 = (boxMax[i] - from[i]) * invDir;

                if (t1 > t2) {
                    std::swap(t1, t2);
                }

                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);

                if (tmin > tmax) {
                    return false;
                }
            }
        }

        return true;
    }

    // Check if a line segment intersects a circle in 2D (XY plane)
    // Returns true if the segment crosses into or through the circle
    static bool lineIntersectsCircle2D(float fromX, float fromY, float toX, float toY,
                                       float centerX, float centerY, float radius) {
        // Vector from center to line start
        float dx = fromX - centerX;
        float dy = fromY - centerY;

        // Direction of line segment
        float dirX = toX - fromX;
        float dirY = toY - fromY;

        // Quadratic coefficients: at^2 + bt + c = 0
        float a = dirX * dirX + dirY * dirY;
        float b = 2.0f * (dx * dirX + dy * dirY);
        float c = dx * dx + dy * dy - radius * radius;

        // Check if either endpoint is inside the circle
        if (c <= 0) return true;  // Start point inside
        float endDx = toX - centerX;
        float endDy = toY - centerY;
        if (endDx * endDx + endDy * endDy <= radius * radius) return true;  // End point inside

        // Check discriminant for intersection
        float discriminant = b * b - 4.0f * a * c;
        if (discriminant < 0) return false;

        // Find intersection parameter
        float sqrtDisc = std::sqrt(discriminant);
        float t1 = (-b - sqrtDisc) / (2.0f * a);
        float t2 = (-b + sqrtDisc) / (2.0f * a);

        // Check if intersection is within segment [0, 1]
        return (t1 >= 0.0f && t1 <= 1.0f) || (t2 >= 0.0f && t2 <= 1.0f);
    }

    // Check if two circles overlap in 2D (XY plane)
    // Used for cylinder vs. cylinder collision detection (with separate Z range check)
    static bool circlesOverlapXY(float center1X, float center1Y, float radius1,
                                 float center2X, float center2Y, float radius2) {
        float dx = center2X - center1X;
        float dy = center2Y - center1Y;
        float distSquared = dx * dx + dy * dy;
        float radiusSum = radius1 + radius2;
        return distSquared <= radiusSum * radiusSum;
    }

    // Check if two Z ranges overlap
    static bool zRangesOverlap(float z1Min, float z1Max, float z2Min, float z2Max) {
        return !(z1Max < z2Min || z2Max < z1Min);
    }

    // Check if a point is inside an AABB (3D)
    static bool pointInAABB(float x, float y, float z,
                            const float* boxMin, const float* boxMax) {
        return x >= boxMin[0] && x <= boxMax[0] &&
               y >= boxMin[1] && y <= boxMax[1] &&
               z >= boxMin[2] && z <= boxMax[2];
    }
};
