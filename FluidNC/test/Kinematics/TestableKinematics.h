// Copyright (c) 2026 - FluidNC Authors
// Testable wrappers for Kinematics classes.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

// =============================================================================
// Include all system and FluidNC infrastructure headers FIRST with normal
// access control.  The #pragma once guards in these headers prevent them
// from being re-parsed when the kinematics headers pull them in below.
// =============================================================================
#include "Configuration/Configurable.h"
#include "Configuration/GenericFactory.h"
#include "MotionControl.h"
#include "System.h"
#include "Planner.h"
#include "Types.h"
#include "Machine/Homing.h"
#include "Machine/MachineConfig.h"

#include <vector>
#include <string>
#include <cmath>

// =============================================================================
// Now include kinematics headers with all members accessible for testing.
// Because the dependency headers above are already included (with proper access
// control) the override only affects the kinematics class definitions.
// =============================================================================
#define private   public
#define protected public

#include "Kinematics/Kinematics.h"
#include "Kinematics/Cartesian.h"
#include "Kinematics/CoreXY.h"
#include "Kinematics/Midtbot.h"
#include "Kinematics/WallPlotter.h"
#include "Kinematics/ParallelDelta.h"

#undef private
#undef protected

// =============================================================================
// Compensated interpolation test helpers
//
// We do not compile Compensated1D.cpp / Compensated2D.cpp because they depend
// on FileStream, Channel and the filesystem layer.  Instead we replicate the
// interpolation algorithms here so that the exact math can be unit-tested.
// =============================================================================

namespace KinematicsTestHelpers {

    // -- Compensated1D interpolation (linear) ---------------------------------
    struct Comp1DGrid {
        float z_min;
        float z_max;
        float granularity;
        std::vector<float> offsets;
    };

    inline float comp1d_interpolate(const Comp1DGrid& g, float z) {
        if (g.offsets.empty()) {
            return 0.0f;
        }
        float idx_f  = (z - g.z_min) / g.granularity;
        int   idx    = int(idx_f);
        int   maxIdx = int(g.offsets.size()) - 1;

        if (idx < 0) return g.offsets[0];
        if (idx >= maxIdx) return g.offsets[maxIdx];

        float t = idx_f - idx;
        return g.offsets[idx] * (1.0f - t) + g.offsets[idx + 1] * t;
    }

    // -- Compensated2D interpolation (bicubic Catmull-Rom) --------------------
    struct Comp2DGrid {
        float x_min, x_max;
        float y_min, y_max;
        int   x_count, y_count;
        std::vector<float> offsets;   // row-major [y * x_count + x]
    };

    inline float comp2d_getOffsetClamped(const Comp2DGrid& g, int ix, int iy) {
        ix = std::clamp(ix, 0, g.x_count - 1);
        iy = std::clamp(iy, 0, g.y_count - 1);
        return g.offsets[iy * g.x_count + ix];
    }

    inline float comp2d_cubicInterpolate(float t, float p0, float p1, float p2, float p3) {
        return p1 + 0.5f * t * (p2 - p0 + t * (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3 +
                                                 t * (3.0f * (p1 - p2) + p3 - p0)));
    }

    inline float comp2d_bicubicInterpolate(const Comp2DGrid& g, int ix, int iy, float tx, float ty) {
        float p[4][4];
        for (int j = -1; j <= 2; j++) {
            for (int i = -1; i <= 2; i++) {
                p[j + 1][i + 1] = comp2d_getOffsetClamped(g, ix + i, iy + j);
            }
        }
        float col[4];
        for (int j = 0; j < 4; j++) {
            col[j] = comp2d_cubicInterpolate(tx, p[j][0], p[j][1], p[j][2], p[j][3]);
        }
        return comp2d_cubicInterpolate(ty, col[0], col[1], col[2], col[3]);
    }

    inline float comp2d_interpolate(const Comp2DGrid& g, float x, float y) {
        if (g.offsets.empty() || g.x_count < 2 || g.y_count < 2) {
            return 0.0f;
        }
        float x_step = (g.x_max - g.x_min) / (g.x_count - 1);
        float y_step = (g.y_max - g.y_min) / (g.y_count - 1);

        float fx = (x - g.x_min) / x_step;
        float fy = (y - g.y_min) / y_step;

        int ix = std::clamp(int(fx), 0, g.x_count - 2);
        int iy = std::clamp(int(fy), 0, g.y_count - 2);

        float tx = std::clamp(fx - ix, 0.0f, 1.0f);
        float ty = std::clamp(fy - iy, 0.0f, 1.0f);

        return comp2d_bicubicInterpolate(g, ix, iy, tx, ty);
    }

}  // namespace KinematicsTestHelpers
