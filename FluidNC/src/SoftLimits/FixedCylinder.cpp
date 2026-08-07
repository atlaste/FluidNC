// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "FixedCylinder.h"
#include "LimitsChecker.h"
#include "../Logging.h"
#include "../Configuration/HandlerBase.h"
#include <cctype>
#include <cmath>
#include <algorithm>

FixedCylinder::~FixedCylinder() {
    LimitsChecker::instance().Unregister(this);
}

void FixedCylinder::group(Configuration::HandlerBase& handler) {
    handler.item("enabled", _enabled);
    handler.item("axis", _axis);
    handler.item("center_x", _centerX);
    handler.item("center_y", _centerY);
    handler.item("start", _startPos);
    handler.item("length", _length);
    handler.item("radius", _radius);
}

void FixedCylinder::afterParse() {
    if (_enabled && _radius > 0 && _length != 0) {
        log_info("SoftLimits: Configured cylinder '" << name() << "' axis=" << _axis
                 << " center=[" << _centerX << "," << _centerY << "]"
                 << " start=" << _startPos << " length=" << _length
                 << " radius=" << _radius);
    }
}

void FixedCylinder::init() {
    if (_enabled && _radius > 0 && _length != 0) {
        LimitsChecker::instance().Register(this);
    }
}

void FixedCylinder::getAxisIndices(int& axisIdx, int& perpIdx1, int& perpIdx2) const {
    switch (_axis) {
        case axis_t::X_AXIS:
            axisIdx = 0;
            perpIdx1 = 1;
            perpIdx2 = 2;
            break;
        case axis_t::Y_AXIS:
            axisIdx = 1;
            perpIdx1 = 0;
            perpIdx2 = 2;
            break;
        case axis_t::Z_AXIS:
        default:
            axisIdx = 2;
            perpIdx1 = 0;
            perpIdx2 = 1;
            break;
    }
}

bool FixedCylinder::TestLimit(const LimitContext& ctx) {
    if (!_enabled || _radius <= 0 || _length == 0) {
        return false;
    }

    // A chuck or fixture is fixed to the machine, so the cutting edge is what
    // has to clear it.
    const float* from = ctx.tipFrom;
    const float* to   = ctx.tipTo;

    int axisIdx, perpIdx1, perpIdx2;
    getAxisIndices(axisIdx, perpIdx1, perpIdx2);

    float axisStart = _startPos;
    float axisEnd = _startPos + _length;  // length can be positive or negative
    if (axisEnd < axisStart) {
        std::swap(axisStart, axisEnd);  // Ensure axisStart <= axisEnd
    }

    float centerPerp1, centerPerp2;
    if (_axis == axis_t::Z_AXIS) {
        centerPerp1 = _centerX;
        centerPerp2 = _centerY;
    } else if (_axis == axis_t::X_AXIS) {
        centerPerp1 = _centerX;
        centerPerp2 = _centerY;
    } else if (_axis == axis_t::Y_AXIS) {
        centerPerp1 = _centerX;
        centerPerp2 = _centerY;
    }
    else {
        return false;
    }

    float fromAxis = from[axisIdx];
    float toAxis = to[axisIdx];
    float segAxisMin = std::min(fromAxis, toAxis);
    float segAxisMax = std::max(fromAxis, toAxis);

    // Motion is completely outside cylinder's axis range if:
    // - motion ends before cylinder starts (segAxisMax < axisStart), OR
    // - motion starts after cylinder ends (segAxisMin > axisEnd)
    if (segAxisMax < axisStart || segAxisMin > axisEnd) {
        return false;
    }

    float fromPerp1 = from[perpIdx1];
    float fromPerp2 = from[perpIdx2];
    float toPerp1 = to[perpIdx1];
    float toPerp2 = to[perpIdx2];

    if (lineIntersectsCircle2D(fromPerp1, fromPerp2, toPerp1, toPerp2,
                                centerPerp1, centerPerp2, _radius)) {
        log_debug("SoftLimits: Motion blocked by cylinder '" << name() << "'");
        return true;
    }

    return false;
}

#ifndef _MSC_VER
namespace {
    ConfigurableModuleFactory::InstanceBuilder<FixedCylinder> __attribute__((init_priority(111))) fixed_cylinder_registration("fixed_cylinder");
}
#endif
