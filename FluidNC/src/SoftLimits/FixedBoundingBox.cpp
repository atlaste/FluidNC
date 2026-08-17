// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "FixedBoundingBox.h"
#include "LimitsChecker.h"
#include "../Logging.h"
#include "../Configuration/HandlerBase.h"
#include <algorithm>

FixedBoundingBox::~FixedBoundingBox() {
    LimitsChecker::instance().Unregister(this);
}

void FixedBoundingBox::group(Configuration::HandlerBase& handler) {
    handler.item("enabled", _enabled);
    handler.item("suspend_during_tool_change", _suspend_during_tool_change);
    handler.item("min_x", _minX);
    handler.item("min_y", _minY);
    handler.item("min_z", _minZ);
    handler.item("max_x", _maxX);
    handler.item("max_y", _maxY);
    handler.item("max_z", _maxZ);
}

void FixedBoundingBox::afterParse() {
    if (_minX > _maxX) std::swap(_minX, _maxX);
    if (_minY > _maxY) std::swap(_minY, _maxY);
    if (_minZ > _maxZ) std::swap(_minZ, _maxZ);
}

void FixedBoundingBox::init() {
    if (_enabled) {
        LimitsChecker::instance().Register(this);
        log_info("SoftLimits: Registered fixed bounding box '" << name() << "' ["
                 << _minX << "," << _minY << "," << _minZ << "] to ["
                 << _maxX << "," << _maxY << "," << _maxZ << "]");
    }
}

bool FixedBoundingBox::TestLimit(const LimitContext& ctx) {
    if (!_enabled || suspendedByToolChange()) {
        return false;
    }

    float boxMin[3] = { _minX, _minY, _minZ };
    float boxMax[3] = { _maxX, _maxY, _maxZ };

    // The box guards something bolted to the table, so what must stay out of it
    // is the cutting edge, not the spindle nose.
    if (lineIntersectsAABB(ctx.tipFrom, ctx.tipTo, boxMin, boxMax)) {
        log_debug("SoftLimits: Motion blocked by bounding box '" << name() << "'");
        return true;
    }

    return false;
}

#ifndef _MSC_VER
namespace {
    ConfigurableModuleFactory::InstanceBuilder<FixedBoundingBox> __attribute__((init_priority(111))) fixed_box_registration("fixed_box");
}
#endif
