// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "MovingBoundingBox.h"
#include "LimitsChecker.h"
#include "../Logging.h"
#include "../Configuration/HandlerBase.h"
#include <algorithm>

MovingBoundingBox::~MovingBoundingBox() {
    LimitsChecker::instance().Unregister(this);
}

void MovingBoundingBox::group(Configuration::HandlerBase& handler) {
    handler.item("enabled", _enabled);
    handler.item("min_x", _minX);
    handler.item("min_y", _minY);
    handler.item("min_z", _minZ);
    handler.item("max_x", _maxX);
    handler.item("max_y", _maxY);
    handler.item("max_z", _maxZ);
    handler.item("tie_x", _tieX);
    handler.item("tie_y", _tieY);
    handler.item("tie_z", _tieZ);
}

void MovingBoundingBox::afterParse() {
    if (_minX > _maxX) std::swap(_minX, _maxX);
    if (_minY > _maxY) std::swap(_minY, _maxY);
    if (_minZ > _maxZ) std::swap(_minZ, _maxZ);
}

void MovingBoundingBox::init() {
    if (_enabled) {
        LimitsChecker::instance().Register(this);
        log_info("SoftLimits: Registered moving bounding box '" << name() << "' ["
                 << _minX << "," << _minY << "," << _minZ << "] to ["
                 << _maxX << "," << _maxY << "," << _maxZ << "] tie_x=" << _tieX
                 << " tie_y=" << _tieY << " tie_z=" << _tieZ);
    }
}

void MovingBoundingBox::getBoxAtPosition(const float* pos, float* boxMin, float* boxMax) const {
    boxMin[0] = _minX + (_tieX ? pos[0] : 0.0f);
    boxMin[1] = _minY + (_tieY ? pos[1] : 0.0f);
    boxMin[2] = _minZ + (_tieZ ? pos[2] : 0.0f);
    boxMax[0] = _maxX + (_tieX ? pos[0] : 0.0f);
    boxMax[1] = _maxY + (_tieY ? pos[1] : 0.0f);
    boxMax[2] = _maxZ + (_tieZ ? pos[2] : 0.0f);
}

bool MovingBoundingBox::TestLimit(const float* from, const float* to) {
    if (!_enabled) {
        return false;
    }

    float fromMin[3], fromMax[3], toMin[3], toMax[3];
    getBoxAtPosition(from, fromMin, fromMax);
    getBoxAtPosition(to, toMin, toMax);

    // Segment intersects the moving volume if it hits the box at start or at end
    if (lineIntersectsAABB(from, to, fromMin, fromMax)) {
        log_debug("SoftLimits: Motion blocked by moving box '" << name() << "' (at from)");
        return true;
    }
    if (lineIntersectsAABB(from, to, toMin, toMax)) {
        log_debug("SoftLimits: Motion blocked by moving box '" << name() << "' (at to)");
        return true;
    }

    return false;
}

namespace {
    ConfigurableModuleFactory::InstanceBuilder<MovingBoundingBox> __attribute__((init_priority(111))) moving_box_registration("moving_box");
}
