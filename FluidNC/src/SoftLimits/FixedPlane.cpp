// Copyright (c) 2026 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "FixedPlane.h"
#include "LimitsChecker.h"
#include "../Logging.h"
#include "../Machine/MachineConfig.h"
#include "../Configuration/HandlerBase.h"

#include <algorithm>
#include <cstring>

FixedPlane::~FixedPlane() {
    LimitsChecker::instance().Unregister(this);
    DynamicLimits::unregisterProvider(this);
}

void FixedPlane::group(Configuration::HandlerBase& handler) {
    handler.item("enabled", _enabled);
    handler.item("suspend_during_tool_change", _suspend_during_tool_change);
    handler.item("min_mm", _min);
    handler.item("max_mm", _max);
    handler.item("unknown_tool_length_mm", _unknown_tool_length);

    std::string axis_str;
    axis_str += _axis_letter;
    handler.item("axis", axis_str);
    if (!axis_str.empty()) {
        _axis_letter = std::toupper(axis_str[0]);
    }
}

void FixedPlane::afterParse() {
    const char* axes = "XYZABCUVW";
    const char* p    = strchr(axes, _axis_letter);
    if (p != nullptr) {
        _axis = axis_t(p - axes);
    } else {
        log_warn("SoftLimits: plane '" << name() << "' has invalid axis '" << _axis_letter << "', defaulting to Z");
        _axis        = Z_AXIS;
        _axis_letter = 'Z';
    }

    if (!std::isnan(_min) && !std::isnan(_max) && _min > _max) {
        std::swap(_min, _max);
    }

    if (_unknown_tool_length < 0.0f) {
        _unknown_tool_length = 0.0f;
    }
}

void FixedPlane::init() {
    if (!_enabled) {
        return;
    }

    if (std::isnan(_min) && std::isnan(_max)) {
        log_warn("SoftLimits: plane '" << name() << "' has neither min_mm nor max_mm and guards nothing");
        return;
    }

    LimitsChecker::instance().Register(this);
    DynamicLimits::registerProvider(this);

    std::string bounds;
    if (!std::isnan(_min)) {
        bounds += " >= " + std::to_string(_min);
    }
    if (!std::isnan(_min) && !std::isnan(_max)) {
        bounds += " and";
    }
    if (!std::isnan(_max)) {
        bounds += " <= " + std::to_string(_max);
    }
    if (_unknown_tool_length > 0.0f) {
        bounds += ", assuming " + std::to_string(_unknown_tool_length) + "mm of tool when no offset is active";
    }

    log_info("SoftLimits: Registered plane '" << name() << "' on " << Machine::Axes::axisName(_axis) << ", tool tip" << bounds);
}

bool FixedPlane::isActive() {
    // Gates the DynamicLimitProvider side as well as TestLimit, so a suspended
    // plane stops clamping jogs at the same moment it stops blocking moves.
    return _enabled && !suspendedByToolChange() && (!std::isnan(_min) || !std::isnan(_max));
}

float FixedPlane::toolBias(const float* tlo) const {
    // An active offset already places the tip correctly, so the bias is only for
    // the case where there is no offset to trust.
    if (_unknown_tool_length > 0.0f && (tlo == nullptr || tlo[_axis] == 0.0f)) {
        return _unknown_tool_length;
    }
    return 0.0f;
}

bool FixedPlane::TestLimit(const LimitContext& ctx) {
    if (!isActive() || int(_axis) >= ctx.nAxis) {
        return false;
    }

    // A half-space is convex, so a straight segment lies entirely inside it
    // whenever both of its endpoints do.  Testing the two endpoints is therefore
    // exact here, not an approximation the way it would be for a box.
    const float bias = toolBias(ctx.tlo);
    const float from = ctx.tipFrom[_axis] - bias;
    const float to   = ctx.tipTo[_axis] - bias;

    if (!std::isnan(_min) && std::min(from, to) < _min) {
        log_debug("SoftLimits: plane '" << name() << "' blocked motion, tip " << std::min(from, to) << " below " << _min);
        return true;
    }

    if (!std::isnan(_max) && std::max(from, to) > _max) {
        log_debug("SoftLimits: plane '" << name() << "' blocked motion, tip " << std::max(from, to) << " above " << _max);
        return true;
    }

    return false;
}

void FixedPlane::getDynamicLimits(const float* current_mpos, const float* active_tlo, float* axis_min, float* axis_max) {
    if (_axis >= Machine::Axes::_numberAxis) {
        return;
    }

    // DynamicLimits converts the target with the offset it was given, so an
    // assumed tool length cannot be applied to the position the way TestLimit
    // does it.  Shifting the published bound by the same amount is equivalent:
    // requiring MPos - assumed >= _min is requiring MPos >= _min + assumed.
    const float bias = toolBias(active_tlo);

    if (!std::isnan(_min)) {
        axis_min[_axis] = _min + bias;
    }
    if (!std::isnan(_max)) {
        axis_max[_axis] = _max + bias;
    }
}

#ifndef _MSC_VER
namespace {
    ConfigurableModuleFactory::InstanceBuilder<FixedPlane> __attribute__((init_priority(111))) fixed_plane_registration("fixed_plane");
}
#endif
