// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "LimitsChecker.h"
#include "../Logging.h"
#include <algorithm>

LimitsChecker& LimitsChecker::instance() {
    static LimitsChecker instance;
    return instance;
}

void LimitsChecker::Register(SoftLimitsComponent* component) {
    if (component == nullptr) {
        return;
    }

    // Check if already registered
    auto it = std::find(_components.begin(), _components.end(), component);
    if (it != _components.end()) {
        return;  // Already registered
    }

    _components.push_back(component);
    log_debug("SoftLimits: Registered component '" << component->componentName() << "'");
}

void LimitsChecker::Unregister(SoftLimitsComponent* component) {
    if (component == nullptr) {
        return;
    }

    auto it = std::find(_components.begin(), _components.end(), component);
    if (it != _components.end()) {
        log_debug("SoftLimits: Unregistered component '" << component->componentName() << "'");
        _components.erase(it);
    }
}

bool LimitsChecker::TestMotion(const float* from, const float* to) {
    _lastViolation = nullptr;

    for (auto* component : _components) {
        if (component->TestLimit(from, to)) {
            _lastViolation = component->componentName();
            log_info("SoftLimits: Motion blocked by '" << _lastViolation << "'");
            return true;
        }
    }

    return false;
}
