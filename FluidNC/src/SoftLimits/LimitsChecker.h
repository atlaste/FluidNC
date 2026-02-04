// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "SoftLimitsComponent.h"
#include <vector>

// Central registry and dispatcher for soft limits components.
// Components register themselves on init and unregister on destruction.
// During motion planning, TestMotion() is called to check all registered components.

class LimitsChecker {
public:
    // Singleton access
    static LimitsChecker& instance();

    // Registration - components call these in init/destructor
    void Register(SoftLimitsComponent* component);
    void Unregister(SoftLimitsComponent* component);

    // Test motion segment against all registered components.
    // Returns true if ANY component reports a limit violation.
    // Coordinates are in machine position (MPos) space.
    bool TestMotion(const float* from, const float* to);

    // Get the name of the component that caused the last limit violation
    const char* lastViolationComponent() const { return _lastViolation; }

private:
    LimitsChecker() = default;
    ~LimitsChecker() = default;

    // Non-copyable
    LimitsChecker(const LimitsChecker&) = delete;
    LimitsChecker& operator=(const LimitsChecker&) = delete;

    std::vector<SoftLimitsComponent*> _components;
    const char* _lastViolation = nullptr;
};
