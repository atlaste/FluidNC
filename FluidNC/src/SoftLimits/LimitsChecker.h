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
    //
    // "from" and "to" are spindle reference points in machine position (MPos)
    // space, as the planner works in.  The tool tip frame is derived here from
    // "tlo" so that every component sees the same conversion; pass a null tlo
    // only when there genuinely is no tool offset in play.
    bool TestMotion(const float* from,
                    const float* to,
                    const float* tlo     = nullptr,
                    int          n_axis  = 3,
                    bool         isProbe = false,
                    bool         isJog   = false);

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
