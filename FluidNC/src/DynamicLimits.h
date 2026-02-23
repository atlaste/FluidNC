#pragma once

#include "Config.h"
#include <vector>
#include <string>
#include <cmath>

// Interface for components that can impose dynamic soft limits
// (e.g., tailstock, gang tool, secondary spindle)
//
// Components register with the DynamicLimits system at initialization.
// The motion planner queries all providers before each move to check
// if the move would violate any dynamic limit.

class DynamicLimitProvider {
public:
    virtual ~DynamicLimitProvider() = default;

    // Get dynamic limits imposed by this component.
    //
    // Parameters:
    //   current_mpos - Current machine position
    //   axis_min/axis_max - Arrays to fill with limits.
    //                       Set to NAN for "no limit from this component"
    //   active_tlo - Currently active tool length offset (from gc_state)
    //
    // The provider should consider:
    //   - Its own position (if it's a movable component like tailstock)
    //   - Tools currently loaded in it (from tool table)
    //   - Required safety margins
    virtual void getDynamicLimits(const float* current_mpos, const float* active_tlo, float* axis_min, float* axis_max) = 0;

    // Human-readable name for error messages
    virtual const char* limitProviderName() = 0;

    // Check if this provider is currently active (imposing limits)
    virtual bool isActive() = 0;
};

// Registry and checker for dynamic soft limits
class DynamicLimits {
private:
    static std::vector<DynamicLimitProvider*> _providers;

public:
    // Register a provider (called during component init)
    static void registerProvider(DynamicLimitProvider* provider);

    // Unregister a provider (called during component cleanup)
    static void unregisterProvider(DynamicLimitProvider* provider);

    // Get effective limits from all active providers
    // Fills axis_min/axis_max with the most restrictive limits
    // (NAN values in output mean "no dynamic limit on this axis")
    static void getEffectiveLimits(const float* current_mpos, const float* active_tlo, float* axis_min, float* axis_max);

    // Check if a move from current_mpos to target_mpos would violate any dynamic limit
    // Returns true if move is OK, false if it would violate a limit
    // If false, error_msg is set to describe the violation
    static bool checkMove(const float* current_mpos, const float* target_mpos, const float* active_tlo, std::string& error_msg);

    // Check if a specific target position violates any dynamic limit
    // (simpler version when you just need to validate a single point)
    static bool checkPosition(const float* target_mpos, const float* active_tlo, std::string& error_msg);

    // Get number of registered providers
    static size_t providerCount() { return _providers.size(); }

    // Clear all providers (for testing/reset)
    static void clearProviders() { _providers.clear(); }
};
