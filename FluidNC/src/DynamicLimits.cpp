#include "DynamicLimits.h"
#include "Machine/MachineConfig.h"
#include "Logging.h"
#include <algorithm>
#include <cmath>

// Static member definition
std::vector<DynamicLimitProvider*> DynamicLimits::_providers;

void DynamicLimits::registerProvider(DynamicLimitProvider* provider) {
    if (provider == nullptr) return;
    
    // Check if already registered
    auto it = std::find(_providers.begin(), _providers.end(), provider);
    if (it == _providers.end()) {
        _providers.push_back(provider);
        log_info("Dynamic limit provider registered: " << provider->limitProviderName());
    }
}

void DynamicLimits::unregisterProvider(DynamicLimitProvider* provider) {
    if (provider == nullptr) return;
    
    auto it = std::find(_providers.begin(), _providers.end(), provider);
    if (it != _providers.end()) {
        _providers.erase(it);
        log_info("Dynamic limit provider unregistered: " << provider->limitProviderName());
    }
}

void DynamicLimits::getEffectiveLimits(
    const float* current_mpos,
    const float* active_tlo,
    float* axis_min,
    float* axis_max
) {
    auto n_axis = Machine::Axes::_numberAxis;
    
    // Initialize with NAN (no limit)
    for (size_t i = 0; i < n_axis; i++) {
        axis_min[i] = NAN;
        axis_max[i] = NAN;
    }
    
    // Temporary arrays for each provider
    float provider_min[MAX_N_AXIS];
    float provider_max[MAX_N_AXIS];
    
    // Query each active provider
    for (auto provider : _providers) {
        if (!provider->isActive()) continue;
        
        // Initialize provider arrays with NAN
        for (size_t i = 0; i < n_axis; i++) {
            provider_min[i] = NAN;
            provider_max[i] = NAN;
        }
        
        provider->getDynamicLimits(current_mpos, active_tlo, provider_min, provider_max);
        
        // Merge with effective limits (take most restrictive)
        for (size_t i = 0; i < n_axis; i++) {
            // For min limits, take the larger value (more restrictive)
            if (!std::isnan(provider_min[i])) {
                if (std::isnan(axis_min[i]) || provider_min[i] > axis_min[i]) {
                    axis_min[i] = provider_min[i];
                }
            }
            
            // For max limits, take the smaller value (more restrictive)
            if (!std::isnan(provider_max[i])) {
                if (std::isnan(axis_max[i]) || provider_max[i] < axis_max[i]) {
                    axis_max[i] = provider_max[i];
                }
            }
        }
    }
}

bool DynamicLimits::checkMove(
    const float* current_mpos,
    const float* target_mpos,
    const float* active_tlo,
    std::string& error_msg
) {
    auto n_axis = Machine::Axes::_numberAxis;
    
    float axis_min[MAX_N_AXIS];
    float axis_max[MAX_N_AXIS];
    
    getEffectiveLimits(current_mpos, active_tlo, axis_min, axis_max);
    
    // Check target position against dynamic limits
    // Account for tool length offset: the tool tip is at target_mpos + active_tlo
    for (size_t i = 0; i < n_axis; i++) {
        float tool_tip = target_mpos[i];
        if (active_tlo != nullptr) {
            tool_tip += active_tlo[i];
        }
        
        if (!std::isnan(axis_min[i]) && tool_tip < axis_min[i]) {
            error_msg = "Dynamic limit violation on " + std::string(Machine::Axes::axisName(i)) + 
                       " (min " + std::to_string(axis_min[i]) + ", target " + std::to_string(tool_tip) + ")";
            return false;
        }
        
        if (!std::isnan(axis_max[i]) && tool_tip > axis_max[i]) {
            error_msg = "Dynamic limit violation on " + std::string(Machine::Axes::axisName(i)) + 
                       " (max " + std::to_string(axis_max[i]) + ", target " + std::to_string(tool_tip) + ")";
            return false;
        }
    }
    
    return true;
}

bool DynamicLimits::checkPosition(
    const float* target_mpos,
    const float* active_tlo,
    std::string& error_msg
) {
    // For single position check, use target as both current and target
    return checkMove(target_mpos, target_mpos, active_tlo, error_msg);
}
