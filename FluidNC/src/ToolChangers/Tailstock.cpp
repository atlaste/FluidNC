#include "Tailstock.h"
#include "ToolTable.h"
#include "Machine/MachineConfig.h"
#include "Logging.h"
#include "../SoftLimits/LimitsChecker.h"
#include <cstring>
#include <cmath>
#include <algorithm>

// Global tailstock instance (set in Tailstock::init when module is configured)
ATCs::Tailstock* tailstock = nullptr;

namespace ATCs {

    void Tailstock::group(Configuration::HandlerBase& handler) {
        handler.item("tool", _loaded_tool);
        handler.item("safety_margin", _safety_margin);
        handler.item("spindle_index", _spindle_index);

        // Axis is specified as a letter
        std::string axis_str;
        axis_str += _axis_letter;
        handler.item("axis", axis_str);
        if (!axis_str.empty()) {
            _axis_letter = std::toupper(axis_str[0]);
        }
    }

    void Tailstock::afterParse() {
        // Convert axis letter to index
        const char* axes = "XYZABCUVW";
        const char* p = strchr(axes, _axis_letter);
        if (p != nullptr) {
            _axis = axis_t(p - axes);
        } else {
            _axis = Z_AXIS;  // Default
            log_warn("Tailstock: Invalid axis '" << _axis_letter << "', defaulting to Z");
        }

        log_info("Tailstock configured on " << Machine::Axes::axisName(_axis)
                 << " axis, tool=" << _loaded_tool
                 << ", margin=" << _safety_margin << "mm"
                 << ", spindle_index=" << _spindle_index);
    }

    void Tailstock::init() {
        DynamicLimits::registerProvider(this);
        LimitsChecker::instance().Register(this);
        tailstock = this;
        log_info("Tailstock initialized and registered for dynamic limits and soft limits");
    }

    Tailstock::~Tailstock() {
        if (tailstock == this) {
            tailstock = nullptr;
        }
        LimitsChecker::instance().Unregister(this);
        DynamicLimits::unregisterProvider(this);
    }

    void Tailstock::getDynamicLimits(
        const float* current_mpos,
        const float* active_tlo,
        float* axis_min,
        float* axis_max
    ) {
        // Only impose limits if we're extended and have a tool loaded
        if (!_extended || _loaded_tool <= 0) {
            return;
        }

        auto n_axis = Machine::Axes::_numberAxis;
        if (_axis >= n_axis) {
            return;  // Invalid axis
        }

        // Get the tailstock tool's TLO from the tool table
        float tailstock_tlo[MAX_N_AXIS] = {};
        if (toolTable != nullptr) {
            toolTable->getToolOffset(_loaded_tool, tailstock_tlo);
        }

        // Calculate the tailstock tool tip position
        float tailstock_tip = _position + tailstock_tlo[_axis];

        // The turret (on the same axis) must stay below the tailstock tip
        float limit = tailstock_tip - _safety_margin;
        axis_max[_axis] = limit;

        log_debug("Tailstock dynamic limit: " << Machine::Axes::axisName(_axis)
                  << " max=" << limit
                  << " (tailstock_pos=" << _position
                  << ", tailstock_tlo=" << tailstock_tlo[_axis]
                  << ", margin=" << _safety_margin << ")");
    }

    bool Tailstock::TestLimit(const float* from, const float* to) {
        if (!_extended || _loaded_tool <= 0) {
            return false;
        }

        auto n_axis = Machine::Axes::_numberAxis;
        if (_axis >= n_axis) {
            return false;
        }

        float tailstock_tlo[MAX_N_AXIS] = {};
        if (toolTable != nullptr) {
            toolTable->getToolOffset(_loaded_tool, tailstock_tlo);
        }

        float tailstock_tip = _position + tailstock_tlo[_axis];
        float limit = tailstock_tip - _safety_margin;

        // Block if the segment crosses into or through the forbidden zone (beyond limit)
        float seg_max = std::max(from[_axis], to[_axis]);
        if (seg_max >= limit) {
            log_debug("Tailstock: Motion blocked, segment max " << seg_max << " >= limit " << limit);
            return true;
        }

        return false;
    }

    namespace {
        ConfigurableModuleFactory::InstanceBuilder<Tailstock> __attribute__((init_priority(110))) tailstock_registration("tailstock");
    }

}  // namespace ATCs
