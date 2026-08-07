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

    bool Tailstock::computeLimit(float& limit) const {
        // Only impose limits if we're extended and have a tool loaded
        if (!_extended || _loaded_tool <= 0) {
            return false;
        }

        if (_axis >= Machine::Axes::_numberAxis) {
            return false;  // Invalid axis
        }

        // Get the tailstock tool's TLO from the tool table
        float tailstock_tlo[MAX_N_AXIS] = {};
        if (toolTable != nullptr) {
            toolTable->getToolOffset(_loaded_tool, tailstock_tlo);
        }

        // The point of the live center, in the same tip frame the turret tool is
        // measured in: tip = MPos - TLO.  Subtracting is what makes a longer
        // center reach further towards the turret and therefore restrict travel
        // more; adding would relax the limit as the center gets longer, which is
        // the wrong way round and would let the turret drive into it.
        const float tailstock_tip = _position - tailstock_tlo[_axis];

        // The turret tool tip must stay below the tailstock tip
        limit = tailstock_tip - _safety_margin;

        log_debug("Tailstock limit: " << Machine::Axes::axisName(_axis) << " max=" << limit << " (tailstock_pos=" << _position
                                      << ", tailstock_tlo=" << tailstock_tlo[_axis] << ", margin=" << _safety_margin << ")");
        return true;
    }

    void Tailstock::getDynamicLimits(const float* current_mpos, const float* active_tlo, float* axis_min, float* axis_max) {
        // active_tlo is deliberately unused: the limit is published in tip space
        // and DynamicLimits converts the move into that frame before comparing.
        float limit;
        if (computeLimit(limit)) {
            axis_max[_axis] = limit;
        }
    }

    bool Tailstock::TestLimit(const LimitContext& ctx) {
        float limit;
        if (!computeLimit(limit)) {
            return false;
        }

        // computeLimit works in tip space, so the turret tool tip is what gets
        // compared against it.
        float seg_max = std::max(ctx.tipFrom[_axis], ctx.tipTo[_axis]);
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
