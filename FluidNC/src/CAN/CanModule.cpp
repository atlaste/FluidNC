// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "CanModule.h"

#include "CanNode.h"
#include "CanScheduler.h"
#include "CanIds.h"

#include "../Machine/MachineConfig.h"
#include "../Machine/AxisSet.h"
#include "../Machine/Axis.h"
#include "../Machine/Axes.h"
#include "../Machine/Homing.h"
#include "../Spindles/Spindle.h"
#include "../Module.h"
#include "../Stepping.h"
#include "../Planner.h"  // plan_sync_position, plan_get_current_block
#include "../GCode.h"    // gc_sync_position
#include "../State.h"    // state_is
#include "../Logging.h"
#include "../NutsBolts.h"  // bitnum_is_true

namespace CAN {
    namespace {
        uint64_t parseUuid(const std::string& text) {
            uint64_t v = 0;
            for (char c : text) {
                int digit;
                if (c >= '0' && c <= '9') {
                    digit = c - '0';
                } else if (c >= 'a' && c <= 'f') {
                    digit = c - 'a' + 10;
                } else if (c >= 'A' && c <= 'F') {
                    digit = c - 'A' + 10;
                } else {
                    continue;  // skip separators such as ':' or '-'
                }
                v = (v << 4) | uint64_t(digit);
            }
            // The admin protocol carries 48 bits of UUID.
            return v & 0xFFFFFFFFFFFFULL;
        }
    }

    // ---------------------------------------------------------------- CanModule

    CanModule::~CanModule() {
        delete _axes;
        // Spindles and ConfigurableModules are owned here too, but only when they are not
        // currently spliced into the global factories.  At teardown the whole config graph is
        // destroyed together, so the factories' own cleanup handles the spliced ones and we
        // must not double-free.  In practice CanModule outlives nothing, so we leave them.
    }

    bool CanModule::definesAxes() const {
        return _axes != nullptr && [&] {
            for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; ++axis) {
                if (_axes->defines(axis)) {
                    return true;
                }
            }
            return false;
        }();
    }

    void CanModule::group(Configuration::HandlerBase& handler) {
        handler.item("label", _label);
        handler.item("uuid", _uuidText);
        handler.item("node", _nodeId);
        handler.item("required", _required);

        handler.section("axes", _axes);

        // Nested spindles and modules are captured into module-owned vectors instead of the
        // global factory lists.  load()/unload() splices them in and out later.
        {
            Spindles::SpindleFactory::Capture capture(_spindles);
            Spindles::SpindleFactory::factory(handler);
        }
        {
            ConfigurableModuleFactory::Capture capture(_modules);
            ConfigurableModuleFactory::factory(handler);
        }
    }

    void CanModule::afterParse() {
        Assert(!_label.empty(), "can_module: 'label' is required so it can be loaded and referenced");
        _uuid = parseUuid(_uuidText);
        Assert(_nodeId >= 1 && _nodeId <= int32_t(MaxNodeId),
               "can_module '%s': node must be between 1 and %d",
               _label.c_str(),
               int(MaxNodeId));
    }

    void CanModule::validate() {
        // Labels must be unique so lookups are unambiguous.
        for (auto other : CanModuleFactory::objects()) {
            Assert(other == this || other->label() != _label, "Duplicate can_module label '%s'", _label.c_str());
        }
    }

    bool CanModule::reserveSpindleTools() {
        // Refuse if an explicitly configured tool number clashes with a spindle already in
        // the global list.  This runs before anything is mutated.
        for (auto s : _spindles) {
            if (s->_tool < 0) {
                continue;
            }
            for (auto existing : Spindles::SpindleFactory::objects()) {
                if (existing->_tool == s->_tool) {
                    log_error("can_module '" << _label << "': tool number " << s->_tool << " already used by /"
                                             << existing->name());
                    return false;
                }
            }
        }
        return true;
    }

    bool CanModule::load() {
        if (_loaded) {
            return true;
        }
        if (!CanModules::quiescent()) {
            log_error("can_module '" << _label << "': can only load when Idle with an empty queue");
            return false;
        }

        // ---- Pre-flight conflict checks (no mutation yet) --------------------------------
        if (_axes) {
            for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; ++axis) {
                if (!_axes->defines(axis)) {
                    continue;
                }
                auto existing = Machine::Axes::_axis[axis];
                if (existing != nullptr && existing->motorCount() > 0) {
                    log_error("can_module '" << _label << "': axis " << Machine::Axes::axisName(axis)
                                             << " is already driven by another axis");
                    return false;
                }
            }
        }
        if (!reserveSpindleTools()) {
            return false;
        }

        log_info("Loading CAN module '" << _label << "' on node " << _nodeId);

        // ---- Splice axes -----------------------------------------------------------------
        _loadedAxisMask = 0;
        if (_axes) {
            for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; ++axis) {
                if (_axes->defines(axis)) {
                    Machine::Axes::spliceIn(axis, _axes->_axis[axis]);
                    _loadedAxisMask |= (1u << uint32_t(axis));
                }
            }
            // Recompute count, fill gaps, rebuild masks.
            Machine::Axes::reconcile();

            // Bring up only the newly spliced axes.
            for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; ++axis) {
                if (bitnum_is_true(_loadedAxisMask, axis)) {
                    auto a = Machine::Axes::_axis[axis];
                    a->init();
                    a->config_motors();
                    Machine::Stepping::setSteps(axis, 0);
                    Machine::Homing::set_axis_unhomed(axis);
                }
            }

            // Soft-limit ranges may have changed; recompute without disturbing position.
            config->_kinematics->recomputeLimits();

            // The planner and parser must adopt the widened axis set.
            plan_sync_position();
            gc_sync_position();
        }

        // ---- Splice spindles -------------------------------------------------------------
        uint32_t nextTool = 100;
        for (auto existing : Spindles::SpindleFactory::objects()) {
            if (existing->_tool >= int32_t(nextTool)) {
                nextTool = uint32_t(existing->_tool) + 1;
            }
        }
        for (auto s : _spindles) {
            if (s->_tool < 0) {
                s->_tool = int32_t(nextTool++);
            }
            Spindles::SpindleFactory::add(s);
            s->init();
        }

        // ---- Splice modules (actuators, pendants) ----------------------------------------
        for (auto m : _modules) {
            ConfigurableModuleFactory::add(m);
            m->init();
        }

        _loaded = true;
        return true;
    }

    bool CanModule::unload() {
        if (!_loaded) {
            return true;
        }
        if (!CanModules::quiescent()) {
            log_error("can_module '" << _label << "': can only unload when Idle with an empty queue");
            return false;
        }

        log_info("Unloading CAN module '" << _label << "'");

        // ---- Remove spindles -------------------------------------------------------------
        bool activeRemoved = false;
        for (auto s : _spindles) {
            if (spindle == s) {
                activeRemoved = true;
            }
            Spindles::SpindleFactory::remove(s);
        }
        if (activeRemoved) {
            // Fall back to a base spindle; this also drives CanPwm::onSpindleChanged, which
            // releases the scheduler PWM binding.
            bool stopped = false, changed = false;
            Spindles::Spindle::switchSpindle(0, Spindles::SpindleFactory::objects(), spindle, stopped, changed);
        }

        // ---- Remove modules --------------------------------------------------------------
        for (auto m : _modules) {
            m->deinit();
            ConfigurableModuleFactory::remove(m);
        }

        // ---- Remove axes -----------------------------------------------------------------
        if (_loadedAxisMask) {
            for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; ++axis) {
                if (bitnum_is_true(_loadedAxisMask, axis)) {
                    Machine::Axes::set_disable(axis, true);
                    CanScheduler::unbindAxis(axis);
                    Machine::Homing::set_axis_unhomed(axis);
                    Machine::Axes::spliceOut(axis);
                }
            }
            _loadedAxisMask = 0;

            Machine::Axes::reconcile();
            config->_kinematics->recomputeLimits();
            plan_sync_position();
            gc_sync_position();
        }

        // ---- Release the node, if nothing else uses it -----------------------------------
        bool shared = false;
        for (auto other : CanModuleFactory::objects()) {
            if (other != this && other->loaded() && other->nodeId() == _nodeId) {
                shared = true;
                break;
            }
        }
        if (!shared) {
            CanNodes::instance().remove(uint8_t(_nodeId));
        }

        _loaded = false;
        return true;
    }

    // --------------------------------------------------------------- CanModules

    const std::vector<CanModule*>& CanModules::all() {
        return CanModuleFactory::objects();
    }

    CanModule* CanModules::byLabel(const std::string& label) {
        for (auto m : all()) {
            if (m->label() == label) {
                return m;
            }
        }
        return nullptr;
    }

    CanModule* CanModules::byNode(int32_t node) {
        for (auto m : all()) {
            if (m->nodeId() == node) {
                return m;
            }
        }
        return nullptr;
    }

    int32_t CanModules::nodeForLabel(const std::string& label) {
        auto m = byLabel(label);
        return m ? m->nodeId() : -1;
    }

    bool CanModules::quiescent() {
        return state_is(State::Idle) && plan_get_current_block() == nullptr;
    }

    std::string CanModules::scan() {
        std::string out;
        for (auto m : all()) {
            uint8_t current = 0;
            bool    present = CanNodes::instance().probe(m->uuid(), current);
            m->setPresent(present);
            if (present && current != uint8_t(m->nodeId())) {
                bool ok = CanNodes::instance().assign(m->uuid(), uint8_t(m->nodeId()));
                out += m->label() + (ok ? ": assigned node " : ": FAILED to assign node ") + std::to_string(m->nodeId()) + "\n";
            } else {
                out += m->label() + (present ? ": present" : ": absent") + "\n";
            }
        }
        return out;
    }

    std::string CanModules::refresh() {
        if (!quiescent()) {
            return "Cannot refresh: machine must be Idle with an empty queue\n";
        }

        std::string out = scan();

        for (auto m : all()) {
            if (m->present() && !m->loaded()) {
                out += m->load() ? ("loaded " + m->label() + "\n") : ("FAILED to load " + m->label() + "\n");
            } else if (!m->present() && m->loaded()) {
                out += m->unload() ? ("unloaded " + m->label() + "\n") : ("FAILED to unload " + m->label() + "\n");
            }
            if (m->required() && !m->present()) {
                out += "WARNING: required module '" + m->label() + "' is absent\n";
            }
        }
        return out;
    }

    uint32_t CanModules::loadedBitmap() {
        uint32_t bits  = 0;
        uint32_t index = 0;
        for (auto m : all()) {
            if (index >= 32) {
                break;
            }
            if (m->loaded()) {
                bits |= (1u << index);
            }
            ++index;
        }
        return bits;
    }

    namespace {
        uint32_t g_previousLoadedBitmap = 0;
    }

    void CanModules::setPreviousLoadedBitmap(uint32_t bits) {
        g_previousLoadedBitmap = bits;
    }

    uint32_t CanModules::previousLoadedBitmap() {
        return g_previousLoadedBitmap;
    }

    bool CanModules::wasLoaded(uint32_t index) {
        return index < 32 && (g_previousLoadedBitmap & (1u << index)) != 0;
    }
}
