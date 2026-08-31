// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../Configuration/Configurable.h"
#include "../Configuration/GenericFactory.h"

#include <cstdint>
#include <string>
#include <vector>

namespace Machine {
    class AxisSet;
}
namespace Spindles {
    class Spindle;
}
class ConfigurableModule;

namespace CAN {
    /*
        A CanModule owns a detached slice of the machine configuration -- some axes, a
        spindle, some actuators -- that physically lives on one CAN node.  The base config in
        config.yaml declares every module the machine might ever wear; which ones are actually
        active is decided at runtime by load() / unload().

        load() splices the module's objects into the global registries (Machine::Axes,
        SpindleFactory, ConfigurableModuleFactory) and runs a narrow, explicit re-init.
        unload() removes them again by pointer.  Nothing is ever deleted, so the pointers stay
        valid across any number of load/unload cycles and reload is cheap.

        This is the mechanism behind tool-change heads, an optional laser, a bolt-on rotary
        axis, and the like: a macro loads the module when the head goes on and unloads it when
        the head comes off.
    */
    class CanModule : public Configuration::Configurable {
    public:
        explicit CanModule(const char* name) : _name(name) {}

        // Identity -------------------------------------------------------------------------
        const std::string& label() const { return _label; }
        uint64_t           uuid() const { return _uuid; }
        int32_t            nodeId() const { return _nodeId; }
        bool               required() const { return _required; }

        // Presence: whether the node was seen on the bus at the last scan.
        bool present() const { return _present; }
        void setPresent(bool p) { _present = p; }

        bool loaded() const { return _loaded; }
        bool definesAxes() const;

        // Splice this module in / out.  Both refuse unless the machine is quiescent (Idle,
        // empty planner) and return false with a logged reason on conflict.
        bool load();
        bool unload();

        // Configuration ---------------------------------------------------------------------
        void group(Configuration::HandlerBase& handler) override;
        void afterParse() override;
        void validate() override;

        const char* name() const { return _name; }

        ~CanModule();

    private:
        const char* _name;

        std::string _label;
        std::string _uuidText;
        uint64_t    _uuid     = 0;
        int32_t     _nodeId   = 0;
        bool        _required = false;

        bool _present = false;
        bool _loaded  = false;

        // Detached, module-owned configuration parsed under this section.
        Machine::AxisSet*                _axes = nullptr;
        std::vector<Spindles::Spindle*>  _spindles;
        std::vector<ConfigurableModule*> _modules;

        // Global axis slots this module occupies while loaded.
        uint32_t _loadedAxisMask = 0;

        // Assigns a free tool number to each module spindle that did not set one, and refuses
        // if an explicitly set tool number clashes with a spindle already present.  Returns
        // false without mutating anything on conflict.
        bool reserveSpindleTools();
    };

    using CanModuleFactory = Configuration::GenericFactory<CanModule>;

    /*
        Static helpers over the set of configured modules.
    */
    class CanModules {
    public:
        static const std::vector<CanModule*>& all();
        static CanModule*                      byLabel(const std::string& label);
        static CanModule*                      byNode(int32_t node);

        // Node id a consumer should use given a module label (or -1 if unknown).
        static int32_t nodeForLabel(const std::string& label);

        // Idle with an empty planner: the only safe moment to reshape the machine.
        static bool quiescent();

        // Probes the bus for each module's UUID and updates present().  When a present node
        // reports a different node id than the module expects, it is (re)assigned.  Returns a
        // human-readable summary.
        static std::string scan();

        // scan(), then load every module that is present but not loaded and unload every
        // module that is loaded but no longer present.  Required-but-absent modules are
        // reported.  Returns a human-readable summary.
        static std::string refresh();

        // Bitmap of loaded modules, for persistence across reboots.
        static uint32_t loadedBitmap();

        // The loaded-module bitmap as restored from FRAM at boot, so a boot-time refresh can
        // tell which modules changed since the machine was last powered down.  Homed state is
        // deliberately not restored for a module whose membership changed.
        static void     setPreviousLoadedBitmap(uint32_t bits);
        static uint32_t previousLoadedBitmap();

        // True if the module at the given zero-based index was loaded before the last reboot.
        static bool wasLoaded(uint32_t index);
    };
}
