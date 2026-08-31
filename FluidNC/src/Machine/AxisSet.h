// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../Configuration/Configurable.h"
#include "Axis.h"

namespace Machine {
    /*
        A detached set of axes owned by a CanModule.

        This parses the same X/Y/Z/A/... sections as Machine::Axes, but into a local array
        rather than the static Machine::Axes::_axis[] table.  Nothing here fills gaps or
        computes an axis count: the module holds these axes dormant until it is loaded, at
        which point CanModule splices the non-null entries into the global axis table and the
        normal gap-filling / count logic runs there.
    */
    class AxisSet : public Configuration::Configurable {
    public:
        AxisSet() {
            for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; ++axis) {
                _axis[axis] = nullptr;
            }
        }

        Axis* _axis[MAX_N_AXIS];

        // The lowest and highest axis index that this set actually defines.  Used by
        // CanModule to detect conflicts and to know which global slots it owns.
        bool defines(axis_t axis) const { return axis < MAX_N_AXIS && _axis[axis] != nullptr; }

        void group(Configuration::HandlerBase& handler) override;

        ~AxisSet() {
            for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; ++axis) {
                if (_axis[axis] != nullptr) {
                    delete _axis[axis];
                    _axis[axis] = nullptr;
                }
            }
        }
    };
}
