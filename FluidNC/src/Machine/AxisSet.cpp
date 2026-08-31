// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "AxisSet.h"
#include "Axes.h"

namespace Machine {
    void AxisSet::group(Configuration::HandlerBase& handler) {
        // Parse every possible axis letter.  Unlike Machine::Axes we always walk the full
        // range, because a module often defines only the high axes (A/B) and _numberAxis has
        // no meaning for a detached set.
        for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; ++axis) {
            handler.section(Axes::_axisNames[axis], _axis[axis], axis);
        }
    }
}
