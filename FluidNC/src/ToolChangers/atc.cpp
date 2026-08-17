// Copyright (c) 2024 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "atc.h"

#include "Machine/MachineConfig.h"

ATCs::ATC* atc = nullptr;

namespace ATCs {
    // Starts invalid, so a machine with enforcement on has to measure before it
    // may cut rather than inheriting trust from nowhere.
    bool ATC::_offset_valid    = false;
    bool ATC::_offset_enforced = false;
    bool ATC::_change_active   = false;

    bool ATC::motion_needs_tool_measurement() {
        return _offset_enforced && !_offset_valid && !_change_active;
    }

    void probe_notification() {}

    bool tool_change(tool_t value, bool pre_select) {
        return true;
    }
}
