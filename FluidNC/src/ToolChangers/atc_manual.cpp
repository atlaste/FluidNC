// Copyright (c) 2026 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "atc_manual.h"

#include "Machine/MachineConfig.h"
#include "Protocol.h"  // protocol_buffer_synchronize
#include "System.h"    // get_mpos
#include "State.h"     // state_is

#include <cstdio>

namespace ATCs {
    void Manual_ATC::validate() {
        ProbingATC::validate();
        Assert(_change_mpos.size() >= 3, "atc_manual: change_mpos_mm needs at least three values");
        Assert(_retrieve_mpos.empty() || _retrieve_mpos.size() >= 3, "atc_manual: retrieve_mpos_mm needs at least three values");
    }

    void Manual_ATC::probe_notification() {}

    void Manual_ATC::retreat_to_safe() {
        try {
            rapid_via_safe_z(retrieve_position());
            log_info("ATC: retreated to the safe position");
        } catch (const AtcFault& fault) { log_error("ATC: could not reach the safe position - " << fault.message); }
    }

    bool Manual_ATC::tool_change(tool_t new_tool, bool pre_select, bool set_tool) {
        if (pre_select) {
            return true;  // nothing to stage on a manual changer
        }

        protocol_buffer_synchronize();

        // M61 adopts a tool number without touching the machine.
        if (set_tool) {
            _last_tool = new_tool;
            if (new_tool == 0) {
                invalidate_tool_length();
            }
            return true;
        }

        if (!ready_to_change()) {
            return false;
        }

        ChangeScope change_scope(*this);

        if (gc_state.modal.plane_select != Plane::XY) {
            fail("this tool changer only works in G17 (XY)");
            return false;
        }

        const ProgramState saved = record_program_state();

        float start[MAX_N_AXIS];
        copyAxes(start, get_mpos());

        try {
            enter_tool_change_state();

            // M6 T0 parks and clears, ready for a new job.
            if (new_tool == 0) {
                rapid_via_safe_z(_change_mpos);
                invalidate_tool_length();
                _last_tool = 0;
                restore_program_state(saved);
                return true;
            }

            rapid_via_safe_z(_change_mpos);

            char message[128];
            snprintf(message, sizeof(message), "install tool #%u, fit the toolsetter, then press start", unsigned(new_tool));
            wait_for_operator(message);

            float length = 0.0;
            if (!measure_tool_length(length)) {
                throw AtcFault { "tool length could not be measured", false };
            }

            apply_tool_length(length);
            _last_tool = new_tool;

            rapid_via_safe_z(retrieve_position());
            wait_for_operator("remove the toolsetter, close the door, then press start");

            // Back to where the job was interrupted.
            move_to_safe_z();
            rapid_xy(start[X_AXIS], start[Y_AXIS]);
            rapid_z(start[Z_AXIS]);

            restore_program_state(saved);

            log_info("ATC: tool #" << new_tool << " ready, length " << length << " mm");
            return true;

        } catch (const AtcFault& fault) {
            if (fault.fatal) {
                set_tlo_valid(false);
                log_error("ATC: tool change aborted - " << fault.message);
                return false;
            }
            invalidate_tool_length();
            retreat_to_safe();
            fail(fault.message);
            return false;
        } catch (const std::exception& e) {
            invalidate_tool_length();
            retreat_to_safe();
            fail(std::string("unexpected error - ") + e.what());
            return false;
        }
    }

    namespace {
        ATCFactory::InstanceBuilder<Manual_ATC> registration("atc_manual");
    }
}
