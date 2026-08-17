// Copyright (c) 2026 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "RapidChange.h"

#include "Machine/MachineConfig.h"
#include "NutsBolts.h"  // delay_ms
#include "Protocol.h"   // protocol_buffer_synchronize
#include "System.h"     // get_mpos

#include <cmath>
#include <cstdio>
#include <cstring>

namespace ATCs {
    void RapidChange::init() {
        ProbingATC::init();

        if (_tool_present_pin.defined()) {
            _tool_present_pin.setAttr(Pin::Attr::Input);
            log_info("ATC: tool detection beam on " << _tool_present_pin.name());
            if (std::isnan(_beam_clear_z)) {
                log_warn("ATC: no beam_clear_mpos_mm, so a cross-threaded holder will not be detected");
            }
        } else {
            log_warn("ATC: no tool_present_pin, so a holder that fails to release or fails to pick up "
                     "will not be noticed until something hits something");
        }

        // Seed occupancy from the config the first time, and re-seed if the
        // magazine has grown or shrunk since whatever was persisted.
        if (_pocket_tools.size() != pocket_count()) {
            _pocket_tools.assign(pocket_count(), 0);
            for (size_t i = 0; i < pocket_count() && i < _pocket_seed.size(); i++) {
                _pocket_tools[i] = _pocket_seed[i];
            }
        }

        log_magazine();
    }

    void RapidChange::validate() {
        ProbingATC::validate();

        Assert(!_pocket_mpos.empty(), "rapidchange: pocket_mpos_mm is required");
        Assert(_pocket_mpos.size() % 3 == 0,
               "rapidchange: pocket_mpos_mm needs three values per pocket, got %d",
               int(_pocket_mpos.size()));
        Assert(_pocket_seed.size() <= pocket_count(),
               "rapidchange: pocket_tools has %d entries but there are only %d pockets",
               int(_pocket_seed.size()),
               int(pocket_count()));
        Assert(_retreat_mpos.size() >= 3, "rapidchange: retreat_mpos_mm needs at least three values");
        Assert(_manual_mpos.empty() || _manual_mpos.size() >= 3, "rapidchange: manual_mpos_mm needs at least three values");
        Assert(std::isnan(_beam_clear_z) || _beam_clear_z > _beam_check_z,
               "rapidchange: beam_clear_mpos_mm must be above beam_check_mpos_mm");

        // The threading travel runs downwards, so these have to be ordered or the
        // cycle drives the wrong way through the pocket.
        Assert(_z_engage < _z_spindle_start, "rapidchange: z_engage_mpos_mm must be below z_spindle_start_mpos_mm");
        Assert(_z_spindle_start <= _z_traverse, "rapidchange: z_spindle_start_mpos_mm must not be above z_traverse_mpos_mm");
        Assert(_z_traverse <= _safe_z, "rapidchange: z_traverse_mpos_mm must not be above safe_z_mpos_mm");

        for (size_t i = 0; i < _pocket_seed.size(); i++) {
            for (size_t j = i + 1; j < _pocket_seed.size(); j++) {
                Assert(_pocket_seed[i] == 0 || _pocket_seed[i] != _pocket_seed[j],
                       "rapidchange: tool %d is listed in two pockets",
                       int(_pocket_seed[i]));
            }
        }
    }

    void RapidChange::probe_notification() {}

    std::vector<float> RapidChange::pocket_position(size_t pocket) const {
        const size_t base = pocket * 3;
        return { _pocket_mpos[base], _pocket_mpos[base + 1], _pocket_mpos[base + 2] };
    }

    int32_t RapidChange::pocket_holding(tool_t tool) const {
        for (size_t i = 0; i < _pocket_tools.size(); i++) {
            if (_pocket_tools[i] == int32_t(tool)) {
                return int32_t(i);
            }
        }
        return -1;
    }

    int32_t RapidChange::pocket_for_putting_away(tool_t tool) const {
        // Where it came from, if that pocket is still free.
        for (size_t i = 0; i < _pocket_seed.size() && i < _pocket_tools.size(); i++) {
            if (_pocket_seed[i] == int32_t(tool) && _pocket_tools[i] == 0) {
                return int32_t(i);
            }
        }
        for (size_t i = 0; i < _pocket_tools.size(); i++) {
            if (_pocket_tools[i] == 0) {
                return int32_t(i);
            }
        }
        return -1;
    }

    void RapidChange::log_magazine() const {
        std::string line;
        for (size_t i = 0; i < _pocket_tools.size(); i++) {
            line += " " + std::to_string(i + 1) + ":";
            line += _pocket_tools[i] == 0 ? "-" : std::to_string(_pocket_tools[i]);
        }
        log_info("ATC: magazine" << line.c_str());
    }

    void RapidChange::feed_z(float z) {
        char line[96];
        snprintf(line, sizeof(line), "G53G1Z%0.3fF%0.3f", z, _engage_feed_rate);
        run(line);
    }

    void RapidChange::dwell_for(uint32_t ms) {
        if (ms == 0) {
            return;
        }
        char line[64];
        snprintf(line, sizeof(line), "G4P%0.3f", ms / 1000.0f);
        run(line);
    }

    bool RapidChange::beam_sees_tool(float z) {
        rapid_z(z);

        // An optical beam on a machine that has just stopped moving needs a
        // moment; reading it immediately reads the vibration.
        delay_ms(_beam_settle_ms);

        return _tool_present_pin.read();
    }

    void RapidChange::verify_spindle(bool expect_tool, const char* context) {
        if (!has_beam()) {
            return;
        }

        const bool present = beam_sees_tool(_beam_check_z);
        if (present != expect_tool) {
            throw AtcFault { std::string(context) + ": the beam sees " + (present ? "a tool" : "an empty spindle") + " but expected " +
                                 (expect_tool ? "a tool" : "an empty spindle"),
                             false };
        }
    }

    void RapidChange::engage(bool loading) {
        char line[96];

        rapid_z(_z_spindle_start);

        // M3 threads on, M4 backs off. The spindle has to already be turning
        // before the threads meet, hence starting it up here rather than at depth.
        snprintf(line, sizeof(line), "%sS%d", loading ? "M3" : "M4", int(loading ? _load_rpm : _unload_rpm));
        run(line);
        dwell_for(_spindle_settle_ms);

        // Threading a holder on takes a second pass to torque it up. Backing one
        // off is done the moment the threads part, so one pass is all it gets.
        const int32_t passes = loading ? _engage_passes : 1;
        for (int32_t pass = 0; pass < passes; pass++) {
            if (pass > 0) {
                feed_z(_z_spindle_start);  // still turning, so the thread stays found
            }
            feed_z(_z_engage);
        }

        run("M5");
    }

    void RapidChange::unload_to_pocket(size_t pocket) {
        const std::vector<float> target = pocket_position(pocket);

        log_info("ATC: putting tool #" << _last_tool << " into pocket " << (pocket + 1));

        move_to_safe_z();
        rapid_xy(target[0], target[1]);
        verify_spindle(true, "before unloading");

        engage(false);

        // The holder is only in the pocket if the spindle comes back up empty.
        // A thread that did not part usually parts on a second attempt, so that
        // is worth trying before giving up and stopping the job.
        if (has_beam() && beam_sees_tool(_beam_check_z)) {
            log_warn("ATC: tool #" << _last_tool << " did not release into pocket " << (pocket + 1) << ", trying once more");
            engage(false);
            if (beam_sees_tool(_beam_check_z)) {
                throw AtcFault { "tool #" + std::to_string(unsigned(_last_tool)) + " will not release into pocket " +
                                     std::to_string(pocket + 1),
                                 false };
            }
        }

        _pocket_tools[pocket] = int32_t(_last_tool);
        _last_tool            = 0;
        invalidate_tool_length();

        // Ends clear of the pockets but still over the magazine, so a load that
        // follows does not need a trip all the way up to safe Z and back.
        rapid_z(_z_traverse);
    }

    void RapidChange::load_from_pocket(size_t pocket, bool already_over_magazine) {
        const std::vector<float> target = pocket_position(pocket);
        const tool_t             tool   = tool_t(_pocket_tools[pocket]);

        log_info("ATC: taking tool #" << tool << " from pocket " << (pocket + 1));

        // Crossing at traverse height is only safe when we know what is below,
        // which is the case exactly when we have just come up out of a pocket.
        if (!already_over_magazine) {
            move_to_safe_z();
        }
        rapid_xy(target[0], target[1]);
        verify_spindle(false, "before loading");

        engage(true);

        verify_spindle(true, "after loading");

        // A holder that threaded on crooked hangs lower than a seated one, so it
        // still breaks the beam at a height where a good one has cleared it.
        // This is the check that catches cross-threading, which is both the
        // characteristic failure of a threaded changer and the one that throws a
        // holder. The length measurement afterwards can miss it, because a
        // crooked tool can still measure inside its plausible range.
        if (has_beam() && !std::isnan(_beam_clear_z) && beam_sees_tool(_beam_clear_z)) {
            throw AtcFault { "tool #" + std::to_string(unsigned(tool)) +
                                 " is cross-threaded; it still breaks the beam at the clear height",
                             false };
        }

        _pocket_tools[pocket] = 0;
        _last_tool            = tool;

        move_to_safe_z();
    }

    void RapidChange::manual_load(tool_t tool) {
        rapid_via_safe_z(manual_position());

        char message[128];
        snprintf(message, sizeof(message), "tool #%u has no pocket; fit it by hand, then press start", unsigned(tool));
        wait_for_operator(message);

        // No beam check here: the beam is over the magazine and we are not.
        // The toolsetter measurement that follows is the real confirmation.
        _last_tool = tool;
    }

    void RapidChange::manual_unload() {
        rapid_via_safe_z(manual_position());

        char message[128];
        snprintf(message, sizeof(message), "tool #%u has nowhere to go; remove it by hand, then press start", unsigned(_last_tool));
        wait_for_operator(message);

        _last_tool = 0;
        invalidate_tool_length();
    }

    bool RapidChange::verify_tool_unchanged() {
        if (_last_tool == 0 || !_tlo_valid) {
            return true;  // nothing recorded to disagree with
        }

        const float recorded = _tool_length;

        float measured = 0.0;
        if (!measure_tool_length(measured)) {
            return false;
        }

        const float drift = measured - recorded;
        if (std::fabs(drift) > _drift_tolerance) {
            log_error("ATC: tool #" << _last_tool << " now measures " << measured << " mm but was " << recorded << " mm, a drift of "
                                    << drift << " mm. It has broken, slipped in the collet, or is not the tool it is recorded as");
            return false;
        }

        log_info("ATC: tool #" << _last_tool << " still measures " << measured << " mm, drift " << drift << " mm");
        return true;
    }

    void RapidChange::retreat_to_safe() {
        try {
            rapid_via_safe_z(_retreat_mpos);
            log_info("ATC: retreated to the safe position");
        } catch (const AtcFault& fault) { log_error("ATC: could not reach the safe position - " << fault.message); }
    }

    bool RapidChange::tool_change(tool_t new_tool, bool pre_select, bool set_tool) {
        if (pre_select) {
            return true;  // nothing to stage; the magazine is not indexed
        }

        protocol_buffer_synchronize();

        // M61 adopts a tool number without touching the machine. Used to tell the
        // changer what is already in the spindle after a manual intervention.
        if (set_tool) {
            const int32_t pocket = pocket_holding(new_tool);
            if (pocket >= 0) {
                _pocket_tools[pocket] = 0;  // it is in the spindle now, not in a pocket
            }
            _last_tool = new_tool;
            if (new_tool == 0) {
                invalidate_tool_length();
            }
            log_magazine();
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
            // Stops the spindle, which is about to be used as a screwdriver, and
            // pins the feed mode: the engagement descent is a G1 with an F word,
            // and under an inherited G93 or G95 that F would mean something else
            // entirely.
            enter_tool_change_state();

            // Re-measure what is in the spindle before anything moves. A tool that
            // snapped or crept invalidates the offset and every tip-space limit
            // derived from it, and it is worth knowing that while the tool is
            // still in a known place.
            if (!verify_tool_unchanged()) {
                throw AtcFault { "the fitted tool is not the length it is recorded as", false };
            }

            if (_last_tool != 0 && tool_t(_last_tool) == new_tool && _tlo_valid) {
                log_info("ATC: tool #" << new_tool << " is already fitted and still measures correctly");
                restore_program_state(saved);
                return true;
            }

            bool over_magazine = false;
            if (_last_tool != 0) {
                const int32_t pocket = pocket_for_putting_away(_last_tool);
                if (pocket < 0) {
                    manual_unload();
                } else {
                    unload_to_pocket(size_t(pocket));
                    over_magazine = true;
                }
            }

            // M6 T0 empties the spindle and parks, ready for a new job.
            if (new_tool == 0) {
                rapid_via_safe_z(_retreat_mpos);
                invalidate_tool_length();
                log_magazine();
                restore_program_state(saved);
                return true;
            }

            const int32_t pocket = pocket_holding(new_tool);
            if (pocket < 0) {
                manual_load(new_tool);
            } else {
                load_from_pocket(size_t(pocket), over_magazine);
            }

            // Always measured, never assumed. Collets get swapped between pockets
            // and picking a holder up is not repeatable to the accuracy the offset
            // needs.
            float length = 0.0;
            if (!measure_tool_length(length)) {
                throw AtcFault { "tool length could not be measured", false };
            }
            apply_tool_length(length);

            // Back to where the job was interrupted.
            move_to_safe_z();
            rapid_xy(start[X_AXIS], start[Y_AXIS]);
            rapid_z(start[Z_AXIS]);

            restore_program_state(saved);

            log_magazine();
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

    void RapidChange::save_atc_data(std::vector<uint8_t>& buffer) {
        ProbingATC::save_atc_data(buffer);

        const uint8_t count  = uint8_t(_pocket_tools.size());
        const auto    offset = buffer.size();
        buffer.resize(offset + sizeof(uint8_t) + count * sizeof(int32_t));
        memcpy(buffer.data() + offset, &count, sizeof(uint8_t));
        for (uint8_t i = 0; i < count; i++) {
            const int32_t tool = _pocket_tools[i];
            memcpy(buffer.data() + offset + sizeof(uint8_t) + i * sizeof(int32_t), &tool, sizeof(int32_t));
        }
    }

    void RapidChange::restore_atc_data(const std::vector<uint8_t>& buffer, size_t& index) {
        ProbingATC::restore_atc_data(buffer, index);

        if (index + sizeof(uint8_t) > buffer.size()) {
            return;  // saved by a build that had no magazine; init() will seed it
        }

        uint8_t count = 0;
        memcpy(&count, buffer.data() + index, sizeof(uint8_t));
        index += sizeof(uint8_t);

        if (index + size_t(count) * sizeof(int32_t) > buffer.size()) {
            log_warn("ATC: persisted magazine state is truncated; falling back to the configured layout");
            return;
        }

        _pocket_tools.resize(count);
        for (uint8_t i = 0; i < count; i++) {
            int32_t tool = 0;
            memcpy(&tool, buffer.data() + index, sizeof(int32_t));
            index += sizeof(int32_t);
            _pocket_tools[i] = tool;
        }
    }

    namespace {
        ATCFactory::InstanceBuilder<RapidChange> registration("rapidchange");
    }
}
