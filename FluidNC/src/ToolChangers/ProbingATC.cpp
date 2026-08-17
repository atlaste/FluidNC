// Copyright (c) 2026 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "ProbingATC.h"

#include "Machine/MachineConfig.h"
#include "MotionControl.h"  // probe_succeeded
#include "Protocol.h"       // protocol_buffer_synchronize, send_alarm
#include "System.h"         // sys, probe_steps, steps_to_mpos
#include "State.h"          // state_is

#include "../SoftLimits/SoftLimitsComponent.h"

#include <cmath>
#include <cstdio>
#include <cstring>

extern parser_block_t gc_block;

namespace ATCs {
    ProbingATC::ChangeScope::ChangeScope(ProbingATC& atc) : _atc(atc) {
        _atc._in_change = true;
        set_change_active(true);
        SoftLimitsComponent::setToolChangeActive(true);
    }

    ProbingATC::ChangeScope::~ChangeScope() {
        _atc._in_change = false;
        set_change_active(false);
        SoftLimitsComponent::setToolChangeActive(false);
    }

    void ProbingATC::init() {
        log_info("ATC:" << name() << " ETS surface Z" << _ets_surface_mpos << " tool length " << _min_tool_length << " to "
                        << _max_tool_length << " mm");
        if (_ets_surface_mpos == 0.0) {
            log_warn("ATC: ets_surface_mpos_mm is 0, which is almost certainly uncalibrated. Tool lengths will be "
                     "rejected as implausible until it is set. Probe a tool of known length and use "
                     "ets_surface_mpos_mm = contact Z - known length");
        }
        if (_tlo_valid) {
            log_info("ATC: restored tool #" << _last_tool << " length " << _tool_length << " mm");
        }

        set_offset_enforced(_must_measure);
        set_offset_valid(_tlo_valid);
        if (_must_measure) {
            log_info("ATC: must_measure_tool is on; g-code motion is refused until a tool has been measured");
        }
    }

    void ProbingATC::validate() {
        Assert(_ets_mpos.size() >= 3, "atc: ets_mpos_mm needs at least three values");
        Assert(_max_tool_length > _min_tool_length, "atc: max_tool_length_mm must exceed min_tool_length_mm");
    }

    void ProbingATC::group(Configuration::HandlerBase& handler) {
        handler.item("safe_z_mpos_mm", _safe_z, -100000, 100000);
        handler.item("probe_seek_rate_mm_per_min", _probe_seek_rate, 1, 10000);
        handler.item("probe_feed_rate_mm_per_min", _probe_feed_rate, 1, 10000);
        handler.item("probe_retract_mm", _probe_retract, 0.1, 100);
        handler.item("probe_repeats", _probe_repeats, 1, 5);
        handler.item("probe_tolerance_mm", _probe_tolerance, 0.001, 5.0);
        handler.item("ets_mpos_mm", _ets_mpos);
        handler.item("ets_rapid_z_mpos_mm", _ets_rapid_z_mpos);
        handler.item("ets_surface_mpos_mm", _ets_surface_mpos, -100000, 100000);
        handler.item("min_tool_length_mm", _min_tool_length, 0, 1000);
        handler.item("max_tool_length_mm", _max_tool_length, 0, 1000);
        handler.item("air_blast_output", _air_blast_output, -1, 3);
        handler.item("air_blast_ms", _air_blast_ms, 0, 10000);
        handler.item("must_measure_tool", _must_measure);
    }

    void ProbingATC::run(const char* gcode) {
        if (sys.abort() || state_is(State::Alarm) || state_is(State::Critical) || state_is(State::ConfigAlarm)) {
            throw AtcFault { "machine stopped", true };
        }

        // gc_execute_line overwrites the global gc_block and we are called from
        // inside M6 processing, so the caller's block has to be put back.
        parser_block_t saved = gc_block;
        Error          err   = gc_execute_line(gcode);
        gc_block             = saved;

        if (err != Error::Ok) {
            throw AtcFault { std::string(gcode) + " failed with error " + std::to_string(int(err)), false };
        }

        protocol_buffer_synchronize();

        if (sys.abort() || state_is(State::Alarm) || state_is(State::Critical)) {
            throw AtcFault { std::string("machine stopped during ") + gcode, true };
        }
    }

    void ProbingATC::move_to_safe_z() {
        rapid_z(_safe_z);
    }

    void ProbingATC::rapid_xy(float x, float y) {
        char line[96];
        snprintf(line, sizeof(line), "G53G0X%0.3fY%0.3f", x, y);
        run(line);
    }

    void ProbingATC::rapid_z(float z) {
        char line[64];
        snprintf(line, sizeof(line), "G53G0Z%0.3f", z);
        run(line);
    }

    void ProbingATC::rapid_via_safe_z(const std::vector<float>& target) {
        move_to_safe_z();
        rapid_xy(target[0], target[1]);
        rapid_z(target[2]);
    }

    void ProbingATC::move_over_toolsetter() {
        move_to_safe_z();
        rapid_xy(_ets_mpos[0], _ets_mpos[1]);
    }

    void ProbingATC::air_blast() {
        if (_air_blast_output < 0 || _air_blast_ms == 0) {
            return;
        }
        char line[64];
        snprintf(line, sizeof(line), "M62P%d", int(_air_blast_output));
        run(line);
        snprintf(line, sizeof(line), "G4P%0.3f", _air_blast_ms / 1000.0f);
        run(line);
        snprintf(line, sizeof(line), "M63P%d", int(_air_blast_output));
        run(line);
    }

    bool ProbingATC::probe_once(float& contact_z) {
        char  line[96];
        float pos[MAX_N_AXIS];

        rapid_z(_ets_rapid_z_mpos);

        // G38.3 rather than G38.2: a probe that finds nothing must be reported
        // back here so the sequence can retreat, not turned into an alarm that
        // kills the job.
        if (_probe_seek_rate > _probe_feed_rate) {
            snprintf(line, sizeof(line), "G53G38.3Z%0.3fF%0.3f", _ets_mpos[2], _probe_seek_rate);
            run(line);
            if (!probe_succeeded) {
                return false;
            }
            steps_to_mpos(pos, probe_steps);
            rapid_z(pos[Z_AXIS] + _probe_retract);
        }

        snprintf(line, sizeof(line), "G53G38.3Z%0.3fF%0.3f", _ets_mpos[2], _probe_feed_rate);
        run(line);
        if (!probe_succeeded) {
            return false;
        }

        steps_to_mpos(pos, probe_steps);
        contact_z = pos[Z_AXIS];

        rapid_z(contact_z + _probe_retract);

        return true;
    }

    bool ProbingATC::measure_tool_length(float& length) {
        move_over_toolsetter();
        air_blast();

        float first = 0.0;
        if (!probe_once(first)) {
            log_error("ATC: toolsetter never tripped; tool missing, too short, or the setter is not in place");
            return false;
        }

        float sum   = first;
        int   count = 1;

        // Repeating the probe catches the readings that are wrong rather than
        // merely imprecise: a chip under the setter, a tool not seated, a setter
        // nudged out of its nest.  One probe cannot tell you it was wrong.
        for (int32_t i = 1; i < _probe_repeats; i++) {
            float next = 0.0;
            if (!probe_once(next)) {
                log_error("ATC: confirming probe never tripped");
                return false;
            }
            if (std::fabs(next - first) > _probe_tolerance) {
                log_error("ATC: probes disagree by " << (next - first) << " mm, tolerance is " << _probe_tolerance << " mm");
                return false;
            }
            sum += next;
            count++;
        }

        float contact_z = sum / count;
        float measured  = contact_z - _ets_surface_mpos;

        log_info("ATC: contact at Z" << contact_z << " gives tool length " << measured << " mm");

        if (measured < _min_tool_length || measured > _max_tool_length) {
            log_error("ATC: tool length " << measured << " mm is outside the plausible range " << _min_tool_length << " to "
                                          << _max_tool_length << " mm; not applying it");
            log_error("ATC: if every tool reads like this, ets_surface_mpos_mm is uncalibrated; set it to " << contact_z
                                                                                                           << " minus this tool's real length");
            return false;
        }

        length = measured;
        return true;
    }

    void ProbingATC::apply_tool_length(float length) {
        char line[64];
        snprintf(line, sizeof(line), "G43.1Z%0.4f", length);
        run(line);
        _tool_length = length;
        set_tlo_valid(true);
    }

    void ProbingATC::invalidate_tool_length() {
        set_tlo_valid(false);
        _tool_length = 0.0;
        try {
            run("G43.1Z0");
        } catch (const AtcFault&) {
            // Offsets stop mattering once the machine is stopping anyway.
        }
        log_warn("ATC: tool length offset cleared and marked unvalidated; probe again before cutting");
    }

    void ProbingATC::wait_for_operator(const char* message) {
        log_info("ATC: " << message);
        run("M0");
    }

    ProbingATC::ProgramState ProbingATC::record_program_state() const {
        ProgramState saved;
        saved.units         = gc_state.modal.units;
        saved.distance      = gc_state.modal.distance;
        saved.feed_rate     = gc_state.modal.feed_rate;
        saved.coolant       = gc_state.modal.coolant;
        saved.spindle       = gc_state.modal.spindle;
        saved.spindle_speed = gc_state.spindle_speed;
        return saved;
    }

    void ProbingATC::enter_tool_change_state() {
        run("M5M9G21G90G94");
    }

    void ProbingATC::restore_program_state(const ProgramState& saved) {
        if (saved.coolant.Flood) {
            run("M8");
        }
        if (saved.coolant.Mist) {
            run("M7");
        }
        if (saved.distance == Distance::Incremental) {
            run("G91");
        }
        if (saved.feed_rate == FeedRate::InverseTime) {
            run("G93");
        } else if (saved.feed_rate == FeedRate::UnitsPerRev) {
            run("G95");
        }
        if (saved.units == Units::Inches) {
            run("G20");
        }
        if (saved.spindle != SpindleState::Disable && saved.spindle != SpindleState::Unknown) {
            char line[64];
            // The spindle applies its own spin-up delay.
            snprintf(line, sizeof(line), "%sS%0.0f", saved.spindle == SpindleState::Ccw ? "M4" : "M3", saved.spindle_speed);
            run(line);
        }
    }

    void ProbingATC::fail(const std::string& message) {
        log_error("ATC: " << message);

        // Ending the job is not enough, because most of the time there is no
        // job: a sender streaming line by line over serial has an empty job
        // stack, so Job::active() is false and nothing would stop the next line
        // of the program from running with no valid offset.  The alarm state is
        // the only thing that gates incoming G-code - execute_line answers
        // SystemGcLock while it is set - so it is what actually makes Candle,
        // UGS and the rest stop streaming.
        //
        // The caller has already retreated, so the alarm finds the machine
        // parked clear of the work rather than wherever it gave up.  That plus
        // the choice of ToolChange over a Critical alarm code is what keeps
        // this cheap to recover from: position and homing both survive, so $X
        // alone puts the machine back in Idle with no re-homing.
        send_alarm(ExecAlarm::ToolChange);

        // send_alarm only queues an event; draining it here means M6 returns
        // with the machine already in Alarm rather than one line later.
        protocol_execute_realtime();

        log_error("ATC: alarmed at the safe position. Correct the cause, $X to unlock, then restart the job");
    }

    bool ProbingATC::ready_to_change() {
        if (sys.abort() || state_is(State::Alarm) || state_is(State::Critical)) {
            log_error("ATC: cannot change tools while the machine is stopped");
            return false;
        }

        if (_in_change) {
            log_error("ATC: an M6 arrived while a tool change was already running; ignoring it");
            return false;
        }

        return true;
    }

    void ProbingATC::save_atc_data(std::vector<uint8_t>& buffer) {
        ATC::save_atc_data(buffer);

        uint8_t valid = _tlo_valid ? 1 : 0;
        auto    size  = buffer.size();
        buffer.resize(size + sizeof(float) + sizeof(uint8_t));
        memcpy(buffer.data() + size, &_tool_length, sizeof(float));
        memcpy(buffer.data() + size + sizeof(float), &valid, sizeof(uint8_t));
    }

    void ProbingATC::restore_atc_data(const std::vector<uint8_t>& buffer, size_t& index) {
        ATC::restore_atc_data(buffer, index);

        if (index + sizeof(float) + sizeof(uint8_t) > buffer.size()) {
            // Saved by an older build that did not store the tool length.  A
            // missing measurement has to read as unvalidated, never as zero.
            _tool_length = 0.0;
            set_tlo_valid(false);
            return;
        }

        uint8_t valid = 0;
        memcpy(&_tool_length, buffer.data() + index, sizeof(float));
        index += sizeof(float);
        memcpy(&valid, buffer.data() + index, sizeof(uint8_t));
        index += sizeof(uint8_t);
        set_tlo_valid(valid != 0);
    }
}
