// Copyright (c) 2026 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "ProbingATC.h"
#include "Pin.h"

namespace ATCs {
    // RapidChange magazine tool changer.
    //
    // The holder is threaded on and off by turning the spindle while the holder
    // is held captive in a pocket, so the whole mechanism is: put the spindle
    // over a pocket, spin it the right way, and feed down.  Everything about
    // measuring the result and failing safely comes from ProbingATC.
    //
    // Three things make this worth more than a macro.  Pocket occupancy has to
    // survive a reboot, because a magazine that has forgotten what is in it will
    // happily thread a second holder onto the first.  A beam sensor above the
    // pockets brackets every engagement, so a holder that failed to release or
    // failed to pick up is caught immediately rather than discovered by a crash.
    // And the tool already in the spindle is re-measured before anything moves,
    // because a tool that snapped or crept in the collet invalidates both the
    // offset and every tip-space limit derived from it.
    //
    // Geometry is one flat vector of x/y/z triplets, so pockets do not have to
    // lie on a line the way a pitch-and-offset scheme would require.
    //
    // rapidchange:
    //   safe_z_mpos_mm: -1.000            # RapidChange calls this z_safe_clearance
    //   z_traverse_mpos_mm: -20.000       # height for moving between pockets
    //   z_spindle_start_mpos_mm: -40.000  # lowest point the spindle may start spinning
    //   z_engage_mpos_mm: -55.000         # bottom of the threading travel
    //   beam_check_mpos_mm: -25.000       # height at which a fitted tool breaks the beam
    //   tool_present_pin: gpio.35:low
    //   pocket_mpos_mm: 10.0 -10.0 -52.0  40.0 -10.0 -52.0  70.0 -10.0 -52.0
    //   pocket_tools: 1 2 3               # which tool starts in which pocket, 0 = empty
    //   engage_feed_rate_mm_per_min: 1800.000
    //   load_rpm: 900
    //   unload_rpm: 900
    //   spindle_settle_ms: 500
    //   engage_passes: 2
    //   tool_length_drift_mm: 0.500
    //   retreat_mpos_mm: 80.000 0.000 -1.000
    //   ... plus the shared ets_* and probe_* keys from ProbingATC
    class RapidChange : public ProbingATC {
    public:
        explicit RapidChange(const char* name) : ProbingATC(name) {}

        RapidChange(const RapidChange&)            = delete;
        RapidChange(RapidChange&&)                 = delete;
        RapidChange& operator=(const RapidChange&) = delete;
        RapidChange& operator=(RapidChange&&)      = delete;

        ~RapidChange() override = default;

        void init() override;
        void validate() override;
        void probe_notification() override;
        bool tool_change(tool_t value, bool pre_select, bool set_tool) override;

        void group(Configuration::HandlerBase& handler) override {
            ProbingATC::group(handler);
            handler.item("pocket_mpos_mm", _pocket_mpos);
            handler.item("pocket_tools", _pocket_seed);
            handler.item("retreat_mpos_mm", _retreat_mpos);
            handler.item("manual_mpos_mm", _manual_mpos);
            handler.item("z_traverse_mpos_mm", _z_traverse, -100000, 100000);
            handler.item("z_spindle_start_mpos_mm", _z_spindle_start, -100000, 100000);
            handler.item("z_engage_mpos_mm", _z_engage, -100000, 100000);
            handler.item("beam_check_mpos_mm", _beam_check_z, -100000, 100000);
            handler.item("beam_clear_mpos_mm", _beam_clear_z);
            handler.item("tool_present_pin", _tool_present_pin);
            handler.item("beam_settle_ms", _beam_settle_ms, 0, 5000);
            handler.item("engage_feed_rate_mm_per_min", _engage_feed_rate, 1, 10000);
            handler.item("load_rpm", _load_rpm, 1, 30000);
            handler.item("unload_rpm", _unload_rpm, 1, 30000);
            handler.item("spindle_settle_ms", _spindle_settle_ms, 0, 30000);
            handler.item("engage_passes", _engage_passes, 1, 5);
            handler.item("tool_length_drift_mm", _drift_tolerance, 0.01, 50.0);
        }

        void save_atc_data(std::vector<uint8_t>& buffer) override;
        void restore_atc_data(const std::vector<uint8_t>& buffer, size_t& index) override;

    private:
        // Pocket centres, three floats per pocket, in machine coordinates.
        std::vector<float> _pocket_mpos = {};

        // Which tool starts in which pocket. Seeds _pocket_tools on first boot and
        // afterwards only says where a tool prefers to be put back.
        std::vector<int32_t> _pocket_seed = {};

        // Live occupancy, persisted. Index is the pocket, value is the tool number
        // sitting in it, 0 for empty.
        std::vector<int32_t> _pocket_tools = {};

        // Where to park when the operator has to decide something.
        std::vector<float> _retreat_mpos = {};

        // Where the operator fits or removes a tool by hand. Empty reuses
        // _retreat_mpos.
        std::vector<float> _manual_mpos = {};

        float _z_traverse      = 0.0;
        float _z_spindle_start = 0.0;
        float _z_engage        = 0.0;
        float _beam_check_z    = 0.0;

        Pin      _tool_present_pin;
        uint32_t _beam_settle_ms = 100;

        // Height at which a correctly seated tool has cleared the beam. A
        // cross-threaded holder hangs lower and is still in it, which is how
        // that failure is told apart from a good pick-up. NaN skips the check.
        float _beam_clear_z = NAN;

        float    _engage_feed_rate  = 1800.0;
        int32_t  _load_rpm          = 900;
        int32_t  _unload_rpm        = 900;
        uint32_t _spindle_settle_ms = 500;

        // Threading a holder on takes a second pass to torque it up; backing one
        // off does not.
        int32_t _engage_passes = 2;

        // How far a re-measured tool may differ from its recorded length before
        // it is treated as broken or moved rather than merely imprecise.
        float _drift_tolerance = 0.5;

        size_t pocket_count() const { return _pocket_mpos.size() / 3; }
        std::vector<float> pocket_position(size_t pocket) const;

        // -1 when there is no such pocket.
        int32_t pocket_holding(tool_t tool) const;
        int32_t pocket_for_putting_away(tool_t tool) const;

        void feed_z(float z);
        void dwell_for(uint32_t ms);

        bool has_beam() const { return _tool_present_pin.defined(); }

        // Moves to the given height and reads the beam there. Callers must have
        // checked has_beam().
        bool beam_sees_tool(float z);

        // Reads the beam at _beam_check_z and throws unless it agrees. Silently
        // does nothing when no pin is configured.
        void verify_spindle(bool expect_tool, const char* context);

        void engage(bool loading);

        // Leaves the spindle at traverse height over the magazine.
        void unload_to_pocket(size_t pocket);

        // already_over_magazine skips the trip to safe Z, which is only correct
        // straight after an unload has left us at traverse height.
        void load_from_pocket(size_t pocket, bool already_over_magazine);

        // Fallbacks for tools the magazine cannot handle, so that a job may mix
        // pocketed tools with one that is too long, too fat, or simply not
        // assigned a pocket. The tool is measured afterwards either way, so a
        // hand-fitted tool ends up with as good an offset as a picked-up one.
        const std::vector<float>& manual_position() const { return _manual_mpos.empty() ? _retreat_mpos : _manual_mpos; }
        void                      manual_load(tool_t tool);
        void                      manual_unload();

        // Re-measures the tool already in the spindle and compares it with what
        // was recorded. False means it is not the tool we think it is.
        bool verify_tool_unchanged();

        void log_magazine() const;

        void retreat_to_safe() override;
    };
}
