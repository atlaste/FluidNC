// Copyright (c) 2026 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "ProbingATC.h"

namespace ATCs {
    // Manual tool change against an electronic toolsetter.
    //
    // Everything about measuring, failing and persisting lives in ProbingATC.
    // What is left here is only the mechanism: park somewhere the operator can
    // reach, stop and ask, and hand back afterwards.
    //
    // atc_manual:
    //   safe_z_mpos_mm: -1.000
    //   change_mpos_mm: 80.000 0.000 -1.000
    //   retrieve_mpos_mm: 80.000 0.000 -1.000
    //   ets_mpos_mm: 5.000 -9.000 -40.000   # X/Y centre of the setter, Z = deepest permitted probe
    //   ets_rapid_z_mpos_mm: -25.000
    //   ets_surface_mpos_mm: -38.000
    //   probe_seek_rate_mm_per_min: 500.000
    //   probe_feed_rate_mm_per_min: 80.000
    //   probe_repeats: 2
    //   probe_tolerance_mm: 0.030
    //   min_tool_length_mm: 5.000
    //   max_tool_length_mm: 120.000
    class Manual_ATC : public ProbingATC {
    public:
        explicit Manual_ATC(const char* name) : ProbingATC(name) {}

        Manual_ATC(const Manual_ATC&)            = delete;
        Manual_ATC(Manual_ATC&&)                 = delete;
        Manual_ATC& operator=(const Manual_ATC&) = delete;
        Manual_ATC& operator=(Manual_ATC&&)      = delete;

        ~Manual_ATC() override = default;

        void probe_notification() override;
        bool tool_change(tool_t value, bool pre_select, bool set_tool) override;

        void validate() override;

        void group(Configuration::HandlerBase& handler) override {
            ProbingATC::group(handler);
            handler.item("change_mpos_mm", _change_mpos);
            handler.item("retrieve_mpos_mm", _retrieve_mpos);
        }

    private:
        // Where the operator fits the tool, and where they collect the setter
        // afterwards.  Both in machine coordinates.
        std::vector<float> _change_mpos   = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        std::vector<float> _retrieve_mpos = {};  // empty falls back to _change_mpos

        const std::vector<float>& retrieve_position() const { return _retrieve_mpos.empty() ? _change_mpos : _retrieve_mpos; }

        void retreat_to_safe() override;
    };
}
