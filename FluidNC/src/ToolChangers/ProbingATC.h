// Copyright (c) 2026 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Config.h"

#include "Configuration/Configurable.h"

#include "Channel.h"
#include "Module.h"
#include "atc.h"

#include "../GCode.h"
#include "../SpindleDatatypes.h"

#include <string>
#include <vector>

namespace ATCs {
    // Raised when a step of a tool change cannot complete.
    //
    // `fatal` distinguishes the two cases that need different handling.  A
    // reset, an e-stop or an alarm has already stopped the machine on its own
    // terms, so no further motion may be commanded and the only correct action
    // is to unwind.  Everything else - a probe that found nothing, a length
    // that is not plausible, two probes that disagree - leaves the machine
    // under control, so the changer can still drive it to a known safe position
    // before raising ExecAlarm::ToolChange from there.
    struct AtcFault {
        std::string message;
        bool        fatal;
    };

    // Everything a tool changer needs in order to measure a tool against an
    // electronic toolsetter and fail safely, without knowing how the tool got
    // into the spindle.  Subclasses supply only the mechanism: an operator and a
    // pair of pauses for the manual changer, a magazine and a threading cycle
    // for RapidChange.
    //
    // Kept in one place because the parts that are easy to get subtly wrong are
    // exactly the shared parts.  Two changers with their own copies of the fault
    // handling would drift, and the direction of the drift would be that one of
    // them stops stopping a streaming sender.
    //
    // Every move is issued as G53 so that the operator's G90/G91 and G20/G21
    // state cannot redirect it, and every step blocks until its motion has
    // finished so the outcome can be checked before the next step is decided.
    //
    // Tool lengths are absolute, measured from ets_surface_mpos_mm rather than
    // from whichever tool happened to be probed first.  This matters beyond
    // convenience: FluidNC's soft-limit components locate the cutting edge as
    // MPos - TLO, so a TLO expressed relative to some reference tool displaces
    // every keep-out volume by that tool's length.  Absolute lengths also
    // survive a reboot, do not care about tool ordering, and can be
    // range-checked for plausibility.
    //
    // Calibrate ets_surface_mpos_mm once, by probing a tool whose length you
    // know:
    //
    //     ets_surface_mpos_mm = <probe contact Z> - <known tool length>
    //
    // The contact Z of every probe is logged, so reading it off is easy.
    class ProbingATC : public ATC {
    public:
        explicit ProbingATC(const char* name) : ATC(name) {}

        void init() override;
        void validate() override;
        void group(Configuration::HandlerBase& handler) override;

        // These changers measure the tool themselves, so M6 must not also apply
        // a tool-table offset on top.
        bool handles_tlo() override { return true; }

        void save_atc_data(std::vector<uint8_t>& buffer) override;
        void restore_atc_data(const std::vector<uint8_t>& buffer, size_t& index) override;

    protected:
        // Height at which the tool clears everything on the table.
        float _safe_z = 50.0;

        // X/Y centre of the toolsetter.  Z is the deepest the spindle may travel
        // before a probe is declared a miss, so set it just short of the collet
        // reaching the setter.
        std::vector<float> _ets_mpos = { 0.0, 0.0, 0.0 };

        // Height to drop to before probing, to keep the slow probe move short.
        float _ets_rapid_z_mpos = 0;

        // Machine Z at which a zero-length tool would trip the setter.  This is
        // the datum that makes measured lengths absolute.
        float _ets_surface_mpos = 0.0;

        float   _probe_seek_rate = 200.0;
        float   _probe_feed_rate = 80.0;
        float   _probe_retract   = 5.0;
        int32_t _probe_repeats   = 2;
        float   _probe_tolerance = 0.03;

        // A measured length outside these bounds is not believed.  The worst-case
        // job that max was once meant to do now belongs to a fixed_plane's
        // unknown_tool_length_mm, which is configured separately because the
        // limit layer has no way to ask an ATC anything.
        float _min_tool_length = 1.0;
        float _max_tool_length = 150.0;

        // Optional air blast to clear the toolsetter before probing.
        int32_t  _air_blast_output = -1;
        uint32_t _air_blast_ms     = 0;

        // Refuse g-code motion until a tool length has been measured, in the same
        // spirit as start/must_home.  Off by default because switching it on locks
        // out all g-code motion from power-up until the first tool change, which is
        // a workflow decision the machine owner has to make knowingly.
        bool _must_measure = false;

        // Runtime state.  Persisted so that a reboot mid-job cannot leave a
        // stale offset applied to a tool it does not describe.
        float _tool_length = 0.0;
        bool  _tlo_valid   = false;

        // Guards against a second M6 arriving while one is in progress.  The
        // protocol executes one line at a time, so this should be unreachable;
        // it is here to fail loudly rather than half-perform a change if that
        // ever stops being true.  Not persisted - a reboot ends any change.
        bool _in_change = false;

        // Marks a change in progress, for the two things that have to know.
        //
        // The must_measure_tool gate is suspended, so the sequence that exists to
        // produce a measurement is not blocked by the absence of one - including
        // the retreat after a failure, which happens with the offset cleared.
        //
        // Soft-limits components that opted into suspend_during_tool_change stand
        // down, because a changer drives into the volumes they defend on purpose:
        // the magazine it threads holders in and out of, and anything below a
        // tip-space floor once a long tool is fitted.
        //
        // Clears on every exit, including the ones taken by throwing out of run().
        class ChangeScope {
            ProbingATC& _atc;

        public:
            explicit ChangeScope(ProbingATC& atc);
            ~ChangeScope();

            ChangeScope(const ChangeScope&)            = delete;
            ChangeScope& operator=(const ChangeScope&) = delete;
        };

        // Keeps the persisted flag and the planner's gate in step. They must never
        // disagree, so nothing assigns _tlo_valid directly.
        void set_tlo_valid(bool valid) {
            _tlo_valid = valid;
            ATC::set_offset_valid(valid);
        }

        // Executes one g-code line synchronously and does not return until the
        // resulting motion has finished, so the caller can act on the outcome.
        // Throws AtcFault on any failure.
        void run(const char* gcode);

        void move_to_safe_z();
        void rapid_xy(float x, float y);
        void rapid_z(float z);

        // Safe Z, then XY, then down to the target Z.  The order is what keeps a
        // traverse from clipping anything between here and there.
        void rapid_via_safe_z(const std::vector<float>& target);

        void move_over_toolsetter();
        void air_blast();

        bool probe_once(float& contact_z);

        // Probes _probe_repeats times, rejects readings that disagree by more
        // than _probe_tolerance, and range-checks the result.  Returns false with
        // an explanatory log on any of those; does not throw.
        bool measure_tool_length(float& length);

        void apply_tool_length(float length);
        void invalidate_tool_length();

        void wait_for_operator(const char* message);

        // The modal state a tool change takes over and then hands back.
        //
        // G94 is the one that matters beyond tidiness.  Under G93 an F word
        // means "finish this move in 1/F minutes", so a feed that reads as
        // 1800 mm/min would drive a probe or a threading descent at whatever
        // the machine can manage.  G95 is worse still, being feed per
        // revolution of a spindle that a changer is about to stop or use as a
        // screwdriver.  Neither is exotic enough to leave to chance.
        //
        // Spindle speed is saved with the state because a changer that spins
        // the spindle itself leaves its own S word behind, and restoring M3
        // without an S would resume the job at the threading speed.
        struct ProgramState {
            Units        units         = Units::Mm;
            Distance     distance      = Distance::Absolute;
            FeedRate     feed_rate     = FeedRate::UnitsPerMin;
            CoolantState coolant       = {};
            SpindleState spindle       = SpindleState::Disable;
            float        spindle_speed = 0.0;
        };

        ProgramState record_program_state() const;

        // Spindle and coolant off, and the modal state every move in this file
        // assumes: millimetres, absolute, units per minute.
        void enter_tool_change_state();

        void restore_program_state(const ProgramState& saved);

        // Retreat, then alarm, in that order.  Callers must have retreated
        // already; see the comment on the definition for why the alarm is not
        // optional.
        void fail(const std::string& message);

        // Where this changer parks when it gives up.  Subclasses know; the base
        // class does not.  Must not throw.
        virtual void retreat_to_safe() = 0;

        // Common opening for tool_change: verifies the machine can move and that
        // no change is already running.  Returns false if the change must not
        // start.
        bool ready_to_change();
    };
}
