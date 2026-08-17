// Copyright (c) 2026 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "SoftLimitsComponent.h"
#include "../DynamicLimits.h"
#include "../Module.h"
#include "../Types.h"

#include <cmath>

// A half-space keep-out for soft limits: one axis, one bound, everything beyond
// it forbidden to the tool tip.
//
// This exists because the obvious alternative, a box, is the wrong shape for the
// job it is usually asked to do.  Guarding the table means saying "the cutter may
// not go below Z-95", which as a box has to be given XY extents that are really
// meant to be infinite, and then relies on a segment-versus-volume intersection
// test where the honest test is a comparison.  A plane has no extents to invent
// and no way for a move that merely grazes the boundary to slip through.
//
// The bound is on the tool tip rather than the spindle nose, so it tracks the
// tool length offset and follows tool changes; that is the whole point, since a
// machine-coordinate Z minimum cannot tell a stub drill from a long end mill.
//
// Registered twice, deliberately.  As a SoftLimitsComponent it blocks G-code
// moves, including the individual segments of an arc.  As a DynamicLimitProvider
// it also reaches Cartesian::constrain_jog, which clamps a jog to the limit
// instead of letting it run into one - a plane is one of the few shapes for
// which clamping is even well defined, and without it every jog towards the
// table would end in a soft-limit alarm.
//
// Example YAML configuration:
//   fixed_plane:
//     axis: Z
//     min_mm: -95.000            # the table: the cutting edge may not go below it
//
//   fixed_plane:                 # repeat the section for more planes
//     axis: X
//     max_mm: 300.000
//
class FixedPlane : public ConfigurableModule, public SoftLimitsComponent, public DynamicLimitProvider {
public:
    explicit FixedPlane(const char* name) : ConfigurableModule(name) {}
    ~FixedPlane() override;

    // Configuration::Configurable interface
    void group(Configuration::HandlerBase& handler) override;
    void afterParse() override;

    // ConfigurableModule interface
    void init() override;

    // SoftLimitsComponent interface
    bool        TestLimit(const LimitContext& ctx) override;
    const char* componentName() const override { return name(); }

    // DynamicLimitProvider interface
    void        getDynamicLimits(const float* current_mpos, const float* active_tlo, float* axis_min, float* axis_max) override;
    const char* limitProviderName() override { return name(); }
    bool        isActive() override;

protected:
    char   _axis_letter = 'Z';
    axis_t _axis        = Z_AXIS;

    // Bounds on the tool tip along _axis, in machine coordinates.  NAN means
    // unbounded on that side, which is how a floor, a ceiling or a slab are all
    // expressed with the same two keys.
    float _min = NAN;
    float _max = NAN;

    // Tool length to assume while no offset is active.
    //
    // With no G43 in effect the tip and the spindle nose coincide as far as the
    // arithmetic is concerned, so a long tool is invisible and the plane would
    // wave it through - exactly the case the plane is there to catch.  Setting
    // this to the longest tool that can be fitted makes the guarantee hold
    // whether or not the operator remembered to probe.  Zero disables it, which
    // is the default because most machines are driven with no offset at all and
    // would otherwise find the plane inexplicably tight.
    float _unknown_tool_length = 0.0f;

    bool _enabled = true;

    // Offset to add to a published bound to account for an unmeasured tool.
    float toolBias(const float* tlo) const;
};
