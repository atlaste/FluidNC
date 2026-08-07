// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "SoftLimitsComponent.h"
#include "../Module.h"
#include <string>

// A bounding box that is tied to one or more axes. The box is defined at origin (0,0,0)
// with given dimensions; the current machine position for each tied axis is added to
// get the box position in machine coordinates.
//
// Example: Tool turret that moves with X and Z. Define box at (0,0,0) with size,
// set tie_x: true, tie_z: true. The box then follows the turret position.
//
// Example YAML:
//   moving_box:
//     name: turret_body
//     min_x: -40
//     min_y: 0
//     min_z: -50
//     max_x: 40
//     max_y: 80
//     max_z: 0
//     tie_x: true
//     tie_y: false
//     tie_z: true

class MovingBoundingBox : public ConfigurableModule, public SoftLimitsComponent {
public:
    explicit MovingBoundingBox(const char* name) : ConfigurableModule(name) {}
    ~MovingBoundingBox() override;

    // Configuration::Configurable interface
    void group(Configuration::HandlerBase& handler) override;
    void afterParse() override;

    // ConfigurableModule interface
    void init() override;

    // SoftLimitsComponent interface
    bool TestLimit(const LimitContext& ctx) override;
    const char* componentName() const override { return name(); }

protected:
    // Box dimensions at origin
    float _minX = 0.0f;
    float _minY = 0.0f;
    float _minZ = 0.0f;
    float _maxX = 0.0f;
    float _maxY = 0.0f;
    float _maxZ = 0.0f;

    // Which axes the box is tied to (box position += current position for that axis)
    bool _tieX = false;
    bool _tieY = false;
    bool _tieZ = false;

    bool _enabled = true;

    // Compute box min/max in machine coords given a position
    void getBoxAtPosition(const float* pos, float* boxMin, float* boxMax) const;
};
