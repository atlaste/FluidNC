// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "SoftLimitsComponent.h"
#include "../Module.h"
#include "../Types.h"

// A configurable cylindrical exclusion zone for soft limits.
// Implemented as a ConfigurableModule; add a section in YAML to enable.
//
// Example YAML configuration for a chuck aligned with Z axis:
//   fixed_cylinder:
//     axis: Z
//     center_x: 0
//     center_y: 0
//     start: 0
//     length: 80
//     radius: 75

class FixedCylinder : public ConfigurableModule, public SoftLimitsComponent {
public:
    explicit FixedCylinder(const char* name) : ConfigurableModule(name) {}
    ~FixedCylinder() override;

    // Configuration::Configurable interface
    void group(Configuration::HandlerBase& handler) override;
    void afterParse() override;

    // ConfigurableModule interface
    void init() override;

    // SoftLimitsComponent interface
    bool TestLimit(const float* from, const float* to) override;
    const char* componentName() const override { return name(); }

private:
    char _axis = 'Z';
    float _centerX = 0.0f;
    float _centerY = 0.0f;
    float _startPos = 0.0f;
    float _length = 0.0f;
    float _radius = 0.0f;

    bool _enabled = true;

    void getAxisIndices(int& axisIdx, int& perpIdx1, int& perpIdx2) const;
};
