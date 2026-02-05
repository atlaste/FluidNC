// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "SoftLimitsComponent.h"
#include "../Module.h"

// A configurable rectangular exclusion zone for soft limits.
// Implemented as a ConfigurableModule; add a section in YAML to enable.
//
// Example YAML configuration:
//   fixed_box:
//     min_x: -50
//     min_y: -100
//     min_z: -200
//     max_x: 50
//     max_y: 100
//     max_z: -150

class FixedBoundingBox : public ConfigurableModule, public SoftLimitsComponent {
public:
    explicit FixedBoundingBox(const char* name) : ConfigurableModule(name) {}
    ~FixedBoundingBox() override;

    // Configuration::Configurable interface
    void group(Configuration::HandlerBase& handler) override;
    void afterParse() override;

    // ConfigurableModule interface
    void init() override;

    // SoftLimitsComponent interface
    bool TestLimit(const float* from, const float* to) override;
    const char* componentName() const override { return name(); }

protected:
    // Bounding box corners in machine coordinates
    float _minX = 0.0f;
    float _minY = 0.0f;
    float _minZ = 0.0f;
    float _maxX = 0.0f;
    float _maxY = 0.0f;
    float _maxZ = 0.0f;

    bool _enabled = true;
};
