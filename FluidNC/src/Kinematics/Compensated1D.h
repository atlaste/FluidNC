// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
    Compensated1D.h

    Kinematic decorator that applies 1D X-axis compensation based on Z position.
    Designed for lathes to correct for spindle runout or bed wear.

    Uses a fixed-granularity lookup table with linear interpolation for O(1) lookup.
*/

#include "Kinematics.h"
#include <vector>
#include <string>

namespace Kinematics {

    class Compensated1D : public KinematicSystem {
    public:
        Compensated1D(const char* name) : KinematicSystem(name) {}

        Compensated1D(const Compensated1D&)            = delete;
        Compensated1D(Compensated1D&&)                 = delete;
        Compensated1D& operator=(const Compensated1D&) = delete;
        Compensated1D& operator=(Compensated1D&&)      = delete;

        // KinematicSystem interface
        bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) override;
        void motors_to_cartesian(float* cartesian, float* motors, axis_t n_axis) override;
        bool transform_cartesian_to_motors(float* motors, float* cartesian) override;

        void init() override;
        void init_position() override;

        // Delegate to wrapped kinematic
        void constrain_jog(float* cartesian, plan_line_data_t* pl_data, float* position) override;
        bool invalid_line(float* cartesian) override;
        bool invalid_arc(float*            target,
                         plan_line_data_t* pl_data,
                         float*            position,
                         float             center[3],
                         float             radius,
                         axis_t            caxes[3],
                         bool              is_clockwise_arc,
                         uint32_t          rotations) override;

        bool canHome(AxisMask axisMask) override;
        void releaseMotors(AxisMask axisMask, MotorMask motors) override;
        bool limitReached(AxisMask& axisMask, MotorMask& motors, MotorMask limited) override;
        bool kinematics_homing(AxisMask& axisMask) override;

        void homing_move(AxisMask axes, MotorMask motors, Machine::Homing::Phase phase, uint32_t settling_ms) override;
        void set_homed_mpos(float* mpos) override;

        // Configuration
        void afterParse() override;
        void group(Configuration::HandlerBase& handler) override;
        void validate() override;

        // Compensation table management
        bool load();
        bool save();
        float interpolate(float z) const;

        // Accessors for commands
        void  setOffset(float z, float x_offset);
        void  clearOffsets();
        float getZMin() const { return _z_min; }
        float getZMax() const { return _z_max; }
        float getGranularity() const { return _granularity; }
        const std::vector<float>& getOffsets() const { return _offsets; }
        const std::string& getFilename() const { return _filename; }

    protected:
        ~Compensated1D() {}

    private:
        KinematicSystem* _wrapped = nullptr;

        // Config parameters
        float       _z_min       = -500.0f;
        float       _z_max       = 500.0f;
        float       _granularity = 0.1f;
        std::string _filename    = "/localfs/comp1d.yaml";

        // Runtime data
        std::vector<float> _offsets;
        bool               _loaded = false;

        int zToIndex(float z) const;
        float indexToZ(int index) const;
    };

}  // namespace Kinematics

// Global pointer for command access (set during init if this kinematic is active)
extern Kinematics::Compensated1D* compensated1D;
