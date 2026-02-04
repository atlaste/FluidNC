// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
    Compensated2D.h

    Kinematic decorator that applies 2D Z-axis compensation based on X,Y position.
    Designed for mills to correct for bed leveling / height map compensation.

    Uses a grid with bicubic Catmull-Rom spline interpolation.
*/

#include "Kinematics.h"
#include <vector>
#include <string>

namespace Kinematics {

    class Compensated2D : public KinematicSystem {
    public:
        Compensated2D(const char* name) : KinematicSystem(name) {}

        Compensated2D(const Compensated2D&)            = delete;
        Compensated2D(Compensated2D&&)                 = delete;
        Compensated2D& operator=(const Compensated2D&) = delete;
        Compensated2D& operator=(Compensated2D&&)      = delete;

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
        bool  load();
        bool  save();
        float interpolate(float x, float y) const;

        // Accessors for commands
        void  setOffset(int ix, int iy, float z_offset);
        void  clearOffsets();
        float getXMin() const { return _x_min; }
        float getXMax() const { return _x_max; }
        float getYMin() const { return _y_min; }
        float getYMax() const { return _y_max; }
        int   getXCount() const { return _x_count; }
        int   getYCount() const { return _y_count; }
        float getOffset(int ix, int iy) const;
        const std::vector<float>& getOffsets() const { return _offsets; }
        const std::string&        getFilename() const { return _filename; }

    protected:
        ~Compensated2D() {}

    private:
        KinematicSystem* _wrapped = nullptr;

        // Config parameters
        float       _x_min   = 0.0f;
        float       _x_max   = 200.0f;
        float       _y_min   = 0.0f;
        float       _y_max   = 200.0f;
        int32_t     _x_count = 10;
        int32_t     _y_count = 10;
        std::string _filename = "/localfs/comp2d.yaml";

        // Runtime data
        std::vector<float> _offsets;  // Row-major: [y * _x_count + x]
        bool               _loaded = false;

        // Bicubic interpolation helpers
        float getOffsetClamped(int ix, int iy) const;
        float cubicInterpolate(float t, float p0, float p1, float p2, float p3) const;
        float bicubicInterpolate(int ix, int iy, float tx, float ty) const;
    };

}  // namespace Kinematics

// Global pointer for command access (set during init if this kinematic is active)
extern Kinematics::Compensated2D* compensated2D;
