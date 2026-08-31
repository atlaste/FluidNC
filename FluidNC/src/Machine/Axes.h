// Copyright (c) 2021 -  Stefan de Bruijn
// Copyright (c) 2021 -  Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"
#include "Axis.h"

namespace MotorDrivers {
    class MotorDriver;
}

namespace Machine {
    class Axes : public Configuration::Configurable {
        bool _switchedStepper = false;

    public:
        static const char* _axisNames[];

        //        static constexpr const char* _names = "XYZABC";

        Axes();

        // Bitmasks to collect information about axes that have limits and homing
        static MotorMask posLimitMask;
        static MotorMask negLimitMask;
        static MotorMask limitMask;
        static MotorMask motorMask;

        static AxisMask homingMask;

        static bool disabled;

        static Pin _sharedStepperDisable;
        static Pin _sharedStepperReset;

        static uint32_t _homing_runs;  // Number of Approach/Pulloff cycles

        static axis_t axisNum(std::string_view axis_name);

        static inline const char* axisName(axis_t axis) {  // returns axis letter as C string
            return axis < MAX_N_AXIS ? _axisNames[axis] : "?";
        }

        // A MotorMask gives every motor its own MOTOR_MASK_STRIDE-wide lane of axis bits, so
        // motor 2 of Z is a different bit from motor 0 of Z.  Prefer motor_mask()/motor_is_set()
        // over the generic bitnum macros for these: the generic ones build their mask from a
        // plain int and would quietly produce garbage for the upper lanes.
        static inline size_t    motor_bit(axis_t axis, motor_t motor) { return size_t(axis) + MOTOR_MASK_STRIDE * size_t(motor); }
        static inline MotorMask motor_mask(axis_t axis, motor_t motor) { return MotorMask(1) << motor_bit(axis, motor); }
        static inline bool      motor_is_set(MotorMask mask, axis_t axis, motor_t motor) { return (mask & motor_mask(axis, motor)) != 0; }

        static inline AxisMask motors_to_axes(MotorMask motors) {
            AxisMask axes = 0;
            for (motor_t motor = 0; motor < MAX_MOTORS_PER_AXIS; ++motor) {
                axes |= AxisMask((motors >> (MOTOR_MASK_STRIDE * motor)) & ((MotorMask(1) << MOTOR_MASK_STRIDE) - 1));
            }
            return axes;
        }

        static inline MotorMask axes_to_motors(AxisMask axes) {
            MotorMask motors = 0;
            for (motor_t motor = 0; motor < MAX_MOTORS_PER_AXIS; ++motor) {
                motors |= MotorMask(axes) << (MOTOR_MASK_STRIDE * motor);
            }
            return motors;
        }

        // How many of an axis' motors appear in mask.  Used to tell a ganged axis from a
        // single-motor one without assuming there are only two.
        static inline motor_t count_motors(MotorMask mask, axis_t axis) {
            motor_t count = 0;
            for (motor_t motor = 0; motor < MAX_MOTORS_PER_AXIS; ++motor) {
                if (motor_is_set(mask, axis, motor)) {
                    ++count;
                }
            }
            return count;
        }

        static axis_t _numberAxis;
        static Axis*  _axis[MAX_N_AXIS];

        // True when the corresponding _axis[] entry was created by Axes itself (a base-config
        // axis or a runtime gap placeholder) and must therefore be deleted by ~Axes.  Axes
        // spliced in from a CanModule are owned by that module's AxisSet, so their entries are
        // marked not-owned and ~Axes leaves them alone.
        static bool _axisOwned[MAX_N_AXIS];

        // Lower bound on the reported axis count, so senders that cache the status field width
        // are not disturbed when a module axis is loaded or unloaded.  Defaults to 3.
        static int32_t _minCount;

        // Some small helpers to find the axis index and axis motor number for a given motor. This
        // is helpful for some motors that need this info, as well as debug information.
        static axis_t  findAxisIndex(const MotorDrivers::MotorDriver* const motor);
        static motor_t findAxisMotor(const MotorDrivers::MotorDriver* const motor);

        static MotorMask hardLimitMask();

        inline bool hasHardLimits() const {
            for (axis_t axis = X_AXIS; axis < _numberAxis; ++axis) {
                auto a = _axis[axis];

                for (int motor = 0; motor < Axis::MAX_MOTORS_PER_AXIS; ++motor) {
                    auto m = a->_motors[motor];
                    if (m && m->_hardLimits) {
                        return true;
                    }
                }
            }
            return false;
        }

        static void init();

        // Zeroes and recomputes the accumulating masks (motorMask, homingMask,
        // Homing::direction_mask) from the live axis set.  Pure bookkeeping: it touches no
        // hardware, so it is safe to call after a CanModule splices axes in or out.
        static void rebuildMasks();

        // Recomputes _numberAxis from the live entries (honouring _minCount), fills any gap
        // below the live maximum with a placeholder Axis, drops placeholders that are no
        // longer needed, and rebuilds the masks.  Called after every splice.
        static void reconcile();

        // Splices a module-owned axis into (out of) the global table.  spliceIn asserts the
        // slot is free; spliceOut leaves the Axis object alive for its owning AxisSet.
        static void spliceIn(axis_t axis, Axis* a);
        static void spliceOut(axis_t axis);

        // These are used during homing cycles.
        // The return value is a bitmask of axes that can home
        static MotorMask set_homing_mode(AxisMask homing_mask, bool isHoming);

        static void set_disable(axis_t axis, bool disable);
        static void set_disable(bool disable);
        static void config_motors();

        static std::string maskToNames(AxisMask mask);

        static bool namesToMask(const char* names, AxisMask& mask);

        static std::string motorMaskToNames(MotorMask mask);

        // Configuration helpers:
        void group(Configuration::HandlerBase& handler) override;
        void afterParse() override;

        ~Axes();
    };
}
