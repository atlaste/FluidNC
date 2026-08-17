// Copyright (c) 2021 -  Stefan de Bruijn
// Copyright (c) 2021 -  Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"
// #include "Axes.h"
#include "Motor.h"
#include "Homing.h"

namespace MotorDrivers {
    class MotorDriver;
}

namespace Machine {
    class Axis : public Configuration::Configurable {
        axis_t  _axis;
        motor_t motorsWithSwitches();

    public:
        Axis(axis_t currentAxis) : _axis(currentAxis) {
            for (motor_t motor = 0; motor < MAX_MOTORS_PER_AXIS; ++motor) {
                _motors[motor] = nullptr;
            }
        }

        static const int MAX_MOTORS_PER_AXIS = ::MAX_MOTORS_PER_AXIS;

        Motor*  _motors[MAX_MOTORS_PER_AXIS];
        Homing* _homing = nullptr;

        float _stepsPerMm   = 80.0f;
        float _maxRate      = 1000.0f;
        float _acceleration = 25.0f;
        float _maxTravel    = 1000.0f;
        bool  _softLimits   = false;

        // Configuration system helpers:
        void group(Configuration::HandlerBase& handler) override;
        void afterParse() override;

        // Checks if a motor matches this axis:
        bool hasMotor(const MotorDrivers::MotorDriver* const driver) const;

        // Number of real (non-null) motors, so 4 for a Voron-style gantry Z.
        motor_t motorCount();
        bool    isGanged() { return motorCount() > 1; }

        // Pulloff distances across the axis' motors.  commonPulloff() is the distance every
        // motor can pull off together; maxPulloff() is the furthest any single motor wants.
        float commonPulloff();
        float maxPulloff();
        float extraPulloff() { return maxPulloff() - commonPulloff(); }

        void init();
        void config_motors();

        ~Axis();
    };
}
