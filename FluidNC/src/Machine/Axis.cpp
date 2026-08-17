#include "Axes.h"
#include "Axis.h"
#include "MachineConfig.h"  // config

#include <cstring>

namespace Machine {
    void Axis::group(Configuration::HandlerBase& handler) {
        handler.item("steps_per_mm", _stepsPerMm, 0.001, 100000.0);
        handler.item("max_rate_mm_per_min", _maxRate, 0.001, 250000.0);
        handler.item("acceleration_mm_per_sec2", _acceleration, 0.001, 100000.0);
        handler.item("max_travel_mm", _maxTravel, 0.1, 10000000.0);
        handler.item("soft_limits", _softLimits);
        handler.section("homing", _homing);

        char tmp[7];
        tmp[0] = 0;
        strcat(tmp, "motor");

        static_assert(MAX_MOTORS_PER_AXIS <= 10, "Section names assume a single motor digit");
        for (size_t i = 0; i < MAX_MOTORS_PER_AXIS; ++i) {
            tmp[5] = char(i + '0');
            tmp[6] = '\0';
            handler.section(tmp, _motors[i], _axis, i);
        }
    }

    void Axis::afterParse() {
        if (_motors[0] == nullptr) {
            _motors[0] = new Machine::Motor(_axis, 0);
        }
    }

    void Axis::init() {
        uint32_t stepRate = uint32_t(_stepsPerMm * _maxRate / 60.0);
        auto     maxRate  = Stepping::maxPulsesPerSec();
        Assert(stepRate <= maxRate, "Stepping rate %d steps/sec exceeds the maximum rate %d", stepRate, maxRate);

        for (size_t i = 0; i < Axis::MAX_MOTORS_PER_AXIS; i++) {
            auto m = _motors[i];
            if (m) {
                log_info("  Motor" << i);
                m->init();
            }
        }
        if (_homing && _homing->_cycle >= 0) {
            _homing->init();
            set_bitnum(Axes::homingMask, _axis);
        }

        // Motors are addressed by index, so a gap would leave a real motor unreachable by the
        // stepping code that walks 0..count-1.
        for (motor_t motor = 1; motor < MAX_MOTORS_PER_AXIS; ++motor) {
            if (_motors[motor] && _motors[motor]->isReal() && !(_motors[motor - 1] && _motors[motor - 1]->isReal())) {
                log_config_error("motor" << int(motor) << " defined without motor" << int(motor - 1));
            }
        }

        // If ganged and only one motor has switches, this is the configuration for a POG style
        // squaring.  The switch should report as being on every motor of the axis.
        if (isGanged() && (motorsWithSwitches() == 1)) {
            for (motor_t motor = 0; motor < MAX_MOTORS_PER_AXIS; ++motor) {
                if (_motors[motor]) {
                    _motors[motor]->makeDualSwitches();
                }
            }
        }

        // Pulloff2 is a single differential phase, so it can only reconcile two distinct
        // pulloff distances.  Three or more would need a phase each, and silently leaving the
        // middle motors at the common distance would quietly skew the gantry.
        if (isGanged()) {
            float distinct[MAX_MOTORS_PER_AXIS];
            int   nDistinct = 0;
            for (motor_t motor = 0; motor < MAX_MOTORS_PER_AXIS; ++motor) {
                auto m = _motors[motor];
                if (!m || !m->isReal()) {
                    continue;
                }
                bool seen = false;
                for (int i = 0; i < nDistinct; ++i) {
                    if (distinct[i] == m->_pulloff) {
                        seen = true;
                        break;
                    }
                }
                if (!seen) {
                    distinct[nDistinct++] = m->_pulloff;
                }
            }
            if (nDistinct > 2) {
                log_config_error("Axis " << Axes::axisName(_axis) << " has " << nDistinct
                                         << " different pulloff_mm values; at most 2 are supported");
            }
        }
    }

    void Axis::config_motors() {
        for (motor_t motor = 0; motor < Axis::MAX_MOTORS_PER_AXIS; ++motor) {
            auto mot = _motors[motor];
            if (mot)
                mot->config_motor();
        }
    }

    // Checks if a motor matches this axis:
    bool Axis::hasMotor(const MotorDrivers::MotorDriver* const driver) const {
        for (size_t i = 0; i < MAX_MOTORS_PER_AXIS; i++) {
            auto m = _motors[i];
            if (m && m->_driver == driver) {
                return true;
            }
        }
        return false;
    }

    // How many real motors does this axis drive?
    motor_t Axis::motorCount() {
        motor_t count = 0;
        for (motor_t motor = 0; motor < MAX_MOTORS_PER_AXIS; ++motor) {
            auto m = _motors[motor];
            if (m && m->isReal()) {
                ++count;
            }
        }
        return count;
    }

    // How many motors have switches defined?
    motor_t Axis::motorsWithSwitches() {
        motor_t count = 0;
        for (motor_t i = 0; i < MAX_MOTORS_PER_AXIS; i++) {
            auto m = _motors[i];
            if (m && m->hasSwitches()) {
                count++;
            }
        }
        return count;
    }

    // The distance every motor of this axis can pull off together.
    float Axis::commonPulloff() {
        float pulloff = _motors[0]->_pulloff;
        for (motor_t motor = 1; motor < MAX_MOTORS_PER_AXIS; ++motor) {
            auto m = _motors[motor];
            if (m && m->isReal()) {
                pulloff = std::min(pulloff, m->_pulloff);
            }
        }
        return pulloff;
    }

    // The furthest any single motor of this axis wants to pull off.
    float Axis::maxPulloff() {
        float pulloff = _motors[0]->_pulloff;
        for (motor_t motor = 1; motor < MAX_MOTORS_PER_AXIS; ++motor) {
            auto m = _motors[motor];
            if (m && m->isReal()) {
                pulloff = std::max(pulloff, m->_pulloff);
            }
        }
        return pulloff;
    }

    Axis::~Axis() {
        for (size_t i = 0; i < MAX_MOTORS_PER_AXIS; i++) {
            if (_motors[i]) {
                delete _motors[i];
            }
        }
    }
}
