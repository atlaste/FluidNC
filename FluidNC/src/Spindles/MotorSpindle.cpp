#include "MotorSpindle.h"

#include "../Machine/MachineConfig.h"

namespace Spindles {
    void MotorSpindle::init() {}

    // Used by Protocol.cpp to restore the state during a restart
    void MotorSpindle::setState(SpindleState state, uint32_t speed) {
        this->_current_speed = speed;
        this->_current_state = state;
    }

    void MotorSpindle::config_message() { log_info("Motor spindle for axis " << axis_); }
    void MotorSpindle::setSpeedfromISR(uint32_t dev_speed) { this->_current_speed = dev_speed; }

    void MotorSpindle::afterParse() {
        motor_ = nullptr;

        if (axis_.length() == 1) {
            auto a = config->_axes;

            for (int axis = 0; axis < a->_numberAxis; ++axis) {
                if (a->axisName(axis) == this->axis_[0]) {
                    if (!a->_axis[axis]->hasDualMotor()) {
                        motor_ = a->_axis[axis]->_motors[0];
                    }
                }
            }
        }
    }

    // Virtual base classes require a virtual destructor.
    MotorSpindle::~MotorSpindle() {}
}
