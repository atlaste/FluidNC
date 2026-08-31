#include "Axes.h"

#include "Motors/MotorDriver.h"
#include "Motors/NullMotor.h"
#include "Config.h"
#include "MotionControl.h"
#include "Stepper.h"  // stepper_id_t
#include "Limit.h"
#include "MachineConfig.h"  // config->
#include <esp_attr.h>
#include <cstring>  // strcmp

// Pre-increment operator
axis_t& IRAM_ATTR operator++(axis_t& axis) {
    // Cast to underlying integer type, increment, then cast back
    axis = static_cast<axis_t>((static_cast<int>(axis) + 1));
    return axis;
}

// Post-increment operator
axis_t IRAM_ATTR operator++(axis_t& axis, int) {
    axis_t old_axis = axis;
    ++axis;
    return old_axis;
}

// Pre-decrement operator
axis_t& IRAM_ATTR operator--(axis_t& axis) {
    // Cast to underlying integer type, increment, then cast back
    axis = static_cast<axis_t>((static_cast<int>(axis) - 1));
    return axis;
}

// Post-deccrement operator
axis_t IRAM_ATTR operator--(axis_t& axis, int) {
    axis_t old_axis = axis;
    --axis;
    return old_axis;
}

namespace Machine {
    MotorMask Axes::posLimitMask = 0;
    MotorMask Axes::negLimitMask = 0;
    MotorMask Axes::limitMask    = 0;
    MotorMask Axes::motorMask    = 0;

    AxisMask Axes::homingMask = 0;

    bool Axes::disabled = false;

    const char* Axes::_axisNames[] = {
        "X", "Y", "Z", "A", "B", "C", "U", "V", "W",
    };
    axis_t Axes::axisNum(std::string_view name) {
        for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; axis++) {
            if (string_util::equal_ignore_case(name, axisName(axis))) {
                return axis;
            }
        }
        return INVALID_AXIS;
    }

    Pin Axes::_sharedStepperDisable;
    Pin Axes::_sharedStepperReset;

    uint32_t Axes::_homing_runs = 2;  // Number of Approach/Pulloff cycles

    axis_t Axes::_numberAxis = X_AXIS;

    Axis* Axes::_axis[MAX_N_AXIS] = { nullptr };

    bool Axes::_axisOwned[MAX_N_AXIS] = { false };

    int32_t Axes::_minCount = A_AXIS;  // three axes, matching the historical floor

    Axes::Axes() {}

    void Axes::init() {
        log_info("Axis count " << Axes::_numberAxis);

        if (_sharedStepperDisable.defined()) {
            _sharedStepperDisable.setAttr(Pin::Attr::Output);
            _sharedStepperDisable.report("Shared stepper disable");
        }

        if (_sharedStepperReset.defined()) {
            _sharedStepperReset.setAttr(Pin::Attr::Output | Pin::Attr::InitialOn);
            _sharedStepperReset.on();
            _sharedStepperReset.report("Shared stepper reset");
        }

        // certain motors need features to be turned on. Check them here
        for (axis_t axis = X_AXIS; axis < _numberAxis; axis++) {
            auto a = _axis[axis];
            if (a) {
                log_info("Axis " << axisName(axis) << " (" << limitsMinPosition(axis) << "," << limitsMaxPosition(axis) << ")");
                a->init();
                auto homing = a->_homing;
                if (homing && !homing->_positiveDirection) {
                    set_bitnum(Homing::direction_mask, axis);
                }
            }
        }

        // Everything present now is base configuration (or a boot-time gap placeholder), so
        // Axes owns it.  Module axes are marked not-owned by spliceIn() later.
        for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; ++axis) {
            _axisOwned[axis] = _axis[axis] != nullptr;
        }

        config_motors();
    }

    void Axes::rebuildMasks() {
        motorMask              = 0;
        homingMask             = 0;
        Homing::direction_mask = 0;

        for (axis_t axis = X_AXIS; axis < _numberAxis; ++axis) {
            auto a = _axis[axis];
            if (!a) {
                continue;
            }
            for (motor_t motor = 0; motor < Axis::MAX_MOTORS_PER_AXIS; ++motor) {
                auto m = a->_motors[motor];
                if (m && m->_driver && strcmp(m->_driver->name(), "null_motor") != 0) {
                    set_bits(motorMask, motor_mask(axis, motor));
                }
            }
            if (a->_homing) {
                if (a->_homing->_cycle >= 0) {
                    set_bitnum(homingMask, axis);
                }
                if (!a->_homing->_positiveDirection) {
                    set_bitnum(Homing::direction_mask, axis);
                }
            }
        }
    }

    void Axes::reconcile() {
        // Highest live index + 1, floored at _minCount for sender stability.
        axis_t highest = X_AXIS;
        for (axis_t axis = MAX_N_AXIS; axis > 0; --axis) {
            if (_axis[axis - 1] != nullptr) {
                highest = axis;
                break;
            }
        }
        axis_t wanted = highest;
        if (wanted < axis_t(_minCount)) {
            wanted = axis_t(_minCount);
        }

        // Fill gaps below the new count with owned placeholders; drop owned placeholders that
        // fell above it.  Never delete a module axis (owned == false): its AxisSet owns it.
        for (axis_t axis = X_AXIS; axis < wanted; ++axis) {
            if (_axis[axis] == nullptr) {
                _axis[axis]      = new Axis(axis);
                _axisOwned[axis] = true;
            }
        }
        for (axis_t axis = wanted; axis < MAX_N_AXIS; ++axis) {
            if (_axis[axis] != nullptr && _axisOwned[axis]) {
                delete _axis[axis];
                _axis[axis]      = nullptr;
                _axisOwned[axis] = false;
            }
        }

        _numberAxis = wanted;
        rebuildMasks();
    }

    void Axes::spliceIn(axis_t axis, Axis* a) {
        Assert(axis < MAX_N_AXIS, "Axis index out of range");
        // A placeholder may occupy the slot; the caller has already checked that no real axis
        // conflicts, so replace an owned placeholder rather than refusing.
        if (_axis[axis] != nullptr && _axisOwned[axis]) {
            delete _axis[axis];
            _axis[axis] = nullptr;
        }
        Assert(_axis[axis] == nullptr, "Axis %s is already occupied", axisName(axis));
        _axis[axis]      = a;
        _axisOwned[axis] = false;
    }

    void Axes::spliceOut(axis_t axis) {
        Assert(axis < MAX_N_AXIS, "Axis index out of range");
        // Only detach; the AxisSet that owns the object is responsible for its lifetime.
        if (!_axisOwned[axis]) {
            _axis[axis] = nullptr;
        }
    }

    void IRAM_ATTR Axes::set_disable(axis_t axis, bool disable) {
        for (motor_t motor = 0; motor < Axis::MAX_MOTORS_PER_AXIS; motor++) {
            auto m = _axis[axis]->_motors[motor];
            if (m) {
                m->_driver->set_disable(disable);
            }
        }
        if (disable) {  // any disable, !disable does not change anything here
            disabled = true;
        }
    }

    void IRAM_ATTR Axes::set_disable(bool disable) {
        for (axis_t axis = X_AXIS; axis < _numberAxis; axis++) {
            set_disable(axis, disable);
        }

        _sharedStepperDisable.synchronousWrite(disable);

        if (!disable && disabled) {
            disabled = false;
            if (Stepping::_disableDelayUsecs) {  // wait for the enable delay
                delay_us(Stepping::_disableDelayUsecs);
            }
        }
    }

    // Put the motors in the given axes into homing mode, returning a
    // mask of which motors can do homing.
    MotorMask Axes::set_homing_mode(AxisMask axisMask, bool isHoming) {
        MotorMask motorsCanHome = 0;

        for (axis_t axis = X_AXIS; axis < _numberAxis; axis++) {
            if (bitnum_is_true(axisMask, axis)) {
                auto a = _axis[axis];
                if (a != nullptr) {
                    for (motor_t motor = 0; motor < Axis::MAX_MOTORS_PER_AXIS; motor++) {
                        Stepping::unblock(axis, motor);
                        auto m = _axis[axis]->_motors[motor];
                        if (m) {
                            if (m->_driver->set_homing_mode(isHoming)) {
                                set_bits(motorsCanHome, motor_mask(axis, motor));
                            }
                        }
                    }
                }
            }
        }

        return motorsCanHome;
    }

    void Axes::config_motors() {
        for (axis_t axis = X_AXIS; axis < _numberAxis; ++axis) {
            _axis[axis]->config_motors();
        }
    }

    // Some small helpers to find the axis index and axis motor index for a given motor. This
    // is helpful for some motors that need this info, as well as debug information.
    axis_t Axes::findAxisIndex(const MotorDrivers::MotorDriver* const driver) {
        for (axis_t axis = X_AXIS; axis < _numberAxis; ++axis) {
            if (_axis[axis] != nullptr && _axis[axis]->hasMotor(driver)) {
                return axis;
            }
        }

        Assert(false, "Cannot find axis for motor driver");
        return INVALID_AXIS;
    }

    motor_t Axes::findAxisMotor(const MotorDrivers::MotorDriver* const driver) {
        for (axis_t axis = X_AXIS; axis < _numberAxis; ++axis) {
            if (_axis[axis] != nullptr && _axis[axis]->hasMotor(driver)) {
                for (motor_t motor = 0; motor < Axis::MAX_MOTORS_PER_AXIS; ++motor) {
                    auto m = _axis[axis]->_motors[motor];
                    if (m && m->_driver == driver) {
                        return motor;
                    }
                }
            }
        }

        Assert(false, "Cannot find axis for motor");
        return INVALID_MOTOR;
    }

    // Configuration helpers:

    void Axes::group(Configuration::HandlerBase& handler) {
        handler.item("shared_stepper_disable_pin", _sharedStepperDisable);
        handler.item("shared_stepper_reset_pin", _sharedStepperReset);
        handler.item("homing_runs", _homing_runs, 1, 5);
        handler.item("min_count", _minCount, 1, MAX_N_AXIS);

        // During the initial configuration parsing phase, _numberAxis is 0 so
        // we try for all the axes.  Subsequently we use the number of axes
        // that are actually present.
        axis_t n_axis = _numberAxis ? _numberAxis : MAX_N_AXIS;
        for (axis_t axis = X_AXIS; axis < n_axis; ++axis) {
            handler.section(_axisNames[axis], _axis[axis], axis);
        }
    }

    void Axes::afterParse() {
        // Find the last axis that was declared and set _numberAxis accordingly
        for (axis_t axis = MAX_N_AXIS; axis > 0; --axis) {
            if (_axis[axis - 1] != nullptr) {
                _numberAxis = axis;
                break;
            }
        }
        // Senders might assume a minimum number of axes in reports
        if (_numberAxis < axis_t(_minCount)) {
            _numberAxis = axis_t(_minCount);
        }

        for (axis_t axis = X_AXIS; axis < _numberAxis; ++axis) {
            if (_axis[axis] == nullptr) {
                _axis[axis] = new Axis(axis);
            }
        }
    }

    std::string Axes::maskToNames(AxisMask mask) {
        std::string retval("");
        auto        n_axis = _numberAxis;
        for (axis_t axis = X_AXIS; axis < n_axis; axis++) {
            if (bitnum_is_true(mask, axis)) {
                retval += axisName(axis);
            }
        }
        return retval;
    }
    std::string Axes::motorMaskToNames(MotorMask mask) {
        std::string retval("");
        for (motor_t motor = 0; motor < Axis::MAX_MOTORS_PER_AXIS; motor++) {
            for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; axis++) {
                if (motor_is_set(mask, axis, motor)) {
                    retval += " ";
                    retval += axisName(axis);
                    // Motor 0 is the bare axis letter, so "Z" "Z2" "Z3" "Z4" reads the way the
                    // motor0..motor3 config sections are numbered.
                    if (motor) {
                        retval += char('1' + motor);
                    }
                }
            }
        }
        return retval;
    }

    MotorMask Axes::hardLimitMask() {
        MotorMask mask = 0;
        for (axis_t axis = X_AXIS; axis < _numberAxis; ++axis) {
            auto a = _axis[axis];

            for (motor_t motor = 0; motor < Axis::MAX_MOTORS_PER_AXIS; ++motor) {
                auto m = a->_motors[motor];
                if (m && m->_hardLimits) {
                    set_bitnum(mask, axis);
                }
            }
        }
        return mask;
    }

    bool Axes::namesToMask(const char* names, AxisMask& mask) {
        bool       retval      = true;
        const auto lenNames    = strlen(names);
        char       axisName[2] = {};
        for (size_t i = 0; i < lenNames; i++) {
            axisName[0] = names[i];
            axis_t axis = axisNum(axisName);
            if (axis == INVALID_AXIS) {
                log_error("Invalid axis name " << axisName);
                retval = false;
            }
            set_bitnum(mask, axis);
        }

        return retval;
    }

    Axes::~Axes() {
        for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; ++axis) {
            // Module-owned axes belong to their CanModule's AxisSet; deleting them here would
            // double-free.  Only delete entries Axes created itself.
            if (_axis[axis] != nullptr && _axisOwned[axis]) {
                delete _axis[axis];
            }
            _axis[axis]      = nullptr;
            _axisOwned[axis] = false;
        }
    }
}
