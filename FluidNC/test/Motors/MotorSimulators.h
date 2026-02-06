// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
// Concrete motor simulators for unit tests. Each motor type provides one.

#pragma once

#include "Motors/MotorTestBase.h"
#include "Motors/MotorSpindleFixture.h"
#include "Motors/NullMotor.h"
#include "Machine/Axes.h"
#include "Machine/Axis.h"
#include "Configuration/Parser.h"
#include "Configuration/ParserHandler.h"
#include "Configuration/AfterParse.h"

namespace MotorTest {

    /** StandardStepper: config with gpio.1/2/3, disable pin at index 3, can home, is real. */
    class StandardStepperSimulator : public IMotorSimulator {
    public:
        void Setup() override {
            MotorSpindleTest::EnsureAxesWithStandardStepper();
            Assert(config != nullptr && config->_axes != nullptr, "config and axes");
            _axis = config->_axes->_axis[X_AXIS];
            Assert(_axis != nullptr && _axis->_motors[0] != nullptr, "X axis motor0");
        }

        MotorDrivers::MotorDriver* GetDriver() override { return _axis ? _axis->_motors[0]->_driver : nullptr; }
        Machine::Axis*             GetAxis() override { return _axis; }
        Machine::Axes*             GetAxes() override { return config->_axes; }
        void                       InitMotor() override {
            if (_axis && _axis->_motors[0])
                _axis->_motors[0]->init();
        }
        int         DisablePinIndex() const override { return MotorSpindleTest::DISABLE_PIN; }
        bool        ExpectedIsReal() const override { return true; }
        bool        ExpectedCanHome() const override { return true; }
        const char* DriverName() const override { return "standard_stepper"; }

    private:
        Machine::Axis* _axis = nullptr;
    };

    /** Nullmotor: no disable pin, not real, cannot home. Run last and use ClearAxes + explicit null_motor YAML. */
    class NullmotorInAxisSimulator : public IMotorSimulator {
    public:
        void Setup() override {
            MotorSpindleTest::ResetGPIO();
            MotorSpindleTest::ClearAxes();
            config->_axes              = new Machine::Axes();
            const char*           yaml = "axes:\n"
                                         "  X:\n"
                                         "    steps_per_mm: 80\n"
                                         "    max_rate_mm_per_min: 1000\n"
                                         "    acceleration_mm_per_sec2: 25\n"
                                         "    max_travel_mm: 100\n"
                                         "    motor0:\n"
                                         "      null_motor:\n";
            Configuration::Parser parser(yaml);
            parser.Tokenize();
            Configuration::ParserHandler handler(parser);
            handler.enterSection("axes", config->_axes);
            config->_axes->afterParse();
            Configuration::AfterParse ap;
            config->_axes->group(ap);
            _axis = config->_axes->_axis[X_AXIS];
            Assert(_axis != nullptr && _axis->_motors[0] != nullptr && _axis->_motors[0]->_driver != nullptr, "null_motor axis");
            Assert(strcmp(_axis->_motors[0]->_driver->name(), "null_motor") == 0, "driver is null_motor");
        }

        MotorDrivers::MotorDriver* GetDriver() override { return _axis && _axis->_motors[0] ? _axis->_motors[0]->_driver : nullptr; }
        Machine::Axis*             GetAxis() override { return _axis; }
        Machine::Axes*             GetAxes() override { return config->_axes; }
        void                       InitMotor() override {}
        int                        DisablePinIndex() const override { return -1; }
        bool                       ExpectedIsReal() const override { return false; }
        bool                       ExpectedCanHome() const override { return false; }
        const char*                DriverName() const override { return "null_motor"; }

    private:
        Machine::Axis* _axis = nullptr;
    };

    /** Nullmotor driver only (no config). For tests that don't need axes. */
    class NullmotorDriverSimulator : public IMotorSimulator {
    public:
        NullmotorDriverSimulator() : _driver("null_motor") {}
        void                       Setup() override {}
        MotorDrivers::MotorDriver* GetDriver() override { return &_driver; }
        Machine::Axis*             GetAxis() override { return nullptr; }
        Machine::Axes*             GetAxes() override { return nullptr; }
        void                       InitMotor() override {}
        int                        DisablePinIndex() const override { return -1; }
        bool                       ExpectedIsReal() const override { return false; }
        bool                       ExpectedCanHome() const override { return false; }
        const char*                DriverName() const override { return "null_motor"; }

    private:
        MotorDrivers::Nullmotor _driver;
    };

}  // namespace MotorTest
