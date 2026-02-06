// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
// Fixture and helpers for motor and spindle unit tests.

#pragma once

#include "TestFramework.h"
#include "Mock/TestLogging.h"
#include "Machine/Axes.h"
#include "Machine/MachineConfig.h"
#include "Configuration/Parser.h"
#include "Configuration/ParserHandler.h"
#include "Configuration/AfterParse.h"
#include "SoftwareGPIO.h"
#include "GCode.h"   // gc_state, parser_state_t
#include "System.h"  // sys

namespace MotorSpindleTest {

    // Pin indices used in minimal axes YAML (gpio.1, gpio.2, gpio.3)
    constexpr int STEP_PIN           = 1;
    constexpr int DIR_PIN            = 2;
    constexpr int DISABLE_PIN        = 3;
    constexpr int SPINDLE_ENABLE_PIN = 4;
    constexpr int SPINDLE_OUTPUT_PIN = 5;

    inline void ResetGPIO() {
        SoftwareGPIO::instance().reset(nullptr, false);

        // Reset global GCode/system state that spindle tests depend on.
        // SpindleState::Disable is NOT 0, so zero-initialised gc_state is wrong.
        gc_state.modal.spindle = SpindleState::Disable;
        sys.set_state(State::Idle);
    }

    /** Return last value written to pin (for disable/enable assertions). */
    inline bool GetPinOutput(int index) {
        return SoftwareGPIO::instance().getOutputValue(index);
    }

    /** Parse minimal axes YAML into config->_axes. Idempotent: if config already has axes, does nothing. */
    inline void EnsureAxesWithStandardStepper() {
        if (config == nullptr || config->_axes != nullptr) {
            return;
        }
        const char*           yaml = "axes:\n"
                                     "  X:\n"
                                     "    steps_per_mm: 80\n"
                                     "    max_rate_mm_per_min: 1000\n"
                                     "    acceleration_mm_per_sec2: 25\n"
                                     "    max_travel_mm: 100\n"
                                     "    motor0:\n"
                                     "      standard_stepper:\n"
                                     "        step_pin: gpio.1\n"
                                     "        direction_pin: gpio.2\n"
                                     "        disable_pin: gpio.3\n";
        Configuration::Parser parser(yaml);
        parser.Tokenize();  // position at first token "axes"
        Configuration::ParserHandler handler(parser);
        config->_axes = new Machine::Axes();
        handler.enterSection("axes", config->_axes);
        config->_axes->afterParse();
        Configuration::AfterParse afterParse;
        config->_axes->group(afterParse);
    }

    /** Clear config->_axes (call before re-parsing in a fresh test). */
    inline void ClearAxes() {
        if (config != nullptr && config->_axes != nullptr) {
            delete config->_axes;
            config->_axes = nullptr;
        }
    }
}
