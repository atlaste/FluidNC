// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
// Base class and simulator interface for spindle unit tests. Shared test logic lives here;
// concrete tests implement the simulator for each spindle type (Null, OnOff, etc.).

#pragma once

#include "TestFramework.h"
#include "Motors/MotorSpindleFixture.h"
#include "Spindles/Spindle.h"
#include "SpindleDatatypes.h"
#include "System.h"
#include <cstring>

namespace SpindleTest {

    /** Simulator (test driver) for a spindle type. Provides the spindle under test. */
    struct ISpindleSimulator {
        virtual ~ISpindleSimulator() = default;

        /** One-time setup: create spindle, init if needed. Call from test. */
        virtual void Setup() = 0;

        /** Spindle under test (must outlive use in test). */
        virtual Spindles::Spindle* GetSpindle() = 0;

        /** True if this spindle drives physical pins (for pin assertions). */
        virtual bool HasPhysicalOutput() const { return false; }
    };

    /** Shared spindle test logic. Run these from concrete tests that provide a simulator. */
    struct SpindleTestBase {
        static void RunSetStateDisable(ISpindleSimulator& sim) {
            sim.Setup();
            Spindles::Spindle* s = sim.GetSpindle();
            Assert(s != nullptr, "spindle");
            s->setState(SpindleState::Disable, 0);
            Assert(s->get_state() == SpindleState::Disable, "state is Disable");
            Assert(sys.spindle_speed() == 0, "sys spindle_speed 0");
        }

        static void RunStopSetsDisable(ISpindleSimulator& sim) {
            sim.Setup();
            Spindles::Spindle* s = sim.GetSpindle();
            s->setState(SpindleState::Cw, 1000);
            s->stop();
            Assert(s->get_state() == SpindleState::Disable, "stop -> Disable");
            Assert(sys.spindle_speed() == 0, "speed 0");
        }

        static void RunNoPhysicalOutput(ISpindleSimulator& sim) {
            MotorSpindleTest::ResetGPIO();
            sim.Setup();
            Spindles::Spindle* s = sim.GetSpindle();
            s->init();
            s->setState(SpindleState::Cw, 1000);
            s->stop();
            Assert(s->get_state() == SpindleState::Disable, "state Disable");
        }

        static void RunMapSpeedDisable(ISpindleSimulator& sim) {
            sim.Setup();
            Spindles::Spindle* s = sim.GetSpindle();
            s->linearSpeeds(1000, 100.0f);
            s->setupSpeeds(1000);
            sys.set_spindle_speed(500);
            uint32_t dev = s->mapSpeed(SpindleState::Disable, 0);
            Assert(sys.spindle_speed() == 0, "mapSpeed Disable sets sys speed 0");
            Assert(dev == 0 || dev == s->offSpeed(), "returns 0 or offSpeed for Disable");
        }

        static void RunOffOnAlarmStop(ISpindleSimulator& sim) {
            sim.Setup();
            Spindles::Spindle* s = sim.GetSpindle();
            s->_off_on_alarm     = true;
            s->setState(SpindleState::Cw, 1000);
            s->stop();
            Assert(s->get_state() == SpindleState::Disable, "off_on_alarm stop -> Disable");
            Assert(sys.spindle_speed() == 0, "speed 0");
        }
    };

}  // namespace SpindleTest
