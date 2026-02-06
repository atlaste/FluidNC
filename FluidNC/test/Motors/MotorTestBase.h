// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
// Base class and simulator interface for motor unit tests. Shared test logic lives here;
// concrete tests implement the simulator for each motor type (stepper, null, rc_servo, etc.).

#pragma once

#include "TestFramework.h"
#include "Motors/MotorSpindleFixture.h"
#include "Motors/MotorDriver.h"
#include "Machine/Axes.h"
#include "Machine/Axis.h"
#include "NutsBolts.h"
#include "Types.h"
#include <cstring>

namespace MotorTest {

    /** Simulator (test driver) for a motor type. Provides the driver under test and expected pin/behavior. */
    struct IMotorSimulator {
        virtual ~IMotorSimulator() = default;

        /** One-time setup: load config, create axes, ensure driver is available. Call from test SetUp. */
        virtual void Setup() = 0;

        /** Driver under test (must outlive use in test). */
        virtual MotorDrivers::MotorDriver* GetDriver() = 0;

        /** Axis for this motor (for Axes::set_disable(axis, ...)). */
        virtual Machine::Axis* GetAxis() = 0;

        /** Axes instance (for set_disable(all), set_homing_mode). */
        virtual Machine::Axes* GetAxes() = 0;

        /** Call motor init so disable pin is Output before writes. No-op if no pin. */
        virtual void InitMotor() = 0;

        /** -1 if no disable pin, else gpio index for asserting disable pin value. */
        virtual int DisablePinIndex() const = 0;

        virtual bool ExpectedIsReal() const = 0;
        /** Expected return from set_homing_mode(true): can this motor home? */
        virtual bool ExpectedCanHome() const = 0;
        /** Expected driver name for assertions. */
        virtual const char* DriverName() const = 0;
    };

    /** Shared motor test logic. Run these from concrete test fixtures that provide a simulator. */
    struct MotorTestBase {
        static void RunSetDisableDriver(IMotorSimulator& sim) {
            MotorSpindleTest::ResetGPIO();
            sim.Setup();
            MotorDrivers::MotorDriver* drv = sim.GetDriver();
            Assert(drv != nullptr, "driver");
            Assert(strcmp(drv->name(), sim.DriverName()) == 0, "driver name");
            sim.InitMotor();
            drv->set_disable(true);
            if (sim.DisablePinIndex() >= 0)
                Assert(MotorSpindleTest::GetPinOutput(sim.DisablePinIndex()) == true, "disable true -> pin high");
            drv->set_disable(false);
            if (sim.DisablePinIndex() >= 0)
                Assert(MotorSpindleTest::GetPinOutput(sim.DisablePinIndex()) == false, "disable false -> pin low");
        }

        static void RunIsReal(IMotorSimulator& sim) {
            sim.Setup();
            Assert(sim.GetDriver()->isReal() == sim.ExpectedIsReal(), "isReal");
        }

        static void RunSetHomingModeDriver(IMotorSimulator& sim) {
            sim.Setup();
            MotorDrivers::MotorDriver* drv = sim.GetDriver();
            Assert(drv->set_homing_mode(true) == sim.ExpectedCanHome(), "set_homing_mode(true)");
            drv->set_homing_mode(false);
        }

    static void RunSetDisablePerAxis(IMotorSimulator& sim) {
        MotorSpindleTest::ResetGPIO();
        sim.Setup();
        sim.InitMotor();
        Machine::Axes* axes = sim.GetAxes();
        Assert(axes != nullptr, "axes");
        // Sync driver's _lastWrittenValue with freshly-reset GPIO state
        sim.GetDriver()->set_disable(false);
        axes->set_disable(X_AXIS, true);
        if (sim.DisablePinIndex() >= 0)
            Assert(MotorSpindleTest::GetPinOutput(sim.DisablePinIndex()) == true, "set_disable(X, true)");
        axes->set_disable(X_AXIS, false);
        if (sim.DisablePinIndex() >= 0)
            Assert(MotorSpindleTest::GetPinOutput(sim.DisablePinIndex()) == false, "set_disable(X, false)");
    }

    static void RunSetDisableAll(IMotorSimulator& sim) {
        MotorSpindleTest::ResetGPIO();
        sim.Setup();
        sim.InitMotor();
        Machine::Axes* axes = sim.GetAxes();
        // Sync driver's _lastWrittenValue with freshly-reset GPIO state
        sim.GetDriver()->set_disable(false);
        axes->set_disable(true);
        if (sim.DisablePinIndex() >= 0)
            Assert(MotorSpindleTest::GetPinOutput(sim.DisablePinIndex()) == true, "set_disable(true) all");
        axes->set_disable(false);
        if (sim.DisablePinIndex() >= 0)
            Assert(MotorSpindleTest::GetPinOutput(sim.DisablePinIndex()) == false, "set_disable(false) all");
    }

        static void RunSetHomingModeAxes(IMotorSimulator& sim) {
            sim.Setup();
            Machine::Axes* axes     = sim.GetAxes();
            AxisMask       mask     = 1u << X_AXIS;
            MotorMask      canHome  = axes->set_homing_mode(mask, true);
            MotorMask      expected = sim.ExpectedCanHome() ? (1u << X_AXIS) : 0u;
            Assert(canHome == expected, "set_homing_mode axes");
            axes->set_homing_mode(mask, false);
        }
    };

}  // namespace MotorTest
