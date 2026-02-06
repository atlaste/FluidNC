// Motor unit tests using MotorTestBase + IMotorSimulator. Shared logic in base; per-type simulators in MotorSimulators.h.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Motors/MotorTestBase.h"
#include "Motors/MotorSimulators.h"
#include "Motors/NullMotor.h"
#include "Machine/Axes.h"
#include "NutsBolts.h"
#include "Types.h"

namespace MotorTest {

    // ---- Nullmotor (driver-only; no axes) ----
    Test(MotorTests, NullmotorSetDisableNoOp) {
        NullmotorDriverSimulator sim;
        MotorTestBase::RunSetDisableDriver(sim);  // no pin, just no crash
    }

    Test(MotorTests, NullmotorSetHomingModeReturnsFalse) {
        NullmotorDriverSimulator sim;
        MotorTestBase::RunSetHomingModeDriver(sim);
    }

    Test(MotorTests, NullmotorIsRealFalse) {
        NullmotorDriverSimulator sim;
        MotorTestBase::RunIsReal(sim);
    }

    // ---- StandardStepper (via config) ----
    Test(MotorTests, StandardStepperSetDisableWritesPin) {
        StandardStepperSimulator sim;
        MotorTestBase::RunSetDisableDriver(sim);
    }

    Test(MotorTests, StandardStepperIsRealAndInitDisablePin) {
        StandardStepperSimulator sim;
        sim.Setup();
        Assert(sim.GetDriver()->isReal() == true, "StandardStepper is real");
        sim.InitMotor();
        sim.GetDriver()->set_disable(true);
        Assert(MotorSpindleTest::GetPinOutput(sim.DisablePinIndex()) == true, "after init disable pin writable");
    }

    Test(MotorTests, AxesSetDisablePerAxis) {
        StandardStepperSimulator sim;
        MotorTestBase::RunSetDisablePerAxis(sim);
    }

    Test(MotorTests, AxesSetDisableAll) {
        StandardStepperSimulator sim;
        MotorTestBase::RunSetDisableAll(sim);
    }

    Test(MotorTests, AxesSetHomingMode) {
        StandardStepperSimulator sim;
        MotorTestBase::RunSetHomingModeAxes(sim);
    }

    // ---- Nullmotor in axis (run last; uses ClearAxes + explicit null_motor YAML) ----
    Test(MotorTests, NullmotorInAxisSetHomingModeReturnsNoHome) {
        NullmotorInAxisSimulator sim;
        MotorTestBase::RunSetHomingModeAxes(sim);
    }

}  // namespace MotorTest
