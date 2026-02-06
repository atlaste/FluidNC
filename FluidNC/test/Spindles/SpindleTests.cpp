// Spindle unit tests using SpindleTestBase + ISpindleSimulator. Shared logic in base; per-type simulators in SpindleSimulators.h.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Spindles/SpindleTestBase.h"
#include "Spindles/SpindleSimulators.h"

namespace SpindleTest {

    // ---- Null spindle (base-class-driven tests) ----
    Test(SpindleTests, NullSpindleSetStateDisableSetsStateAndSpeedZero) {
        NullSpindleSimulator sim;
        SpindleTestBase::RunSetStateDisable(sim);
    }

    Test(SpindleTests, NullSpindleStopSetsDisable) {
        NullSpindleSimulator sim;
        SpindleTestBase::RunStopSetsDisable(sim);
    }

    Test(SpindleTests, NullSpindleNoPhysicalOutput) {
        NullSpindleSimulator sim;
        SpindleTestBase::RunNoPhysicalOutput(sim);
    }

    Test(SpindleTests, MapSpeedDisableWithZeroSpeedWithDisableReturnsZeroAndSetsSysSpeedZero) {
        NullSpindleSimulator sim;
        SpindleTestBase::RunMapSpeedDisable(sim);
    }

    Test(SpindleTests, OffOnAlarmStopPutsSpindleInDisable) {
        NullSpindleSimulator sim;
        SpindleTestBase::RunOffOnAlarmStop(sim);
    }

    // OnOff with pins (setState Disable, _disable_with_zero_speed, abort) require Pin::create from test;
    // when run in this executable Pin::create can throw. Add OnOffSpindleSimulator when fixture supports it.

}  // namespace SpindleTest
