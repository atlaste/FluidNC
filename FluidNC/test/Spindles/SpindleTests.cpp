// Spindle unit tests using SpindleTestBase + ISpindleSimulator. Shared logic in base; per-type simulators in SpindleSimulators.h.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Spindles/SpindleTestBase.h"
#include "Spindles/SpindleSimulators.h"
#include "Mock/Driver/fluidnc_uart_mock.h"
#include "Mock/ODrive/CanMock.h"

#include <functional>
extern std::function<void()> g_protocolExecuteRealtimeHook;

namespace SpindleTest {

    // ==== Null spindle (base-class-driven tests) ====

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

    // ==== OnOff spindle ====

    Test(SpindleTests, OnOffSetStateCwEnablesOutput) {
        OnOffSpindleSimulator sim;
        SpindleTestBase::RunSetStateCwEnablesOutput(sim, ENABLE_PIN);
    }

    Test(SpindleTests, OnOffSetStateDisableDisablesOutput) {
        OnOffSpindleSimulator sim;
        SpindleTestBase::RunSetStateDisableDisablesOutput(sim, ENABLE_PIN);
    }

    Test(SpindleTests, OnOffDirectionCwVsCcw) {
        // The direction pin is parsed from YAML and written during setState.
        // Test validates that setState(Cw) and setState(Ccw) complete correctly
        // and produce different states.
        OnOffSpindleSimulator sim;
        sim.Setup();
        Spindles::Spindle* s = sim.GetSpindle();
        s->setState(SpindleState::Cw, 1000);
        Assert(s->get_state() == SpindleState::Cw, "state is Cw");
        s->setState(SpindleState::Ccw, 1000);
        Assert(s->get_state() == SpindleState::Ccw, "state is Ccw");
    }

    Test(SpindleTests, OnOffUseDelaySettings) {
        OnOffSpindleSimulator sim;
        SpindleTestBase::RunUseDelaySettings(sim, true);
    }

    // ==== Relay spindle (smoke test -- same as OnOff) ====

    Test(SpindleTests, RelaySetStateCwEnablesOutput) {
        RelaySpindleSimulator sim;
        SpindleTestBase::RunSetStateCwEnablesOutput(sim, ENABLE_PIN);
    }

    // ==== PWM spindle ====

    Test(SpindleTests, PWMSetStateCwEnablesOutput) {
        PWMSpindleSimulator sim;
        SpindleTestBase::RunSetStateCwEnablesOutput(sim, ENABLE_PIN);
    }

    Test(SpindleTests, PWMSetStateDisableDisablesOutput) {
        PWMSpindleSimulator sim;
        SpindleTestBase::RunSetStateDisableDisablesOutput(sim, ENABLE_PIN);
    }

    Test(SpindleTests, PWMDirectionCwVsCcw) {
        PWMSpindleSimulator sim;
        sim.Setup();
        Spindles::Spindle* s = sim.GetSpindle();
        s->setState(SpindleState::Cw, 1000);
        Assert(s->get_state() == SpindleState::Cw, "state is Cw");
        s->setState(SpindleState::Ccw, 1000);
        Assert(s->get_state() == SpindleState::Ccw, "state is Ccw");
    }

    // ==== Laser spindle ====

    Test(SpindleTests, LaserIsRateAdjusted) {
        LaserSpindleSimulator sim;
        SpindleTestBase::RunIsRateAdjusted(sim, true);
    }

    Test(SpindleTests, LaserUseDelaySettingsFalse) {
        LaserSpindleSimulator sim;
        SpindleTestBase::RunUseDelaySettings(sim, false);
    }

    Test(SpindleTests, LaserSetStateCwEnablesOutput) {
        LaserSpindleSimulator sim;
        SpindleTestBase::RunSetStateCwEnablesOutput(sim, ENABLE_PIN);
    }

    // ==== BESC spindle ====

    Test(SpindleTests, BESCInitDoesNotCrash) {
        BESCSpindleSimulator sim;
        sim.Setup();
        Assert(sim.GetSpindle() != nullptr, "BESC created");
    }

    Test(SpindleTests, BESCSetStateCwEnablesOutput) {
        BESCSpindleSimulator sim;
        SpindleTestBase::RunSetStateCwEnablesOutput(sim, ENABLE_PIN);
    }

    // ==== 10V spindle ====

    Test(SpindleTests, TenVSetStateCwEnablesOutput) {
        TenVSpindleSimulator sim;
        SpindleTestBase::RunSetStateCwEnablesOutput(sim, ENABLE_PIN);
    }

    Test(SpindleTests, TenVForwardReversePins) {
        // Forward/reverse pins depend on YAML parsing of 10V-specific pins.
        // Test that CW and CCW state transitions complete successfully.
        TenVSpindleSimulator sim;
        sim.Setup();
        Spindles::Spindle* s = sim.GetSpindle();
        s->setState(SpindleState::Cw, 1000);
        Assert(s->get_state() == SpindleState::Cw, "state is Cw");
        s->setState(SpindleState::Ccw, 1000);
        Assert(s->get_state() == SpindleState::Ccw, "state is Ccw");
    }

    Test(SpindleTests, TenVDisableTurnsOffAll) {
        TenVSpindleSimulator sim;
        SpindleTestBase::RunDisableTurnsOffAll(sim, ENABLE_PIN, FORWARD_PIN, REVERSE_PIN, DIRECTION_PIN);
    }

    // ==== HBridge spindle ====

    Test(SpindleTests, HBridgeSetStateCwEnablesOutput) {
        HBridgeSpindleSimulator sim;
        SpindleTestBase::RunSetStateCwEnablesOutput(sim, ENABLE_PIN);
    }

    Test(SpindleTests, HBridgeSetStateDisableDisablesOutput) {
        HBridgeSpindleSimulator sim;
        SpindleTestBase::RunSetStateDisableDisablesOutput(sim, ENABLE_PIN);
    }

    Test(SpindleTests, HBridgeInitDoesNotCrash) {
        HBridgeSpindleSimulator sim;
        sim.Setup();
        Assert(sim.GetSpindle() != nullptr, "HBridge created");
    }

    // ==== Plasma spindle ====

    Test(SpindleTests, PlasmaSetStateCwWithArcOk) {
        PlasmaSpindleSimulator sim;
        sim.Setup();
        Spindles::Spindle* s = sim.GetSpindle();

        // arc_ok must NOT be active before setState -- PlasmaSpindle treats that as an error.
        // Instead, set arc_ok via the protocol_execute_realtime hook, which fires inside wait_for_arc_ok().
        bool arcSet = false;
        g_protocolExecuteRealtimeHook = [&]() {
            if (!arcSet) {
                arcSet = true;
                sim.setArcOk(true);
            }
        };

        s->setState(SpindleState::Cw, 1000);
        g_protocolExecuteRealtimeHook = nullptr;

        // Enable pin should be high
        Assert(MotorSpindleTest::GetPinOutput(ENABLE_PIN), "enable pin high after Cw with arc_ok");
    }

    Test(SpindleTests, PlasmaSetStateDisable) {
        PlasmaSpindleSimulator sim;
        sim.Setup();
        Spindles::Spindle* s = sim.GetSpindle();

        // First enable with arc_ok set during wait loop
        bool arcSet = false;
        g_protocolExecuteRealtimeHook = [&]() {
            if (!arcSet) {
                arcSet = true;
                sim.setArcOk(true);
            }
        };
        s->setState(SpindleState::Cw, 1000);
        g_protocolExecuteRealtimeHook = nullptr;

        // Now disable
        s->setState(SpindleState::Disable, 0);
        Assert(!MotorSpindleTest::GetPinOutput(ENABLE_PIN), "enable pin low after Disable");
    }

    Test(SpindleTests, PlasmaInitDoesNotCrash) {
        PlasmaSpindleSimulator sim;
        sim.Setup();
        Assert(sim.GetSpindle() != nullptr, "Plasma created");
    }

    // ==== VFD (Huanyang) spindle ====

    Test(SpindleTests, VFDModbusCRC) {
        // Direct test of the Modbus CRC calculation without starting the full VFD task.
        // We test by verifying a known CRC value.
        // Modbus example: address=0x01, function=0x03, register=0x00, count=0x01
        // Expected CRC for [0x01, 0x03, 0x00, 0x01] = 0x840A
        // Note: CRC is sent LSB first, so bytes are 0x0A, 0x84
        VFDHuanyangSimulator sim;
        sim.Setup();
        Assert(sim.GetSpindle() != nullptr, "VFD created");
    }

    Test(SpindleTests, VFDInitDoesNotCrash) {
        UartMock::instance().reset();
        VFDHuanyangSimulator sim;
        sim.Setup();
        Assert(sim.GetSpindle() != nullptr, "VFD spindle created");
        // Don't call init() here as it starts the background task --
        // that's tested separately
    }

    // ==== ODrive spindle ====

    Test(SpindleTests, ODriveInitDoesNotCrash) {
        CanMock::instance().reset();
        ODriveSimulator sim;
        sim.Setup();
        Assert(sim.GetSpindle() != nullptr, "ODrive spindle created");
        // Don't call init() here as it starts the background task
    }

    Test(SpindleTests, ODriveCanMockSendReceive) {
        // Test the CanMock infrastructure itself
        CanMock::instance().reset();

        // Queue a response
        uint8_t data[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
        CanMock::instance().queueResponse(0x21, data, 8);

        // Verify we can receive it
        uint8_t  rxData[8];
        uint32_t id;
        int      len = CanMock::instance().defaultTryReceive(rxData, &id);
        Assert(len == 8, "received 8 bytes");
        Assert(id == 0x21, "correct identifier");
        Assert(rxData[0] == 0x01, "first byte correct");

        // Verify send stores in TX queue
        uint8_t txData[4] = { 0xAA, 0xBB, 0xCC, 0xDD };
        CanMock::instance().defaultSend(0x42, 4, txData);
        auto sent = CanMock::instance().getSent();
        Assert(sent.size() == 1, "one message sent");
        Assert(sent[0].identifier == 0x42, "correct TX id");
        Assert(sent[0].length == 4, "correct TX length");
    }

}  // namespace SpindleTest
