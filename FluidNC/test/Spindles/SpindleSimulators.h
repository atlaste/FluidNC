// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
// Concrete spindle simulators for unit tests.

#pragma once

#include "Spindles/SpindleTestBase.h"
#include "Spindles/NullSpindle.h"
#include "Spindles/OnOffSpindle.h"
#include "Spindles/RelaySpindle.h"
#include "Spindles/PWMSpindle.h"
#include "Spindles/LaserSpindle.h"
#include "Spindles/BESCSpindle.h"
#include "Spindles/10vSpindle.h"
#include "Spindles/HBridgeSpindle.h"
#include "Spindles/PlasmaSpindle.h"
#include "Spindles/VFDSpindle.h"
#include "Spindles/VFD/HuanyangProtocol.h"
#include "Spindles/ODriveSpindle.h"
#include "Motors/MotorSpindleFixture.h"
#include "Configuration/Parser.h"
#include "Configuration/ParserHandler.h"
#include "Configuration/AfterParse.h"

namespace SpindleTest {

    // Pin indices used for spindle tests
    constexpr int ENABLE_PIN     = 4;
    constexpr int OUTPUT_PIN     = 5;
    constexpr int DIRECTION_PIN  = 6;
    constexpr int FORWARD_PIN    = 7;
    constexpr int REVERSE_PIN    = 8;
    constexpr int ARC_OK_PIN     = 9;
    // HBridge uses OUTPUT_PIN for CW, DIRECTION_PIN for CCW

    // Helper: parse a YAML snippet into any Configurable via its group() handler
    inline void ParseYaml(const char* yaml, const char* sectionName, Configuration::Configurable* target) {
        Configuration::Parser parser(yaml);
        parser.Tokenize();
        Configuration::ParserHandler handler(parser);
        handler.enterSection(sectionName, target);
        target->afterParse();
    }

    // Convenience overload for spindles
    inline void ParseSpindleYaml(const char* yaml, const char* sectionName, Spindles::Spindle* spindle) {
        ParseYaml(yaml, sectionName, spindle);
    }

    // ---- Null spindle ----

    class NullSpindleSimulator : public ISpindleSimulator {
    public:
        NullSpindleSimulator() : _spindle("NoSpindle") {}
        void               Setup() override {}
        Spindles::Spindle* GetSpindle() override { return &_spindle; }
        bool               HasPhysicalOutput() const override { return false; }

    private:
        Spindles::Null _spindle;
    };

    // ---- OnOff spindle ----

    class OnOffSpindleSimulator : public ISpindleSimulator {
    public:
        OnOffSpindleSimulator() : _spindle("OnOff") {}

        void Setup() override {
            MotorSpindleTest::ResetGPIO();
            const char* yaml = "spindle:\n"
                               "  output_pin: gpio.5\n"
                               "  enable_pin: gpio.4\n"
                               "  direction_pin: gpio.6\n";
            ParseSpindleYaml(yaml, "spindle", &_spindle);
            _spindle.init();
        }

        Spindles::Spindle* GetSpindle() override { return &_spindle; }
        bool               HasPhysicalOutput() const override { return true; }

    private:
        Spindles::OnOff _spindle;
    };

    // ---- Relay spindle (empty subclass of OnOff) ----

    class RelaySpindleSimulator : public ISpindleSimulator {
    public:
        RelaySpindleSimulator() : _spindle("Relay") {}

        void Setup() override {
            MotorSpindleTest::ResetGPIO();
            const char* yaml = "spindle:\n"
                               "  output_pin: gpio.5\n"
                               "  enable_pin: gpio.4\n"
                               "  direction_pin: gpio.6\n";
            ParseSpindleYaml(yaml, "spindle", &_spindle);
            _spindle.init();
        }

        Spindles::Spindle* GetSpindle() override { return &_spindle; }
        bool               HasPhysicalOutput() const override { return true; }

    private:
        Spindles::Relay _spindle;
    };

    // ---- PWM spindle ----

    class PWMSpindleSimulator : public ISpindleSimulator {
    public:
        PWMSpindleSimulator() : _spindle("PWM") {}

        void Setup() override {
            MotorSpindleTest::ResetGPIO();
            const char* yaml = "spindle:\n"
                               "  pwm_hz: 5000\n"
                               "  output_pin: gpio.5\n"
                               "  enable_pin: gpio.4\n"
                               "  direction_pin: gpio.6\n";
            ParseSpindleYaml(yaml, "spindle", &_spindle);
            _spindle.init();
        }

        Spindles::Spindle* GetSpindle() override { return &_spindle; }
        bool               HasPhysicalOutput() const override { return true; }

    private:
        Spindles::PWM _spindle;
    };

    // ---- Laser spindle ----

    class LaserSpindleSimulator : public ISpindleSimulator {
    public:
        LaserSpindleSimulator() : _spindle("Laser") {}

        void Setup() override {
            MotorSpindleTest::ResetGPIO();
            const char* yaml = "spindle:\n"
                               "  pwm_hz: 5000\n"
                               "  output_pin: gpio.5\n"
                               "  enable_pin: gpio.4\n";
            ParseSpindleYaml(yaml, "spindle", &_spindle);
            _spindle.init();
        }

        Spindles::Spindle* GetSpindle() override { return &_spindle; }
        bool               HasPhysicalOutput() const override { return true; }

    private:
        Spindles::Laser _spindle;
    };

    // ---- BESC spindle ----

    class BESCSpindleSimulator : public ISpindleSimulator {
    public:
        BESCSpindleSimulator() : _spindle("BESC") {}

        void Setup() override {
            MotorSpindleTest::ResetGPIO();
            const char* yaml = "spindle:\n"
                               "  pwm_hz: 50\n"
                               "  output_pin: gpio.5\n"
                               "  enable_pin: gpio.4\n"
                               "  direction_pin: gpio.6\n";
            ParseSpindleYaml(yaml, "spindle", &_spindle);
            _spindle.init();
        }

        Spindles::Spindle* GetSpindle() override { return &_spindle; }
        bool               HasPhysicalOutput() const override { return true; }

    private:
        Spindles::BESC _spindle;
    };

    // ---- 10V spindle ----

    class TenVSpindleSimulator : public ISpindleSimulator {
    public:
        TenVSpindleSimulator() : _spindle("10V") {}

        void Setup() override {
            MotorSpindleTest::ResetGPIO();
            const char* yaml = "spindle:\n"
                               "  pwm_hz: 5000\n"
                               "  output_pin: gpio.5\n"
                               "  enable_pin: gpio.4\n"
                               "  direction_pin: gpio.6\n"
                               "  forward_pin: gpio.7\n"
                               "  reverse_pin: gpio.8\n";
            ParseSpindleYaml(yaml, "spindle", &_spindle);
            _spindle.init();
        }

        Spindles::Spindle* GetSpindle() override { return &_spindle; }
        bool               HasPhysicalOutput() const override { return true; }

    private:
        Spindles::_10v _spindle;
    };

    // ---- HBridge spindle ----

    class HBridgeSpindleSimulator : public ISpindleSimulator {
    public:
        HBridgeSpindleSimulator() : _spindle("HBridge") {}

        void Setup() override {
            MotorSpindleTest::ResetGPIO();
            const char* yaml = "spindle:\n"
                               "  pwm_hz: 5000\n"
                               "  output_cw_pin: gpio.5\n"
                               "  output_ccw_pin: gpio.6\n"
                               "  enable_pin: gpio.4\n";
            ParseSpindleYaml(yaml, "spindle", &_spindle);
            _spindle.init();
        }

        Spindles::Spindle* GetSpindle() override { return &_spindle; }
        bool               HasPhysicalOutput() const override { return true; }

    private:
        Spindles::HBridge _spindle;
    };

    // ---- Plasma spindle ----

    class PlasmaSpindleSimulator : public ISpindleSimulator {
    public:
        PlasmaSpindleSimulator() : _spindle("PlasmaSpindle") {}

        void Setup() override {
            MotorSpindleTest::ResetGPIO();
            const char* yaml = "spindle:\n"
                               "  enable_pin: gpio.4\n"
                               "  arc_ok_pin: gpio.9\n"
                               "  arc_wait_ms: 100\n";
            ParseSpindleYaml(yaml, "spindle", &_spindle);
            _spindle.init();
        }

        Spindles::Spindle* GetSpindle() override { return &_spindle; }
        bool               HasPhysicalOutput() const override { return true; }

        // Set the arc_ok signal (simulates the arc being established)
        void setArcOk(bool state) {
            SoftwareGPIO::instance().setPadValue(ARC_OK_PIN, state);
        }

    private:
        Spindles::PlasmaSpindle _spindle;
    };

    // ---- VFD (Huanyang) spindle ----
    // Uses UartMock for UART communication. The VFD task runs on a real thread.

    // Test subclass to expose protected _uart for test setup
    class TestableVFDSpindle : public Spindles::VFDSpindle {
    public:
        TestableVFDSpindle(const char* name, Spindles::VFD::VFDProtocol* detail) : VFDSpindle(name, detail) {}
        void setTestUart(Uart* uart) { _uart = uart; }
    };

    class VFDHuanyangSimulator : public ISpindleSimulator {
    public:
        VFDHuanyangSimulator() : _protocol(), _spindle("Huanyang", &_protocol) {}

        void Setup() override {
            MotorSpindleTest::ResetGPIO();
            // VFD spindle needs a Uart with TX/RX pins configured so that
            // Uart::begin() can call getNative() without failing.
            if (!_uart) {
                _uart = new Uart(0);
                // Configure the Uart's TX/RX pins via YAML parsing
                const char* uartYaml = "uart:\n"
                                       "  txd_pin: gpio.16\n"
                                       "  rxd_pin: gpio.17\n"
                                       "  baud: 9600\n";
                ParseYaml(uartYaml, "uart", _uart);
            }
            _spindle.setTestUart(_uart);

            // Reset the shutdown flag from any previous test
            Spindles::VFD::VFDProtocol::resetShutdown();
        }

        Spindles::Spindle* GetSpindle() override { return &_spindle; }
        bool               HasPhysicalOutput() const override { return false; }

        void Teardown() {
            Spindles::VFD::VFDProtocol::requestShutdown();
            // Give the task time to exit
            delay_ms(50);
        }

        ~VFDHuanyangSimulator() {
            Teardown();
            delete _uart;
            _uart = nullptr;
        }

    private:
        Spindles::VFD::HuanyangProtocol _protocol;
        TestableVFDSpindle              _spindle;
        Uart*                           _uart = nullptr;
    };

    // ---- ODrive spindle ----
    // Uses CanMock for CAN communication. The ODrive task runs on a real thread.

    class ODriveSimulator : public ISpindleSimulator {
    public:
        ODriveSimulator() : _spindle("odrive") {}

        void Setup() override {
            MotorSpindleTest::ResetGPIO();

            // ODrive needs CAN TX/RX pins configured
            const char* yaml = "spindle:\n"
                               "  can_tx: gpio.5\n"
                               "  can_rx: gpio.6\n"
                               "  odrive_node_id: 1\n"
                               "  max_speed: 4000\n";
            ParseSpindleYaml(yaml, "spindle", &_spindle);

            // Reset the shutdown flag from any previous test
            Spindles::ODriveSpindle::resetShutdown();
        }

        Spindles::Spindle* GetSpindle() override { return &_spindle; }
        bool               HasPhysicalOutput() const override { return false; }

        void Teardown() {
            Spindles::ODriveSpindle::requestShutdown();
            delay_ms(50);
        }

        ~ODriveSimulator() {
            Teardown();
        }

    private:
        Spindles::ODriveSpindle _spindle;
    };

}  // namespace SpindleTest
