// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Extenders.h"
#include "PinExtenderDriver.h"
#include "Configuration/Configurable.h"
#include "Machine/EventPin.h"
#include "Machine/MachineConfig.h"
#include "Machine/I2CBus.h"
#include "Platform.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <bitset>

namespace Extenders {
    class I2CPinExtenderBase;

    // The extender's interrupt line only reports that *something* on the device changed,
    // never which pin.  It therefore exists to shorten the monitor task's wait rather than
    // to identify a pin, and the task still reads the input registers to find the change.
    class ExtenderInterruptPin : public InputPin {
        I2CPinExtenderBase* _container = nullptr;

    public:
        ExtenderInterruptPin() : InputPin("extender interrupt") {}

        void attach(I2CPinExtenderBase* container) { _container = container; }

        void trigger(bool active) override;
    };

    // Pin extenders...
    //
    // The PCA9539 is identical to the PCA9555 in terms of API. It provides 2 address
    // pins, so a maximum of 4 possible values. Per PCA, there are 16 I/O ports in 2
    // separate registers, so that's a total of 16*4 = 64 values.
    // Datasheet: https://www.ti.com/lit/ds/symlink/pca9539.pdf
    // Speed: 400 kHz
    //
    // The PCA8574 is quite similar as well, but only has 8 bits per device, so a single
    // register. It has 3 address pins, so 8 possible values. 8*8=64 bits.
    // Datasheet: https://www.nxp.com/docs/en/data-sheet/PCA8574_PCA8574A.pdf
    // Speed: 400 kHz
    //
    // Input pins are serviced by a monitor task that reads the input registers, compares
    // them against the previous reading and turns each changed bit into a pin event.  An
    // optional per-device 'interruptN' line wakes that task early; without one the task
    // falls back on its poll interval, which is the only option on boards that do not
    // route the interrupt or whose interrupt GPIO is unavailable.
    //
    // NOTE: The data sheet explains that interrupts can be chained. If that is the case, the
    // interrupt will have the effect that ALL PCA's in the chain have to be queried. Needless
    // to say, this is usually a bad idea, because things like endstops become much slower
    // as a result. For now, I just felt like not supporting it.
    //
    // The MCP23017 has two interrupt lines, one for register A and register B. Apart from
    // that it appears to be quite similar as well. It has 3 address lines and 16 I/O ports,
    // so that's a total of 8 * 16 = 128 I/O ports.
    // Datasheet: https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf
    // Speed: 100 kHz, 400 kHz, 1.7 MHz.
    //
    // MCP23S17 is similar to MCP23017 but works using SPI instead of I2C (10 MHz). MCP23S08
    // seems to be the same, but 8-bit.
    //
    // MAX7301 is SPI based, and like all the others, it can generate an ISR when the state
    // changes (pin 31). Address is selected like any other SPI device by CS. MAX7301 includes
    // pullups and schmitt triggers.
    // Datasheet: https://datasheet.lcsc.com/lcsc/1804140032_Maxim-Integrated-MAX7301AAX-_C143583.pdf
    //
    // There's a gazillion different pin extenders out there... I chose to just implement a few
    // of them for now, and only the ones that support interrupts; not using ISR's doesn't make a
    // lot of sense. Add more derived classes as we go.
    class I2CPinExtenderBase : public PinExtenderDriver {
        // Address can be set for up to 4 devices. Each device supports 16 pins.

        static const int numberDevices = 4;
        static const int numberPins    = 16 * numberDevices;

        uint64_t _claimed = 0;

        Machine::I2CBus* _i2cBus   = nullptr;
        int32_t          _i2cBusId = 0;

        // Upper bound on how long an input change can go unnoticed when no interrupt line
        // is wired.  Endstops and probes ride on this, so it defaults low.
        int32_t _pollMs = 5;

        static uint8_t IRAM_ATTR I2CGetValue(Machine::I2CBus* bus, uint8_t address, uint8_t reg);
        static void IRAM_ATTR    I2CSetValue(Machine::I2CBus* bus, uint8_t address, uint8_t reg, uint8_t value);

        // Registers:
        // 4x16 = 64 bits. Fits perfectly into an uint64.
        uint64_t          _configuration = 0;
        uint64_t          _invert        = 0;
        volatile uint64_t _value         = 0;

        // 4 devices, 2 registers per device. 8 bits is enough:
        uint8_t _dirtyRegisters = 0;

        // Pins that asked to be told about changes, and the last state the monitor task
        // dispatched for them.  Kept separate from _value so that a concurrent readPin()
        // refreshing the cache cannot swallow an edge.
        InputPin* _eventPins[numberPins] = { nullptr };
        uint16_t  _lastEventValue[numberDevices] = { 0 };

        // Devices worth reading in the monitor loop: those with event pins or an interrupt.
        uint8_t _monitoredDevices = 0;

        ExtenderInterruptPin _interruptPins[numberDevices];

        TaskHandle_t _monitorTask = nullptr;

        static void monitorTaskLoop(void* arg);

        void pollDevices();
        void refreshDevice(int device);

        const char* _name;

    protected:
        uint8_t _baseAddress = 0x00;  // Set for each pin extender!

    public:
        I2CPinExtenderBase(const char* name) : _name(name) {}

        const char* name() { return _name; }

        void claim(pinnum_t index) override;
        void free(pinnum_t index) override;

        void group(Configuration::HandlerBase& handler) override;

        void init();

        void IRAM_ATTR setupPin(pinnum_t index, Pins::PinAttributes attr) override;
        void IRAM_ATTR writePin(pinnum_t index, bool high) override;
        bool IRAM_ATTR readPin(pinnum_t index) override;
        void IRAM_ATTR flushWrites() override;

        void registerEvent(pinnum_t index, InputPin* obj) override;

        // Called from an interrupt pin's event to cut short the monitor task's wait.
        void wake();

        ~I2CPinExtenderBase();
    };
}
