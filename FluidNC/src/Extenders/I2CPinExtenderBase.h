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
    // them against the previous reading and turns each changed bit into a pin event.  A
    // device with an optional per-device 'interruptN' line is read when that line fires; a
    // device without one is polled every poll_ms, which is the only option on boards that do
    // not route the interrupt or whose interrupt GPIO is unavailable.
    //
    // Nothing else reads the device.  readPin() answers from the value that task maintains, because
    // it is called from limit and probe checks that cannot afford to wait on a bus transfer.
    //
    // Losing a device
    // ---------------
    // Every device is read on a slow timer as well, however it is wired.  The interrupt cannot
    // report that the chip itself has gone: one that has lost power, been reset or fallen off the
    // bus leaves its interrupt idle, which looks exactly like inputs that have not moved.  So a
    // device that stops answering would otherwise leave the machine cutting on endstop and probe
    // values that are no longer coming from anywhere.
    //
    // When one does stop answering, its inputs are forced to failsafe_state and the resulting
    // transitions are dispatched normally, and losing it raises an ExpanderReset alarm.  This is the
    // same contract the CAN pin extender offers for a node that stops reporting, for the same
    // reason, and both are configured with the same items.
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

        // Overrides the address the driver would use by itself.  Needed because these parts are
        // pin compatible but not address compatible: a PCA9535/9555 uses pin 3 as a third address
        // strap where a PCA9539 uses it as a reset, so a board that pulls that pin high puts the
        // chip at an address outside the four a two-strap part can occupy.  -1 keeps the default.
        int32_t _address = -1;

        // Upper bound on how long an input change can go unnoticed on a device with no interrupt
        // line.  Endstops and probes ride on this, so it defaults low.  It does not govern a device
        // whose interrupt is wired; that one is read when the interrupt says to.
        int32_t _pollMs = 5;

        // False when the transfer failed, so that a dead device is distinguishable from one reporting
        // zeroes.  A zero would read as "no input asserted", which is the wrong way to be wrong.
        static bool IRAM_ATTR I2CGetValue(Machine::I2CBus* bus, uint8_t address, uint8_t reg, uint8_t& value);
        static void IRAM_ATTR I2CSetValue(Machine::I2CBus* bus, uint8_t address, uint8_t reg, uint8_t value);

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

        // Devices worth reading in the monitor loop: any with an input pin, an event pin or an
        // interrupt line.  A device carrying only outputs is never read.
        uint8_t _monitoredDevices = 0;

        // Of those, the ones whose interrupt line is wired, and which therefore are read when it fires
        // rather than at the poll interval.  They are still read slowly to prove they are alive.
        uint8_t _interruptDevices = 0;

        // Devices that have stopped answering, and how many consecutive reads have failed.
        uint8_t _lostDevices                 = 0;
        uint8_t _failureCount[numberDevices] = { 0 };

        // Deadlines for the two timed reads: when the devices with no interrupt line are next read,
        // and when the ones that have a line are read whether or not it asked for it.
        int32_t _nextPoll        = 0;
        int32_t _nextHealthCheck = 0;

        // What the inputs are forced to when a device stops answering, in the pins' own electrical
        // sense; the :low option is applied on top exactly as it is to a real reading.  So the default
        // of zero, on an input wired normally closed and declared active low, asserts that input just
        // as cutting its wire would - losing the device must never read as "all clear".
        uint32_t _failsafeStateLow  = 0;
        uint32_t _failsafeStateHigh = 0;
        bool     _failsafeInputs    = true;
        bool     _alarmOnLoss       = true;

        uint64_t failsafeState() const { return (uint64_t(_failsafeStateHigh) << 32) | uint64_t(_failsafeStateLow); }

        // Which of the family's addresses answered when init() probed them.  A board normally fits
        // fewer chips than the address pins allow, and talking to the absent ones only produces
        // failed transfers.
        uint8_t _presentDevices = 0;

        bool devicePresent(int device) const { return (_presentDevices & (uint8_t(1) << device)) != 0; }

        ExtenderInterruptPin _interruptPins[numberDevices];

        TaskHandle_t _monitorTask = nullptr;

        static void monitorTaskLoop(void* arg);

        int32_t waitMs() const;
        void    pollDevices(bool fromInterrupt);
        bool    refreshDevice(int device);
        void    dispatchChanges(int device, uint16_t value);
        void    noteResponse(int device);
        void    noteFailure(int device);

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

        pinnum_t pinCount() const override { return numberPins; }

        // Pins on an address that did not answer are not available, which is what keeps a board with
        // one chip from offering the 48 pins of the three it does not have.
        bool pinAvailable(pinnum_t index) const override {
            return index < numberPins && devicePresent(index / 16) && (_claimed & (uint64_t(1) << index)) == 0;
        }

        // Called from an interrupt pin's event to cut short the monitor task's wait.
        void wake();

        ~I2CPinExtenderBase();
    };
}
