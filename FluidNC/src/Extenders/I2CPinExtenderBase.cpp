// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"
#if MAX_N_I2C
#    include "Extenders.h"
#    include "I2CPinExtenderBase.h"
#    include "Logging.h"
#    include "MotionControl.h"  // mc_critical
#    include "NutsBolts.h"      // to_hex
#    include "Protocol.h"       // protocol_send_event, pinActiveEvent, pinInactiveEvent

#    include <freertos/FreeRTOS.h>

namespace Extenders {
    // Register addresses shared by every device in this family.
    static const uint8_t InputReg  = 0;
    static const uint8_t OutputReg = 2;
    static const uint8_t ConfigReg = 6;

    void ExtenderInterruptPin::trigger(bool active) {
        InputPin::trigger(active);
        if (_container) {
            _container->wake();
        }
    }

    void I2CPinExtenderBase::claim(pinnum_t index) {
        Assert(index >= 0 && index < numberPins, "I2C pin extender IO index should be [0-%d]; %d is out of range", numberPins - 1, index);

        uint64_t mask = uint64_t(1) << index;
        Assert((_claimed & mask) == 0, "I2C pin extender IO port %d is already used", index);

        _claimed |= mask;
    }

    void I2CPinExtenderBase::free(pinnum_t index) {
        uint64_t mask = uint64_t(1) << index;
        _claimed &= ~mask;
    }

    // Neither of these logs a failed transfer.  I2CBus already reports one, naming the address and the
    // cause and rate limiting itself, and these are called from the monitor task often enough that a
    // second message per failure was most of the console traffic on a board with a missing device.

    bool I2CPinExtenderBase::I2CGetValue(Machine::I2CBus* bus, uint8_t address, uint8_t reg, uint8_t& value) {
        // Selecting the register and reading it are two transfers, and the device remembers the
        // selection in between, so they have to be kept together.
        Machine::I2CBus::Lock lock(*bus);

        // write() returns number of bytes written on success, negative on error
        if (bus->write(address, &reg, 1) != 1) {
            return false;
        }

        return bus->read(address, &value, 1) == 1;
    }

    void I2CPinExtenderBase::I2CSetValue(Machine::I2CBus* bus, uint8_t address, uint8_t reg, uint8_t value) {
        uint8_t data[2];
        data[0] = reg;
        data[1] = uint8_t(value);
        bus->write(address, data, 2);
    }

    void I2CPinExtenderBase::group(Configuration::HandlerBase& handler) {
        handler.item("busId", _i2cBusId);
        handler.item("address", _address, -1, 0x77);
        handler.item("poll_ms", _pollMs, 1, 1000);
        handler.item("failsafe_inputs", _failsafeInputs);
        handler.item("alarm_on_loss", _alarmOnLoss);

        // Split because the configuration system has no 64 bit item.  Same names and meaning as the
        // CAN pin extender's, which faces the identical problem of a device that stops reporting.
        handler.item("failsafe_state_low", _failsafeStateLow);
        handler.item("failsafe_state_high", _failsafeStateHigh);
        handler.item("interrupt0", _interruptPins[0]);
        handler.item("interrupt1", _interruptPins[1]);
        handler.item("interrupt2", _interruptPins[2]);
        handler.item("interrupt3", _interruptPins[3]);
    }

    void I2CPinExtenderBase::init() {
        Assert(_i2cBusId >= 0 && _i2cBusId < MAX_N_I2C, "I2C bus ID out of range");
        this->_i2cBus = config->_i2c[_i2cBusId];

        Assert(_i2cBus != nullptr, "I2C pin extender only works when I2C bus is configured");

        if (_address >= 0) {
            _baseAddress = uint8_t(_address);
        }

        log_info("Setting up I2C pin extender on I2C" << _i2cBusId);

        // Most boards fit one chip out of the four addresses the family allows, so ask which are
        // actually there before talking to them.  Every later operation is gated on the answer:
        // without that, a board with a single device generates a failed transfer for each of the
        // three absent ones on every pass of the monitor task, which floods the console at the poll
        // rate and makes a healthy board look broken.
        for (int device = 0; device < numberDevices; ++device) {
            if (_i2cBus->probe(uint8_t(_baseAddress + device))) {
                _presentDevices |= uint8_t(1) << device;
            }
        }

        if (_presentDevices == 0) {
            // to_hex returns a shared buffer, so it can only appear once per statement.
            log_error(name() << ": nothing answered on any of the " << numberDevices << " addresses starting at "
                             << to_hex(_baseAddress) << ".  Run $I2C/Scan to see what is on the bus");
            return;
        }

        for (int device = 0; device < numberDevices; ++device) {
            if ((_presentDevices & (uint8_t(1) << device)) == 0) {
                continue;
            }

            log_info(name() << " device " << device << " at address " << to_hex(_baseAddress + device));

            // No pin has been set up yet, so this reads nothing into the cache and only confirms that
            // the device that answered the probe also answers a register read.  Seeding the cache and
            // the change detector has to wait for setupPin(), which is where the inputs become known.
            refreshDevice(device);

            auto& pin = _interruptPins[device];
            if (!pin.undefined()) {
                pin.attach(this);
                pin.init();
                _monitoredDevices |= uint8_t(1) << device;
                _interruptDevices |= uint8_t(1) << device;
            }
        }

        // The task is started unconditionally because pins register their events during
        // Axes/control init, which happens after this point; _monitoredDevices is consulted
        // on every pass rather than once here.
        xTaskCreatePinnedToCore(monitorTaskLoop,
                                "i2c_ext",
                                configMINIMAL_STACK_SIZE + 2048,
                                this,
                                1,
                                &_monitorTask,
                                SUPPORT_TASK_CORE);
    }

    void I2CPinExtenderBase::wake() {
        if (_monitorTask) {
            xTaskNotifyGive(_monitorTask);
        }
    }

    void I2CPinExtenderBase::registerEvent(pinnum_t index, InputPin* obj) {
        Assert(index >= 0 && index < numberPins, "I2C pin extender pin %d out of range", int(index));
        _eventPins[index] = obj;
        _monitoredDevices |= uint8_t(1) << (index / 16);
    }

    // How often a device whose interrupt line is wired gets read anyway, to prove it is still there.
    // Roughly ten times a second: the alarm for a device that has gone is allowed to be a little late,
    // since the normal case is that everything is alive and this read only exists to notice when it is
    // not.  Deliberately not a round 100 ms, which is StatePersistence's save interval - two periodic
    // users of one bus at the same period would sit in a fixed phase relationship and either always or
    // never collide, and the collision is the case that costs a save its latency.
    static const int32_t healthCheckMs = 97;

    // How many consecutive failed reads mean the device has gone rather than that a transfer glitched.
    // Costs a little detection latency in exchange for not alarming over a one-off.
    static const uint8_t failuresBeforeLost = 3;

    int32_t I2CPinExtenderBase::waitMs() const {
        // A watched device with no interrupt line is the only reason to wake on a timer at all.
        // Reading the others that fast would put constant traffic on a bus shared with other
        // peripherals for no benefit, which is the whole point of wiring the interrupt up.
        uint8_t watched = _monitoredDevices & _presentDevices;
        return (watched & ~_interruptDevices) ? _pollMs : healthCheckMs;
    }

    void I2CPinExtenderBase::monitorTaskLoop(void* arg) {
        auto inst = static_cast<I2CPinExtenderBase*>(arg);

        while (true) {
            // Which devices are worth reading depends on why we woke, so pass that on: a notification
            // means an interrupt line went active, a timeout means a polled device is due.
            bool fromInterrupt = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(inst->waitMs())) != 0;
            inst->pollDevices(fromInterrupt);
        }
    }

    bool I2CPinExtenderBase::refreshDevice(int device) {
        uint8_t address = uint8_t(_baseAddress + device);

        uint8_t low  = 0;
        uint8_t high = 0;
        if (!I2CGetValue(_i2cBus, address, InputReg, low) || !I2CGetValue(_i2cBus, address, InputReg + 1, high)) {
            // Say nothing here.  I2CBus has already reported the transfer, and the caller decides
            // whether one failure is a glitch or the device has gone.
            return false;
        }

        uint16_t raw   = uint16_t(low) | (uint16_t(high) << 8);
        uint16_t value = raw ^ uint16_t(_invert >> (device * 16));

        // Only the bits configured as inputs are ours to update; leaving the output bits
        // alone keeps a concurrent writePin() from losing its pending value.
        uint64_t inputMask = _configuration & (uint64_t(0xFFFF) << (device * 16));
        _value             = (_value & ~inputMask) | ((uint64_t(value) << (device * 16)) & inputMask);

        return true;
    }

    void I2CPinExtenderBase::pollDevices(bool fromInterrupt) {
        // Each device is handled according to how it was wired, because that is a per-device choice in
        // the config and there is no telling what a given board did.
        uint8_t watched   = _monitoredDevices & _presentDevices;
        uint8_t polled    = watched & ~_interruptDevices;
        uint8_t interrupt = watched & _interruptDevices;

        int32_t now = int32_t(xTaskGetTickCount() * portTICK_PERIOD_MS);
        uint8_t due = 0;

        if (fromInterrupt) {
            // A line went active.  It says something changed but not on which device, and by now the
            // input may even have changed back, so read every device that could have raised it rather
            // than trusting the line's present level.
            due |= interrupt;
        }

        // The remaining two reasons to read go by their own deadlines rather than by why this wake
        // happened, so that a device whose interrupt fires steadily cannot starve either of them by
        // cutting every wait short.

        // A device with no interrupt line reports nothing except when it is read.
        if ((now - _nextPoll) >= 0) {
            due |= polled;
            _nextPoll = now + _pollMs;
        }

        // A device with one is read on a slow timer anyway, because the line says nothing about
        // whether the chip is still there: one that has lost power, been reset or fallen off the bus
        // leaves its interrupt idle, which is indistinguishable from inputs that have not moved.
        // Without this the machine would go on cutting against endstop values that are no longer
        // coming from anywhere.  The same read covers an interrupt edge that went missing.
        if ((now - _nextHealthCheck) >= 0) {
            due |= interrupt;
            _nextHealthCheck = now + healthCheckMs;
        }

        for (int device = 0; device < numberDevices; ++device) {
            if ((due & (uint8_t(1) << device)) == 0) {
                continue;
            }

            if (refreshDevice(device)) {
                noteResponse(device);
                dispatchChanges(device, uint16_t(_value >> (device * 16)));
            } else {
                noteFailure(device);
            }
        }
    }

    void I2CPinExtenderBase::dispatchChanges(int device, uint16_t value) {
        uint16_t changed = value ^ _lastEventValue[device];
        if (changed == 0) {
            return;
        }
        _lastEventValue[device] = value;

        for (int bit = 0; bit < 16; ++bit) {
            uint16_t mask = uint16_t(1) << bit;
            if ((changed & mask) == 0) {
                continue;
            }

            InputPin* obj = _eventPins[device * 16 + bit];
            if (obj == nullptr) {
                continue;
            }

            // Hand the transition to the protocol task; this runs on a support task and
            // must not do the work of an alarm or a feed hold itself.
            bool active = (value & mask) != 0;
            protocol_send_event(active ? &pinActiveEvent : &pinInactiveEvent, obj);
        }
    }

    void I2CPinExtenderBase::noteResponse(int device) {
        uint8_t mask = uint8_t(1) << device;

        _failureCount[device] = 0;

        if (_lostDevices & mask) {
            _lostDevices &= ~mask;
            log_info(name() << " device " << device << " is answering again");
        }
    }

    void I2CPinExtenderBase::noteFailure(int device) {
        uint8_t mask = uint8_t(1) << device;

        if (_lostDevices & mask) {
            return;  // Already reported; nothing to add until it comes back.
        }
        if (++_failureCount[device] < failuresBeforeLost) {
            return;  // Tolerate the odd glitch rather than halting the machine over one.
        }

        _lostDevices |= mask;
        log_error(name() << " device " << device << " has stopped responding at address " << to_hex(_baseAddress + device));

        if (_failsafeInputs) {
            // Drive this device's inputs to the failsafe pattern and dispatch the transitions, so an
            // input wired normally closed and declared active low asserts exactly as a cut wire
            // would, rather than reading clear because nothing is answering.
            uint64_t logical   = failsafeState() ^ _invert;
            uint64_t inputMask = _configuration & (uint64_t(0xFFFF) << (device * 16));
            _value             = (_value & ~inputMask) | (logical & inputMask);

            dispatchChanges(device, uint16_t(_value >> (device * 16)));
        }
        if (_alarmOnLoss) {
            // Whatever these pins carry, they are no longer being reported, and continuing to move on
            // stale endstop and probe values is not a thing to do quietly.
            mc_critical(ExecAlarm::ExpanderReset);
        }
    }

    void I2CPinExtenderBase::setupPin(pinnum_t index, Pins::PinAttributes attr) {
        bool activeLow = attr.has(Pins::PinAttributes::ActiveLow);
        bool output    = attr.has(Pins::PinAttributes::Output);

        uint64_t mask  = uint64_t(1) << index;
        _invert        = (_invert & ~mask) | (activeLow ? mask : 0);
        _configuration = (_configuration & ~mask) | (output ? 0 : mask);

        if (!output) {
            // readPin() answers from the cache, so any device carrying an input needs the monitor
            // task to keep that cache current - not just the ones whose pins registered for events.
            _monitoredDevices |= uint8_t(1) << (index / 16);
        }

        const uint8_t deviceId = index / 16;
        if (!devicePresent(deviceId)) {
            return;
        }

        uint8_t address = _baseAddress + deviceId;

        uint8_t value = uint8_t(_configuration >> (8 * (index / 8)));
        uint8_t reg   = ConfigReg + ((index / 8) & 1);

        // log_info("Setup reg " << int(reg) << " with value " << int(value));

        I2CSetValue(_i2cBus, address, reg, value);

        if (!output) {
            // Seed the cache readPin() answers from, now that the device knows this pin is an input.
            // Without this it would read as zero - "input not asserted" - until the monitor task's
            // first pass, and the startup limit check happens inside that window.  This runs while the
            // configuration is being applied, so it is the one place a blocking transfer costs nothing.
            refreshDevice(deviceId);
            _lastEventValue[deviceId] = uint16_t(_value >> (deviceId * 16));
        }
    }

    void I2CPinExtenderBase::writePin(pinnum_t index, bool high) {
        uint64_t mask   = uint64_t(1) << index;
        uint64_t oldVal = _value;
        uint64_t newVal = high ? mask : uint64_t(0);
        _value          = (_value & ~mask) | newVal;

        _dirtyRegisters |= ((_value != oldVal) ? 1 : 0) << (index / 8);
    }

    bool I2CPinExtenderBase::readPin(pinnum_t index) {
        // Deliberately does not touch the bus.  Limit and probe checks and status reports call this
        // from tasks that must not stall, and a transfer costs hundreds of microseconds at best and
        // far longer when it has to queue behind another user of a shared bus - which on this board
        // includes the FRAM the position is saved to.  The monitor task keeps _value current instead:
        // on the interrupt for a device that has its line wired, every poll_ms for one that does not.
        //
        // A stale cache is the risk this trades for, so nothing is allowed to leave it unattended: a
        // device with any input on it is read by the monitor task whether or not its pins asked for
        // events, and one that stops answering raises an alarm instead of quietly freezing its inputs.
        return (_value & (uint64_t(1) << index)) != 0;
    }

    void I2CPinExtenderBase::flushWrites() {
        uint64_t write = _value ^ _invert;
        for (int i = 0; i < 8; ++i) {
            if ((_dirtyRegisters & (1 << i)) != 0 && devicePresent(i / 2)) {
                uint8_t address = _baseAddress + (i / 2);

                uint8_t val = uint8_t(write >> (8 * i));
                uint8_t reg = OutputReg + (i & 1);
                I2CSetValue(_i2cBus, address, reg, val);
            }
        }

        _dirtyRegisters = 0;
    }

    I2CPinExtenderBase::~I2CPinExtenderBase() {
        if (_monitorTask) {
            vTaskDelete(_monitorTask);
            _monitorTask = nullptr;
        }
    }
}
#endif
