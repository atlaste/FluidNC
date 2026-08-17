// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"
#if MAX_N_I2C
#    include "Extenders.h"
#    include "I2CPinExtenderBase.h"
#    include "Logging.h"
#    include "Protocol.h"  // protocol_send_event, pinActiveEvent, pinInactiveEvent

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

    uint8_t I2CPinExtenderBase::I2CGetValue(Machine::I2CBus* bus, uint8_t address, uint8_t reg) {
        // write() returns number of bytes written on success, negative on error
        auto written = bus->write(address, &reg, 1);

        if (written != 1) {
            log_info("Error writing to i2c bus. Code: " << written);
            return 0;
        }

        uint8_t inputData;
        if (bus->read(address, &inputData, 1) != 1) {
            log_info("Error reading from i2c bus.");
        }

        return inputData;
    }

    void I2CPinExtenderBase::I2CSetValue(Machine::I2CBus* bus, uint8_t address, uint8_t reg, uint8_t value) {
        uint8_t data[2];
        data[0] = reg;
        data[1] = uint8_t(value);
        // write() returns number of bytes written on success, negative on error
        auto written = bus->write(address, data, 2);

        if (written != 2) {
            log_error("Error writing to i2c bus; I2C pin extender failed. Code: " << written);
        }
    }

    void I2CPinExtenderBase::group(Configuration::HandlerBase& handler) {
        handler.item("busId", _i2cBusId);
        handler.item("poll_ms", _pollMs, 1, 1000);
        handler.item("interrupt0", _interruptPins[0]);
        handler.item("interrupt1", _interruptPins[1]);
        handler.item("interrupt2", _interruptPins[2]);
        handler.item("interrupt3", _interruptPins[3]);
    }

    void I2CPinExtenderBase::init() {
        Assert(_i2cBusId >= 0 && _i2cBusId < MAX_N_I2C, "I2C bus ID out of range");
        this->_i2cBus = config->_i2c[_i2cBusId];

        Assert(_i2cBus != nullptr, "I2C pin extender only works when I2C bus is configured");

        log_info("Setting up I2C pin extender on I2C" << _i2cBusId);

        for (int device = 0; device < numberDevices; ++device) {
            // Seed the change detector from the real device state so that the first poll
            // does not report every asserted input as a fresh edge.
            refreshDevice(device);
            _lastEventValue[device] = uint16_t(_value >> (device * 16));

            auto& pin = _interruptPins[device];
            if (!pin.undefined()) {
                pin.attach(this);
                pin.init();
                _monitoredDevices |= uint8_t(1) << device;
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

    void I2CPinExtenderBase::monitorTaskLoop(void* arg) {
        auto inst = static_cast<I2CPinExtenderBase*>(arg);

        while (true) {
            // Either an interrupt line woke us or the poll interval expired; both mean
            // "go look at the inputs".
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(inst->_pollMs));
            inst->pollDevices();
        }
    }

    void I2CPinExtenderBase::refreshDevice(int device) {
        uint8_t address = uint8_t(_baseAddress + device);

        uint16_t raw = uint16_t(I2CGetValue(_i2cBus, address, InputReg));
        raw |= uint16_t(I2CGetValue(_i2cBus, address, InputReg + 1)) << 8;

        uint16_t value = raw ^ uint16_t(_invert >> (device * 16));

        // Only the bits configured as inputs are ours to update; leaving the output bits
        // alone keeps a concurrent writePin() from losing its pending value.
        uint64_t inputMask = _configuration & (uint64_t(0xFFFF) << (device * 16));
        _value             = (_value & ~inputMask) | ((uint64_t(value) << (device * 16)) & inputMask);
    }

    void I2CPinExtenderBase::pollDevices() {
        for (int device = 0; device < numberDevices; ++device) {
            if ((_monitoredDevices & (uint8_t(1) << device)) == 0) {
                continue;
            }

            refreshDevice(device);

            uint16_t value   = uint16_t(_value >> (device * 16));
            uint16_t changed = value ^ _lastEventValue[device];
            if (changed == 0) {
                continue;
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
    }

    void I2CPinExtenderBase::setupPin(pinnum_t index, Pins::PinAttributes attr) {
        bool activeLow = attr.has(Pins::PinAttributes::ActiveLow);
        bool output    = attr.has(Pins::PinAttributes::Output);

        uint64_t mask  = uint64_t(1) << index;
        _invert        = (_invert & ~mask) | (activeLow ? mask : 0);
        _configuration = (_configuration & ~mask) | (output ? 0 : mask);

        const uint8_t deviceId = index / 16;

        uint8_t address = _baseAddress + deviceId;

        uint8_t value = uint8_t(_configuration >> (8 * (index / 8)));
        uint8_t reg   = ConfigReg + ((index / 8) & 1);

        // log_info("Setup reg " << int(reg) << " with value " << int(value));

        I2CSetValue(_i2cBus, address, reg, value);
    }

    void I2CPinExtenderBase::writePin(pinnum_t index, bool high) {
        uint64_t mask   = uint64_t(1) << index;
        uint64_t oldVal = _value;
        uint64_t newVal = high ? mask : uint64_t(0);
        _value          = (_value & ~mask) | newVal;

        _dirtyRegisters |= ((_value != oldVal) ? 1 : 0) << (index / 8);
    }

    bool I2CPinExtenderBase::readPin(pinnum_t index) {
        uint8_t reg      = uint8_t(index / 8);
        uint8_t deviceId = reg / 2;
        uint8_t address  = _baseAddress + deviceId;

        // Always ask the device.  The cached value exists for the monitor task's edge
        // detection, and trusting it here is what previously made reads on an
        // interrupt-driven device return whatever was true at startup.
        auto     readReg  = InputReg + (reg & 1);
        auto     value    = I2CGetValue(_i2cBus, address, readReg);
        uint64_t newValue = uint64_t(value) << (int(reg) * 8);
        uint64_t mask     = uint64_t(0xff) << (int(reg) * 8);

        _value = ((newValue ^ _invert) & mask) | (_value & ~mask);

        return (_value & (1ull << index)) != 0;
    }

    void I2CPinExtenderBase::flushWrites() {
        uint64_t write = _value ^ _invert;
        for (int i = 0; i < 8; ++i) {
            if ((_dirtyRegisters & (1 << i)) != 0) {
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
