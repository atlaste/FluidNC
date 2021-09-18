// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Extenders.h"
#include "PCA9539.h"

namespace Extenders {
    void PCA9539::claim(pinnum_t index) {
        Assert(index >= 0 && index < 16 * 4, "PCA9539 IO index should be [0-63]; %d is out of range", index);
        Assert(!_claimed[index], "PCA9539 IO port %d is already used.", index);
        _claimed[index] = true;
    }

    void PCA9539::free(pinnum_t index) { _claimed[index] = false; }

    uint8_t PCA9539::I2CGetValue(uint8_t address, uint8_t reg) {
        auto err = _i2cBus->write(address, &reg, 1);

        if (err) {
            /* TODO FIXME: NOT SURE... */
        }

        uint8_t inputData;
        if (_i2cBus->read(address, &inputData, 1) != 1) {
            /* TODO FIXME: NOT SURE... */
        }

        return inputData;
    }

    void PCA9539::I2CSetValue(uint8_t address, uint8_t reg, uint8_t value) {
        uint8_t data[2];
        data[0]  = reg;
        data[1]  = uint8_t(value);
        auto err = _i2cBus->write(address, data, 2);

        if (err) {
            /* TODO FIXME: NOT SURE... */
        }
    }

    void PCA9539::validate() const {
        auto i2c = config->_i2c;
        Assert(i2c != nullptr, "PCA9539 works through I2C, but I2C is not configured.");
    }

    void PCA9539::group(Configuration::HandlerBase& handler) {}

    void PCA9539::init() { this->_i2cBus = config->_i2c; }

    void PCA9539::setupPin(pinnum_t index, Pins::PinAttributes attr) {
        bool activeLow = attr.has(Pins::PinAttributes::ActiveLow);
        bool output    = attr.has(Pins::PinAttributes::Output);

        uint64_t mask  = uint64_t(1) << index;
        _invert        = (_invert & ~mask) | (activeLow ? mask : 0);
        _configuration = (_configuration & ~mask) | (output ? 0 : mask);

        const uint8_t deviceId = index / 16;

        const uint8_t ConfigReg = 6;
        uint8_t       address   = 0x74 + deviceId;

        uint8_t value = uint8_t(_configuration >> (8 * (index / 8)));
        uint8_t reg   = ConfigReg + ((index / 8) & 1);

        I2CSetValue(address, reg, value);
    }

    void PCA9539::writePin(pinnum_t index, bool high) {
        uint64_t mask   = uint64_t(1) << index;
        uint64_t oldVal = _value;
        uint64_t newVal = high ? mask : uint64_t(0);
        _value          = (_value & ~mask) | newVal;

        _dirtyRegisters |= ((_value != oldVal) ? 1 : 0) << (index / 8);
    }

    bool PCA9539::readPin(pinnum_t index) {
        uint8_t reg      = uint8_t(index / 8);
        uint8_t deviceId = reg / 2;

        const uint8_t InputReg = 0;
        uint8_t       address  = 0x74 + deviceId;

        auto     value    = I2CGetValue(address, InputReg + (reg & 1));
        uint64_t newValue = uint64_t(value) << (int(reg) * 8);
        uint64_t mask     = uint64_t(0xff) << (int(reg) * 8);

        _value = ((newValue ^ _invert) & mask) | (_value & ~mask);
        return (_value & (1 << index)) != 0;
    }

    void PCA9539::flushWrites() {
        uint64_t write = _value ^ _invert;
        for (int i = 0; i < 8; ++i) {
            if ((_dirtyRegisters & (1 << i)) != 0) {
                const uint8_t OutputReg = 2;
                uint8_t       address   = 0x74 + (i / 2);

                uint8_t val = uint8_t(write >> (8 * i));
                uint8_t reg = OutputReg + (i & 1);
                I2CSetValue(address, reg, val);
            }
        }

        _dirtyRegisters = 0;
    }

    const char* PCA9539::name() const { return "pca9539"; }

    namespace {
        PinExtenderFactory::InstanceBuilder<PCA9539> registration("pca9539");
    }
}
