// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "ExtPinDetail.h"

namespace Pins {
    ExtPinDetail::ExtPinDetail(uint32_t device, pinnum_t index, const PinOptionsParser& options) :
        PinDetail(index), _device(device), _capabilities(PinCapabilities::Output | PinCapabilities::Input | PinCapabilities::ISR),
        _attributes(Pins::PinAttributes::Undefined) {
        // User defined pin capabilities
        for (auto opt : options) {
            if (opt.is("low")) {
                _attributes = _attributes | PinAttributes::ActiveLow;
            } else if (opt.is("high")) {
                // Default: Active HIGH.
            } else {
                Assert(false, "Unsupported pin extender option '%s'", opt());
            }
        }
    }

    Extenders::PinExtenderDriver* ExtPinDetail::driver() const {
        if (_owner == nullptr) {
            auto ext = config->_extenders;
            Assert(ext != nullptr && ext->_pinDrivers[_device] != nullptr && ext->_pinDrivers[_device]->_driver != nullptr,
                   "Cannot find pin extender definition in configuration for pin pinext%d.%d",
                   int(_device),
                   int(_index));
            _owner = ext->_pinDrivers[_device]->_driver;
        }
        return _owner;
    }

    PinCapabilities ExtPinDetail::capabilities() const {
        return driver()->capabilities() | PinCapabilities::ISR;
    }

    // I/O:
    void ExtPinDetail::write(bool high) {
        Assert(_owner != nullptr, "Cannot write to uninitialized pin");
        _owner->writePin(_index, high);
    }

    void ExtPinDetail::synchronousWrite(bool high) {
        Assert(_owner != nullptr, "Cannot write to uninitialized pin");
        _owner->writePin(_index, high);
        _owner->flushWrites();
    }

    bool ExtPinDetail::read() {
        Assert(_owner != nullptr, "Cannot read from uninitialized pin");
        return _owner->readPin(_index);
    }

    void ExtPinDetail::setAttr(PinAttributes value, uint32_t frequency) {
        // We setup the driver in setAttr. Before this time, the owner might not be valid.

        // Check the attributes first:
        Assert(value.has(PinAttributes::Input) || value.has(PinAttributes::Output),
               "Pin extender pins can be used as either input or output");
        Assert(value.has(PinAttributes::Input) != value.has(PinAttributes::Output),
               "Pin extender pins can be used as either input or output");
        Assert(value.validateWith(this->_capabilities), "Requested attributes do not match the pin extender capabilities");
        Assert(!_attributes.conflictsWith(value), "Attributes on this pin have been set before, and there's a conflict");

        _attributes = value;

        if (!_claimed) {
            driver()->claim(_index);
            _claimed = true;
        }

        driver()->setupPin(_index, _attributes);
        driver()->writePin(_index, value.has(PinAttributes::InitialOn));
    }

    void ExtPinDetail::registerEvent(InputPin* obj) {
        driver()->registerEvent(_index, obj);
    }

    PinAttributes ExtPinDetail::getAttr() const {
        return _attributes;
    }

#if 0
    void ExtPinDetail::attachInterrupt(void (*callback)(void*, bool), void* arg, uint8_t mode) {
        Assert(_owner != nullptr, "Cannot attach ISR on uninitialized pin");
        _owner->attachInterrupt(_index, callback, arg, mode);
    }
    void ExtPinDetail::detachInterrupt() {
        Assert(_owner != nullptr, "Cannot detach ISR on uninitialized pin");
        _owner->detachInterrupt(_index);
    }
#endif

    std::string ExtPinDetail::toString() {
        char buf[20];
        snprintf(buf, 20, "pinext%d.%d", int(_device), int(_index));
        std::string s(buf);
        if (_attributes.has(PinAttributes::ActiveLow)) {
            s += ":low";
        }
        return s;
    }

    ExtPinDetail::~ExtPinDetail() {
        if (_claimed && _owner) {
            _owner->free(_index);
        }
    }
}
