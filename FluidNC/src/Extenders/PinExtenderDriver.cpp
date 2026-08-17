// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "PinExtenderDriver.h"

namespace Extenders {

    void PinExtenderDriver::attachInterrupt(pinnum_t index, void (*callback)(void*, bool), void* arg, uint8_t mode) {
        Assert(false, "Interrupts are not supported by pin extender for pin %d", index);
    }
    void PinExtenderDriver::detachInterrupt(pinnum_t index) {
        Assert(false, "Interrupts are not supported by pin extender");
    }

    Pins::PinCapabilities PinExtenderDriver::capabilities() const {
        return Pins::PinCapabilities::Input | Pins::PinCapabilities::Output;
    }

    void PinExtenderDriver::registerEvent(pinnum_t index, InputPin* obj) {
        Assert(false, "Pin events are not supported by pin extender for pin %d", index);
    }
}
