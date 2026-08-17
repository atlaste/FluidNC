// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"
#include "Pins/PinAttributes.h"
#include "Pins/PinCapabilities.h"

#include "Platform.h"

class InputPin;

namespace Extenders {
    class PinExtenderDriver : public Configuration::Configurable {
    public:
        virtual void init() = 0;

        virtual void claim(pinnum_t index) = 0;
        virtual void free(pinnum_t index)  = 0;

        virtual void IRAM_ATTR setupPin(pinnum_t index, Pins::PinAttributes attr) = 0;
        virtual void IRAM_ATTR writePin(pinnum_t index, bool high)                = 0;
        virtual bool IRAM_ATTR readPin(pinnum_t index)                            = 0;
        virtual void IRAM_ATTR flushWrites()                                      = 0;

        virtual void attachInterrupt(pinnum_t index, void (*callback)(void*, bool), void* arg, uint8_t mode);
        virtual void detachInterrupt(pinnum_t index);

        // Each class of extender advertises its own capability bit so that consumers can
        // reject pin classes that are unsuitable for their purpose.
        virtual Pins::PinCapabilities capabilities() const;

        // Event-driven input.  Drivers that can report input changes asynchronously override
        // this and call obj->trigger() (via protocol_send_event) when the pin changes state.
        virtual void registerEvent(pinnum_t index, InputPin* obj);

        // Name is required for the configuration factory to work.
        virtual const char* name() const = 0;

        // Virtual base classes require a virtual destructor.
        virtual ~PinExtenderDriver() {}
    };
}
