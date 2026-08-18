// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class Channel;

namespace Machine {
    class I2CBus : public Configuration::Configurable {
    protected:
        bool _error = false;

        // A device that stops answering is usually asked again straight away - the pin extender's
        // monitor task reads its inputs every few milliseconds - so logging each failure buries
        // everything else in the console and slows the system down with the logging itself.  Report
        // the first failure in full, then only a periodic count, and say when the bus recovers.
        uint32_t _failures        = 0;
        int32_t  _nextFailureLog  = 0;

        void reportFailure(const char* operation, uint8_t address, int code);
        void reportSuccess();

        SemaphoreHandle_t _mutex = nullptr;

    public:
        I2CBus(objnum_t busNumber);

        void acquire();
        void release();

        // Holds the bus across a sequence of transfers that must not be split.  Pointing a device at a
        // register and then reading it takes two, and several tasks share a bus here - the pin
        // extender's monitor task, state persistence, and whichever channel ran $I2C/Scan - so without
        // this one of them can land a transfer in the gap and both come away with the wrong bytes.
        //
        // write() and read() take it for their own transfer too, so that a caller which does not know
        // it should be holding it cannot interleave with one that is.  Queueing on this rather than in
        // the driver also keeps a transfer from being timed out by the wait: the driver's own timeout
        // is per transfer and short, so a burst from one task used to make another's transfer fail
        // outright instead of simply waiting its turn.
        class Lock {
            I2CBus* _bus;

        public:
            explicit Lock(I2CBus& bus) : _bus(&bus) { _bus->acquire(); }
            ~Lock() { _bus->release(); }

            Lock(const Lock&)            = delete;
            Lock& operator=(const Lock&) = delete;
        };

        objnum_t _busNumber = 0;
        Pin      _sda;
        Pin      _scl;
        uint32_t _frequency = 100000;

        void init();
        void validate() override;
        void group(Configuration::HandlerBase& handler) override;

        static const char* ErrorDescription(int code);

        int write(uint8_t address, const uint8_t* data, size_t count);
        int read(uint8_t address, uint8_t* data, size_t count);

        // True when a device acknowledges its address.  Uses an address-only transfer, so it does not
        // disturb whatever register pointer the device is holding.
        bool probe(uint8_t address);

        // List the devices that answer, for working out whether a missing device is absent,
        // misaddressed or holding the bus down.
        void scan(Channel& out);

        ~I2CBus() = default;
    };
}
