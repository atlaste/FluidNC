// Copyright (c) 2022 - Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"
#if MAX_N_I2C
#    include "I2CBus.h"
#    include "Driver/fluidnc_i2c.h"
#    include "Channel.h"    // log_msg_to
#    include "NutsBolts.h"  // to_hex
#    include <esp_err.h>
#    include <freertos/FreeRTOS.h>
#    include <freertos/task.h>  // xTaskGetTickCount

namespace Machine {
    I2CBus::I2CBus(objnum_t busNumber) : _busNumber(busNumber) {
        // Recursive, because write() and read() each take it for their own transfer while a caller
        // may already be holding it across a sequence of them.
        _mutex = xSemaphoreCreateRecursiveMutex();
    }

    void I2CBus::acquire() {
        if (_mutex) {
            xSemaphoreTakeRecursive(_mutex, portMAX_DELAY);
        }
    }

    void I2CBus::release() {
        if (_mutex) {
            xSemaphoreGiveRecursive(_mutex);
        }
    }

    void I2CBus::validate() {
        if (_sda.defined() || _scl.defined()) {
            Assert(_sda.defined(), "I2C SDA pin configured multiple times");
            Assert(_scl.defined(), "I2C SCL pin configured multiple times");
        }
    }

    void I2CBus::group(Configuration::HandlerBase& handler) {
        handler.item("sda_pin", _sda);
        handler.item("scl_pin", _scl);
        handler.item("frequency", _frequency);
    }

    void I2CBus::init() {
        _error      = false;
        auto sdaPin = _sda.getNative(Pin::Capabilities::Native | Pin::Capabilities::Input | Pin::Capabilities::Output);
        auto sclPin = _scl.getNative(Pin::Capabilities::Native | Pin::Capabilities::Input | Pin::Capabilities::Output);

        log_info("I2C SDA: " << _sda.name() << ", SCL: " << _scl.name() << ", Freq: " << _frequency << ", Bus #: " << _busNumber);

        _error = i2c_master_init(_busNumber, sdaPin, sclPin, _frequency);
        if (_error) {
            log_error("I2C init failed");
        }

    }

    // How long to stay quiet between summaries while a device keeps failing.
    static const int32_t failureLogIntervalMs = 5000;

    void I2CBus::reportFailure(const char* operation, uint8_t address, int code) {
        ++_failures;

        int32_t now = int32_t(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if (_failures == 1) {
            log_warn("I2C" << _busNumber << ' ' << operation << " to address " << to_hex(address)
                           << " failed: " << ErrorDescription(code));
            _nextFailureLog = now + failureLogIntervalMs;
            return;
        }
        if ((now - _nextFailureLog) >= 0) {
            log_warn("I2C" << _busNumber << ": " << _failures << " failed transfers, most recently " << operation
                           << " to address " << to_hex(address));
            _nextFailureLog = now + failureLogIntervalMs;
        }
    }

    void I2CBus::reportSuccess() {
        if (_failures) {
            log_info("I2C" << _busNumber << " answering again after " << _failures << " failed transfers");
            _failures = 0;
        }
    }

    int I2CBus::write(uint8_t address, const uint8_t* data, size_t count) {
        if (_error) {
            return -1;
        }

        log_verbose("I2C write: " << int(address); for (int i = 0; i < count; ++i) { ss << ' ' << int(data[i]); });

        Lock lock(*this);

        auto ret = i2c_write(_busNumber, address, data, count);
        if (ret != int(count)) {
            reportFailure("write", address, ret);
        } else {
            reportSuccess();
        }
        return ret;
    }

    int I2CBus::read(uint8_t address, uint8_t* data, size_t count) {
        if (_error) {
            return -1;
        }
        log_verbose("I2C read: " << int(address));

        Lock lock(*this);

        auto ret = i2c_read(_busNumber, address, data, count);
        if (ret != int(count)) {
            reportFailure("read", address, ret);
        } else {
            reportSuccess();
        }
        return ret;
    }

    bool I2CBus::probe(uint8_t address) {
        if (_error) {
            return false;
        }
        // Deliberately not routed through write(), so that probing absent addresses does not count
        // against the failure reporting or reset it on success.
        //
        // Tried twice because an address-only transfer is a weaker test than a real one, so it is worth
        // being sure before concluding that a device is absent.
        Lock lock(*this);

        return i2c_write(_busNumber, address, nullptr, 0) == 0 || i2c_write(_busNumber, address, nullptr, 0) == 0;
    }

    void I2CBus::scan(Channel& out) {
        if (_error) {
            log_error_to(out, "I2C" << _busNumber << " did not initialise");
            return;
        }

        log_msg_to(out, "Scanning I2C" << _busNumber << " (SDA " << _sda.name() << ", SCL " << _scl.name() << ")");

        int found = 0;
        // 0x00 is the general call address and 0x78-0x7f are reserved, so neither is a device.
        for (uint8_t address = 1; address < 0x78; address++) {
            if (probe(address)) {
                log_msg_to(out, "  device at " << to_hex(address));
                ++found;
            }
        }
        if (found == 0) {
            log_msg_to(out, "No devices answered.  Check wiring, pull-ups and that the devices are powered");
        }
    }

    const char* I2CBus::ErrorDescription(int code) {
        // write() and read() negate the esp_err_t so that a failure cannot be mistaken for a byte
        // count.  ESP_FAIL is already -1 and passes through unchanged.
        return esp_err_to_name(code < ESP_FAIL ? esp_err_t(-code) : esp_err_t(code));
    }
}
#endif
