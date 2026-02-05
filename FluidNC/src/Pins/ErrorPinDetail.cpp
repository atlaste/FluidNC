// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "ErrorPinDetail.h"
#include "Config.h"
#include <stdexcept>

namespace Pins {
    ErrorPinDetail::ErrorPinDetail(std::string_view descr) : PinDetail(0), _description(descr) {}

    PinCapabilities ErrorPinDetail::capabilities() const {
        return PinCapabilities::Error;
    }

    void IRAM_ATTR ErrorPinDetail::write(bool high) {
        std::string msg = "Cannot write to pin " + _description + ". The config is incorrect.";
        log_error(msg.c_str());
        throw std::runtime_error(msg);
    }
    bool ErrorPinDetail::read() {
        std::string msg = "Cannot read from pin " + _description + ". The config is incorrect.";
        log_error(msg.c_str());
        throw std::runtime_error(msg);
    }
    void ErrorPinDetail::setAttr(PinAttributes value, uint32_t frequency) {
        // setAttr doesn't throw - it's needed to configure the pin before use
        // The actual error happens on read/write
        log_error("Cannot set mode on pin " << _description.c_str() << ". The config is incorrect.");
    }

    PinAttributes ErrorPinDetail::getAttr() const {
        return PinAttributes::None;
    }

    std::string ErrorPinDetail::toString() {
        std::string s("ERROR_PIN (for ");
        return s + _description + ")";
    }
}
