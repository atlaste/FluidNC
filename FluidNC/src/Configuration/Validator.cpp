// Copyright (c) 2021 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Validator.h"

#include "Configurable.h"
#include "../Logging.h"

#include <cstdarg>
#include <cstring>
#include <atomic>

namespace Configuration {
    Validator::Validator() {
        // Read fence for config. Shouldn't be necessary, but better safe than sorry.
        std::atomic_thread_fence(std::memory_order::memory_order_seq_cst);
    }

    void Validator::validate(bool condition, const char* error, ...) {
        if (!condition) {
            char    tmp[255];
            va_list arg;
            va_start(arg, error);
            size_t len = vsnprintf(tmp, 255, error, arg);
            va_end(arg);
            tmp[254] = 0;

            log_error("Validation error at "; for (auto it : _path) { ss << '/' << it; } ss << ": " << tmp);
            _validationFailed = true;
        }
    }

    void Validator::enterSection(const char* name, Configurable* value) {
        _path.push_back(name);  // For error handling

        try {
            value->validate(*this);
        } catch (const AssertionFailed& ex) {
            // Log something meaningful to the user:
            log_error("Validation error at "; for (auto it : _path) { ss << '/' << it; } ss << ": " << ex.msg);
            _validationFailed = true;

            // Set the state to config alarm, so users can't run time machine.
            // sys.state = State::ConfigAlarm; // TODO FIXME SdB
        }

        value->group(*this);

        _path.erase(_path.begin() + (_path.size() - 1));
    }

    void Validator::item(const char* name, int32_t& value, int32_t minValue, int32_t maxValue) {
        validate(value >= minValue && value <= maxValue,
                 "Configuration value %s with value %d should be in the range [%d, %d]",
                 name,
                 value,
                 minValue,
                 maxValue);
    }
    void Validator::item(const char* name, float& value, float minValue, float maxValue) {
        validate(value >= minValue && value <= maxValue,
                 "Configuration value %s with value %.3f should be in the range [%.3f, %.3f]",
                 name,
                 value,
                 minValue,
                 maxValue);
    }
    void Validator::item(const char* name, std::vector<speedEntry>& value) {
        for (auto it : value) {
            validate(
                it.percent >= 0 && it.percent <= 100, "Speed map %s has percentage %.3f which is out of range (0%-100%).", name, it.percent);
        }
    }
    void Validator::item(const char* name, String& value, int minLength, int maxLength) {
        auto len = value.length();
        validate(len >= minLength && len <= maxLength,
                 "Configuration value %s with value '%s' should have a length between %d and %d characters.",
                 name,
                 value,
                 minLength,
                 maxLength);
    }

    void Validator::item(const char* name, int& value, const EnumItem* e) {
        validate(EnumItem::find(e, value).name != nullptr, "Enum value for key %s is not defined", name);
    }

    void Validator::finishValidation() {
        Assert(!_validationFailed, "Configuration validation failed. Please check your configuration file and the error log!");
    }
}
