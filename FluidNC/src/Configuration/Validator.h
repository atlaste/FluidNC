// Copyright (c) 2021 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../Pin.h"
#include "HandlerBase.h"

#include <vector>

namespace Configuration {
    class Configurable;

    class Validator : public HandlerBase {
        Validator(const Validator&) = delete;
        Validator& operator=(const Validator&) = delete;

        std::vector<const char*> _path;
        bool                     _validationFailed = false;

    protected:
        void        enterSection(const char* name, Configurable* value) override;
        bool        matchesUninitialized(const char* name) override { return false; }
        HandlerType handlerType() override { return HandlerType::Validator; }

    public:
        Validator();

        void validate(bool condition, const char* error, ...);

        void item(const char* name, bool& value) override {}
        void item(const char* name, int32_t& value, int32_t minValue = 0, int32_t maxValue = INT32_MAX) override;
        void item(const char* name, float& value, float minValue = -3e38, float maxValue = 3e38) override;
        void item(const char* name, std::vector<speedEntry>& value) override;
        void item(const char* name, String& value, int minLength = 0, int maxLength = 255) override;
        void item(const char* name, int& value, const EnumItem* e) override;
        void item(const char* name, UartData& wordLength, UartParity& parity, UartStop& stopBits) override {}
        void item(const char* name, Pin& value) override {}
        void item(const char* name, IPAddress& value) override {}

        void finishValidation();
    };
}
