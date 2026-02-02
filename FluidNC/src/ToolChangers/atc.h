// Copyright (c) 2024 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Config.h"

#include "Configuration/Configurable.h"

#include "Channel.h"
#include "Module.h"

#include <cstring>

namespace ATCs {
    class ATC : public Configuration::Configurable {
    protected:
        const char* _name;
        uint32_t    _last_tool = 0;
        bool        _error     = false;

    public:
        ATC(const char* name) : _name(name) {}

        ATC(const ATC&)            = delete;
        ATC(ATC&&)                 = delete;
        ATC& operator=(const ATC&) = delete;
        ATC& operator=(ATC&&)      = delete;

        const char* name() { return _name; }

        virtual void init() = 0;

        // ATC* _atc; // huh? That doesn't belong here.

        // Configuration handlers:
        void validate() override {}
        void afterParse() override {}
        void group(Configuration::HandlerBase& handler) override {}

        virtual void probe_notification()                                      = 0;
        virtual bool tool_change(tool_t value, bool pre_select, bool set_tool) = 0;
        
        // Returns true if the ATC handles TLO internally (e.g., after probing).
        // When true, the M6 handler will not automatically apply TLO from the tool table.
        virtual bool handles_tlo() { return false; }
        
        virtual void save_atc_data(std::vector<uint8_t>& buffer) { 
            auto size = buffer.size();
            buffer.resize(buffer.size() + 4);
            memcpy(buffer.data() + size, &_last_tool, 4);

        }
        virtual void restore_atc_data(const std::vector<uint8_t>& buffer, size_t& index) { 
            memcpy(&_last_tool, buffer.data() + index, 4);
            index += 4;
        }

        virtual ~ATC() = default;
    };

    using ATCFactory = Configuration::GenericFactory<ATC>;
}
