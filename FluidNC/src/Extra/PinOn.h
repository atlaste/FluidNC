#pragma once

#include "Configuration/HandlerBase.h"
#include "Module.h"
#include "Pin.h"
#include "State.h"

#include <string>
#include <string_view>

namespace Extra {
    // Drives output pins to a fixed level.  A pin that is given a list of
    // states instead follows the machine state, being on only while the machine
    // is in one of those states, which suits indicator lamps.
    class PinOn : public ConfigurableModule {
        static const size_t nPins = 5;

        Pin         _pins[nPins];
        int32_t     _values[nPins] = { 1, 1, 1, 1, 1 };
        std::string _states[nPins];

        // Bit per State value, parsed from _states by init().  Zero means the
        // pin is not state driven, so it just holds its configured value.
        uint32_t _stateMasks[nPins] = { 0 };

        bool _anyStateDriven = false;

        // The state that the pins were last written for
        State _lastState = State::Idle;

        static uint32_t parseStates(std::string_view names);

        void applyStates(State state);

    public:
        PinOn(const char* name);

        void init() override;
        void poll() override;

        void group(Configuration::HandlerBase& handler) override {
            handler.item("pin1", _pins[0]);
            handler.item("value1", _values[0]);
            handler.item("states1", _states[0]);

            handler.item("pin2", _pins[1]);
            handler.item("value2", _values[1]);
            handler.item("states2", _states[1]);

            handler.item("pin3", _pins[2]);
            handler.item("value3", _values[2]);
            handler.item("states3", _states[2]);

            handler.item("pin4", _pins[3]);
            handler.item("value4", _values[3]);
            handler.item("states4", _states[3]);

            handler.item("pin5", _pins[4]);
            handler.item("value5", _values[4]);
            handler.item("states5", _states[4]);
        }

        ~PinOn() {}
    };
}
