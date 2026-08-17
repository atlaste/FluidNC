#include "PinOn.h"

#include "System.h"  // sys, StateName
#include "string_util.h"

#include <algorithm>

namespace Extra {
    PinOn::PinOn(const char* name) : ConfigurableModule(name) {}

    namespace {
        std::string knownStates() {
            std::string names;
            for (const auto& entry : StateName) {
                if (!names.empty()) {
                    names += " ";
                }
                names += entry.second;
            }
            return names;
        }
    }

    uint32_t PinOn::parseStates(std::string_view names) {
        uint32_t mask = 0;

        while (!names.empty()) {
            auto pos  = names.find_first_of(" \t,|");
            auto name = string_util::trim(names.substr(0, pos));
            names     = pos == std::string_view::npos ? "" : names.substr(pos + 1);
            if (name.empty()) {
                continue;
            }

            auto entry = std::find_if(
                StateName.begin(), StateName.end(), [name](const auto& e) { return string_util::equal_ignore_case(name, e.second); });
            if (entry == StateName.end()) {
                log_error("Unknown state " << name << " - expected one of " << knownStates());
                continue;
            }
            mask |= 1u << int(entry->first);
        }

        return mask;
    }

    void PinOn::init() {
        for (size_t i = 0; i < nPins; i++) {
            Pin& pin = _pins[i];
            if (pin.undefined()) {
                continue;
            }
            pin.setAttr(Pin::Attr::Output);

            auto states = string_util::trim(_states[i]);
            if (states.empty()) {
                log_info("Setting pin " << pin.name() << " to " << int(_values[i]));
                pin.write(_values[i] != 0);
                continue;
            }

            // A pin whose state names are all bad keeps a zero mask, so it stays
            // off rather than following the configured value.
            _stateMasks[i]  = parseStates(states);
            _anyStateDriven = true;
            log_info("Setting pin " << pin.name() << " on in states " << states);
        }

        if (_anyStateDriven) {
            applyStates(sys.state());
        }
    }

    void PinOn::poll() {
        if (!_anyStateDriven || sys.state() == _lastState) {
            return;
        }
        applyStates(sys.state());
    }

    void PinOn::applyStates(State state) {
        _lastState = state;

        uint32_t bit = 1u << int(state);
        for (size_t i = 0; i < nPins; i++) {
            if (_stateMasks[i]) {
                _pins[i].write((_stateMasks[i] & bit) != 0);
            }
        }
    }

    // Configuration registration
    namespace {
        ConfigurableModuleFactory::InstanceBuilder<PinOn> registration("pin_on");
    }

}
