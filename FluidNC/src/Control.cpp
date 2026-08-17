// Copyright (c) 2021 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Control.h"

#include "Protocol.h"        // *Event
#include "Machine/Macros.h"  // macro0Event

// Numbered variants of the pins that a machine plausibly has more than one of.  With pin
// extenders it is common to have far more fault sources than a single GPIO budget could
// ever cover -- drive faults, contactor feedback, air pressure, thermal switches -- and
// capturing all of them is worth more than saving a few objects here.
static const int numFaultPins      = 16;
static const int numEstopPins      = 4;
static const int numSafetyDoorPins = 4;

void Control::addNumbered(const Event* event, const char* base, char letter, int count) {
    char legend[32];
    for (int i = 0; i < count; ++i) {
        snprintf(legend, sizeof(legend), "%s%d", base, i);
        _pins.push_back(new ControlPin(event, legend, letter));
    }
}

Control::Control() {
    _pins.push_back(new ControlPin(&safetyDoorEvent, "safety_door_pin", 'D'));
    _pins.push_back(new ControlPin(&rtResetEvent, "reset_pin", 'R'));
    _pins.push_back(new ControlPin(&feedHoldEvent, "feed_hold_pin", 'H'));
    _pins.push_back(new ControlPin(&cycleStartEvent, "cycle_start_pin", 'S'));
    _pins.push_back(new ControlPin(&macro0Event, "macro0_pin", '0'));
    _pins.push_back(new ControlPin(&macro1Event, "macro1_pin", '1'));
    _pins.push_back(new ControlPin(&macro2Event, "macro2_pin", '2'));
    _pins.push_back(new ControlPin(&macro3Event, "macro3_pin", '3'));
    _pins.push_back(new ControlPin(&faultPinEvent, "fault_pin", 'F'));
    _pins.push_back(new ControlPin(&faultPinEvent, "estop_pin", 'E'));
    _pins.push_back(new ControlPin(&homingButtonEvent, "homing_button_pin", 'O'));

    addNumbered(&faultPinEvent, "fault_pin", 'F', numFaultPins);
    addNumbered(&faultPinEvent, "estop_pin", 'E', numEstopPins);
    addNumbered(&safetyDoorEvent, "safety_door_pin", 'D', numSafetyDoorPins);
}

void Control::init() {
    for (auto pin : _pins) {
        pin->init();
    }
}

void Control::group(Configuration::HandlerBase& handler) {
    for (auto pin : _pins) {
        handler.item(pin->legend().c_str(), *pin);
    }
}

std::string Control::report_status() {
    // A letter appears once however many pins of that kind are active, so that a machine
    // with sixteen fault inputs does not produce a status line full of Fs.
    std::string ret = "";
    for (auto pin : _pins) {
        if (pin->get() && ret.find(pin->letter()) == std::string::npos) {
            ret += pin->letter();
        }
    }
    return ret;
}

bool Control::pins_block_unlock() {
    std::string blockers("FE");  // Fault, E-Stop block unlock and homing
    for (auto pin : _pins) {
        if (pin->get() && blockers.find(pin->letter()) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool Control::stuck() {
    for (auto pin : _pins) {
        if (pin->get()) {
            return true;
        }
    }
    return false;
}

bool Control::startup_check() {
    bool ret = false;
    for (auto pin : _pins) {
        if (pin->get()) {
            delay_ms(1000);
            if (pin->get()) {
                log_error(pin->legend() << " is active at startup");
                ret = true;
            }
        }
    }
    return ret;
}

// Returns if safety door is ajar(T) or closed(F), based on pin state.
bool Control::safety_door_ajar() {
    // Any one of the door pins being open means the door is ajar.  Pins that are not
    // configured never change from their default of inactive.
    for (auto pin : _pins) {
        if (pin->letter() == 'D' && pin->get()) {
            return true;
        }
    }
    return false;
}
