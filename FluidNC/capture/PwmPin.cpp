// Copyright 2022 - Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
  PWM capabilities provided by the ESP32 LEDC controller via the ESP-IDF driver.
  Capture-layer implementation: stores duty values so unit tests can observe them.
*/

#include "Driver/PwmPin.h"
#include "Config.h"

std::unordered_map<pinnum_t, uint32_t> PwmPin::_dutyByGpio;

PwmPin::PwmPin(pinnum_t gpio, bool invert, uint32_t frequency) : _gpio(gpio), _frequency(frequency) {
    _period = 1000000 / frequency;
}

// cppcheck-suppress unusedFunction
void PwmPin::setDuty(uint32_t duty) {
    _dutyByGpio[_gpio] = duty;
}

uint32_t PwmPin::lastDuty(pinnum_t gpio) {
    auto it = _dutyByGpio.find(gpio);
    return (it != _dutyByGpio.end()) ? it->second : 0;
}

void PwmPin::resetAll() {
    _dutyByGpio.clear();
}

PwmPin::~PwmPin() {}
