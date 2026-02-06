// Copyright 2022 - Mitch Bradley
// Mock implementation for Windows unit testing with SoftwareGPIO integration
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "fluidnc_gpio.h"
#include <SoftwareGPIO.h>
#include "Machine/EventPin.h"  // InputPin

// Storage for ISR callbacks - converts from SoftwareGPIO's (void*, bool) to gpio's (void*)
static struct {
    void (*callback)(void*);
    void* arg;
} isr_storage[256] = {};

static void gpio_isr_wrapper(void* arg, bool value) {
    intptr_t pin = reinterpret_cast<intptr_t>(arg);
    if (pin >= 0 && pin < 256 && isr_storage[pin].callback) {
        isr_storage[pin].callback(isr_storage[pin].arg);
    }
}

// Storage for gpio event system (used by InputPin::registerEvent -> gpio_set_event)
static struct {
    void* arg;       // InputPin* stored as void*
    bool  invert;    // active-low flag
} event_storage[256] = {};

// ISR callback that fires when a pin with a registered event changes state.
// Directly calls InputPin::trigger(active) so that InputPin::get() reflects the new state.
static void gpio_event_isr_wrapper(void* cbarg, bool rising) {
    intptr_t gpio_num = reinterpret_cast<intptr_t>(cbarg);
    if (gpio_num >= 0 && gpio_num < 256 && event_storage[gpio_num].arg) {
        // 'rising' is true for a low-to-high transition.
        // If pin is inverted (active-low), active = !rising, otherwise active = rising.
        bool      active = rising != event_storage[gpio_num].invert;
        InputPin* pin    = static_cast<InputPin*>(event_storage[gpio_num].arg);
        pin->trigger(active);
    }
}

// GPIO interface - implementations that use SoftwareGPIO for testing

void gpio_write(pinnum_t pin, bool value) {
    if (pin >= 0 && pin < 256) {
        SoftwareGPIO::instance().writeOutput(pin, value);
    }
}

bool gpio_read(pinnum_t pin) {
    if (pin >= 0 && pin < 256) {
        return SoftwareGPIO::instance().read(pin);
    }
    return false;
}

void gpio_mode(pinnum_t pin, bool input, bool output, bool pullup, bool pulldown, bool opendrain) {
    if (pin >= 0 && pin < 256) {
        uint8_t mode = 0;
        if (input) mode |= INPUT;
        if (output) mode |= OUTPUT;
        if (pullup) mode |= PULLUP;
        if (pulldown) mode |= PULLDOWN;
        SoftwareGPIO::instance().setMode(pin, mode);
    }
}

void gpio_drive_strength(pinnum_t pin, uint8_t strength) {
    // Mock - no drive strength in software
}

void gpio_set_interrupt_type(pinnum_t pin, uint8_t mode) {
    // Handled by gpio_add_interrupt
}

void gpio_add_interrupt(pinnum_t pin, int8_t mode, void (*callback)(void*), void* arg) {
    if (pin >= 0 && pin < 256) {
        isr_storage[pin].callback = callback;
        isr_storage[pin].arg = arg;
        SoftwareGPIO::instance().attachISR(pin, gpio_isr_wrapper, reinterpret_cast<void*>(static_cast<intptr_t>(pin)), mode);
    }
}

void gpio_remove_interrupt(pinnum_t pin) {
    if (pin >= 0 && pin < 256) {
        SoftwareGPIO::instance().detachISR(pin);
        isr_storage[pin].callback = nullptr;
        isr_storage[pin].arg = nullptr;
    }
}

void gpio_route(pinnum_t pin, uint32_t signal) {
    // Mock - no IO matrix routing in software
}

void gpio_set_event(int32_t gpio_num, void* arg, bool invert) {
    if (gpio_num >= 0 && gpio_num < 256) {
        event_storage[gpio_num].arg    = arg;
        event_storage[gpio_num].invert = invert;
        // Register a SoftwareGPIO ISR so that setPadValue triggers InputPin::trigger
        SoftwareGPIO::instance().attachISR(
            gpio_num, gpio_event_isr_wrapper, reinterpret_cast<void*>(static_cast<intptr_t>(gpio_num)), CHANGE);
    }
}

void gpio_clear_event(int32_t gpio_num) {
    if (gpio_num >= 0 && gpio_num < 256) {
        event_storage[gpio_num].arg = nullptr;
        SoftwareGPIO::instance().detachISR(gpio_num);
    }
}

void poll_gpios() {
    // Events are delivered immediately via the SoftwareGPIO ISR chain,
    // so explicit polling is not needed in the test environment.
}
