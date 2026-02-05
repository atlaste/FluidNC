// Copyright 2022 - Mitch Bradley
// Mock implementation for Windows unit testing with SoftwareGPIO integration
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "fluidnc_gpio.h"
#include <SoftwareGPIO.h>

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
    // Mock - no event support
}

void gpio_clear_event(int32_t gpio_num) {
    // Mock - no event support
}

void poll_gpios() {
    // Mock - no polling needed in software
}
