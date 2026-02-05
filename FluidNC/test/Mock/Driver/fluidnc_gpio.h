// Copyright 2022 - Mitch Bradley
// Mock for Windows unit testing
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
typedef int8_t pinnum_t;
#define INVALID_PINNUM -1

// GPIO interface - mocked stubs

inline void gpio_write(pinnum_t pin, bool value) {}
inline bool gpio_read(pinnum_t pin) { return false; }
inline void gpio_mode(pinnum_t pin, bool input, bool output, bool pullup, bool pulldown, bool opendrain) {}
inline void gpio_drive_strength(pinnum_t pin, uint8_t strength) {}
inline void gpio_set_interrupt_type(pinnum_t pin, uint8_t mode) {}
inline void gpio_add_interrupt(pinnum_t pin, int8_t mode, void (*callback)(void*), void* arg) {}
inline void gpio_remove_interrupt(pinnum_t pin) {}
inline void gpio_route(pinnum_t pin, uint32_t signal) {}

inline void gpio_set_event(int32_t gpio_num, void* arg, bool invert) {}
inline void gpio_clear_event(int32_t gpio_num) {}
inline void poll_gpios() {}

#ifdef __cplusplus
}
#endif
