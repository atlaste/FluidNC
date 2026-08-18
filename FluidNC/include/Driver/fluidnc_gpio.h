// Copyright 2022 - Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
typedef int8_t pinnum_t;
#define INVALID_PINNUM -1

// GPIO interface

void gpio_write(pinnum_t pin, bool value);
bool gpio_read(pinnum_t pin);
void gpio_mode(pinnum_t pin, bool input, bool output, bool pullup, bool pulldown, bool opendrain);
void gpio_drive_strength(pinnum_t pin, uint8_t strength);
void gpio_set_interrupt_type(pinnum_t pin, uint8_t mode);
void gpio_add_interrupt(pinnum_t pin, int8_t mode, void (*callback)(void*), void* arg);
void gpio_remove_interrupt(pinnum_t pin);
void gpio_route(pinnum_t pin, uint32_t signal);

// True when the chip has committed the pad to something outside FluidNC's pin system, so that
// reconfiguring it would break a peripheral that nothing in the configuration mentions.  Peripherals
// FluidNC sets up itself do not need this: those pins are claimed through the pin system and are
// already visible as taken.
bool gpio_reserved_by_platform(pinnum_t pin);

void gpio_set_event(int32_t gpio_num, void* arg, bool invert);
void gpio_clear_event(int32_t gpio_num);
void poll_gpios();

// Input debouncing.  A level must be seen on "samples" consecutive samples,
// taken every "sample_us" microseconds, before an event is sent.  Passing
// sample_us == 0 disables debouncing and restores unfiltered polling.
// "samples" is rounded up to a power of two, at most 8.  Returns false if the
// requested filtering could not be started, in which case polling is
// unfiltered.
bool gpio_debounce_config(uint32_t sample_us, uint32_t samples);

// Number of times a partially accumulated level change was abandoned because
// the input went back to its previous state, i.e. the count of rejected
// glitches.  A steadily climbing count means the input is picking up noise.
uint32_t gpio_glitch_count(int32_t gpio_num);
uint32_t gpio_glitch_total(void);
void     gpio_glitch_reset(void);

#ifdef __cplusplus
}
#endif
