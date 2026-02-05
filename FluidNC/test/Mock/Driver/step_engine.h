// Copyright (c) 2024 - Mitch Bradley
// Mock step_engine.h for Windows unit testing
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "Driver/fluidnc_gpio.h"

typedef struct step_engine {
    const char* name;

    // Prepare the engine for use
    uint32_t (*init)(uint32_t dir_delay_us, uint32_t pulse_delay_us, uint32_t frequency, bool (*fn)(void));

    // Setup the step pin
    uint32_t (*init_step_pin)(pinnum_t pin, bool inverted);

    // Set direction pin state
    void (*set_dir_pin)(pinnum_t pin, bool level);

    // Commit direction changes
    void (*finish_dir)();

    // Start step pulse
    void (*start_step)();

    // Set step pin state
    void (*set_step_pin)(pinnum_t pin, bool level);

    // Finish step pulse
    void (*finish_step)();

    // Start unstep
    bool (*start_unstep)();

    // Finish unstep
    void (*finish_unstep)();

    // Max step rate
    uint32_t (*max_pulses_per_sec)();

    // Timer control
    void (*set_timer_ticks)(uint32_t ticks);
    void (*start_timer)();
    void (*stop_timer)();

    // Link to next engine
    struct step_engine* link;
} step_engine_t;

// Mock: empty list of step engines for testing
inline step_engine_t* step_engines = nullptr;

// Mock: registration macro does nothing in tests
#define REGISTER_STEP_ENGINE(name, engine)
