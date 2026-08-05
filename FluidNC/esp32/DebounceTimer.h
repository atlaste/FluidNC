// Copyright 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Periodic timer for the GPIO input sampler, on general purpose timer group 0 /
// timer 1.  Group 0 / timer 0 belongs to the step engine and group 1 belongs to
// the performance profiler, so timer 1 of group 0 is the one that is free.
//
// The callback is invoked directly from the timer interrupt, at a priority below
// the step timer so that sampling can never delay a step pulse.  It must
// therefore be in IRAM and must only call things that are safe from an ISR.
//
// May be called more than once to change the period.  Returns false if the
// interrupt could not be allocated, in which case no sampling happens.
bool debounceTimerInit(uint32_t period_us, void (*fn)(void));

void debounceTimerStop(void);

#ifdef __cplusplus
}
#endif
