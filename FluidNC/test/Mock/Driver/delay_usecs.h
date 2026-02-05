// Mock delay_usecs.h for Windows unit testing
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
// Note: Implementations are in FluidNC/capture/timing.cpp

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t ticks_per_us;

void timing_init();
void spinUntil(int32_t endTicks);

// Blocking delay for very short time intervals
void delay_us(int32_t us);

int32_t usToCpuTicks(int32_t us);
int32_t usToEndTicks(int32_t us);
int32_t getCpuTicks();

#ifdef __cplusplus
}
#endif
