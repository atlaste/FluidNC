// Copyright 2026 - FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
//
// Periodic timer for the GPIO input sampler, using the timer_ll API in the same
// way StepTimer.cpp does.  The step engine claims group 0 / timer 0 without
// going through the gptimer driver, so the driver's allocator does not know that
// timer is taken and would hand it out again; group 0 / timer 1 is therefore
// named explicitly here rather than allocated.
//
// The timer counts microseconds, so the alarm value is the period in
// microseconds.  The interrupt is allocated one priority level below the step
// timer, so a step pulse always preempts a sample rather than the reverse.

#ifdef __cplusplus
extern "C" {
#endif

#include "DebounceTimer.h"

#include "hal/timer_ll.h"
#include "hal/timer_types.h"
#include "esp_intr_alloc.h"

#include <esp_idf_version.h>

#if ESP_IDF_VERSION_MAJOR < 5

#    include "soc/timer_periph.h"

static const uint32_t fTimers = 80000000;  // the frequency of ESP32 timers

static void (*debounce_isr_callback)(void) = NULL;
static bool intr_allocated                 = false;

static void IRAM_ATTR debounce_timer_isr(void* arg) {
    // esp_intr_alloc_intrstatus() takes care of filtering based on the interrupt status register
    timer_ll_clear_intr_status(&TIMERG0, TIMER_1);

    // The alarm enable clears itself when the alarm fires, so rearm it to get
    // the next period.
    timer_ll_set_alarm_enable(&TIMERG0, TIMER_1, true);

    debounce_isr_callback();
}

bool debounceTimerInit(uint32_t period_us, void (*fn)(void)) {
    debounce_isr_callback = fn;

    timer_ll_intr_disable(&TIMERG0, TIMER_1);
    timer_ll_set_counter_enable(&TIMERG0, TIMER_1, false);
    timer_ll_set_alarm_enable(&TIMERG0, TIMER_1, false);
    timer_ll_set_counter_value(&TIMERG0, TIMER_1, 0ULL);

    // APB is the reset default for the timer groups, and this version of
    // timer_ll has no setter for it.
    timer_ll_set_divider(&TIMERG0, TIMER_1, fTimers / 1000000);
    timer_ll_set_counter_increase(&TIMERG0, TIMER_1, true);
    timer_ll_set_auto_reload(&TIMERG0, TIMER_1, true);
    timer_ll_clear_intr_status(&TIMERG0, TIMER_1);
    timer_ll_set_alarm_value(&TIMERG0, TIMER_1, (uint64_t)period_us);

    if (!intr_allocated) {
        if (esp_intr_alloc_intrstatus(timer_group_periph_signals.groups[TIMER_GROUP_0].t0_irq_id + TIMER_1,
                                      ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL2,
                                      timer_ll_get_intr_status_reg(&TIMERG0),
                                      1 << TIMER_1,
                                      debounce_timer_isr,
                                      NULL,
                                      NULL) != ESP_OK) {
            return false;
        }
        intr_allocated = true;
    }

    timer_ll_intr_enable(&TIMERG0, TIMER_1);
    timer_ll_set_alarm_enable(&TIMERG0, TIMER_1, true);
    timer_ll_set_counter_enable(&TIMERG0, TIMER_1, true);
    return true;
}

void debounceTimerStop(void) {
    timer_ll_set_counter_enable(&TIMERG0, TIMER_1, false);
    timer_ll_set_alarm_enable(&TIMERG0, TIMER_1, false);
    timer_ll_intr_disable(&TIMERG0, TIMER_1);
}

#    ifdef __cplusplus
}
#    endif

#else
#    include <esp_attr.h>
#    include "esp_intr_types.h"
#    include "soc/timer_periph.h"

#    define TIMER_1 1

static const uint32_t fTimers = 80000000;  // the frequency of ESP32 timers

static void (*debounce_isr_callback)(void) = NULL;
static bool intr_allocated                 = false;

static void IRAM_ATTR debounce_timer_isr(void* arg) {
    // esp_intr_alloc_intrstatus() takes care of filtering based on the interrupt status register
    timer_ll_clear_intr_status(&TIMERG0, TIMER_LL_EVENT_ALARM(TIMER_1));

    // Auto-reload is unreliable in IDF v5, so force the counter back to 0
    timer_ll_trigger_soft_reload(&TIMERG0, TIMER_1);

    // The alarm enable clears itself when the alarm fires, so rearm it to get
    // the next period.
    timer_ll_enable_alarm(&TIMERG0, TIMER_1, true);

    debounce_isr_callback();
}

bool debounceTimerInit(uint32_t period_us, void (*fn)(void)) {
    debounce_isr_callback = fn;

    timer_ll_enable_intr(&TIMERG0, TIMER_LL_EVENT_ALARM(TIMER_1), false);
    timer_ll_enable_counter(&TIMERG0, TIMER_1, false);
    timer_ll_enable_alarm(&TIMERG0, TIMER_1, false);
    timer_ll_set_reload_value(&TIMERG0, TIMER_1, 0ULL);
    timer_ll_trigger_soft_reload(&TIMERG0, TIMER_1);

    timer_ll_set_clock_source(&TIMERG0, TIMER_1, GPTIMER_CLK_SRC_APB);
    timer_ll_set_clock_prescale(&TIMERG0, TIMER_1, fTimers / 1000000);
    timer_ll_set_count_direction(&TIMERG0, TIMER_1, gptimer_count_direction_t::GPTIMER_COUNT_UP);
    timer_ll_enable_auto_reload(&TIMERG0, TIMER_1, true);
    timer_ll_clear_intr_status(&TIMERG0, TIMER_LL_EVENT_ALARM(TIMER_1));
    timer_ll_set_alarm_value(&TIMERG0, TIMER_1, (uint64_t)period_us);

    if (!intr_allocated) {
        if (esp_intr_alloc_intrstatus(timer_group_periph_signals.groups[0].timer_irq_id[TIMER_1],
                                      ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL2,
                                      (uint32_t)timer_ll_get_intr_status_reg(&TIMERG0),
                                      TIMER_LL_EVENT_ALARM(TIMER_1),
                                      debounce_timer_isr,
                                      NULL,
                                      NULL) != ESP_OK) {
            return false;
        }
        intr_allocated = true;
    }

    timer_ll_enable_intr(&TIMERG0, TIMER_LL_EVENT_ALARM(TIMER_1), true);
    timer_ll_enable_alarm(&TIMERG0, TIMER_1, true);
    timer_ll_enable_counter(&TIMERG0, TIMER_1, true);
    return true;
}

void debounceTimerStop(void) {
    timer_ll_enable_counter(&TIMERG0, TIMER_1, false);
    timer_ll_enable_alarm(&TIMERG0, TIMER_1, false);
    timer_ll_enable_intr(&TIMERG0, TIMER_LL_EVENT_ALARM(TIMER_1), false);
}

#    ifdef __cplusplus
}
#    endif

#endif
