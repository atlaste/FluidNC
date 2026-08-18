// Copyright 2022 - Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"
#include "Uart.h"
#include "Protocol.h"
#include "Driver/fluidnc_gpio.h"
#include "Pin.h"

#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include "rom/gpio.h"  // gpio_matrix_*
#include "DebounceTimer.h"

#include <freertos/FreeRTOS.h>  // portMUX_TYPE, portENTER_CRITICAL_SAFE
#include <freertos/task.h>      // xTaskGetTickCount()

static gpio_dev_t* _gpio_dev = GPIO_HAL_GET_HW(GPIO_PORT_0);

void IRAM_ATTR gpio_write(pinnum_t pin, bool value) {
    gpio_ll_set_level(_gpio_dev, (gpio_num_t)pin, (uint32_t)value);
}
bool IRAM_ATTR gpio_read(pinnum_t pin) {
    return gpio_ll_get_level(_gpio_dev, (gpio_num_t)pin);
}

void gpio_mode(pinnum_t pin, bool input, bool output, bool pullup, bool pulldown, bool opendrain) {
    gpio_config_t conf {};
    conf.pin_bit_mask = (1ULL << pin);
    conf.intr_type    = GPIO_INTR_DISABLE;

    if (input) {
        conf.mode = (gpio_mode_t)((int)conf.mode | GPIO_MODE_DEF_INPUT);
    }
    if (output) {
        conf.mode = (gpio_mode_t)((int)conf.mode | GPIO_MODE_DEF_OUTPUT);
    }
    if (pullup) {
        conf.pull_up_en = GPIO_PULLUP_ENABLE;
    }
    if (pulldown) {
        conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    }
    if (opendrain) {
        conf.mode = (gpio_mode_t)((int)conf.mode | GPIO_MODE_DEF_OD);
    }
    gpio_config(&conf);
}
void gpio_drive_strength(pinnum_t pin, uint8_t strength) {
    gpio_set_drive_capability((gpio_num_t)pin, (gpio_drive_cap_t)strength);
}
#if 0
void gpio_add_interrupt(pinnum_t pin, uint8_t mode, void (*callback)(void*), void* arg) {
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);  // Will return an err if already called

    gpio_num_t gpio = (gpio_num_t)pin;
    gpio_isr_handler_add(gpio, callback, arg);

    //FIX interrupts on peripherals outputs (eg. LEDC,...)
    //Enable input in GPIO register
    gpio_hal_context_t gpiohal;
    gpiohal.dev = GPIO_LL_GET_HW(GPIO_PORT_0);
    gpio_hal_input_enable(&gpiohal, gpio);
}
void gpio_remove_interrupt(pinnum_t pin) {
    gpio_num_t gpio = (gpio_num_t)pin;
    gpio_isr_handler_remove(gpio);  //remove handle and disable isr for pin
    gpio_set_intr_type(gpio, GPIO_INTR_DISABLE);
}
#endif
void gpio_route(pinnum_t pin, uint32_t signal) {
    if (pin == INVALID_PINNUM) {
        return;
    }
    gpio_num_t gpio = (gpio_num_t)pin;
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction(gpio, (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
    gpio_matrix_out(gpio, signal, 0, 0);
}

typedef uint64_t gpio_mask_t;

static volatile gpio_mask_t gpios_inverted = 0;  // GPIOs that are active low
static volatile gpio_mask_t gpios_interest = 0;  // GPIOs with an action
static volatile gpio_mask_t gpios_current  = 0;  // The last GPIO action events that were sent

static int32_t gpio_next_event_ticks[MAX_N_GPIO + 1] = { 0 };
static int32_t gpio_deltat_ticks[MAX_N_GPIO + 1]     = { 0 };

// Do not send events for changes that occur too soon
static void gpio_set_rate_limit(int32_t gpio_num, uint32_t ms) {
    gpio_deltat_ticks[gpio_num] = pdMS_TO_TICKS(ms);
}

// get_gpios() and gpio_mask() are reached from the debounce sampler, which runs in an interrupt
// that stays enabled while the flash cache is off, so they have to live in IRAM.  They are small
// enough that the compiler usually inlines them, but "usually" is not good enough here: at -Os a
// helper with more than one caller gets emitted as a real function, and a call into flash with the
// cache disabled panics with "Cache disabled but cached memory region accessed".
static inline gpio_mask_t IRAM_ATTR get_gpios() {
    return ((((uint64_t)REG_READ(GPIO_IN1_REG)) << 32) | REG_READ(GPIO_IN_REG)) ^ gpios_inverted;
}
static gpio_mask_t IRAM_ATTR gpio_mask(int32_t gpio_num) {
    return 1ULL << gpio_num;
}
static inline bool gpio_is_active(int32_t gpio_num) {
    return get_gpios() & gpio_mask(gpio_num);
}
static void gpios_update(volatile gpio_mask_t& gpios, int32_t gpio_num, bool active) {
    if (active) {
        gpios |= gpio_mask(gpio_num);
    } else {
        gpios &= ~gpio_mask(gpio_num);
    }
}

// ---------------------------------------------------------------------------
// Input debouncing
//
// VFD switching noise couples sub-microsecond spikes into opto-isolated
// endstop, probe and input lines.  A periodic sampler filters every GPIO that
// has a registered event: a level must be observed on a fixed number of
// consecutive samples before it is believed.  Because both the sample period
// and the required count are fixed, the delay from a real edge to the event is
// a constant, which keeps the machine position at which an input trips
// repeatable.  Only the phase of the real edge within one sample period
// remains uncertain, so the sample period sets the position jitter.
//
// That argument only holds if the samples really are equally spaced, so the
// sampler runs from a hardware timer interrupt rather than a task; see
// DebounceTimer.h.  A sample delayed by the scheduler would silently widen the
// confirmation window and with it the position spread.  The sampler only
// filters and never dispatches, which is what keeps it short enough to belong
// in an ISR; poll_gpios turns the confirmed changes into events.
//
// The counters are packed four bits per GPIO: GPIO p uses nibble (p / 4) of
// word (p % 4).  Shifting the raw GPIO word right by the word index and
// masking with 0x1111... leaves each surviving bit already sitting on a nibble
// boundary, so the same value doubles as the addend that increments all 16
// lanes of a word in a single add.  Rounding the required count to a power of
// two makes the threshold test a single mask as well, so a full pass over all
// 64 possible GPIOs costs about thirty branchless operations.
// ---------------------------------------------------------------------------

static const int      NIBBLE_LANES = 4;
static const uint64_t NIBBLE_LSB   = 0x1111111111111111ULL;

// One entry per bit of gpio_mask_t rather than per real GPIO, so an index
// derived from the mask can never run off the end.
static const int32_t N_MASK_BITS = 64;

static uint64_t             debounce_count[NIBBLE_LANES] = { 0 };
static uint32_t             debounce_threshold_bit       = 3;  // log2 of the required sample count
static volatile gpio_mask_t gpios_debounced              = 0;  // Filtered levels, owned by the sampler
static volatile bool        debounce_running             = false;

// gpio_mask_t is wider than a machine word, so reads and writes of the filtered
// levels are not atomic.  The sampler runs in interrupt context, which means a
// torn read elsewhere could momentarily invent a level change and dispatch an
// event for it.  The guarded regions are only a few instructions long, and the
// _SAFE variants disable interrupts on the current core, so the sampler cannot
// deadlock against a holder running on that same core.
static portMUX_TYPE gpios_debounced_mux = portMUX_INITIALIZER_UNLOCKED;

// In IRAM because the sampler reads the levels; see get_gpios() above.
static inline gpio_mask_t IRAM_ATTR gpios_debounced_get() {
    portENTER_CRITICAL_SAFE(&gpios_debounced_mux);
    const gpio_mask_t levels = gpios_debounced;
    portEXIT_CRITICAL_SAFE(&gpios_debounced_mux);
    return levels;
}

static uint32_t gpio_glitches[N_MASK_BITS] = { 0 };
static uint32_t gpio_glitches_total        = 0;

static void IRAM_ATTR gpio_debounce_sample(void) {
    const gpio_mask_t raw  = get_gpios();
    const gpio_mask_t diff = (raw ^ gpios_debounced_get()) & gpios_interest;

    const uint64_t threshold = NIBBLE_LSB << debounce_threshold_bit;

    gpio_mask_t settled = 0;
    gpio_mask_t aborted = 0;

    for (int lane = 0; lane < NIBBLE_LANES; lane++) {
        uint64_t c = debounce_count[lane];

        // One bit per GPIO of this lane, on the nibble boundary, set where the
        // raw level disagrees with the level we currently believe.
        const uint64_t advance = (diff >> lane) & NIBBLE_LSB;

        // A counter that had started accumulating but now agrees again saw a
        // glitch.  Fold each nibble down onto its low bit to find those lanes.
        uint64_t nonzero = c | (c >> 1);
        nonzero |= nonzero >> 2;
        nonzero &= NIBBLE_LSB;
        aborted |= (nonzero & ~advance) << lane;

        // Zero the lanes that agree, then increment the rest.  Multiplying by
        // 0xF spreads each nibble's low bit across the whole nibble without
        // carrying into its neighbour, because 1 * 15 still fits in four bits.
        c &= advance * 0xF;
        c += advance;

        // A power-of-two threshold is reached exactly when one known bit sets.
        const uint64_t hit = (c & threshold) >> debounce_threshold_bit;
        settled |= hit << lane;
        c &= ~(hit * 0xF);

        debounce_count[lane] = c;
    }

    // Only bits we have been counting can appear here, and a pin cannot be
    // counting until it has an interest bit, so this cannot collide with a pin
    // being registered concurrently.
    if (settled) {
        portENTER_CRITICAL_SAFE(&gpios_debounced_mux);
        gpios_debounced ^= settled;
        portEXIT_CRITICAL_SAFE(&gpios_debounced_mux);
    }

    // Normally empty, so this loop costs nothing unless an input is noisy.
    while (aborted) {
        const int32_t gpio_num = 63 - __builtin_clzll(aborted);
        ++gpio_glitches[gpio_num];
        ++gpio_glitches_total;
        aborted &= ~gpio_mask(gpio_num);
    }
}

bool gpio_debounce_config(uint32_t sample_us, uint32_t samples) {
    // Stop before touching the shared state below, so the sampler cannot fire
    // partway through the reset and count against a half-updated threshold.
    debounceTimerStop();
    debounce_running = false;

    if (sample_us == 0) {
        return true;
    }

    uint32_t bit = 0;
    while ((1u << bit) < samples && bit < 3) {
        ++bit;
    }
    debounce_threshold_bit = bit;

    for (int lane = 0; lane < NIBBLE_LANES; lane++) {
        debounce_count[lane] = 0;
    }
    portENTER_CRITICAL_SAFE(&gpios_debounced_mux);
    gpios_debounced = get_gpios();
    portEXIT_CRITICAL_SAFE(&gpios_debounced_mux);

    debounce_running = debounceTimerInit(sample_us, gpio_debounce_sample);
    return debounce_running;
}

uint32_t gpio_glitch_count(int32_t gpio_num) {
    return (gpio_num >= 0 && gpio_num < N_MASK_BITS) ? gpio_glitches[gpio_num] : 0;
}
uint32_t gpio_glitch_total(void) {
    return gpio_glitches_total;
}
void gpio_glitch_reset(void) {
    for (int32_t i = 0; i < N_MASK_BITS; i++) {
        gpio_glitches[i] = 0;
    }
    gpio_glitches_total = 0;
}

static void* gpioArgs[MAX_N_GPIO + 1];

void gpio_set_event(int32_t gpio_num, void* arg, bool invert) {
    gpioArgs[gpio_num] = arg;

    gpios_update(gpios_interest, gpio_num, true);
    gpios_update(gpios_inverted, gpio_num, invert);
    gpio_set_rate_limit(gpio_num, 5);
    auto active = gpio_is_active(gpio_num);

    // The sampler must start from the real level, otherwise it would spend the
    // first few samples confirming a state it never actually saw.
    portENTER_CRITICAL_SAFE(&gpios_debounced_mux);
    gpios_update(gpios_debounced, gpio_num, active);
    portEXIT_CRITICAL_SAFE(&gpios_debounced_mux);

    // Set current to the opposite of the current state so the first poll will send the current state
    gpios_update(gpios_current, gpio_num, !active);
}
void gpio_clear_event(int32_t gpio_num) {
    gpioArgs[gpio_num] = nullptr;
    gpios_update(gpios_interest, gpio_num, false);
}

static void gpio_send_event(int32_t gpio_num, bool active) {
    auto    end_ticks  = gpio_next_event_ticks[gpio_num];
    int32_t this_ticks = int32_t(xTaskGetTickCount());
    // The rate limiter is a crude stand-in for debouncing that lets the first
    // edge through unfiltered.  When the sampler is running it has already
    // vetted this change, and the lockout would only delay a legitimate
    // release, so skip it.
    if (debounce_running || end_ticks == 0 || ((this_ticks - end_ticks) > 0)) {
        end_ticks = this_ticks + gpio_deltat_ticks[gpio_num];
        if (end_ticks == 0) {
            end_ticks = 1;
        }
        gpio_next_event_ticks[gpio_num] = end_ticks;

        auto arg = gpioArgs[gpio_num];
        if (arg) {
            protocol_send_event_from_ISR(active ? &pinActiveEvent : &pinInactiveEvent, arg);
        }
        gpios_update(gpios_current, gpio_num, active);
    }
}

void poll_gpios() {
    // With the sampler running, it owns the filtered levels and this only has
    // to turn confirmed changes into events.  Without it, read the pins here.
    gpio_mask_t gpios_active  = debounce_running ? gpios_debounced_get() : get_gpios();
    gpio_mask_t gpios_changed = (gpios_active ^ gpios_current) & gpios_interest;

    // Process each changed GPIO. We check gpios_changed != 0 explicitly because
    // __builtin_clzll(0) is undefined behavior - the optimizer can assume it never
    // happens and turn this into an infinite loop in release builds.
    while (gpios_changed) {
        int zeros = __builtin_clzll(gpios_changed);  // Safe: gpios_changed is non-zero here
        int32_t gpio_num = 63 - zeros;
        gpio_send_event(gpio_num, gpios_active & gpio_mask(gpio_num));
        // Clear the bit we just processed
        gpios_changed &= ~gpio_mask(gpio_num);
    }
}
