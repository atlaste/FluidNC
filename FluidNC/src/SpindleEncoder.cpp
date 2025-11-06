#include "SpindleEncoder.h"

#include "Assertion.h"
#include "Logging.h"

#include <driver/gpio.h>
#include <driver/pulse_cnt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <hal/pcnt_hal.h>
#include <hal/pcnt_ll.h>
#include <soc/pcnt_periph.h>

#include <cstdint>
#include <cstring>  // memset
#include <esp_err.h>
#include <esp_rom_gpio.h>
#include <esp_sleep.h>
#include <iostream>
#include <esp_attr.h>
#include <sdkconfig.h>

// We need access to the low-level PCNT hardware structures for this:
extern "C" {
typedef struct pcnt_unit_t  pcnt_unit_t;
typedef struct pcnt_group_t pcnt_group_t;

struct pcnt_group_t {
    int                group_id;
    int                intr_priority;
    portMUX_TYPE       spinlock;
    pcnt_hal_context_t hal;
    pcnt_unit_t*       units[SOC_PCNT_UNITS_PER_GROUP];
};

struct pcnt_unit_t {
    pcnt_group_t* group;
    portMUX_TYPE  spinlock;
    int           unit_id;
    int           low_limit;
    int           high_limit;
    // ... other fields
};
}

bool IRAM_ATTR SpindleEncoder::pcnt_on_overflow(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx) {
    auto enc = static_cast<SpindleEncoder*>(user_ctx);

    if (edata->watch_point_value == 32767) {
        enc->totalCount += 32767;
    } else if (edata->watch_point_value == -32767) {
        enc->totalCount -= 32767;
    }
    return pdFALSE;
}

bool IRAM_ATTR SpindleEncoder::pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx) {
    auto enc = static_cast<SpindleEncoder*>(user_ctx);

    pcnt_unit_t*  pcnt_unit = (pcnt_unit_t*)unit;
    pcnt_group_t* group     = pcnt_unit->group;
    int           unit_id   = pcnt_unit->unit_id;

    // Disable all threshold watch points (ISR-safe)
    pcnt_ll_enable_thres_event(group->hal.dev, unit_id, 0, false);
    pcnt_ll_enable_thres_event(group->hal.dev, unit_id, 1, false);

    // TODO: Some real work here.

    // Calculate new watch point
    int output = 0;
    pcnt_unit_get_count(enc->pcnt_total, &output);
    int64_t sum       = enc->totalCount + int64_t(output);
    int     remainder = int(100 - (sum % 100));

    // Set new threshold values and enable them (ISR-safe)
    pcnt_ll_set_thres_value(group->hal.dev, unit_id, 0, remainder);
    pcnt_ll_set_thres_value(group->hal.dev, unit_id, 1, -remainder);
    pcnt_ll_enable_thres_event(group->hal.dev, unit_id, 0, true);
    pcnt_ll_enable_thres_event(group->hal.dev, unit_id, 1, true);

    // Clear the counter (ISR-safe)
    pcnt_ll_clear_count(group->hal.dev, unit_id);

    return pdFALSE;
}

void SpindleEncoder::init() {
    if (pin_a.undefined() || pin_b.undefined()) {
        log_error("Spindle encoder pins not defined, skipping initialization");
        return;
    }

    auto gpio_a = pin_a.getNative(Pin::Capabilities::Input);
    auto gpio_b = pin_b.getNative(Pin::Capabilities::Input);

    log_info("Installing encoder");
    pcnt_unit_config_t unit_config1 = { .low_limit = -32767, .high_limit = 32767, .intr_priority = 0, .flags = { .accum_count = 0 } };
    AssertOK(pcnt_new_unit(&unit_config1, &pcnt_total));

    pcnt_unit_config_t unit_config2 = { .low_limit = -32767, .high_limit = 32767, .intr_priority = 0, .flags = { .accum_count = 0 } };
    AssertOK(pcnt_new_unit(&unit_config2, &pcnt_alm));

    log_debug("Install pcnt channels using virtual GPIOs");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num  = -1,
        .level_gpio_num = -1,
        .flags = { .invert_edge_input = 0, .invert_level_input = 0, .virt_edge_io_level = 0, .virt_level_io_level = 0, .io_loop_back = 0 },
    };
    pcnt_channel_handle_t pcnt_total_chan_a = NULL;
    pcnt_channel_handle_t pcnt_alm_chan_a   = NULL;
    AssertOK(pcnt_new_channel(pcnt_total, &chan_a_config, &pcnt_total_chan_a));
    AssertOK(pcnt_new_channel(pcnt_alm, &chan_a_config, &pcnt_alm_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num  = -1,
        .level_gpio_num = -1,
        .flags = { .invert_edge_input = 0, .invert_level_input = 0, .virt_edge_io_level = 0, .virt_level_io_level = 0, .io_loop_back = 0 },
    };
    pcnt_channel_handle_t pcnt_total_chan_b = NULL;
    pcnt_channel_handle_t pcnt_alm_chan_b   = NULL;
    AssertOK(pcnt_new_channel(pcnt_total, &chan_b_config, &pcnt_total_chan_b));
    AssertOK(pcnt_new_channel(pcnt_alm, &chan_b_config, &pcnt_alm_chan_b));

    log_debug("Configure GPIOs for pcnt");
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << gpio_a) | (1ULL << gpio_b),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    AssertOK(gpio_config(&gpio_conf));

    log_debug("Connect GPIOs to PCNT units via GPIO matrix");
    esp_rom_gpio_connect_in_signal(gpio_a, pcnt_periph_signals.groups[0].units[0].channels[0].pulse_sig, false);
    esp_rom_gpio_connect_in_signal(gpio_b, pcnt_periph_signals.groups[0].units[0].channels[0].control_sig, false);
    esp_rom_gpio_connect_in_signal(gpio_b, pcnt_periph_signals.groups[0].units[0].channels[1].pulse_sig, false);
    esp_rom_gpio_connect_in_signal(gpio_a, pcnt_periph_signals.groups[0].units[0].channels[1].control_sig, false);

    esp_rom_gpio_connect_in_signal(gpio_a, pcnt_periph_signals.groups[0].units[1].channels[0].pulse_sig, false);
    esp_rom_gpio_connect_in_signal(gpio_b, pcnt_periph_signals.groups[0].units[1].channels[0].control_sig, false);
    esp_rom_gpio_connect_in_signal(gpio_b, pcnt_periph_signals.groups[0].units[1].channels[1].pulse_sig, false);
    esp_rom_gpio_connect_in_signal(gpio_a, pcnt_periph_signals.groups[0].units[1].channels[1].control_sig, false);

    log_debug("Set edge and level actions for pcnt channels");
    AssertOK(pcnt_channel_set_edge_action(pcnt_total_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    AssertOK(pcnt_channel_set_level_action(pcnt_total_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    AssertOK(pcnt_channel_set_edge_action(pcnt_total_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    AssertOK(pcnt_channel_set_level_action(pcnt_total_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    AssertOK(pcnt_channel_set_edge_action(pcnt_alm_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    AssertOK(pcnt_channel_set_level_action(pcnt_alm_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    AssertOK(pcnt_channel_set_edge_action(pcnt_alm_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    AssertOK(pcnt_channel_set_level_action(pcnt_alm_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // 32767 is the max count for 16-bit (symmetric). We need a watch point to handle overflows.
    log_debug("Add watch points and register callbacks");
    AssertOK(pcnt_unit_add_watch_point(pcnt_total, 32767));
    AssertOK(pcnt_unit_add_watch_point(pcnt_total, -32767));
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_overflow,
    };
    AssertOK(pcnt_unit_register_event_callbacks(pcnt_total, &cbs, this));

    // Initial watch points for alarm unit
    // TODO: 32767 is just an arbitrary placeholder.
    int initial_watch = 32767;
    AssertOK(pcnt_unit_add_watch_point(pcnt_alm, initial_watch));
    AssertOK(pcnt_unit_add_watch_point(pcnt_alm, -initial_watch));

    pcnt_event_callbacks_t cbs2 = {
        .on_reach = pcnt_on_reach,
    };

    // NOTE: We just put it on a queue for now. We'll need to do something more useful later.
    AssertOK(pcnt_unit_register_event_callbacks(pcnt_alm, &cbs2, this));

    log_debug("Start pcnt units");
    AssertOK(pcnt_unit_enable(pcnt_total));
    AssertOK(pcnt_unit_clear_count(pcnt_total));
    AssertOK(pcnt_unit_start(pcnt_total));

    AssertOK(pcnt_unit_enable(pcnt_alm));
    AssertOK(pcnt_unit_clear_count(pcnt_alm));
    AssertOK(pcnt_unit_start(pcnt_alm));

    // Register and then we're done.
    spindle_encoder = this;
}

void SpindleEncoder::deinit() {
    // De-init:
    pcnt_unit_stop(pcnt_alm);
    pcnt_unit_disable(pcnt_alm);

    pcnt_unit_stop(pcnt_total);
    pcnt_unit_disable(pcnt_total);
}

int64_t SpindleEncoder::getCount() {
tryAgain:
    auto before = totalCount;
    int  value;
    if (pcnt_unit_get_count(pcnt_total, &value) == ESP_OK) {
        auto after = totalCount;

        // The ISR might have updated the total count during the get count. There's no telling what
        // the right number is if it did.
        if (before != after) {
            goto tryAgain;
        }
        return before + int64_t(value);
    } else {
        Assert(false, "PCNT didn't return a workable value. Can't continue reliably.");
    }
}

bool IRAM_ATTR SpindleEncoder::validateSpeed(int32_t usecs) {
    auto spindle = ::spindle;
    if (spindle == nullptr || !spindle->speedIsValid()) {
        return true;
    }

    int32_t rpm   = int32_t(spindle->_current_speed);

    auto state = spindle->_current_state;
    if (state == SpindleState::Ccw) {
        rpm = -rpm;
    } else if (state != SpindleState::Cw) {
        return true;  // Nothing to validate.
    }

    int64_t count = getCount();
    int32_t delta = int32_t(count - lastCount);

    // At low RPM, delta might be 0 if sampled too frequently
    // Only validate if we have enough ticks (at least 20 to be meaningful)
    if (delta < 20 && delta > -20) {
        lastCount = count;
        return false;  // Not enough data to validate yet
    }

    auto minRPM = (rpm * (100 - tolerance)) / 100;
    auto maxRPM = (rpm * (100 + tolerance)) / 100;

    // Spindle is running at speed [rpm]. The encoder has CPR ticks per rev. So
    // per minute we process [rpm] * CPR * ratio pulses per minute. What we have
    // is [delta] ticks in [usecs] microseconds. The speed that matches these ticks
    // is:
    //
    // tpm = delta * 60'000'000 / usecs [ticks/minute]
    // rpm = tpm / (CPR * ratio)

    // Use int64_t for calculation to prevent overflow
    int32_t revsPerMinute = int32_t((delta * 60'000'000LL * 10'000LL) / (usecs * countPerRevolution * ratio));

    lastEncoderSpeed_ = revsPerMinute;

    lastCount = count;

    // Return true if out of tolerance (caller should handle the error outside ISR)
    return (revsPerMinute >= minRPM && revsPerMinute <= maxRPM);
}

SpindleEncoder* spindle_encoder = nullptr;

ConfigurableModuleFactory::InstanceBuilder<SpindleEncoder> __attribute__((init_priority(200))) spindle_encoder_module("spindle_encoder");
