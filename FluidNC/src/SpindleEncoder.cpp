#include "SpindleEncoder.h"

#include "Assertion.h"
#include "Logging.h"
#include "Scheduler/ISchedulable.h"
#include "Scheduler/Timer.h"
#include "MotionControl.h"
#include "State.h"

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

typedef struct {
    pcnt_ll_watch_event_id_t event_id;           // event type
    int                      watch_point_value;  // value to be watched
} pcnt_watch_point_t;

struct pcnt_unit_t {
    pcnt_group_t*      group;                  // which group the pcnt unit belongs to
    portMUX_TYPE       spinlock;               // Spinlock, stop one unit from accessing different parts of a same register concurrently
    int                unit_id;                // allocated unit numerical ID
    int                low_limit;              // low limit value
    int                high_limit;             // high limit value
    int                clear_signal_gpio_num;  // which gpio clear signal input
    int                accum_value;            // accumulated count value
    pcnt_chan_t*       channels[SOC_PCNT_CHANNELS_PER_UNIT];  // array of PCNT channels
    pcnt_watch_point_t watchers[PCNT_LL_WATCH_EVENT_MAX];     // array of PCNT watchers
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

    // CRITICAL: Disable thresholds and clear counter FIRST to prevent overflow
    // The counter keeps running during ISR, so we need to reset it before processing
    pcnt_ll_enable_thres_event(group->hal.dev, unit_id, 0, false);
    pcnt_ll_enable_thres_event(group->hal.dev, unit_id, 1, false);

    // Update encoder counts remaining using watch_point_value
    // (we update watchers[] when setting thresholds, so this is accurate)
    auto ecr = enc->encoder_counts_remaining - edata->watch_point_value;

    if (ecr != 0 && (enc->encoder_counts_remaining > 100 || enc->encoder_counts_remaining < -100)) {
        ets_printf("pcnt_on_reach: countdown wp=%d ecr=%d->%d\n",
                   (int)edata->watch_point_value, (int)enc->encoder_counts_remaining, (int)ecr);
    }

    enc->encoder_counts_remaining = ecr;

    // Check if we've reached target (ecr == 0)
    // Since we use watch_point_value (which we control), ecr hits exactly 0
    // NOTE: Should be == 0 because direction matters!
    if (ecr == 0) {
        // Target reached - execute step callback
        auto cb = enc->encoder_callback;
        bool cb_result = cb ? cb() : false;
        ets_printf("pcnt_on_reach: ecr=0 wp=%d cb=%d cb_ret=%d step_fp=%d\n",
                   (int)edata->watch_point_value, (int)(cb != nullptr), (int)cb_result, (int)enc->current_step_fp);
        if (!cb || !cb_result || enc->current_step_fp == 0) {
            ets_printf("pcnt_on_reach: STOPPING alarm (cb=%d ret=%d fp=%d)\n",
                       (int)(cb != nullptr), (int)cb_result, (int)enc->current_step_fp);
            pcnt_ll_clear_count(group->hal.dev, unit_id);
            pcnt_unit_stop(enc->pcnt_alm);
            return pdFALSE;
        }

        // Calculate next step target using fixed-point remainder accumulation
        // counts_fp is scaled by 1024, so divide to get integer encoder counts
        int32_t counts_fp = enc->current_step_fp_remainder + enc->current_step_fp;
        int32_t value = counts_fp / 1024;
        int32_t remainder = counts_fp - (value * 1024);  // Remainder keeps sign

        // Ensure we always move at least one count
        if (value == 0) {
            value = (enc->current_step_fp < 0) ? -1 : 1;
        }

        // Update the remainder for next step
        enc->current_step_fp_remainder = remainder;

        // Set next threshold
        ecr = value;
        enc->encoder_counts_remaining = ecr;
    }

    // Set up threshold for next interrupt
    // Use absolute value for threshold, set both +/- to catch either direction
    int32_t next_threshold = (ecr < 0) ? -ecr : ecr;
    if (next_threshold > 16383) {
        next_threshold = 16383;
    }
    if (next_threshold < 1) {
        next_threshold = 1;
    }

    // Update ESP-IDF driver's watchers so next ISR gets correct watch_point_value
    // ESP32/S3: watchers[0] = THRES1 event, watchers[1] = THRES0 event
    pcnt_unit->watchers[PCNT_LL_WATCH_EVENT_THRES0].watch_point_value = next_threshold;   // THRES0 = positive
    pcnt_unit->watchers[PCNT_LL_WATCH_EVENT_THRES1].watch_point_value = -next_threshold;  // THRES1 = negative

    // Set hardware thresholds and enable events
    // Counter was already cleared at ISR start, now counting from 0
    pcnt_ll_set_thres_value(group->hal.dev, unit_id, 0, next_threshold);
    pcnt_ll_set_thres_value(group->hal.dev, unit_id, 1, -next_threshold);
    pcnt_ll_enable_thres_event(group->hal.dev, unit_id, 0, true);
    pcnt_ll_enable_thres_event(group->hal.dev, unit_id, 1, true);

    // Clear counter AFTER setting thresholds - starts fresh from 0
    pcnt_ll_clear_count(group->hal.dev, unit_id);

    // Debug (commented out to avoid WDT timeout)
    // ets_printf("next_thresh=%d ecr=%d\n", next_threshold, ecr);

    return pdFALSE;
}

Scheduler::Schedulable<void> SpindleEncoder::monitorSpeed() {
    using namespace Scheduler;

    int64_t lastCheckTime = Timer::currentTime();

    while (true) {
        // Yield for 100ms between checks
        co_delay_msec(100);

        auto spindle = ::spindle;
        if (!spindle || !spindle->speedIsValid()) {
            // No spindle or speed not set/ramping, nothing to validate
            // speedIsValid() returns false during ramp-up
            lastCheckTime = Timer::currentTime();
            lastCountIdle = getCount();
            continue;
        }

        {
            int64_t currentTime = Timer::currentTime();
            int32_t deltaUs     = int32_t(currentTime - lastCheckTime);
            lastCheckTime       = currentTime;

            if (!validateSpeed(deltaUs, false)) {
                log_error("Spindle encoder: speed out of tolerance");
                // mc_critical(ExecAlarm::SpindleControl);
                // TODO FIXME: Figure out a good way to do this.
            }
        }
    }
}

void SpindleEncoder::init() {
    if (pin_a.undefined() || pin_b.undefined()) {
        log_error("Spindle encoder pins not defined, skipping initialization");
        return;
    }

    auto gpio_a = pin_a.getNative(Pin::Capabilities::Input);
    auto gpio_b = pin_b.getNative(Pin::Capabilities::Input);

    log_info("Installing encoder on gpio " << int(gpio_a) << " and " << int(gpio_b));
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
    AssertOK(pcnt_unit_register_event_callbacks(pcnt_alm, &cbs2, this));

    log_debug("Start pcnt units");
    AssertOK(pcnt_unit_enable(pcnt_total));
    AssertOK(pcnt_unit_clear_count(pcnt_total));
    AssertOK(pcnt_unit_start(pcnt_total));

    AssertOK(pcnt_unit_enable(pcnt_alm));
    AssertOK(pcnt_unit_clear_count(pcnt_alm));
    // AssertOK(pcnt_unit_start(pcnt_alm));

    // Register and then we're done.
    spindle_encoder = this;

    // Schedule the monitoring coroutine for idle-time validation
    monitorTask_ = Scheduler::schedule(monitorSpeed());
    log_info("Spindle encoder monitoring task scheduled");
}

void SpindleEncoder::registerStepCallback(encoder_step_callback_t callback) {
    encoder_callback = callback;
}

void SpindleEncoder::unregisterStepCallback() {
    encoder_callback         = nullptr;
    encoder_counts_remaining = 0;
}

void IRAM_ATTR SpindleEncoder::armAlarm(int64_t targetCount) {
    int output = 0;
    pcnt_unit_get_count(pcnt_total, &output);
    int64_t sum = getCount();

    // Store the last total count as the starting point
    auto alarmValue          = int32_t(targetCount - sum);
    encoder_counts_remaining = alarmValue;

    // Get the PCNT unit internals for low-level access
    pcnt_unit_t*  pcnt_unit = (pcnt_unit_t*)pcnt_alm;
    pcnt_group_t* group     = pcnt_unit->group;
    int           unit_id   = pcnt_unit->unit_id;

    // Stop unit first before reconfiguring thresholds
    pcnt_unit_stop(pcnt_alm);

    // Clear the counter
    pcnt_ll_clear_count(group->hal.dev, unit_id);

    // Clear any pending interrupts
    pcnt_ll_clear_intr_status(group->hal.dev, (1 << unit_id));

    // CRITICAL: Disable limit events - we only want threshold events
    // Limit events fire at ±32767 and cause the overflow behavior we're seeing
    // pcnt_ll_enable_high_limit_event(group->hal.dev, unit_id, false);
    // pcnt_ll_enable_low_limit_event(group->hal.dev, unit_id, false);

    // Set up initial threshold using alarmValue (set by setStepAlarmValue)
    // This ensures the ISR fires after alarmValue counts instead of waiting for 32767
    // Use absolute value - we set both +/- thresholds for bidirectional
    int32_t initial_threshold = (alarmValue < 0) ? -alarmValue : alarmValue;
    if (initial_threshold > 16383) {
        initial_threshold = 16383;
    }
    if (initial_threshold < 1) {
        initial_threshold = 1;
    }

    // Update the ESP-IDF driver's internal watch point tracking so the callback fires
    // ESP32/S3: watchers[0] = THRES1 event, watchers[1] = THRES0 event
    // We set: THRES0 = positive threshold, THRES1 = negative threshold
    pcnt_unit->watchers[PCNT_LL_WATCH_EVENT_THRES0].watch_point_value = initial_threshold;   // THRES0 = positive
    pcnt_unit->watchers[PCNT_LL_WATCH_EVENT_THRES1].watch_point_value = -initial_threshold;  // THRES1 = negative

    pcnt_ll_set_thres_value(group->hal.dev, unit_id, 0, initial_threshold);
    pcnt_ll_set_thres_value(group->hal.dev, unit_id, 1, -initial_threshold);
    pcnt_ll_enable_thres_event(group->hal.dev, unit_id, 0, true);
    pcnt_ll_enable_thres_event(group->hal.dev, unit_id, 1, true);

    // Clear counter AFTER setting thresholds - ensures we start from 0
    pcnt_ll_clear_count(group->hal.dev, unit_id);

    // Start the ALM counter
    AssertOK(pcnt_unit_start(pcnt_alm));

    if (alarmValue > 100 || alarmValue < -100) {
        ets_printf("armAlarm: target=%lld sum=%lld delta=%d thresh=%d\n",
                   (long long)targetCount, (long long)sum, alarmValue, initial_threshold);
    }
}

void IRAM_ATTR SpindleEncoder::startStepCallback(int64_t target) {
    armAlarm(target);
}

void IRAM_ATTR SpindleEncoder::stopStepCallback() {
    pcnt_unit_stop(pcnt_alm);
    encoder_counts_remaining = 0;
}

int64_t IRAM_ATTR SpindleEncoder::setStepAlarmValue(int32_t counts_fp) {
    // counts_fp already carries the correct sign via the gear ratio from countsPerStep().
    // Negative ratio → negative counts_fp → targets move in the negative counting direction.
    int32_t value = counts_fp / 1024;
    int32_t remainder = counts_fp - (value * 1024);

    if (value == 0) {
        value = (counts_fp < 0) ? -1 : 1;
    }

    this->current_step_fp_remainder = remainder;
    this->current_step_fp = counts_fp;

    auto count = getCount();
    ets_printf("setStepAlarm: fp=%d val=%d count=%lld target=%lld\n",
               counts_fp, value, (long long)count, (long long)(count + value));
    return count + value;
}

int64_t IRAM_ATTR SpindleEncoder::setIndexAlarm() {
    auto count     = getCount();
    auto remainder = count % countPerRevolution;
    auto base      = count - remainder;  // Most recent revolution boundary we crossed
    auto dir       = countDirection_;

    this->current_step_fp_remainder = 0;

    // Next revolution boundary in the counting direction
    auto target = base + dir * countPerRevolution;
    ets_printf("setIndexAlarm: count=%lld CPR=%d rem=%lld dir=%d target=%lld\n",
               (long long)count, (int)countPerRevolution, (long long)remainder, (int)dir, (long long)target);
    return target;
}

int64_t IRAM_ATTR SpindleEncoder::setCountAlarm(int32_t target_count) {
    auto count     = getCount();
    auto remainder = count % countPerRevolution;
    auto dir       = countDirection_;
    auto base      = count - remainder;

    this->current_step_fp_remainder = 0;

    // Offset from base in counting direction; wrap by one revolution if needed
    auto result = base + dir * target_count;
    if ((dir > 0 && result <= count) || (dir < 0 && result >= count)) {
        result += dir * countPerRevolution;
    }
    return result;
}

void SpindleEncoder::deinit() {
    // Unschedule the monitoring task
    if (monitorTask_ && Scheduler::slowScheduler) {
        Scheduler::slowScheduler->unschedule(monitorTask_);
        monitorTask_ = nullptr;
        log_info("Spindle encoder monitoring task unscheduled");
    }

    // De-init hardware:
    pcnt_unit_stop(pcnt_alm);
    pcnt_unit_disable(pcnt_alm);

    pcnt_unit_stop(pcnt_total);
    pcnt_unit_disable(pcnt_total);
}

int64_t IRAM_ATTR SpindleEncoder::getCount() {
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

bool IRAM_ATTR SpindleEncoder::validateSpeed(int32_t usecs, bool fromISR) {
    auto     spindle      = ::spindle;
    int64_t& lastCountRef = fromISR ? lastCount : lastCountIdle;
    int64_t  count        = getCount();

    if (spindle == nullptr || !spindle->speedIsValid()) {
        // Update the appropriate lastCount; otherwise the first reading will be off.
        lastCountRef = count;
        return true;
    }

    int32_t rpm = int32_t(spindle->_current_speed);

    auto state = spindle->_current_state;
    if (state == SpindleState::Ccw) {
        rpm = -rpm;
    } else if (state != SpindleState::Cw) {
        return true;  // Nothing to validate.
    }

    // Use separate lastCount tracking for ISR vs idle coroutine to avoid race conditions
    int32_t delta = int32_t(count - lastCountRef);

    // Track counting direction for alarm computations (setIndexAlarm, setStepAlarmValue)
    if (delta > 20) {
        countDirection_ = 1;
    } else if (delta < -20) {
        countDirection_ = -1;
    }

    // At low RPM, delta might be 0 if sampled too frequently
    // Only validate if we have enough ticks (at least 20 to be meaningful)
    if (delta < 20 && delta > -20) {
        // Don't update lastCount - let ticks accumulate for next measurement -- BUT we should update usecs as well then...
        lastCountRef = count;
        return true;  // Not enough data to validate yet
    }

    auto minRPM = (rpm * (100 - tolerance)) / 100;
    auto maxRPM = (rpm * (100 + tolerance)) / 100;

    if (minRPM > maxRPM) {
        auto tmp = maxRPM;
        maxRPM   = minRPM;
        minRPM   = tmp;
    }

    // Spindle is running at speed [rpm]. The encoder has CPR ticks per rev. So
    // per minute we process [rpm] * CPR * ratio pulses per minute. What we have
    // is [delta] ticks in [usecs] microseconds. The speed that matches these ticks
    // is:
    //
    // tpm = delta * 60'000'000 / usecs [ticks/minute]
    // rpm = tpm / (CPR * ratio)

    // Use int64_t for calculation to prevent overflow
    // CRITICAL: Cast denominator to int64_t to prevent 32-bit overflow
    // (usecs * countPerRevolution * ratio can exceed 2^31)
    int64_t numerator     = delta * 60'000'000LL * 10'000LL;
    int64_t denominator   = int64_t(usecs) * countPerRevolution * ratio;
    int32_t revsPerMinute = int32_t(numerator / denominator);

    // Once every 10 seconds:
    // 
    // static int64_t lastLog = 0;
    // if (esp_timer_get_time() - lastLog > 10000000) {
    //     lastLog = esp_timer_get_time();
    //     log_verbose("ValidateSpeed: delta=" << delta << ", usecs=" << usecs << ", CPR=" << countPerRevolution << ", ratio=" << ratio
    //                                         << ", numerator=" << numerator << ", denominator=" << denominator << ", rpm_calc=" << revsPerMinute
    //                                         << ", target_rpm=" << rpm << ", minRPM=" << minRPM << ", maxRPM=" << maxRPM
    //                                         << ", fromISR=" << fromISR);
    // }
    
    lastEncoderSpeed_ = revsPerMinute;
    lastCountRef      = count;  // Update the appropriate lastCount

    // Return true if out of tolerance (caller should handle the error outside ISR)
    if (revsPerMinute >= minRPM && revsPerMinute <= maxRPM) {
        errorCount = 0;
        return true;
    } else {
        ++errorCount;
        if (errorCount >= allowedErrors) {
            if (!fromISR) {
                log_warn("ValidateSpeed errors: delta=" << delta << ", usecs=" << usecs << ", CPR=" << countPerRevolution
                                                        << ", ratio=" << ratio << ", numerator=" << numerator
                                                        << ", denominator=" << denominator << ", rpm_calc=" << revsPerMinute
                                                        << ", target_rpm=" << rpm << ", minRPM=" << minRPM << ", maxRPM=" << maxRPM);
            }
            return false;
        } else {
            log_verbose("Ignoring spindle speed error; a few errors are allowed.");
            return true;
        }
    }
}

SpindleEncoder* spindle_encoder = nullptr;

ConfigurableModuleFactory::InstanceBuilder<SpindleEncoder> __attribute__((init_priority(200))) spindle_encoder_module("spindle_encoder");
