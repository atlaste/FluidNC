#include "SpindleEncoder.h"

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

namespace {
    static bool pcnt_on_overflow(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx) {
        if (edata->watch_point_value == 32767) {
            totalCount += 32767;
        } else if (edata->watch_point_value == -32767) {
            totalCount -= 32767;
        }
        return pdFALSE;
    }

    static bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx) {
        pcnt_unit_t*  pcnt_unit = (pcnt_unit_t*)unit;
        pcnt_group_t* group     = pcnt_unit->group;
        int           unit_id   = pcnt_unit->unit_id;

        // Disable all threshold watch points (ISR-safe)
        pcnt_ll_enable_thres_event(group->hal.dev, unit_id, 0, false);
        pcnt_ll_enable_thres_event(group->hal.dev, unit_id, 1, false);

        // TODO: Some real work here.


        // Calculate new watch point
        int output = 0;
        pcnt_unit_get_count(pcnt_total, &output);
        long sum       = (totalCount + output);
        int  remainder = int(100 - (sum % 100));

        // Set new threshold values and enable them (ISR-safe)
        pcnt_ll_set_thres_value(group->hal.dev, unit_id, 0, remainder);
        pcnt_ll_set_thres_value(group->hal.dev, unit_id, 1, -remainder);
        pcnt_ll_enable_thres_event(group->hal.dev, unit_id, 0, true);
        pcnt_ll_enable_thres_event(group->hal.dev, unit_id, 1, true);

        // Clear the counter (ISR-safe)
        pcnt_ll_clear_count(group->hal.dev, unit_id);

        // TODO: We just add it to the queue; might need to do something that makes more sense...
        BaseType_t    high_task_wakeup;
        QueueHandle_t queue = (QueueHandle_t)user_ctx;
        xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
        return (high_task_wakeup == pdTRUE);
    }

}

void SpindleEncoder::init() {
    if (pin_a.undefined() || pin_b.undefined()) {
        log_error("Spindle encoder pins not defined, skipping initialization");
        return;
    }

    auto gpio_a = pin_a.getNative(Pin::Capabilities::Input);
    auto gpio_b = pin_b.getNative(Pin::Capabilities::Input);

    log_info("install pcnt units");
    pcnt_unit_config_t unit_config1 = { .low_limit = -32767, .high_limit = 32767, .intr_priority = 0, .flags = { .accum_count = 0 } };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config1, &pcnt_total));

    pcnt_unit_config_t unit_config2 = { .low_limit = -32767, .high_limit = 32767, .intr_priority = 0, .flags = { .accum_count = 0 } };
    pcnt_unit_handle_t pcnt_alm     = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config2, &pcnt_alm));

    log_info("install pcnt channels using virtual GPIOs");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num  = -1,
        .level_gpio_num = -1,
        .flags = { .invert_edge_input = 0, .invert_level_input = 0, .virt_edge_io_level = 0, .virt_level_io_level = 0, .io_loop_back = 0 },
    };
    pcnt_channel_handle_t pcnt_total_chan_a = NULL;
    pcnt_channel_handle_t pcnt_alm_chan_a   = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_total, &chan_a_config, &pcnt_total_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_alm, &chan_a_config, &pcnt_alm_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num  = -1,
        .level_gpio_num = -1,
        .flags = { .invert_edge_input = 0, .invert_level_input = 0, .virt_edge_io_level = 0, .virt_level_io_level = 0, .io_loop_back = 0 },
    };
    pcnt_channel_handle_t pcnt_total_chan_b = NULL;
    pcnt_channel_handle_t pcnt_alm_chan_b   = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_total, &chan_b_config, &pcnt_total_chan_b));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_alm, &chan_b_config, &pcnt_alm_chan_b));

    log_info("configure actual GPIOs");
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << gpio_a) | (1ULL << gpio_b),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    log_info("manually connect GPIOs to PCNT units via GPIO matrix");
    esp_rom_gpio_connect_in_signal(gpio_a, pcnt_periph_signals.groups[0].units[0].channels[0].pulse_sig, false);
    esp_rom_gpio_connect_in_signal(gpio_b, pcnt_periph_signals.groups[0].units[0].channels[0].control_sig, false);
    esp_rom_gpio_connect_in_signal(gpio_b, pcnt_periph_signals.groups[0].units[0].channels[1].pulse_sig, false);
    esp_rom_gpio_connect_in_signal(gpio_a, pcnt_periph_signals.groups[0].units[0].channels[1].control_sig, false);

    esp_rom_gpio_connect_in_signal(gpio_a, pcnt_periph_signals.groups[0].units[1].channels[0].pulse_sig, false);
    esp_rom_gpio_connect_in_signal(gpio_b, pcnt_periph_signals.groups[0].units[1].channels[0].control_sig, false);
    esp_rom_gpio_connect_in_signal(gpio_b, pcnt_periph_signals.groups[0].units[1].channels[1].pulse_sig, false);
    esp_rom_gpio_connect_in_signal(gpio_a, pcnt_periph_signals.groups[0].units[1].channels[1].control_sig, false);

    log_info("set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_total_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_total_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_total_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_total_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_alm_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_alm_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_alm_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_alm_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // 32767 is the max count for 16-bit (symmetric). We need a watch point to handle overflows. 
    log_info("add watch points and register callbacks");
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_total, 32767));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_total, -32767));
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_overflow,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_total, &cbs, nullptr));

    // Initial watch points for alarm unit
    // TODO: 32767 is just an arbitrary placeholder.
    int initial_watch = 32767;
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_alm, initial_watch));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_alm, -initial_watch));

    pcnt_event_callbacks_t cbs2 = {
        .on_reach = pcnt_on_reach,
    };

    // NOTE: We just put it on a queue for now. We'll need to do something more useful later.
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_alm, &cbs2, queue));

    log_info("start pcnt units");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_total));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_total));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_total));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_alm));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_alm));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_alm));

}

void SpindleEncoder::deinit() {
    // De-init:
    pcnt_unit_stop(pcnt_alm);
    pcnt_unit_disable(pcnt_alm);

    pcnt_unit_stop(pcnt_total);
    pcnt_unit_disable(pcnt_total);
}
