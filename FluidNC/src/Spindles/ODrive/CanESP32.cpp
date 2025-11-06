// Copyright (c) 2020 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "CanESP32.h"

#include <driver/gpio.h>
#include <driver/twai.h>

#include "Logging.h"

namespace Spindles::ODrive {
    bool CanESP32::send(uint32_t id, uint8_t length, const uint8_t* data) {
        // Configure message to transmit
        twai_message_t message;
        memset(&message, 0, sizeof(message));

        bool rtr = data == nullptr;
        if (id & 0x80000000) {
            // Message type and format settings
            message.extd             = uint32_t(1);                // Standard vs extended format
            message.rtr              = uint32_t(rtr ? 1 : 0);      // Data vs RTR frame
            message.ss               = uint32_t(0);                // Whether the message is single shot (i.e., does not repeat on error)
            message.self             = uint32_t(0);                // Whether the message is a self reception request (loopback)
            message.dlc_non_comp     = uint32_t(0);                // DLC is less than 8
            message.identifier       = uint32_t(id & 0x1fffffff);  // Message ID and payload
            message.data_length_code = length;
            memcpy(message.data, data, length);
        } else {
            // Message type and format settings
            message.extd             = uint32_t(0);            // Standard vs extended format
            message.rtr              = uint32_t(rtr ? 1 : 0);  // Data vs RTR frame
            message.ss               = uint32_t(0);            // Whether the message is single shot (i.e., does not repeat on error)
            message.self             = uint32_t(0);            // Whether the message is a self reception request (loopback)
            message.dlc_non_comp     = uint32_t(0);            // DLC is less than 8
            message.identifier       = uint32_t(id);           // Message ID and payload
            message.data_length_code = length;
            memcpy(message.data, data, length);
        }

        // Queue message for transmission
        return (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK);
    }

    bool CanESP32::init(int txPin, int rxPin) {
        // auto tx       = gpio_num_t(9);
        // auto rx       = gpio_num_t(47);
        auto tx       = gpio_num_t(txPin);
        auto rx       = gpio_num_t(rxPin);
        int  baudKbit = 500;

        /*
    Available baud rates:

    TWAI_TIMING_CONFIG_1MBITS()
    TWAI_TIMING_CONFIG_800KBITS()
    TWAI_TIMING_CONFIG_500KBITS()
    TWAI_TIMING_CONFIG_250KBITS()
    TWAI_TIMING_CONFIG_125KBITS()
    TWAI_TIMING_CONFIG_100KBITS()
    TWAI_TIMING_CONFIG_50KBITS()
    TWAI_TIMING_CONFIG_25KBITS()
    */

        // Initialize configuration structures using macro initializers
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
        twai_timing_config_t  t_config;

        switch (baudKbit) {
            case 25:
                t_config = TWAI_TIMING_CONFIG_25KBITS();
                break;
            case 50:
                t_config = TWAI_TIMING_CONFIG_50KBITS();
                break;
            case 100:
                t_config = TWAI_TIMING_CONFIG_100KBITS();
                break;
            case 125:
                t_config = TWAI_TIMING_CONFIG_125KBITS();
                break;
            case 250:
                t_config = TWAI_TIMING_CONFIG_250KBITS();
                break;
            case 500:
                t_config = TWAI_TIMING_CONFIG_500KBITS();
                break;
            case 800:
                t_config = TWAI_TIMING_CONFIG_800KBITS();
                break;
            case 1000:
                t_config = TWAI_TIMING_CONFIG_1MBITS();
                break;
            default:
                log_warn("Incorrect baud rate given. Falling back on 1 mbit.");
                t_config = TWAI_TIMING_CONFIG_1MBITS();
                break;
        }
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        // Install TWAI driver
        if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
            log_info("Driver installed");
        } else {
            log_error("Failed to install driver");
            return false;
        }

        // Start TWAI driver
        if (twai_start() == ESP_OK) {
            log_info("Driver started");
        } else {
            log_error("Failed to start driver");
            return false;
        }

        twai_status_info_t status_info;
        twai_get_status_info(&status_info);
        log_info("TWAI Status - State: " << int(status_info.state) << ", TX Queue: " << int(status_info.state) << ", RX Queue: "
                                         << int(status_info.msgs_to_tx) << ", TX Err: " << int(status_info.tx_error_counter)
                                         << ", RX Err:" << int(status_info.rx_error_counter));

        return true;
    }

    int CanESP32::tryReceive(uint8_t* data, uint32_t* identifier) {
        twai_message_t message;
        if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
            *identifier = message.identifier;

            memcpy(data, message.data, message.data_length_code);
            return message.data_length_code;
        } else {
            return -1;
        }
    }

    CanESP32::~CanESP32() {
        // Stop the TWAI driver
        if (twai_stop() == ESP_OK) {
            log_info("Driver stopped");
        } else {
            log_error("Failed to stop driver");
            return;
        }

        // Uninstall the TWAI driver
        if (twai_driver_uninstall() == ESP_OK) {
            log_info("Driver uninstalled");
        } else {
            log_error("Failed to uninstall driver");
            return;
        }
    }
}
