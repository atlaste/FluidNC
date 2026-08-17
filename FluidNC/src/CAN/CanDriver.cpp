// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "CanDriver.h"

#include <cstring>
#include <driver/gpio.h>
#include <driver/twai.h>

#include "../Logging.h"

namespace CAN {
    namespace Driver {
        static bool _installed = false;

        static bool timingFor(int baud_kbit, twai_timing_config_t& t_config) {
            switch (baud_kbit) {
                case 25:
                    t_config = TWAI_TIMING_CONFIG_25KBITS();
                    return true;
                case 50:
                    t_config = TWAI_TIMING_CONFIG_50KBITS();
                    return true;
                case 100:
                    t_config = TWAI_TIMING_CONFIG_100KBITS();
                    return true;
                case 125:
                    t_config = TWAI_TIMING_CONFIG_125KBITS();
                    return true;
                case 250:
                    t_config = TWAI_TIMING_CONFIG_250KBITS();
                    return true;
                case 500:
                    t_config = TWAI_TIMING_CONFIG_500KBITS();
                    return true;
                case 800:
                    t_config = TWAI_TIMING_CONFIG_800KBITS();
                    return true;
                case 1000:
                    t_config = TWAI_TIMING_CONFIG_1MBITS();
                    return true;
                default:
                    return false;
            }
        }

        bool init(int tx_pin, int rx_pin, int baud_kbit) {
            if (_installed) {
                log_error("CAN driver is already installed");
                return false;
            }

            twai_timing_config_t t_config;
            if (!timingFor(baud_kbit, t_config)) {
                log_error("CAN: unsupported baud rate " << baud_kbit << " kbit");
                return false;
            }

            twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t(tx_pin), gpio_num_t(rx_pin), TWAI_MODE_NORMAL);

            // The default queue depths are tiny.  Scheduled motion bursts a few frames per
            // segment and the RX side must not drop a node's input bitmap while the RX task
            // is briefly descheduled.
            g_config.tx_queue_len = 32;
            g_config.rx_queue_len = 64;

            twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

            if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
                log_error("CAN: failed to install TWAI driver");
                return false;
            }
            if (twai_start() != ESP_OK) {
                log_error("CAN: failed to start TWAI driver");
                twai_driver_uninstall();
                return false;
            }

            _installed = true;
            log_info("CAN bus tx:" << tx_pin << " rx:" << rx_pin << " " << baud_kbit << " kbit");
            return true;
        }

        void deinit() {
            if (!_installed) {
                return;
            }
            twai_stop();
            twai_driver_uninstall();
            _installed = false;
        }

        bool send(uint32_t id, uint8_t length, const uint8_t* data, uint32_t timeout_ms) {
            if (!_installed) {
                return false;
            }

            twai_message_t message;
            memset(&message, 0, sizeof(message));

            message.rtr              = data == nullptr ? 1 : 0;
            message.extd             = (id & 0x80000000) ? 1 : 0;
            message.identifier       = message.extd ? (id & 0x1fffffff) : id;
            message.data_length_code = length;
            if (data != nullptr && length != 0) {
                memcpy(message.data, data, length);
            }

            return twai_transmit(&message, pdMS_TO_TICKS(timeout_ms)) == ESP_OK;
        }

        int receive(uint8_t* data, uint32_t* identifier, uint32_t timeout_ms) {
            if (!_installed) {
                return -1;
            }

            twai_message_t message;
            if (twai_receive(&message, pdMS_TO_TICKS(timeout_ms)) != ESP_OK) {
                return -1;
            }

            *identifier = message.identifier;
            memcpy(data, message.data, message.data_length_code);
            return message.data_length_code;
        }

        static bool stateIs(twai_state_t want) {
            if (!_installed) {
                return false;
            }
            twai_status_info_t status;
            if (twai_get_status_info(&status) != ESP_OK) {
                return false;
            }
            return status.state == want;
        }

        bool busOff() {
            return stateIs(TWAI_STATE_BUS_OFF);
        }

        bool stopped() {
            return stateIs(TWAI_STATE_STOPPED);
        }

        void initiateRecovery() {
            if (_installed) {
                twai_initiate_recovery();
            }
        }

        bool restart() {
            return _installed && twai_start() == ESP_OK;
        }
    }
}
