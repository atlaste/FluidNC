#include "ODriveESP32.h"

#include "Logging.h"

#include <driver/gpio.h>
#include <driver/twai.h>

void ODriveESP32::init(const Pin& txPin, const Pin& rxPin, int baudKbit) {
    auto tx = gpio_num_t(txPin.getNative(Pin::Capabilities::Native | Pin::Capabilities::Output));
    auto rx = gpio_num_t(rxPin.getNative(Pin::Capabilities::Native | Pin::Capabilities::Input));

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
        case 25: t_config = TWAI_TIMING_CONFIG_25KBITS(); break;
        case 50: t_config = TWAI_TIMING_CONFIG_50KBITS(); break;
        case 100: t_config = TWAI_TIMING_CONFIG_100KBITS(); break;
        case 125: t_config = TWAI_TIMING_CONFIG_125KBITS(); break;
        case 250: t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
        case 500: t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
        case 800: t_config = TWAI_TIMING_CONFIG_800KBITS(); break;
        case 1000:
            t_config = TWAI_TIMING_CONFIG_1MBITS();
            break;
        default:
            printf("Incorrect baud rate given. Falling back on 1 mbit.\n");
            t_config = TWAI_TIMING_CONFIG_1MBITS();
            break;
    }
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return;
    }
}

bool ODriveESP32::transmit(uint32_t id, uint8_t len, const uint8_t* buffer) {
    // Configure message to transmit
    twai_message_t message;
    memset(&message, 0, sizeof(message));

    // Message type and format settings
    message.extd             = uint32_t(1);   // Standard vs extended format
    message.rtr              = uint32_t(0);   // Data vs RTR frame
    message.ss               = uint32_t(0);   // Whether the message is single shot (i.e., does not repeat on error)
    message.self             = uint32_t(0);   // Whether the message is a self reception request (loopback)
    message.dlc_non_comp     = uint32_t(0);   // DLC is less than 8
    message.identifier       = uint16_t(id);  // Message ID and payload
    message.data_length_code = len;
    memcpy(message.data, buffer, len);

    // Queue message for transmission
    return (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK);
}

bool ODriveESP32::waitForResponse() {
    auto endTicks = usToEndTicks(10'000);  // 10 ms
    do {
        pump();
    } while ((getCpuTicks() - endTicks) < 0 && requestedMessageId != REQUEST_PENDING);
    return requestedMessageId != REQUEST_PENDING;
}

void ODriveESP32::pump() {
    // Wait for the message to be received
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
        printf("Message received\n");
        failCount = 0;
    } else {
        ++failCount;
        if ((failCount % 10) == 1) {
            printf("Failed to receive message\n");
        }
        return;
    }

    // Process received message
    if (message.extd) {
        printf("Message is in Extended Format\n");
    } else {
        printf("Message is in Standard Format\n");
    }

    printf("ID is %ld\n", message.identifier);
    if (!(message.rtr)) {
        for (int i = 0; i < message.data_length_code; i++) {
            printf("Data byte %d = %d\n", i, message.data[i]);
        }
    }

    uint32_t nodeId = (message.identifier >> kNodeIdShift);
    switch (message.identifier & kCmdIdBits) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            Get_Encoder_Estimates_msg_t estimates;
            estimates.decode_buf(message.data);
            if (feedback_callback_)
                feedback_callback_(nodeId, estimates, feedback_user_data_);
            break;
        }
        case Heartbeat_msg_t::cmd_id: {
            Heartbeat_msg_t status;
            status.decode_buf(message.data);
            if (axis_state_callback_ != nullptr)
                axis_state_callback_(nodeId, status, axis_state_user_data_);
            else
                log_warn("ODrive: missing callback");
            break;
        }
        default: {
            if (requestedMessageId == REQUEST_PENDING)
                return;

#ifdef DEBUG
            Serial.print("waiting for: 0x");
            Serial.println(requestedMessageId, HEX);
#endif  // DEBUG
            if (message.identifier != requestedMessageId)
                return;

            memcpy(message.data, responseData, message.data_length_code);
            requestedMessageId = REQUEST_PENDING;
        }
    }
}

ODriveESP32::~ODriveESP32() {
    // Stop the TWAI driver
    if (twai_stop() == ESP_OK) {
        printf("Driver stopped\n");
    } else {
        printf("Failed to stop driver\n");
        return;
    }

    // Uninstall the TWAI driver
    if (twai_driver_uninstall() == ESP_OK) {
        printf("Driver uninstalled\n");
    } else {
        printf("Failed to uninstall driver\n");
        return;
    }
}
