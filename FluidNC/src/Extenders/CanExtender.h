#pragma once

#include "../Logging.h"

#include <driver/gpio.h>
#include <driver/can.h>
#include <esp_err.h>

class CanExtender {
    // Our CanExtender class uses the extended CAN protocol that has up to 8 bytes per package.
    //
    // Before you consider messing around here, read:
    // https://docs.espressif.com/projects/esp-idf/en/release-v3.3/api-reference/peripherals/can.html
    //
    // The basic functionality is according to the state machine as described in "Driver operation".
    // A task handles the RX/TX buffer of the master.
    //
    //
    // CAN can do hardware filtering of messages through the ID's. Filtering works based on a
    // mask, that has everything to do with this ID. As master, we have ID 1. In other words, all
    // packages that send to ID 1 are therefore received by us. Having a bit hit on ID 1 doesn't
    // mean that it's send _to_ us, it just means that there's a high chance that we should care.
    //
    // Notice that in CAN, messages with a lower ID have priority over messages with a higher ID.
    //
    // Significant ID's:
    // - 1<< 0:  Alarm. All nodes listen to this!
    // - 1<< 1:  Slow down.
    // - 1<< 2:  Axis.
    // - 1<< 3:  Spindle.
    // - 1<< 4:  Output pin extender.
    // - ...     [reserved for future use]
    // - 1<< 9:  Firmware update.
    // - 1<< 10: Status.
    // - 1<< 11: Sync timestamp.
    //
    // Id's alone say nothing about the protocol itself obviously. The protocol itself is bound to
    // max 8 bytes packages.
    //
    // Nodes have a unique, physical node id. This is not to be confused with the logical id above, that
    // is mainly relevant for filtering.
    //
    // The protocol that is used depends on the logical id, and works as follows (wherever 'node id' is
    // mentioned below, we use the physical node id):
    //
    // 
    // Alarm:
    //
    // - 1 byte: originator node id.
    // - 1 byte: packet type
    //
    // Packet type 0x01: [raise alarm]
    // - 1 byte: type of alarm. See Error.h for error codes.
    //
    // When an alarm is raised, packet type 1 is sent to all alarm nodes, and they have to respond by
    // putting the firmware in most dominant alarm mode that is available.
    //
    // Packet type 0x02: [alarm details]
    // - 1-6 bytes: char[] data with description, 0-terminated.
    //
    // Packet type 0x03: [node online request]
    // - 1 byte: target node id. Requests if a target node is online. The node should respond with (#4).
    //
    // Packet type 0x04: [node came online]
    // - 4 bytes: firmware sequence id. The firmware sequence id is stored in NVS on all nodes, and
    //            should match. Initially it's set on 0 for a client, and 1 for a master. In other
    //            words, the master will push its firmware to all clients initially. This packet is
    //            sent as soon as a node comes online, and can be requested using packet type 3.
    // - 4 bytes: 29 bits with the message id's that are relevant. This can be used by the master to
    //            decide what to send and what not.
    //
    // Packet type 0x05: [feed hold]
    // - no payload
    //
    // Packet type 0x06: [probe hit]
    // - 4 bytes: timestamp
    //
    // Packet type 0x07: [alarm reset]
    // - no payload
    //
    // Packet type 0x08: [firmware update]
    // - no payload
    //
    //
    // Slow down:
    //
    // If axis cannot keep up with motion, they issue a 'slow down' command.
    //
    //
    // Axis:
    //
    // A can bus node can be used to drive an axis, and an arbitrary number of motors per axis. Note
    // that can bus motors are configured as-if they are motors.
    //
    // - 1 byte: node id.
    // - 1 byte: dwell
    // - 2 byte: delta time
    // - 2 byte: delta position
    // - 2 byte: delta velocity
    //
    // The dwell is the time modulo 256 to wait before commencing the operation. This can be used for
    // synchronizing axis: an operation can be sent _before_ it is actually put into motion. Once the
    // timer hits the dwell value, it will start running. Since timers are synchronized, this implies
    // that messages need to be sent before motion starts.
    //
    // Motion assumes linear acceleration.
    //
    //
    // TODO FIXME - SLOW DOWN AND AXIS NEED MORE CAREFUL CONSIDERATION.
    //
    //
    // Spindle:
    //
    // Common:
    // - 1 byte: node id.
    // - 1 byte: operation. 0x80 is 'response' if required.
    //
    // Operation 0x00: get spindle type
    // - 1 byte: spindle type. PWM, 5V, on/off, VFD, etc.
    // - 1 byte: capabilities, bitmask.
    //
    // Response:
    // - No response
    //
    // Operation 0x01: set spindle speed (un-syncd)
    // - 4 byte: speed in RPM
    //
    // Response:
    // - No response
    //
    // Operation 0x02: set spindle speed (syncd)
    // - 4 byte: speed in RPM
    //
    // Response:
    // - Responds 0x83 packages until spindle speed is met. Then it responds with 0x82:
    // - 4 byte: speed in RPM
    //
    // Operation 0x03: current spindle speed
    // - no payload
    //
    // Response:
    // - 4 byte: speed in RPM
    //
    // Operation 0x04: max spindle speed
    // - no payload
    //
    // Response:
    // - 4 byte: speed in RPM
    //
    //
    // Output pin extender.
    //
    // Common:
    // - 1 byte: node id.
    // - 1 byte: operation. 0x80 is 'response' if required.
    //
    // Operation 0x00 - 0x0F: set output pin
    // - 5 bytes: bitmask. Together with the 0xF, this is 5*8 = 40 bytes per message.
    //   We have 16 messages, so that's a grand total of 16*40=640 different outputs we
    //   can set.
    //
    // Each output corresponds with 1 bit.
    //
    // Response:
    // - 1 byte: 1 = ok, 0 = error.
    //
    // Operation 0x10: reset all outputs
    // - no payload
    //
    // Response:
    // - 1 byte: 1 = ok, 0 = error.
    //
    //
    // Firmware update.
    //
    // Any node on the network can initiate a firmware update, as long as the sequence is correct.
    // This allows a user to plug in a temporary ESP32 with a new firmware, which then does the
    // update, and is then removed from the network again.
    //
    // Progress of firmware updates is sent over the logging.
    //
    // Unlike what one would perhaps expect, firmware updates are always multicast. Basically
    // the master checks if a client has an outdated firmware, and just starts the process.
    //
    // It's also noteworthy to say that firmware updates stop all other communications (Alarm 0x08).
    //
    // The firmware update works with the following protocol:
    // Nodes always response with a single byte: OK or Not OK. The response is sent to ID = 1.
    //
    // - 1 byte: operation.
    //
    // Operation 0x01: Notifies that a firmware update is about to begin. At this point, the master
    // already knows what clients should participate. No payload.
    //
    // Operation 0x02: Start flash batch block
    // - 4 bytes: offset of block
    // - 2 bytes: length of block (0-64KB)
    //
    // The size of the batch is what is left, bounded by 64 KB. _After_ the data is received, a
    // checksum follows to ensure the content was correctly sent:
    //
    // Operation 0x03: CRC checksum
    // - 4 bytes: CRC32
    //
    // If the content was correct, the client continues to write the data to flash. It then confirms
    // the payload was correct using 0x81.
    //
    // If one of the clients has an incorrect checksum, it responds with 0x81 / 0x02 (not correct), and
    // the firmware block will be resent.
    //
    // Operation 0x09: [firmware update complete]
    // - 1 = OK, 2 = FAILED
    //
    // Operation response 0x81: Firmware update progress OK/NOK response.
    //
    // Payload:
    // - 1 byte: 0x01 = OK, 0x02 = Not OK.
    //
    //
    // Status.
    //
    // Status is issued for OLED's and the like. Status information is basically broadcasted, and whomever
    // is interested will listen. Status can be just about anything from IP addresses to positions.
    // Status messages should not exceed -say- 10 kHz though, to ensure they don't make the bus too busy.
    //
    // Only the master can send status messages.
    //
    // - 1 byte: status type
    // - 1 byte: payload length
    // - 6 bytes: payload in packet 1
    // - N bytes in subsequent packages: remaining payload
    // - 2 bytes: CRC16 checksum for validation.
    //
    //
    // Sync timestamp.
    //
    // Sync timestamp is used to keep the clocks of the nodes more or less in sync. The sync timestamp is
    // a simple 4-byte message, unidirectional, with a clock value.

public:
    CanExtender() {}

    void init() {
        //Initialize configuration
        can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, CAN_MODE_NORMAL);
        can_timing_config_t  t_config = CAN_TIMING_CONFIG_1MBITS();
        can_filter_config_t  f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

        //Install CAN driver
        if (can_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
            log_info("CANBus driver installed");
        } else {
            log_error("Failed to install CANBus driver");
            return;
        }

        //Start CAN driver
        if (can_start() == ESP_OK) {
            log_info("CANBus driver started");
        } else {
            log_error("Failed to start CANBus driver");
            return;
        }
    }
    // error frame = alarm

    void send(const char* message, size_t size) {
        can_message_t message;
        message.identifier       = 0xAAAA;
        message.flags            = CAN_MSG_FLAG_EXTD;
        message.data_length_code = 4;
        for (int i = 0; i < 4; i++) {
            message.data[i] = 0;
        }

        //Queue message for transmission
        if (can_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
            log_info("Message queued for transmission");
        } else {
            log_error("Failed to queue message for transmission");
        }
    }

    void receive(const char* message, size_t size) {
        can_message_t message;
        if (can_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK) {
            log_info("Message received");
        } else {
            log_error("Failed to receive message");
            return;
        }

        //Process received message
        if (message.flags & CAN_MSG_FLAG_EXTD) {
            log_info("Message is in Extended Format");
        } else {
            log_error("Message is in Standard Format");
        }
        log_info("ID is %d", message.identifier);
        if (!(message.flags & CAN_MSG_FLAG_RTR)) {
            for (int i = 0; i < message.data_length_code; i++) {
                log_info("Data byte %d = %d", i, message.data[i]);
            }
        }
    }

    /* alerts:
    
    
        //Reconfigure alerts to detect Error Passive and Bus-Off error states
        uint32_t alerts_to_enable = CAN_ALERT_ERR_PASS | CAN_ALERT_BUS_OFF;
        if (can_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
            printf("Alerts reconfigured");
        } else {
            printf("Failed to reconfigure alerts");
        }

        //Block indefinitely until an alert occurs
        uint32_t alerts_triggered;
        can_read_alerts(&alerts_triggered, portMAX_DELAY);
*/
    ~CanExtender() {
        //Stop the CAN driver
        if (can_stop() == ESP_OK) {
            log_info("Driver stopped");
        } else {
            log_error("Failed to stop driver");
            return;
        }

        //Uninstall the CAN driver
        if (can_driver_uninstall() == ESP_OK) {
            log_info("Driver uninstalled");
        } else {
            log_error("Failed to uninstall driver");
            return;
        }
    }
};
