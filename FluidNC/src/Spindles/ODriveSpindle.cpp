// Copyright (c) 2025 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "ODriveSpindle.h"
#include "ODrive/CanESP32.h"

#include "Machine/MachineConfig.h"
#include "Protocol.h"  // rtAlarm
#include "Report.h"    // hex message
#include "Configuration/HandlerType.h"

#include <freertos/task.h>
#include <freertos/queue.h>
#include <atomic>

#include <cstdio>

#include <iostream>
#include <cmath>
#include <esp_timer.h>
#include <esp_attr.h>
#include <freertos/FreeRTOS.h>

#include "ODrive/ODriveEnums.h"
#include "ODrive/CanESP32.h"

namespace Spindles {
    struct ODriveAction {
        enum Action {
            None = 0,
            SetMode,
            SetSpeed,
        };
        Action   action   = None;
        uint32_t arg      = 0;
        bool     critical = false;

    };

    // ODrive CAN Messages:
    int ODriveSpindle::receive(uint8_t* responseData, uint32_t* messageId, uint32_t* nodeId, int timeout_ms = 1000) {
        auto deadline = esp_timer_get_time() + (timeout_ms * 1000);

        while ((esp_timer_get_time() - deadline) < 0) {
            uint32_t identifier = 0;
            int      res        = can->tryReceive(responseData, &identifier);
            *nodeId             = (identifier >> kNodeIdShift);
            *messageId          = identifier & kCmdIdBits;

            // Note: we only support 1 odrive at the moment.
            if (res >= 0 && ODriveNodeId == *nodeId) {
                if (*messageId == Heartbeat_msg_t::cmd_id) {
                    lastHeartbeat = esp_timer_get_time();

                    Heartbeat_msg_t hb;
                    hb.decode_buf(responseData);
                    lastAxisState = hb.Axis_State;
                    lastAxisError = hb.Axis_Error;
                } else if (*messageId == Get_Encoder_Estimates_msg_t::cmd_id) {
                    Get_Encoder_Estimates_msg_t estimates;
                    estimates.decode_buf(responseData);
                    lastPosition = estimates.Pos_Estimate;
                    lastVelocity = estimates.Vel_Estimate;
                } else {
                    printf("Received message from node %d, id %d: ", int(*nodeId), int(*messageId));
                    for (int i = 0; i < 8; ++i) {
                        printf("0x%02X ", responseData[i]);
                    }
                    printf("\n");

                    return res;
                }
            }
        }
        return 0;
    }
    void ODriveSpindle::pump() {
        uint8_t  responseData[8];
        uint32_t messageId;
        uint32_t nodeId;
        receive(responseData, &messageId, &nodeId);
    }

    bool ODriveSpindle::setState(ODrive::ODriveAxisState state) {
        // Enter closed loop control
        if (lastAxisState != uint8_t(state)) {
            std::cout << "Entering state " << int(state) << "..." << std::endl;
            Set_Axis_State_msg_t setState;
            setState.Axis_Requested_State = state;
            send(setState);

            // Verify closed loop
            auto deadline = esp_timer_get_time() + 2000000;
            while ((deadline - esp_timer_get_time()) > 0 && lastAxisState != uint8_t(state)) {
                pump();
            }
        }

        return lastAxisState == uint8_t(state);
    }

    bool ODriveSpindle::setSpeed(int rpm) {
        // Set speed
        std::cout << "Ramping to " << rpm << " RPM (" << (float(rpm) / 60) << " rev/s)..." << std::endl;

        float       targetVelocity = float(rpm) / 60.0f;
        const float tolerance      = 1.0f;  // +/- 1 rev/s tolerance
        const int   timeout        = 5;     // 5 seconds timeout
        bool        speedReached   = false;

        Set_Input_Vel_msg_t velCmd;
        velCmd.Input_Vel       = targetVelocity;  // 1000 RPM = 16.67 rev/s
        velCmd.Input_Torque_FF = 0.0f;
        send(velCmd);

        // Wait till speed reaches target RPM
        auto deadline = esp_timer_get_time() + (timeout * 1000000);
        while ((deadline - esp_timer_get_time()) > 0) {
            pump();

            if (fabs(lastVelocity - targetVelocity) < tolerance) {
                speedReached = true;
                std::cout << "Target speed reached: " << lastVelocity << " rev/s" << std::endl;
                break;
            }

            std::cout << "Current speed: " << lastVelocity << " rev/s (target: " << targetVelocity << " rev/s)" << std::endl;
        }

        if (!speedReached) {
            std::cout << "Warning: Target speed not reached within timeout" << std::endl;
        }

        return speedReached;
    }

    void ODriveSpindle::setClosedLoopControl() {
        // Switch from velocity control to position control
        Set_Controller_Mode_msg_t modeMsg;
        modeMsg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
        modeMsg.Input_Mode   = INPUT_MODE_VEL_RAMP;
        send(modeMsg);

        // Enter closed loop control
        std::cout << "Entering closed loop control..." << std::endl;

        if (!setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)) {
            std::cout << "Failed to set closed loop state. State = " << lastAxisState << std::endl;
            while (1) {}
        }
    }

    void ODriveSpindle::setIdleControl()
    {
        // Set idle
        std::cout << "Setting idle state." << std::endl;
        setState(ODriveAxisState::AXIS_STATE_IDLE);
    }

    void ODriveSpindle::initializationSequence() {
        printf("ODrive Test Application Started\n");

        // Wait for up to 3 seconds for the serial port to be opened on the PC side.
        // If no PC connects, continue anyway.
        std::cout << "Starting ODriveCAN demo" << std::endl;

        if (can == nullptr) {
            can = new ODrive::CanESP32(txPin.getNative(), rxPin.getNative());
        }

        // Configure and initialize the CAN bus interface. This function depends on
        // your hardware and the CAN stack that you're using.
        std::cout << "Starting CAN bus" << std::endl;
        if (!can->init()) {
            std::cout << "CAN failed to initialize: reset required" << std::endl;
            while (true) {}  // Spin indefinitely.
        }

        lastHeartbeat = 0;
        uint8_t  buf[8];
        uint32_t msgId;
        uint32_t nodeId;

        std::cout << "Waiting for ODrive..." << std::endl;
        while (lastHeartbeat == 0) {
            receive(buf, &msgId, &nodeId);
        }
        std::cout << "ODrive node " << nodeId << " found." << std::endl;

        // request bus voltage and current (1sec timeout)
        std::cout << "Attempting to read bus voltage and current" << std::endl;
        Get_Bus_Voltage_Current_msg_t vbus;
        if (!request(vbus)) {
            std::cout << "vbus request failed!" << std::endl;
            while (true)
                ;  // spin indefinitely
        }

        std::cout << "DC voltage [V]: " << vbus.Bus_Voltage << std::endl;
        std::cout << "DC current [A]: " << vbus.Bus_Current << std::endl;

        // Clear errors.
        Clear_Errors_msg_t clear;
        clear.Identify = 0;
        if (!send(clear)) {
            std::cout << "Failed to send Clear Errors message" << std::endl;
            while (1) {}
        } else {
            std::cout << "Cleared errors." << std::endl;
        }

        std::cout << "Reading error..." << std::endl;
        Get_Error_msg_t error;
        if (request(error)) {
            std::cout << "Errors: " << error.Active_Errors << ", " << error.Disarm_Reason << std::endl;

            if (error.Active_Errors != 0 || error.Disarm_Reason != 0) {
                while (1) {}
            }
        }

        std::cout << "\nSUCCESS! All CAN communication tests passed\n" << std::endl;
        std::cout << "Done." << std::endl;

        setIdleControl();
    }

    void ODriveSpindle::invokeAction(ODriveAction& action) {
        switch (action.action) {
            case ODriveAction::SetMode:
            case ODriveAction::SetSpeed:
            default:
                break;
        }
    }

    // ODrive command task: 
    // TODO FIXME; look at VFDProtocol for inspiration.

    // Spindle implementation:
    void ODriveSpindle::init() {
        _sync_dev_speed = 0;
        _syncing        = false;

        // These ODrives are always reversible, but most can be set via the operator panel
        // to only allow one direction.  In principle we could check that setting and
        // automatically set is_reversable.
        is_reversable = true;

        _current_state = SpindleState::Disable;

        // Initialization is complete, so now it's okay to run the queue task:
        if (!cmd_queue) {  // init can happen many times, we only want to start one task
            const int ODRIVE_QUEUE_SIZE = 32;
            cmd_queue                   = xQueueCreate(ODRIVE_QUEUE_SIZE, sizeof(ODriveAction));
            speed_queue                 = xQueueCreate(ODRIVE_QUEUE_SIZE, sizeof(uint32_t));

            xTaskCreatePinnedToCore(cmd_task,                // task
                                    "ODrive_cmdTaskHandle",  // name for task
                                    2048,                    // size of task stack
                                    this,                    // parameters
                                    1,                       // priority
                                    &cmdTaskHandle,
                                    SUPPORT_TASK_CORE  // core
            );
        }

        init_atc();
        config_message();

        set_mode(SpindleState::Disable, true);

        _default_ramp_delay = 5000;
    }

    void ODriveSpindle::config_message() {
        std::string usage(" ODriveSpindle");
        usage += atc_info();
    }

    void ODriveSpindle::set_mode(SpindleState mode, bool critical) {
        _last_override_value = sys.spindle_speed_ovr();  // sync these on mode changes
        if (cmd_queue) {
            ODriveAction action;
            action.action   = ODriveAction::SetMode;
            action.arg      = uint32_t(mode);
            action.critical = critical;
            if (xQueueSend(cmd_queue, &action, 0) != pdTRUE) {
                log_info("ODrive Queue Full");
            }
        }
    }

    void ODriveSpindle::setState(SpindleState state, SpindleSpeed speed) {
        log_debug(name() << ": setState:" << uint8_t(state) << " SpindleSpeed:" << speed);
        if (sys.abort()) {
            return;  // Block during abort.
        }

        if (speed == 0 && _disable_with_zero_speed) {
            log_debug("Disabling because speed is 0");
            state = SpindleState::Disable;
        }

        bool critical = (state_is(State::Cycle) || state != SpindleState::Disable);

        uint32_t dev_speed = mapSpeed(state, speed);

        if (_current_dev_speed != dev_speed) {
            log_debug("setSpeed " << int(dev_speed));
            setSpeed(dev_speed);
        }

        if (_current_state != state) {
            log_debug("set_mode " << int(state));
            set_mode(state, critical);  // critical if we are in a job
            _current_state = state;
        }

        //if (use_delay_settings()) {
        //    spindleDelay(state, speed);
        //    return;
        //}

        // _sync_dev_speed is set by a callback that handles
        // responses from periodic get_current_speed() requests.
        // It changes as the actual speed ramps toward the target.

        _syncing = true;  // poll for speed

        // Invalidate the speed; we're ramping:
        startRamp(100000);

        auto minSpeedAllowed = dev_speed > _slop ? (dev_speed - _slop) : 0;
        auto maxSpeedAllowed = dev_speed + _slop;

        int unchanged = 0;
        //            const int limit     = 150;  // 15 sec / 100 ms
        const int limit = 100;

        if (_debug > 1 && _sync_dev_speed != UINT32_MAX) {
            log_info("Syncing to " << int(dev_speed));
        }

        while ((_last_override_value == sys.spindle_speed_ovr()) &&  // skip if the override changes
               ((_sync_dev_speed < minSpeedAllowed || _sync_dev_speed > maxSpeedAllowed) && unchanged < limit)) {
            if (!xQueueReceive(speed_queue, &_sync_dev_speed, 3000)) {
                mc_critical(ExecAlarm::SpindleControl);
                log_error(name() << ": spindle did not reach device units " << dev_speed << ". Reported value is " << _sync_dev_speed);
                _syncing = false;

                // Let's just say it's valid again; otherwise we get issues later.
                startRamp(0);
                _speedIsValidAfter = 0;
                return;
            }
        }
        _last_override_value = sys.spindle_speed_ovr();
        _current_speed       = speed;
        if (_debug > 1) {
            log_info("Synced speed to " << int(dev_speed));
        }

        // Make the speed valid again:
        startRamp(0);
        _speedIsValidAfter = 0;

        _syncing = false;
    }

    void IRAM_ATTR ODriveSpindle::setSpeedfromISR(uint32_t dev_speed) {
        if (_current_dev_speed == dev_speed || _last_speed == dev_speed) {
            return;
        }

        _last_speed = dev_speed;

        // Let's just set some large number to invalidate the speed for 5 seconds; it
        // will be enabled later in the queue:
        startRamp(_default_ramp_delay);

        if (cmd_queue) {
            ODriveAction action;
            action.action   = ODriveAction::SetSpeed;
            action.arg      = dev_speed;
            action.critical = (dev_speed == 0);
            // Ignore errors because reporting is not safe from an ISR.
            // Perhaps set a flag instead?
            xQueueSendFromISR(cmd_queue, &action, 0);
        }
    }

    void ODriveSpindle::setSpeed(uint32_t dev_speed) {
        if (cmd_queue) {
            ODriveAction action;
            action.action   = ODriveAction::SetSpeed;
            action.arg      = dev_speed;
            action.critical = dev_speed == 0;
            if (xQueueSend(cmd_queue, &action, 0) != pdTRUE) {
                log_info("ODrive Queue Full");
            } else {
                // Let's just set some large number to invalidate the speed (for 10 seconds); it
                // will be enabled later in the queue:
                startRamp(10'000);
            }
        }
    }

    void ODriveSpindle::validate() {
        Spindle::validate();
        Assert(_uart != nullptr || _uart_num != -1, "ODrive: missing UART configuration");
    }

    void ODriveSpindle::afterParse() {}

    void ODriveSpindle::group(Configuration::HandlerBase& handler) {
        handler.item("debug", _debug, 0, 5);
        handler.item("retries", _retries);
        handler.item("can_tx", txPin);
        handler.item("can_rx", rxPin);
        handler.item("odrive_node_id", ODriveNodeId);

        Spindle::group(handler);
    }
}
