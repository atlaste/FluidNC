// Copyright (c) 2025 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "ODriveSpindle.h"
#include "ODrive/CanESP32.h"
#include "ODrive/ODriveEnums.h"

#include "Protocol.h"       // rtAlarm
#include "MotionControl.h"  // mc_critical
#include "System.h"         // sys.*
#include "Logging.h"

#include <cstdio>
#include <iostream>
#include <cmath>
#include <esp_timer.h>
#include <esp_attr.h>

namespace Spindles {
    struct ODriveAction {
        enum Action {
            None = 0,
            SetMode,
            SetSpeed,
            SetSpeedNoSync,
        };
        Action  action   = None;
        int32_t arg      = 0;
        bool    critical = false;
    };

    // Static member initialization
    QueueHandle_t ODriveSpindle::cmd_queue     = nullptr;
    QueueHandle_t ODriveSpindle::speed_queue   = nullptr;
    TaskHandle_t  ODriveSpindle::cmdTaskHandle = nullptr;

    // ODrive CAN Messages:
    int ODriveSpindle::receive(uint8_t* responseData, uint32_t* messageId, uint32_t* nodeId, int timeout_ms) {
        auto deadline = esp_timer_get_time() + (timeout_ms * 1000);

        while ((esp_timer_get_time() - deadline) < 0) {
            uint32_t identifier = 0;
            int      res        = can->tryReceive(responseData, &identifier);
            *nodeId             = (identifier >> kNodeIdShift);
            *messageId          = identifier & kCmdIdBits;

            // Note: we only support 1 odrive at the moment.
            if (res >= 0 && ODriveNodeId == *nodeId) {
                if (*messageId == ODrive::Heartbeat_msg_t::cmd_id) {
                    lastHeartbeat = esp_timer_get_time();

                    ODrive::Heartbeat_msg_t hb;
                    hb.decode_buf(responseData);
                    lastAxisState = hb.Axis_State;
                    lastAxisError = hb.Axis_Error;
                } else if (*messageId == ODrive::Get_Encoder_Estimates_msg_t::cmd_id) {
                    ODrive::Get_Encoder_Estimates_msg_t estimates;
                    estimates.decode_buf(responseData);
                    lastPosition = estimates.Pos_Estimate;
                    lastVelocity = estimates.Vel_Estimate;
                } else {
                    log_info("Received message from node " << int(*nodeId) << " with id: " << int(*messageId) << ".");
                    // for (int i = 0; i < 8; ++i) {
                    //     printf("0x%02X ", responseData[i]);
                    // }
                    // printf("\n");

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
            log_info("Entering state " << int(state) << "...");
            ODrive::Set_Axis_State_msg_t setState;
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

    bool ODriveSpindle::setSpeedCommand(int32_t rpm, bool sync) {
        if (_current_state == SpindleState::Ccw) {
            rpm = -rpm;
        }
        rpm = int(rpm * gearFactor);

        // Set speed
        log_info("Ramping to " << rpm << " RPM (" << (double(rpm) / 60.0) << " rev/s)...");

        double       targetVelocity = double(rpm) / 60.0;
        const double tolerance      = 1.0;  // +/- 1 rev/s tolerance
        const int    timeout        = 15;   // 15 seconds timeout
        bool         speedReached   = false;

        ODrive::Set_Input_Vel_msg_t velCmd;
        velCmd.Input_Vel       = targetVelocity;  // 1000 RPM = 16.67 rev/s
        velCmd.Input_Torque_FF = 0.0;
        send(velCmd);
        
        // Mark speed as invalid during ramp (for encoder validation)
        startRamp(timeout * 1000);  // timeout converted to milliseconds

        if (sync) {
            // Wait till speed reaches target RPM
            auto deadline = esp_timer_get_time() + (timeout * 1000000);
            while ((deadline - esp_timer_get_time()) > 0) {
                pump();

                auto diff = lastVelocity - targetVelocity;
                if (diff < 0) {
                    diff = -diff;
                }
                if (diff < tolerance) {
                    speedReached = true;
                    endRamp();  // Mark speed as valid now
                    log_info("Target speed reached: " << lastVelocity << " rev/s");
                    break;
                }

                log_info("Current speed: " << lastVelocity << " rev/s (target: " << targetVelocity << " rev/s)");
            }

            if (!speedReached) {
                log_warn("Warning: Target speed not reached within timeout");
            }

            return speedReached;
        } else {
            return true;
        }
    }

    void ODriveSpindle::setClosedLoopControl() {
        // Switch from velocity control to position control
        ODrive::Set_Controller_Mode_msg_t modeMsg;
        modeMsg.Control_Mode = ODrive::CONTROL_MODE_VELOCITY_CONTROL;
        modeMsg.Input_Mode   = ODrive::INPUT_MODE_VEL_RAMP;
        send(modeMsg);

        // Enter closed loop control
        log_info("Entering closed loop control...");

        if (!setState(ODrive::ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)) {
            log_error("Failed to set closed loop state. State = " << lastAxisState);
            while (1) {}
        }
    }

    void ODriveSpindle::setIdleControl() {
        // Set idle
        log_info("Setting idle state.");
        setState(ODrive::ODriveAxisState::AXIS_STATE_IDLE);
    }

    void ODriveSpindle::initializationSequence() {
        if (can == nullptr) {
            can = new ODrive::CanESP32();
        }

        // Configure and initialize the CAN bus interface. This function depends on
        // your hardware and the CAN stack that you're using.
        if (!can->init(txPin.getNative(Pin::Capabilities::Output), rxPin.getNative(Pin::Capabilities::Input))) {
            log_error("CAN failed to initialize: reset required");
            while (true) {}  // Spin indefinitely.
        }

        lastHeartbeat = 0;
        uint8_t  buf[8];
        uint32_t msgId;
        uint32_t nodeId;

        log_info("Waiting for ODrive...");
        while (lastHeartbeat == 0) {
            receive(buf, &msgId, &nodeId);
        }
        log_info("ODrive node " << nodeId << " found.");

        // request bus voltage and current (1sec timeout)
        log_info("Attempting to read bus voltage and current");
        ODrive::Get_Bus_Voltage_Current_msg_t vbus;
        if (!request(vbus)) {
            log_error("VBus request failed!");
            while (true)
                ;  // spin indefinitely
        }

        log_info("DC voltage [V]: " << vbus.Bus_Voltage);
        log_info("DC current [A]: " << vbus.Bus_Current);

        // Clear errors.
        ODrive::Clear_Errors_msg_t clear;
        if (!send(clear)) {
            log_error("Failed to send Clear Errors message");
            while (1) {}
        } else {
            log_info("Cleared errors.");
        }

        log_info("Reading error...");
        ODrive::Get_Error_msg_t error;
        if (request(error)) {
            if (error.Active_Errors != 0 || error.Disarm_Reason != 0) {
                log_error("Active errors: " << error.Active_Errors << ", disarm reason: " << error.Disarm_Reason);
                while (1) {}
            }
        }

        log_info("SUCCESS! All CAN communication tests passed");

        setIdleControl();
    }

    void ODriveSpindle::invokeAction(ODriveAction& action) {
        switch (action.action) {
            case ODriveAction::SetMode: {
                SpindleState mode = SpindleState(action.arg);

                // Handle mode changes
                if (mode == SpindleState::Disable) {
                    // Stop the spindle and set idle
                    setIdleControl();

                    // Clear the command queue
                    if (cmd_queue) {
                        xQueueReset(cmd_queue);
                    }
                } else {
                    // Enable spindle
                    setClosedLoopControl();
                }

                _current_state = mode;
                break;
            }
            case ODriveAction::SetSpeedNoSync:
            case ODriveAction::SetSpeed: {
                int32_t rpm = action.arg;

                _current_dev_speed = rpm;

                // Set the speed (convert device units to RPM)
                bool success = setSpeedCommand(int(rpm), action.action != ODriveAction::SetSpeedNoSync);

                if (!success) {
                    if (action.critical) {
                        mc_critical(ExecAlarm::SpindleControl);
                        log_error("Critical ODrive spindle speed not reached: " << int(rpm) << " RPM");
                    } else {
                        log_warn("ODrive spindle speed not reached: " << int(rpm) << " RPM");
                    }
                }

                if (speed_queue) {
                    // rpm cannot be queued. TODO FIXME: Queueing a pointer to a local is NOT okay.
                    xQueueSend(speed_queue, &rpm, 0);
                }
                break;
            }
            default:
                break;
        }
    }

    // ODrive command task
    void ODriveSpindle::cmd_task(void* pvParameters) {
        ODriveSpindle* instance = static_cast<ODriveSpindle*>(pvParameters);

        // Run initialization sequence
        instance->initializationSequence();

        const TickType_t poll_delay = pdMS_TO_TICKS(100);  // 100ms polling interval

        // Main command processing loop
        for (;;) {
            ODriveAction action;

            // Check for commands in the queue
            if (xQueueReceive(cmd_queue, &action, poll_delay)) {
                // Process the action
                instance->invokeAction(action);
            } else {
                // No command in queue, just pump to handle incoming messages
                instance->pump();
            }

            // If syncing, periodically poll for speed updates
            if (instance->_syncing && instance->speed_queue) {
                uint32_t currentSpeed = uint32_t(instance->lastVelocity * 60.0f);  // Convert rev/s to RPM
                xQueueSend(speed_queue, &currentSpeed, 0);
            }
        }
    }

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
                                    4096,                    // size of task stack
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

        if (_speeds.size() == 0) {
            // The default speed map for an odrive spindle is linear from 0=0% to [max]=100%
            linearSpeeds(maxSpeed, 100.0f);
        }

        setupSpeeds(maxSpeed);
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
        if (sys.abort()) {
            return;  // Block during abort.
        }

        if (speed == 0 && _disable_with_zero_speed) {
            log_debug("Disabling because speed is 0");
            state = SpindleState::Disable;
        }

        bool critical = (state_is(State::Cycle) || state != SpindleState::Disable);

        int32_t dev_speed = int32_t(mapSpeed(state, speed));

        log_debug(name() << ": setState:" << uint8_t(state) << " SpindleSpeed:" << speed
                         << ". Current dev speed: " << int(_current_dev_speed) << "; dev speed: " << int(dev_speed));

        bool change_direction = false;
        if (_current_state != state) {
            // Check if we're going from 'disable' (M5) to 'enable' (M3/M4).
            if ((_current_state == SpindleState::Cw || _current_state == SpindleState::Ccw) !=
                (state == SpindleState::Cw || state == SpindleState::Ccw)) {
                // Set speed *FIRST* when going from disable to Cw/Ccw.
                log_debug("Set speed " << int(dev_speed));
                _current_state = state;
                setSpeed(dev_speed, false);

                log_debug("Set mode " << int(state));
                set_mode(state, critical);  // critical if we are in a job
            } else {
                _current_state   = state;
            }
            change_direction = true;
        }

        if (_current_dev_speed != dev_speed || change_direction) {
            // Invalidate the speed; we're ramping:
            startRamp(100000);

            // It's okay to set the speed again.
            log_debug("Set speed " << int(dev_speed));
            setSpeed(dev_speed);
        }

        //if (use_delay_settings()) {
        //    spindleDelay(state, speed);
        //    return;
        //}

        // _sync_dev_speed is set by a callback that handles
        // responses from periodic get_current_speed() requests.
        // It changes as the actual speed ramps toward the target.

        _syncing = true;  // poll for speed


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
                endRamp();
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
        endRamp();
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

    void ODriveSpindle::setSpeed(int32_t dev_speed, bool sync) {
        if (cmd_queue) {
            ODriveAction action;
            action.action   = sync ? ODriveAction::SetSpeed : ODriveAction::SetSpeedNoSync;
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
        Assert(txPin.defined() && rxPin.defined(), "ODrive: missing CAN TX/RX pin configuration");
    }

    void ODriveSpindle::afterParse() {}

    void ODriveSpindle::group(Configuration::HandlerBase& handler) {
        handler.item("debug", _debug, 0, 5);
        handler.item("retries", _retries);
        handler.item("can_tx", txPin);
        handler.item("can_rx", rxPin);
        handler.item("odrive_node_id", ODriveNodeId);
        handler.item("max_speed", maxSpeed);

        float gf = float(gearFactor);
        handler.item("gear_factor", gf);
        gearFactor = double(gf);

        Spindle::group(handler);
    }

    namespace {
        SpindleFactory::InstanceBuilder<ODriveSpindle> registration("odrive");
    }
}
