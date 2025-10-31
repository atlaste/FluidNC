#include "ODriveSpindle.h"

#include "../NutsBolts.h"
#include "ODrive/ODriveESP32.h"
#include "ODrive/ODriveEnums.h"

#include <freertos/queue.h>
#include <freertos/task.h>

#include <atomic>
#include <esp_timer.h>

namespace Spindles
{
    QueueHandle_t ODriveSpindle::oDriveCommandQueue      = nullptr;
    TaskHandle_t  ODriveSpindle::oDriveCommandTaskHandle = nullptr;

    const int ODRIVE_QUEUE_SIZE = 10;
    void      ODriveSpindle::init()
    {
        lastHeartbeat = esp_timer_get_time() - 3'600ull * 1'000'000ull;  // 1 hour ago

        // Initialization is complete, so now it's okay to run the queue task:
        if (!ODriveSpindle::oDriveCommandQueue)
        {  // init can happen many times, we only want to start one task
            ODriveSpindle::oDriveCommandQueue = xQueueCreate(ODRIVE_QUEUE_SIZE, sizeof(ODriveAction));
            xTaskCreatePinnedToCore(oDriveCommandTaskHelper,  // task
                                    "odriveTaskHandle",       // name for task
                                    4096,                     // size of task stack
                                    this,                     // parameters
                                    1,                        // priority
                                    &oDriveCommandTaskHandle,
                                    SUPPORT_TASK_CORE  // core
            );
        }
    }

    void ODriveSpindle::config_message() {}

    void ODriveSpindle::setSpeedfromISR(uint32_t dev_speed)
    {
        // requested speed is in currentState
        if (lastSetSpeedRevMin == dev_speed)
        {
            return;
        }

        lastSetSpeedRevMin = dev_speed;
        if (oDriveCommandQueue)
        {
            ODriveAction action;
            action.speedRevMin = dev_speed;
            action.action      = ODriveActionType::SetSpeed;
            xQueueSendFromISR(oDriveCommandQueue, &action, 0);
        }
    }
    void ODriveSpindle::setState(SpindleState state, SpindleSpeed speed)
    {
        // requested speed is in currentState
        uint32_t speedRevMin = speed;

        if (lastSetSpeedRevMin == speedRevMin)
        {
            return;
        }

        if (oDriveCommandQueue)
        {
            ODriveAction action;
            action.speedRevMin = speedRevMin;
            switch (state)
            {
                case SpindleState::Cw:
                    action.action = ODriveActionType::RunCW;
                    break;
                case SpindleState::Ccw:
                    action.action = ODriveActionType::RunCCW;
                    break;
                case SpindleState::Disable:
                default:
                    action.action = ODriveActionType::Stop;
                    break;
            }
            if (xQueueSend(oDriveCommandQueue, &action, 0) != pdTRUE)
            {
                log_info("ODrive queue full");
            }
        }
    }

    //// Called every time a Heartbeat message arrives from the ODrive
    void ODriveSpindle::onHeartbeat(uint8_t nodeId, Heartbeat_msg_t& msg, void* user_data)
    {
        auto ud = static_cast<ODriveSpindle*>(user_data);
        if (nodeId == ud->odriveNodeId)
        {
            ud->lastHeartbeat = esp_timer_get_time();
            ud->lastAxisState = ODriveAxisState(msg.Axis_State);
            ud->lastAxisError = ODriveError(msg.Axis_Error);
        }
    }

    void ODriveSpindle::onFeedback(uint8_t nodeId, Get_Encoder_Estimates_msg_t& feedback, void* user_data)
    {
        auto ud = static_cast<ODriveSpindle*>(user_data);
        if (nodeId == ud->odriveNodeId)
        {
            ud->lastFeedback     = esp_timer_get_time();
            ud->feedbackPosition = feedback.Pos_Estimate;
            ud->feedbackVelocity = feedback.Vel_Estimate;
        }
    }

    bool ODriveSpindle::heartBeatValid()
    {
        // Hearbeats are normally sent at 10 Hz. We check if it's valid within 0.5 second.
        auto now = esp_timer_get_time();
        return now - lastHeartbeat < 500'000ull;
    }

    //// Called every time a feedback message arrives from the ODrive
    //void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
    //    ODriveUserData* odrv_user_data    = static_cast<ODriveUserData*>(user_data);
    //    odrv_user_data->last_feedback     = msg;
    //    odrv_user_data->received_feedback = true;
    //}

    void ODriveSpindle::oDriveCommandTask()
    {
        initializationSequence();

        while (true)
        {
            ODriveAction action;
            if (xQueueReceive(oDriveCommandQueue, &action, 0))
            {
                switch (lastRequestedState)
                {
                    case ODriveState::NotInitialized:
                        initializationSequence();
                        break;
                    case ODriveState::Idle:
                        setAxisMode(ODriveAxisState::AXIS_STATE_IDLE);
                        break;
                    case ODriveState::ClosedLoop:
                        setVelocity(action.speedRevMin);
                        setAxisMode(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
                        break;
                    case ODriveState::RunningCCW:
                        setVelocity(-action.speedRevMin);
                        break;
                    case ODriveState::RunningCW:
                        setVelocity(action.speedRevMin);
                        break;
                    case ODriveState::Alarm:
                        setVelocity(0);
                        break;
                }
            }

            intf.pump();
        }
    }
    void ODriveSpindle::initializationSequence()
    {
        if (lastRequestedState == ODriveState::NotInitialized)
        {
            log_info("Setting up ODrive spindle on tx=" << canTx.name() << ", rx=" << canRx.name() << ", baud=" << baudKbit << "kb");

            // Register callbacks for the heartbeat and encoder feedback messages
            intf.onFeedback(onFeedback, this);
            intf.onStatus(onHeartbeat, this);

            intf.init(canTx, canRx, baudKbit);

            log_info("Waiting for ODrive to come online.");
            while (!heartBeatValid())
            {
                intf.pump();
                delay_ms(100);
            }

            {
                // Get version
                Get_Version_msg_t msg;
                if (!intf.transmit(odriveNodeId, msg))
                {
                    log_error("Failed to get ODrive version");
                }

                log_info("Found ODrive v" << int(msg.Fw_Version_Major) << "." << int(msg.Fw_Version_Minor) << "."
                                          << int(msg.Fw_Version_Revision) << ", hardware v" << int(msg.Hw_Version_Major) << "."
                                          << int(msg.Hw_Version_Minor) << "." << int(msg.Hw_Version_Variant) << ", protocol v"
                                          << int(msg.Protocol_Version) << ". Attempting to read bus voltage and current.");
            }

            Get_Bus_Voltage_Current_msg_t vbus;
            if (!intf.transmit(odriveNodeId, vbus))
            {
                log_error("Failed to read bus voltage and current. Fatal error!");
                send_alarm_from_ISR(ExecAlarm::AbortCycle);
                return;
            }

            log_info("DC voltage [V]: " << vbus.Bus_Voltage << "DC current [A]: " << vbus.Bus_Current);
        }

        lastRequestedState = ODriveState::Idle;
    }

    void ODriveSpindle::setAxisMode(ODriveAxisState axisMode)
    {
        do
        {
            {
                // Clear errors:
                Clear_Errors_msg_t msg;
                intf.send(odriveNodeId, msg);
            }

            delay_ms(10);

            {
                // Set closed loop control:
                Set_Axis_State_msg_t msg;
                msg.Axis_Requested_State = (uint32_t)ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
                intf.send(odriveNodeId, msg);
            }

            // Pump events for 150ms. This delay is needed for two reasons;
            // 1. If there is an error condition, such as missing DC power, the ODrive might
            //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
            //    on the first heartbeat response, so we want to receive at least two
            //    heartbeats (100ms default interval).
            // 2. If the bus is congested, the setState command won't get through
            //    immediately but can be delayed.
            for (int i = 0; i < 15; ++i)
            {
                delay_ms(10);
                intf.pump();
            }
        } while (lastAxisState != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

        // Set to velocity control:
        // {
        //     Set_Controller_Mode_msg_t msg;
        //     msg.Control_Mode = ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL;  // ODriveControlMode
        //     msg.Input_Mode   = ODriveInputMode::INPUT_MODE_VEL_RAMP;              // ODriveInputMode
        //     intf.send(odriveNodeId, msg);
        // }

        // set limits:
        // {
        //     Set_Limits_msg_t msg;
        //
        //     msg.Velocity_Limit = 3000;
        //     msg.Current_Limit  = 11;
        //     intf.send(odriveNodeId, msg);
        // }
    }

    void ODriveSpindle::setVelocity(int32_t revPerMin)
    {
        float revPerSec = revPerMin / 60.0f;

        // set velocity:
        {
            Set_Input_Vel_msg_t msg;
            msg.Input_Vel       = revPerSec;
            msg.Input_Torque_FF = 0.0f;  // torque_feedforward
            intf.send(odriveNodeId, msg);
        }
    }

    void ODriveSpindle::checkState()
    {
        {
            // get feedback:
            Get_Encoder_Estimates_msg_t msg;
            intf.transmit(odriveNodeId, msg);
            // msg.Pos_Estimate
            // msg.Vel_Estimate

            // TODO: If we're in 'spindle follower mode', the velocity of the CNC should match
            // the velocity estimate. Or better perhaps, it should match the position... but
            // that *does* require very frequent polling.
        }
    }

    /*
    // set pos gain:
    {
        Set_Pos_Gain_msg_t msg;
        msg.Pos_Gain = pos_gain;
        intf.send(odriveNodeId, msg);
    }

    // set velocity gain:
    {
        Set_Vel_Gains_msg_t msg;

        msg.Vel_Gain            = vel_gain;
        msg.Vel_Integrator_Gain = vel_integrator_gain;

        intf.send(odriveNodeId, msg);
    }

    {
        Set_Absolute_Position_msg_t msg;

        msg.Position = abs_pos;

        intf.send(odriveNodeId, msg);
    }

    {
        Set_Traj_Vel_Limit_msg_t msg;

        msg.Traj_Vel_Limit = vel_limit;

        intf.send(odriveNodeId, msg);
    }

    {
        Set_Traj_Accel_Limits_msg_t msg;

        msg.Traj_Accel_Limit = accel_limit;
        msg.Traj_Decel_Limit = decel_limit;

        intf.send(odriveNodeId, msg);
    }

    // set limits:
    {
        Set_Limits_msg_t msg;

        msg.Velocity_Limit = 3000;
        msg.Current_Limit  = 11;
        intf.send(odriveNodeId, msg);
    }

    // set velocity:
    {
        Set_Input_Vel_msg_t msg;
        msg.Input_Vel       = velocity;
        msg.Input_Torque_FF = 0.0f;  // torque_feedforward
        intf.send(odriveNodeId, msg);
    }

    // set controller mode:
    {
        Set_Controller_Mode_msg_t msg;
        msg.Control_Mode = control_mode;  // ODriveControlMode
        msg.Input_Mode   = input_mode;    // ODriveInputMode
        intf.send(odriveNodeId, msg);
    }

    {
    // get feedback:
        Get_Encoder_Estimates_msg_t msg;
        intf.transmit(odriveNodeId, msg);
        // msg.Pos_Estimate
        // msg.Vel_Estimate
    }

    {
        // Get currents
        Get_Iq_msg_t msg;
        intf.transmit(odriveNodeId, msg);
    }

    {
        // Get temperature
        Get_Temperature_msg_t msg;
        intf.transmit(odriveNodeId, msg);
    }

    {
        // Get error
        Get_Error_msg_t msg;
        intf.transmit(odriveNodeId, msg);
    }
    */

    // Configuration registration
    namespace
    {
        SpindleFactory::InstanceBuilder<ODriveSpindle> registration("ODrive");
    }
}
