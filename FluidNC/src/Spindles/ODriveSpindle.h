#pragma once

#include "Spindle.h"
#include "ODrive/ODriveESP32.h"
#include "ODrive/ODriveEnums.h"
#include "ODrive/can_simple_messages.hpp"

#include <cstdint>

namespace Spindles
{
    enum class ODriveState
    {
        NotInitialized,
        Idle,
        ClosedLoop,
        RunningCW,
        RunningCCW,
        Alarm,
    };

    enum class ODriveActionType
    {
        Initialize,
        Enable,
        RunCW,
        RunCCW,
        Stop,
        Disable,
        SetSpeed,
    };

    class ODriveAction
    {
    public:
        ODriveActionType action      = ODriveActionType::Initialize;
        uint32_t         speedRevMin = 0;
    };

    // This adds support for ODriveSpindle spindles
    class ODriveSpindle : public Spindle
    {
    private:
        static QueueHandle_t oDriveCommandQueue;
        static TaskHandle_t  oDriveCommandTaskHandle;
        void                 oDriveCommandTask();
        static void          oDriveCommandTaskHelper(void* pvParameters)
        {
            // Use member function please:
            static_cast<ODriveSpindle*>(pvParameters)->oDriveCommandTask();
        }

        Pin canRx;
        Pin canTx;
        int32_t odriveNodeId = -1;

        ODriveESP32 intf;

        int64_t         lastHeartbeat;
        ODriveAxisState lastAxisState;
        ODriveError     lastAxisError;

        static void onHeartbeat(uint8_t nodeId, Heartbeat_msg_t& msg, void* user_data);
        bool        heartBeatValid();

        static void onFeedback(uint8_t nodeId, Get_Encoder_Estimates_msg_t& feedback, void* user_data);

        // state:
        ODriveState lastRequestedState = ODriveState::NotInitialized;
        uint32_t    lastSetSpeedRevMin = 0;

        int64_t lastFeedback;
        float   feedbackPosition = 0.0f;
        float   feedbackVelocity = 0.0f;

        void initializationSequence();
        void setAxisMode(ODriveAxisState axisMode);
        void setVelocity(int32_t revPerSec);
        void checkState();

    public:
        ODriveSpindle(const char* name) : Spindle(name) {}

        ODriveSpindle(const ODriveSpindle&)            = delete;
        ODriveSpindle(ODriveSpindle&&)                 = delete;
        ODriveSpindle& operator=(const ODriveSpindle&) = delete;
        ODriveSpindle& operator=(ODriveSpindle&&)      = delete;

        void init() override;
        void setSpeedfromISR(uint32_t dev_speed) override;
        void setState(SpindleState state, SpindleSpeed speed) override;
        void config_message() override;

        // Configuration handlers:
        void validate() override
        {
            Assert(odriveNodeId > 0 && odriveNodeId < 255, "ODrive node id must be set as [1-254]");
            Assert(canTx.defined(), "CAN bus TX pin must be set for ODrive");
            Assert(canRx.defined(), "CAN bus RX pin must be set for ODrive");
            Spindle::validate();
        }

        void group(Configuration::HandlerBase& handler) override
        {
            handler.item("can_rx", canRx);
            handler.item("can_tx", canTx);
            handler.item("odrive_node_id", odriveNodeId);

            Spindle::group(handler);
        }

        virtual ~ODriveSpindle() {}
    };
}
