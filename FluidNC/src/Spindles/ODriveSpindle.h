// Copyright (c) 2025 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Spindles/Spindle.h"
#include "ODrive/can_simple_messages.hpp"
#include "ODrive/ODriveEnums.h"
#include "../CAN/CanBus.h"

#include "Logging.h"

#include <atomic>
#include <freertos/FreeRTOS.h>  // must be first

// queue and task
#include <freertos/task.h>
#include <freertos/queue.h>

namespace Spindles {
    struct ODriveAction;

    class ODriveSpindle : public Spindle, public CAN::CanListener {
    private:
        struct CanResponse {
            uint32_t cmd_id;
            uint8_t  len;
            uint8_t  data[8];
        };
	    enum class ODriveState {
			Uninitialized,
			Initialized,
			Error,
	    };

	    
        static const uint8_t kNodeIdShift = 5;
        static const uint8_t kCmdIdBits   = 0x1F;

        int32_t  _current_dev_speed   = -1;
        uint32_t _last_speed          = 0;
        Percent  _last_override_value = 100;  // no override is 100 percent

        static const int32_t _default_ramp_delay = 1'000;  // 1 second - safe for int32_t tick overflow

        void set_mode(SpindleState mode, bool critical);

        static QueueHandle_t    cmd_queue;
        static QueueHandle_t    speed_queue;
        static TaskHandle_t     cmdTaskHandle;
        static std::atomic<bool> _shutdown;
        static void             cmd_task(void* pvParameters);

        // Bus access lives in the .cpp so that this header does not have to pull in
        // MachineConfig.h, which would create an include cycle through Spindle.h.
        bool sendFrame(uint32_t cmd_id, uint8_t len, const uint8_t* data);
        bool awaitResponse(uint32_t cmd_id, uint8_t* data, uint8_t* len, int timeout_ms);
        bool awaitHeartbeat(int timeout_ms);

        template <typename T>
        bool send(T& msg) {
            uint8_t data[8] = { 0 };
            msg.encode_buf(data);
            return sendFrame(msg.cmd_id, msg.msg_length, data);
        }

        template <typename T>
        bool request(T& msg, int timeout_ms = 1000) {
            // A remote transmission request; the ODrive answers with the same command id.
            if (!sendFrame(msg.cmd_id, 0, nullptr)) {
                log_warn("ODrive: could not queue request " << int(msg.cmd_id));
                return false;
            }

            uint8_t responseData[8] = { 0 };
            uint8_t count           = 0;
            if (!awaitResponse(msg.cmd_id, responseData, &count, timeout_ms)) {
                log_warn("ODrive: request " << int(msg.cmd_id) << " timed out");
                return false;
            }
            if (count < msg.msg_length) {
                log_warn("ODrive: incomplete response, " << int(count) << " bytes");
                return false;
            }
            msg.decode_buf(responseData);
            return true;
        }

        bool setState(ODrive::ODriveAxisState state);
        bool setSpeedCommand(int32_t rpm, bool sync);
        void setClosedLoopControl();
        void setIdleControl();
        void initializationSequence();
        void invokeAction(ODriveAction& action);
        
        // Helper for setState
        void setSpeed(int32_t dev_speed, bool sync = true);

    protected:
        uint32_t _retries = 5;

        int32_t         ODriveNodeId  = 1;
        int64_t         lastHeartbeat = 0;
        volatile double lastPosition  = 0.0;
        volatile double lastVelocity  = 0.0;
        uint8_t         lastAxisState = 0;
        uint32_t        lastAxisError = 0;
        int32_t         maxSpeed      = 4000;
        double          gearFactor    = 1.0;

        // Responses to request<T>() are handed over from the shared CAN RX task.
        QueueHandle_t _responseQueue = nullptr;

        // Deprecated: superseded by the top-level "can:" section.  Still honoured so that
        // existing machine configs keep working; see init().
        Pin txPin;
        Pin rxPin;

        volatile bool _syncing;

        ODriveState state = ODriveState::Uninitialized;

    public:
        uint8_t _debug = 1;

        ODriveSpindle(const char* name) : Spindle(name) {}
        ODriveSpindle(const ODriveSpindle&)            = delete;
        ODriveSpindle(ODriveSpindle&&)                 = delete;
        ODriveSpindle& operator=(const ODriveSpindle&) = delete;
        ODriveSpindle& operator=(ODriveSpindle&&)      = delete;

        void init();
        void config_message();
        void setState(SpindleState state, SpindleSpeed speed);

        void onCanFrame(uint32_t id, uint8_t len, const uint8_t* data, int64_t rx_time_us) override;
        void setSpeedfromISR(uint32_t dev_speed) override;
        void reset() override;

        // ISR-safe callback for constant surface speed support
        SpeedCallbackInfo getISRSpeedCallback() override;
        static void isrSpeedCallback(uint32_t dev_speed, void* userData);

        // volatile uint32_t _sync_dev_speed;
        uint32_t     _sync_dev_speed;

        // Configuration handlers:
        void validate() override;
        void afterParse() override;
        void group(Configuration::HandlerBase& handler) override;

        // Graceful shutdown for the background task (used by unit tests)
        static void requestShutdown();
        static bool isShutdownRequested();
        static void resetShutdown();

        virtual ~ODriveSpindle() {}
    };
}
