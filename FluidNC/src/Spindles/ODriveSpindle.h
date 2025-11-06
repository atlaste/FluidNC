// Copyright (c) 2025 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Spindles/Spindle.h"
#include "ODrive/CanESP32.h"

#include "Uart.h"

namespace Spindles {
    class ODriveSpindle : public Spindle {
    private:
        static const uint8_t kNodeIdShift         = 5;
        static const uint8_t kCmdIdBits           = 0x1F;

        int32_t  _current_dev_speed   = -1;
        uint32_t _last_speed          = 0;
        Percent  _last_override_value = 100;  // no override is 100 percent

        int32_t _default_ramp_delay = 5000;

        void set_mode(SpindleState mode, bool critical);

        
    template <typename T>
        bool send(T& msg) {
            uint8_t data[8] = { 0 };
            msg.encode_buf(data);
            return can->send((ODriveNodeId << kNodeIdShift) | msg.cmd_id, msg.msg_length, data);
        }

        template <typename T>
        bool request(T& msg, int timeout_ms = 1000) {
            can->send((ODriveNodeId << kNodeIdShift) | msg.cmd_id,
                      0,       // no data
                      nullptr  // RTR=1
            );

            uint8_t  responseData[8];
            uint32_t messageId;
            uint32_t recvNodeId;
            int      count = receive(responseData, &messageId, &recvNodeId, timeout_ms);
            if (count == msg.msg_length) {
                if (messageId == msg.cmd_id) {
                    msg.decode_buf(responseData);
                    return true;
                } else {
                    std::cout << "Received unexpected message ID: " << messageId << std::endl;
                }
            } else if (count < msg.msg_length) {
                std::cout << "Received incomplete message: " << count << " bytes" << std::endl;
            } else {
                std::cout << "Request timed out" << std::endl;
            }

            return false;
        }

        int receive(uint8_t* responseData, uint32_t* messageId, uint32_t* nodeId, int timeout_ms = 1000);
        void pump();
        bool setState(ODrive::ODriveAxisState state);
        bool setSpeed(int rpm);
        void setClosedLoopControl();
        void setIdleControl();
        void initializationSequence();
        void invokeAction(ODriveAction& action);

    protected:
        uint32_t _retries = 5;

        int32_t           ODriveNodeId  = 1;
        ODrive::CanESP32* can           = nullptr;
        int64_t           lastHeartbeat = 0;
        float             lastPosition  = 0.0f;
        float             lastVelocity  = 0.0f;
        uint8_t           lastAxisState = 0;
        uint32_t          lastAxisError = 0;

        Pin txPin;
        Pin rxPin;

        void setSpeed(uint32_t dev_speed);

        volatile bool _syncing;

    public:
        uint8_t _debug = 0;

        ODriveSpindle(const char* name) : Spindle(name) {}
        ODriveSpindle(const ODriveSpindle&)            = delete;
        ODriveSpindle(ODriveSpindle&&)                 = delete;
        ODriveSpindle& operator=(const ODriveSpindle&) = delete;
        ODriveSpindle& operator=(ODriveSpindle&&)      = delete;

        void init();
        void config_message();
        void setState(SpindleState state, SpindleSpeed speed);
        void setSpeedfromISR(uint32_t dev_speed) override;

        // volatile uint32_t _sync_dev_speed;
        uint32_t     _sync_dev_speed;
        SpindleSpeed _slop;

        // Configuration handlers:
        void validate() override;
        void afterParse() override;
        void group(Configuration::HandlerBase& handler) override;

        virtual ~ODriveSpindle() {}
    };
}
