#pragma once

#include "Pin.h"
#include "can_simple_messages.hpp"

#include <cstdlib>

class ODriveESP32
{
    static const uint8_t kNodeIdShift = 5;
    static const uint8_t kCmdIdBits   = 0x1F;

    static const uint8_t REQUEST_PENDING = 0xFF;

    uint32_t requestedMessageId = REQUEST_PENDING;
    uint8_t  responseData[8];

    bool transmit(uint32_t id, uint8_t len, const uint8_t* buffer);

    void* axis_state_user_data_;
    void* feedback_user_data_;

    void (*axis_state_callback_)(uint8_t nodeId, Heartbeat_msg_t& feedback, void* user_data)           = nullptr;
    void (*feedback_callback_)(uint8_t nodeId, Get_Encoder_Estimates_msg_t& feedback, void* user_data) = nullptr;

public:
    void onFeedback(void (*callback)(uint8_t nodeId, Get_Encoder_Estimates_msg_t& feedback, void* user_data), void* user_data = nullptr)
    {
        feedback_callback_  = callback;
        feedback_user_data_ = user_data;
    }

    void onStatus(void (*callback)(uint8_t nodeId, Heartbeat_msg_t& feedback, void* user_data), void* user_data = nullptr)
    {
        axis_state_callback_  = callback;
        axis_state_user_data_ = user_data;
    }

    void init(const Pin& tx, const Pin& rx);

    template <typename T>
    bool transmit(uint8_t nodeId, T& msg)
    {
        uint32_t id = (uint32_t(nodeId) << kNodeIdShift) | uint32_t(msg.cmd_id);
        uint8_t  buf[8];
        msg.encode_buf(buf);

        requestedMessageId = id;

        if (transmit(id, msg.msg_length, buf))
        {
            if (waitForResponse())
            {
                msg.decode_buf(responseData);
                return true;
            }
        }
        return false;
    }

    template <typename T>
    bool send(uint8_t nodeId, T& msg)
    {
        uint32_t id = (uint32_t(nodeId) << kNodeIdShift) | uint32_t(msg.cmd_id);
        uint8_t  buf[8];
        msg.encode_buf(buf);

        return transmit(id, msg.msg_length, buf);
    }

    bool waitForResponse();

    void pump();

    ~ODriveESP32();
};
