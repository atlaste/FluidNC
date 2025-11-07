// Copyright (c) 2024. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Config.h"
#include "Module.h"
#include "Configuration/Configurable.h"
#include "AsyncWebSocket.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <cstdint>

class AsyncWebSocket;
class AsyncWebSocketClient;

// Forward declarations for usb_stream types
struct uvc_frame;
typedef struct uvc_frame uvc_frame_t;

namespace WebUI {
    class UVCCamera : public ConfigurableModule {
    public:
        UVCCamera(const char* name);
        ~UVCCamera();

        UVCCamera(const UVCCamera&)            = delete;
        UVCCamera(UVCCamera&&)                 = delete;
        UVCCamera& operator=(const UVCCamera&) = delete;
        UVCCamera& operator=(UVCCamera&&)      = delete;

        // Module interface
        void init() override;
        void deinit() override;
        int  init_priority() override { return 0x8000; }  // After WebUI_Server

        // Configuration interface
        void validate() override;
        void group(Configuration::HandlerBase& handler) override;

        // Frame buffer structure (needs to be public for callbacks)
        struct FrameBuffer {
            uint8_t* data;
            size_t   size;
            size_t   capacity;
            bool     ready;
        };

    private:
        // Configuration items
        bool    _enable           = false;
        int32_t _frame_rate_limit = 5;  // fps (1-30)
        int32_t _preferred_width  = 640;
        int32_t _preferred_height = 480;
        int32_t _buffer_count     = 2;  // 2-4 buffers

        // Runtime state
        bool              _initialized    = false;
        bool              _camera_present = false;
        bool              _error          = false;
        AsyncWebSocket*   _camera_socket  = nullptr;
        TaskHandle_t      _capture_task   = nullptr;
        SemaphoreHandle_t _buffer_mutex   = nullptr;

        // Frame buffer ring
        FrameBuffer* _frame_buffers = nullptr;
        uint8_t      _write_index   = 0;
        uint8_t      _read_index    = 0;

        // UVC streaming state and buffers
        bool     _uvc_initialized = false;
        uint8_t* _uvc_xfer_buffer_a = nullptr;
        uint8_t* _uvc_xfer_buffer_b = nullptr;
        uint8_t* _uvc_frame_buffer = nullptr;

        // Client tracking
        static constexpr size_t MAX_CLIENTS                  = 4;
        uint32_t                _active_clients[MAX_CLIENTS] = { 0 };
        size_t                  _active_client_count         = 0;

        // Private methods
        bool initUVC();
        void deinitUVC();
        bool setupWebSocket();
        void cleanupWebSocket();

        bool allocateFrameBuffers();
        void freeFrameBuffers();

        FrameBuffer* getReadBuffer();
        void         advanceReadIndex();

        bool addClient(uint32_t client_id);
        void removeClient(uint32_t client_id);

        void streamFrameToClients();

        // Static task wrapper
        static void captureTaskWrapper(void* param);
        void        captureTask();

        // Websocket event handler
        void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);

    public:
        // Public accessors for C-style callbacks
        void              setCameraPresent(bool present) { _camera_present = present; }
        bool              hasActiveClients();
        FrameBuffer*      getWriteBuffer();
        void              advanceWriteIndex();
        SemaphoreHandle_t getBufferMutex() { return _buffer_mutex; }
    };
}
