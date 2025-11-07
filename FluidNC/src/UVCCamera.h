// Copyright (c) 2024. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Config.h"
#include "Module.h"
#include "Configuration/Configurable.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class AsyncWebSocket;
class AsyncWebSocketClient;

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
    int  init_priority() override { return 0x5200; }  // After WebUI_Server (0x5100)

    // Configuration interface
    void validate() override;
    void group(Configuration::HandlerBase& handler) override;

private:
    // Configuration items
    bool    _enable            = false;
    int32_t _frame_rate_limit  = 1;      // fps (1-30)
    int32_t _preferred_width   = 1920;
    int32_t _preferred_height  = 1080;
    int32_t _buffer_count      = 3;      // 2-4 buffers

    // Runtime state
    bool               _initialized     = false;
    bool               _camera_present  = false;
    bool               _error           = false;
    AsyncWebSocket*    _camera_socket   = nullptr;
    TaskHandle_t       _capture_task    = nullptr;
    SemaphoreHandle_t  _buffer_mutex    = nullptr;

    // Frame buffer ring
    struct FrameBuffer {
        uint8_t* data;
        size_t   size;
        size_t   capacity;
        bool     ready;
    };
    FrameBuffer* _frame_buffers = nullptr;
    uint8_t      _write_index   = 0;
    uint8_t      _read_index    = 0;

    // UVC device handle (opaque pointer to avoid including UVC headers everywhere)
    void* _uvc_device_handle = nullptr;

    // Client tracking
    static constexpr size_t MAX_CLIENTS = 4;
    uint32_t _active_clients[MAX_CLIENTS] = {0};
    size_t   _active_client_count         = 0;

    // Private methods
    bool initUVC();
    void deinitUVC();
    bool setupWebSocket();
    void cleanupWebSocket();
    
    bool allocateFrameBuffers();
    void freeFrameBuffers();
    
    FrameBuffer* getWriteBuffer();
    FrameBuffer* getReadBuffer();
    void advanceWriteIndex();
    void advanceReadIndex();
    
    bool addClient(uint32_t client_id);
    void removeClient(uint32_t client_id);
    bool hasActiveClients();
    
    void streamFrameToClients();
    
    // Static task wrapper
    static void captureTaskWrapper(void* param);
    void captureTask();
    
    // Static websocket event handler
    static void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, 
                                 int type, void* arg, uint8_t* data, size_t len);
};

