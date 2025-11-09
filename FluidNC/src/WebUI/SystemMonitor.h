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

namespace WebUI {
    class SystemMonitor : public ConfigurableModule {
    public:
        SystemMonitor(const char* name);
        ~SystemMonitor();

        SystemMonitor(const SystemMonitor&)            = delete;
        SystemMonitor(SystemMonitor&&)                 = delete;
        SystemMonitor& operator=(const SystemMonitor&) = delete;
        SystemMonitor& operator=(SystemMonitor&&)      = delete;

        // Module interface
        void init() override;
        void deinit() override;
        int  init_priority() override { return 0x5300; }  // After WebUI_Server

        // Configuration interface
        void validate() override;
        void group(Configuration::HandlerBase& handler) override;

    private:
        // Configuration items
        bool    _enable        = false;
        int32_t _update_rate   = 1000;  // Update rate in milliseconds (100-5000)
        bool    _track_cpu     = true;
        bool    _track_memory  = true;
        bool    _track_tasks   = true;

        // Runtime state
        bool            _initialized = false;
        bool            _error       = false;
        AsyncWebSocket* _monitor_socket = nullptr;
        TaskHandle_t    _monitor_task = nullptr;

        // Client tracking
        static constexpr size_t MAX_CLIENTS                  = 4;
        uint32_t                _active_clients[MAX_CLIENTS] = { 0 };
        size_t                  _active_client_count         = 0;

        // Private methods
        bool setupWebSocket();
        void cleanupWebSocket();

        bool addClient(uint32_t client_id);
        void removeClient(uint32_t client_id);
        bool hasActiveClients();

        void collectAndBroadcastStats();
        void collectSystemStats(char* json_buffer, size_t buffer_size);

        // Static task wrapper
        static void monitorTaskWrapper(void* param);
        void        monitorTask();

        // Websocket event handler
        void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);
    };
}

