// Copyright (c) 2024. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

// System Monitor - Tracks CPU and memory usage per task, similar to Task Manager/top

#include "SystemMonitor.h"
#include "WebUI/WebUIServer.h"
#include "Logging.h"

#include <ESPAsyncWebServer.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>
#include <cstdio>

namespace WebUI {
    SystemMonitor::SystemMonitor(const char* name) : ConfigurableModule(name) {}

    SystemMonitor::~SystemMonitor() {
        deinit();
    }

    void SystemMonitor::validate() {
        if (_update_rate < 100 || _update_rate > 5000) {
            log_error("SystemMonitor update_rate must be between 100 and 5000 ms");
            _error = true;
        }
    }

    void SystemMonitor::group(Configuration::HandlerBase& handler) {
        handler.item("enable", _enable);
        handler.item("update_rate", _update_rate, 100, 5000);
        handler.item("track_cpu", _track_cpu);
        handler.item("track_memory", _track_memory);
        handler.item("track_tasks", _track_tasks);
    }

    void SystemMonitor::init() {
        if (_error || !_enable) {
            if (!_enable) {
                log_info("SystemMonitor module disabled in configuration");
            }
            return;
        }

        log_info("Initializing SystemMonitor module");
        log_info("  Update rate: " << _update_rate << " ms");
        log_info("  Track CPU: " << (_track_cpu ? "yes" : "no"));
        log_info("  Track Memory: " << (_track_memory ? "yes" : "no"));
        log_info("  Track Tasks: " << (_track_tasks ? "yes" : "no"));

        // Setup websocket endpoint
        if (!setupWebSocket()) {
            log_error("Failed to setup websocket");
            _error = true;
            return;
        }

        // Create monitor task
        BaseType_t result = xTaskCreatePinnedToCore(
            monitorTaskWrapper,
            "sys_monitor",
            4096,
            this,
            1,  // Low priority - don't interfere with system
            &_monitor_task,
            0   // Core 0
        );

        if (result != pdPASS) {
            log_error("Failed to create SystemMonitor task");
            _error = true;
            cleanupWebSocket();
            return;
        }

        _initialized = true;
        log_info("SystemMonitor module initialized");
    }

    void SystemMonitor::deinit() {
        if (!_initialized) {
            return;
        }

        log_info("Deinitializing SystemMonitor module");

        // Stop monitor task
        if (_monitor_task) {
            vTaskDelete(_monitor_task);
            _monitor_task = nullptr;
        }

        // Cleanup websocket
        cleanupWebSocket();

        _initialized = false;
    }

    bool SystemMonitor::setupWebSocket() {
        // Get the webserver instance
        AsyncWebServer* webserver = WebUI::WebUI_Server::getWebServer();
        if (!webserver) {
            log_error("WebUI server not available");
            return false;
        }

        // Create websocket handler for /sysmon endpoint
        _monitor_socket = new AsyncWebSocket("/sysmon");
        if (!_monitor_socket) {
            log_error("Failed to create websocket");
            return false;
        }

        // Set event handler
        _monitor_socket->onEvent(
            [this](AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
                this->onWebSocketEvent(server, client, type, arg, data, len);
            });

        // Add handler to webserver
        webserver->addHandler(_monitor_socket);

        log_info("SystemMonitor websocket endpoint /sysmon registered");
        return true;
    }

    void SystemMonitor::cleanupWebSocket() {
        if (_monitor_socket) {
            _monitor_socket->closeAll();
            _monitor_socket = nullptr;
        }
        _active_client_count = 0;
    }

    bool SystemMonitor::addClient(uint32_t client_id) {
        if (_active_client_count >= MAX_CLIENTS) {
            return false;
        }

        // Check if already present
        for (size_t i = 0; i < _active_client_count; i++) {
            if (_active_clients[i] == client_id) {
                return true;
            }
        }

        _active_clients[_active_client_count++] = client_id;
        log_info("SystemMonitor client connected: " << client_id << " (total: " << _active_client_count << ")");
        return true;
    }

    void SystemMonitor::removeClient(uint32_t client_id) {
        for (size_t i = 0; i < _active_client_count; i++) {
            if (_active_clients[i] == client_id) {
                // Shift remaining clients down
                for (size_t j = i; j < _active_client_count - 1; j++) {
                    _active_clients[j] = _active_clients[j + 1];
                }
                _active_client_count--;
                log_info("SystemMonitor client disconnected: " << client_id << " (total: " << _active_client_count << ")");
                break;
            }
        }
    }

    bool SystemMonitor::hasActiveClients() {
        return _active_client_count > 0;
    }

    void SystemMonitor::collectSystemStats(char* json_buffer, size_t buffer_size) {
        size_t offset = 0;
        
        // Start JSON
        offset += snprintf(json_buffer + offset, buffer_size - offset, "{");

        // Memory statistics
        if (_track_memory) {
            multi_heap_info_t heap_info;
            heap_caps_get_info(&heap_info, MALLOC_CAP_INTERNAL);
            
            size_t total_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
            size_t total_size = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
            size_t largest_free = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
            
            // PSRAM stats if available
            size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
            size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
            
            offset += snprintf(json_buffer + offset, buffer_size - offset,
                "\"memory\":{\"total\":%u,\"free\":%u,\"used\":%u,\"largest_free\":%u,\"psram_total\":%u,\"psram_free\":%u},",
                total_size, total_free, total_size - total_free, largest_free, psram_total, psram_free);
        }

        // CPU and task statistics
        if (_track_cpu || _track_tasks) {
            offset += snprintf(json_buffer + offset, buffer_size - offset, "\"tasks\":[");
            
            // Get number of tasks
            UBaseType_t task_count = uxTaskGetNumberOfTasks();
            
            // Allocate array for task status
            TaskStatus_t* task_array = (TaskStatus_t*)malloc(task_count * sizeof(TaskStatus_t));
            if (task_array) {
                uint32_t total_runtime;
                task_count = uxTaskGetSystemState(task_array, task_count, &total_runtime);
                
                // Avoid divide by zero
                if (total_runtime == 0) {
                    total_runtime = 1;
                }
                
                for (UBaseType_t i = 0; i < task_count; i++) {
                    uint32_t cpu_percent = (task_array[i].ulRunTimeCounter * 100) / total_runtime;
                    uint32_t stack_free = task_array[i].usStackHighWaterMark;
                    
                    // Task state as string
                    const char* state_str = "Unknown";
                    switch (task_array[i].eCurrentState) {
                        case eRunning:   state_str = "Running"; break;
                        case eReady:     state_str = "Ready"; break;
                        case eBlocked:   state_str = "Blocked"; break;
                        case eSuspended: state_str = "Suspended"; break;
                        case eDeleted:   state_str = "Deleted"; break;
                    }
                    
                    offset += snprintf(json_buffer + offset, buffer_size - offset,
                        "%s{\"name\":\"%s\",\"priority\":%u,\"cpu\":%u,\"stack_free\":%u,\"state\":\"%s\"}",
                        (i > 0 ? "," : ""),
                        task_array[i].pcTaskName,
                        task_array[i].uxCurrentPriority,
                        cpu_percent,
                        stack_free,
                        state_str);
                }
                
                free(task_array);
            }
            
            offset += snprintf(json_buffer + offset, buffer_size - offset, "]");
        }

        // System uptime (in seconds)
        uint32_t uptime_sec = esp_log_timestamp() / 1000;
        offset += snprintf(json_buffer + offset, buffer_size - offset, ",\"uptime\":%u", uptime_sec);

        // End JSON
        offset += snprintf(json_buffer + offset, buffer_size - offset, "}");
    }

    void SystemMonitor::collectAndBroadcastStats() {
        if (!_monitor_socket || !hasActiveClients()) {
            return;
        }

        // Allocate buffer for JSON (8KB should be enough for stats)
        const size_t buffer_size = 8192;
        char* json_buffer = (char*)malloc(buffer_size);
        if (!json_buffer) {
            log_error("Failed to allocate stats buffer");
            return;
        }

        // Collect stats into JSON
        collectSystemStats(json_buffer, buffer_size);

        // Broadcast to all connected clients
        for (size_t i = 0; i < _active_client_count; i++) {
            AsyncWebSocketClient* client = _monitor_socket->client(_active_clients[i]);
            if (client && client->canSend()) {
                client->text(json_buffer);
            }
        }

        free(json_buffer);
    }

    void SystemMonitor::onWebSocketEvent(
        AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
        switch (type) {
            case WS_EVT_CONNECT:
                log_info("SystemMonitor websocket client connected from " << client->remoteIP().toString().c_str());
                addClient(client->id());
                break;

            case WS_EVT_DISCONNECT:
                log_info("SystemMonitor websocket client disconnected");
                removeClient(client->id());
                break;

            case WS_EVT_ERROR:
                log_error("SystemMonitor websocket error for client " << client->id());
                removeClient(client->id());
                break;

            case WS_EVT_DATA:
                // We don't expect data from clients (monitoring is one-way)
                break;

            case WS_EVT_PONG:
            case WS_EVT_PING:
                // Keep-alive
                break;
        }
    }

    void SystemMonitor::monitorTaskWrapper(void* param) {
        SystemMonitor* monitor = static_cast<SystemMonitor*>(param);
        monitor->monitorTask();
        vTaskDelete(NULL);
    }

    void SystemMonitor::monitorTask() {
        log_info("SystemMonitor task started");

        TickType_t update_delay = pdMS_TO_TICKS(_update_rate);
        TickType_t last_update = xTaskGetTickCount();

        while (true) {
            // Only collect and send stats if we have clients
            if (hasActiveClients()) {
                TickType_t current_time = xTaskGetTickCount();
                if (current_time - last_update >= update_delay) {
                    collectAndBroadcastStats();
                    last_update = current_time;
                }
            }

            // Small delay to prevent busy-waiting
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // Module registration
    ConfigurableModuleFactory::InstanceBuilder<SystemMonitor> system_monitor_module __attribute__((init_priority(0x5300))) ("system_monitor");
}

