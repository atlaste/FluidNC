// Copyright (c) 2024. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Config.h"
#include "Module.h"
#include "Configuration/Configurable.h"
#include "AsyncWebSocket.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <cstdint>
#include <atomic>

// ESP-IDF performance monitoring
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
#include <perfmon.h>
#include <esp_debug_helpers.h>
#include <driver/timer.h>
#define HAS_PERFMON 1
#endif

class AsyncWebSocket;
class AsyncWebSocketClient;

namespace WebUI {
    class PerformanceProfiler : public ConfigurableModule {
    public:
        PerformanceProfiler(const char* name);
        ~PerformanceProfiler();

        PerformanceProfiler(const PerformanceProfiler&)            = delete;
        PerformanceProfiler(PerformanceProfiler&&)                 = delete;
        PerformanceProfiler& operator=(const PerformanceProfiler&) = delete;
        PerformanceProfiler& operator=(PerformanceProfiler&&)      = delete;

        // Module interface
        void init() override;
        void deinit() override;
        int  init_priority() override { return 0x5600; }  // After WebUI_Server (0x5500)

        // Configuration interface
        void validate() override;
        void group(Configuration::HandlerBase& handler) override;

        // Profiling control (called via WebSocket commands)
        bool startProfiling();
        void stopProfiling();
        bool isProfiling() const { return _profiling; }

    private:
        // Configuration items
        bool    _enable             = false;
        bool    _enable_pc_sampling = false;  // Enable PC sampling (optional)
        int32_t _update_rate        = 1000;   // How often to send results to browser (ms)
        
        // Runtime state
        bool            _initialized = false;
        bool            _error       = false;
        AsyncWebSocket* _profiler_socket = nullptr;

        // Profiling state
        volatile bool   _profiling = false;
        
        // Stack profiling (hash table approach from PerfProf)
        struct StackItem {
            intptr_t      caller;
            intptr_t      callee;
            uint_fast32_t calls;
            uint_fast32_t cycles;
        };
        
        static const int HashSize      = 7459;  // Prime number
        static const int MaxCollisions = 5;
        
        StackItem*          _hashTable = nullptr;
        std::atomic<bool>   _locked = true;
        volatile uint64_t   _total_cycles = 0;  // Total cycles across all samples (for debugging)
        volatile uint32_t   _sample_count = 0;  // Number of stack samples taken (for percentage calc)
        
        // Performance counters broadcast task
        TaskHandle_t    _monitor_task = nullptr;
        uint64_t        _last_cycles = 0;
        uint32_t        _last_timestamp = 0;
        
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

        void broadcastStatus();
        void broadcastHashTableResults();
        void broadcastPerfCounters();

        // Stack profiling
        void walkStack();
        static bool timerISRCallback(void* args);
        static void initializeProfilerTimer(void* parameters);
        
        // Performance counter monitoring
        static void monitorTaskWrapper(void* param);
        void monitorTask();
        
        // Websocket event handler
        void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);
        void handleCommand(AsyncWebSocketClient* client, const char* data, size_t len);
    };
}

