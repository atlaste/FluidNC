// Copyright (c) 2024. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

// Performance Profiler - Statistical CPU sampling for performance analysis

#include "PerformanceProfiler.h"
#include "WebUI/WebUIServer.h"
#include "Logging.h"

#include <ESPAsyncWebServer.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <cstdio>

#ifdef HAS_PERFMON
#include <esp_cpu_utils.h>
#include <cstring>

// Compatibility for ESP-IDF stack walking
#ifndef esp_cpu_process_stack_pc
// #define esp_cpu_process_stack_pc(pc) (pc)
#endif

// Branch prediction hints
#ifndef likely
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#endif
#endif

namespace WebUI {
    PerformanceProfiler::PerformanceProfiler(const char* name) : ConfigurableModule(name) {}

    PerformanceProfiler::~PerformanceProfiler() {
        deinit();
    }

    void PerformanceProfiler::validate() {
        if (_enable_pc_sampling) {
            if (_max_samples < 100 || _max_samples > 100000) {
                log_error("PerformanceProfiler max_samples must be between 100 and 100000");
                _error = true;
            }
            if (_default_rate < 10 || _default_rate > _max_rate) {
                log_error("PerformanceProfiler default_rate must be between 10 and max_rate");
                _error = true;
            }
            if (_max_rate < 100 || _max_rate > 50000) {
                log_error("PerformanceProfiler max_rate must be between 100 and 50000");
                _error = true;
            }
        }
        if (_update_rate < 100 || _update_rate > 10000) {
            log_error("PerformanceProfiler update_rate must be between 100 and 10000");
            _error = true;
        }
    }

    void PerformanceProfiler::group(Configuration::HandlerBase& handler) {
        handler.item("enable", _enable);
        handler.item("enable_pc_sampling", _enable_pc_sampling);
        handler.item("max_samples", _max_samples, 100, 100000);
        handler.item("default_rate", _default_rate, 10, 50000);
        handler.item("max_rate", _max_rate, 100, 50000);
        handler.item("update_rate", _update_rate, 100, 10000);
    }

    void PerformanceProfiler::init() {
        if (_error || !_enable) {
            if (!_enable) {
                log_info("PerformanceProfiler module disabled in configuration");
            }
            return;
        }

        log_info("Initializing PerformanceProfiler module");
        log_info("  Stack profiling: " << (_enable_pc_sampling ? "enabled" : "disabled"));
        log_info("  Update rate: " << _update_rate << " ms");
        log_info("  HAS_PERFMON: " << 
#ifdef HAS_PERFMON
            "YES"
#else
            "NO"
#endif
        );

#ifdef HAS_PERFMON
        if (_enable_pc_sampling) {
            // Start locked so we don't collect during initialization
            _locked = true;

            // Allocate hash table for stack profiling
            _hashTable = new StackItem[HashSize + 1];  // +1 is sentinel
            if (!_hashTable) {
                log_error("Failed to allocate profiler hash table");
                _error = true;
                return;
            }
            memset(_hashTable, 0, sizeof(StackItem) * (HashSize + 1));

            // Initialize Xtensa performance monitoring
            esp_err_t err;
            err = xtensa_perfmon_init(0, XTPERF_CNT_CYCLES, XTPERF_MASK_CYCLES, 0, -1);  // CPU cycles
            if (err != ESP_OK) {
                log_error("Failed to init perfmon counter 0");
                delete[] _hashTable;
                _hashTable = nullptr;
                _error = true;
                return;
            }

            err = xtensa_perfmon_init(1, XTPERF_CNT_INSN, XTPERF_MASK_INSN_ALL, 0, -1);  // Instructions
            if (err != ESP_OK) {
                log_error("Failed to init perfmon counter 1");
                delete[] _hashTable;
                _hashTable = nullptr;
                _error = true;
                return;
            }

            xtensa_perfmon_start();
            log_info("  Perfmon counters started");
        }
#endif

        // Setup websocket endpoint
        if (!setupWebSocket()) {
            log_error("Failed to setup websocket");
            _error = true;
            if (_hashTable) {
                delete[] _hashTable;
                _hashTable = nullptr;
            }
            return;
        }

#ifdef HAS_PERFMON
        if (_enable_pc_sampling) {
            // Create timer tasks on each core (hardware timer initialization)
            xTaskCreatePinnedToCore(initializeProfilerTimer, "prof0", 2048, this, configMAX_PRIORITIES - 1, NULL, 0);

#ifndef CONFIG_FREERTOS_UNICORE
            xTaskCreatePinnedToCore(initializeProfilerTimer, "prof1", 2048, this, configMAX_PRIORITIES - 1, NULL, 1);
#endif
            // Wait for timer tasks to start
            vTaskDelay(pdMS_TO_TICKS(100));

            // Unlock to start collecting
            _locked = false;
            log_info("  Profiler timers started");
        }

        // Create monitoring task for continuous stats broadcast
        BaseType_t result = xTaskCreatePinnedToCore(
            monitorTaskWrapper,
            "perf_monitor",
            4096,
            this,
            1,  // Low priority
            &_monitor_task,
            0   // Core 0
        );

        if (result != pdPASS) {
            log_error("Failed to create performance monitor task");
            _error = true;
            cleanupWebSocket();
            if (_hashTable) {
                delete[] _hashTable;
                _hashTable = nullptr;
            }
            return;
        }
#endif

        _initialized = true;
        log_info("PerformanceProfiler module initialized");
    }

    void PerformanceProfiler::deinit() {
        if (!_initialized) {
            return;
        }

        log_info("Deinitializing PerformanceProfiler module");

        // Stop profiling if active
        stopProfiling();

        // Stop monitor task
        if (_monitor_task) {
            vTaskDelete(_monitor_task);
            _monitor_task = nullptr;
        }

        // Free hash table
        if (_hashTable) {
            delete[] _hashTable;
            _hashTable = nullptr;
        }

        // Cleanup websocket
        cleanupWebSocket();

        _initialized = false;
    }

    bool PerformanceProfiler::setupWebSocket() {
        // Get the webserver instance
        AsyncWebServer* webserver = WebUI::WebUI_Server::getWebServer();
        if (!webserver) {
            log_error("WebUI server not available");
            return false;
        }

        // Create websocket handler for /profiler endpoint
        _profiler_socket = new AsyncWebSocket("/profiler");
        if (!_profiler_socket) {
            log_error("Failed to create websocket");
            return false;
        }

        // Set event handler
        _profiler_socket->onEvent(
            [this](AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
                this->onWebSocketEvent(server, client, type, arg, data, len);
            });

        // Add handler to webserver
        webserver->addHandler(_profiler_socket);

        log_info("PerformanceProfiler websocket endpoint /profiler registered");
        return true;
    }

    void PerformanceProfiler::cleanupWebSocket() {
        if (_profiler_socket) {
            _profiler_socket->closeAll();
            _profiler_socket = nullptr;
        }
        _active_client_count = 0;
    }

    bool PerformanceProfiler::addClient(uint32_t client_id) {
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
        log_info("PerformanceProfiler client connected: " << client_id);
        return true;
    }

    void PerformanceProfiler::removeClient(uint32_t client_id) {
        for (size_t i = 0; i < _active_client_count; i++) {
            if (_active_clients[i] == client_id) {
                // Shift remaining clients down
                for (size_t j = i; j < _active_client_count - 1; j++) {
                    _active_clients[j] = _active_clients[j + 1];
                }
                _active_client_count--;
                log_info("PerformanceProfiler client disconnected: " << client_id);
                break;
            }
        }
    }

    bool PerformanceProfiler::hasActiveClients() {
        return _active_client_count > 0;
    }


    bool PerformanceProfiler::startProfiling(uint32_t duration_ms, uint32_t sample_rate_hz) {
        if (_profiling) {
            log_warn("Profiling already in progress");
            return false;
        }

        if (!_enable_pc_sampling || !_hashTable) {
            log_error("Stack profiling not enabled or not initialized");
            return false;
        }

        log_info("Starting stack profiling for " << duration_ms << "ms");

        // Clear hash table
        memset(_hashTable, 0, sizeof(StackItem) * (HashSize + 1));

        // Unlock to start collecting (timers are already running)
        _locked = false;
        _profiling = true;

        // Broadcast status
        broadcastStatus();
        
        return true;
    }

    void PerformanceProfiler::stopProfiling() {
        if (!_profiling) {
            return;
        }

        log_info("Stopping stack profiling");

        // Lock collection (timers keep running, just not collecting)
        _locked = true;
        _profiling = false;

        // Wait a bit to ensure no active collection
        vTaskDelay(pdMS_TO_TICKS(50));

        // Debug: Check if hash table has any entries before broadcast
        if (_enable_pc_sampling && _hashTable) {
            size_t entries = 0;
            for (int i = 0; i < HashSize; i++) {
                if (_hashTable[i].caller != 0) entries++;
            }
            log_info("Hash table has " << entries << " entries before broadcast");
            
            broadcastHashTableResults();
        }
        broadcastStatus();
    }

    void PerformanceProfiler::broadcastStatus() {
        if (!_profiler_socket || !hasActiveClients()) {
            return;
        }

        // Send status as JSON
        char buffer[256];
        snprintf(buffer, sizeof(buffer),
            "{\"type\":\"status\",\"profiling\":%s,\"enabled\":%s}",
            _profiling ? "true" : "false",
            _enable_pc_sampling ? "true" : "false");

        _profiler_socket->textAll(buffer);
    }

    void PerformanceProfiler::broadcastHashTableResults() {
        if (!_profiler_socket || !hasActiveClients() || !_hashTable) {
            return;
        }

        // Count valid items
        size_t total_items = 0;
        for (int i = 0; i < HashSize; i++) {
            if (_hashTable[i].caller != 0) total_items++;
        }

        log_info("Broadcasting " << total_items << " stack profiler results");

        // Build complete JSON (each WebSocket message must be complete, valid JSON)
        // Estimate: ~100 bytes per item, so reserve appropriate space
        String json;
        json.reserve(total_items * 120 + 256);  // Pre-allocate to avoid reallocations
        
        char header[128];
        snprintf(header, sizeof(header), "{\"type\":\"results\",\"count\":%u,\"items\":[", 
                static_cast<unsigned int>(total_items));
        json = header;
        
        size_t sent = 0;
        for (int i = 0; i < HashSize && sent < total_items; i++) {
            if (_hashTable[i].caller != 0) {
                if (sent > 0) json += ",";

                char item[128];
                snprintf(item, sizeof(item), 
                    "{\"caller\":\"0x%08x\",\"callee\":\"0x%08x\",\"calls\":%u,\"cycles\":%u}",
                    static_cast<unsigned int>(_hashTable[i].caller),
                    static_cast<unsigned int>(_hashTable[i].callee),
                    static_cast<unsigned int>(_hashTable[i].calls),
                    static_cast<unsigned int>(_hashTable[i].cycles));
                json += item;
                sent++;
            }
        }

        // Close JSON array
        json += "]}";
        
        // Send complete JSON as single WebSocket message
        log_info("Sending JSON payload: " << json.length() << " bytes");
        _profiler_socket->textAll(json);
    }

    void PerformanceProfiler::onWebSocketEvent(
        AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
        
        switch (type) {
            case WS_EVT_CONNECT:
                log_info("PerformanceProfiler client connected: " << client->id());
                addClient(client->id());
                broadcastStatus();
                break;

            case WS_EVT_DISCONNECT:
                log_info("PerformanceProfiler client disconnected: " << client->id());
                removeClient(client->id());
                break;

            case WS_EVT_ERROR:
                log_error("PerformanceProfiler websocket error for client " << client->id());
                removeClient(client->id());
                break;

            case WS_EVT_DATA:
                handleCommand(client, (const char*)data, len);
                break;

            case WS_EVT_PONG:
            case WS_EVT_PING:
                break;
        }
    }

    void PerformanceProfiler::handleCommand(AsyncWebSocketClient* client, const char* data, size_t len) {
        // Parse simple JSON commands
        // Format: {"cmd":"start","duration":10000,"rate":1000}
        //         {"cmd":"stop"}
        
        if (len < 10) return;

        // Simple parser - look for command
        if (strstr(data, "\"cmd\":\"start\"")) {
            // Extract duration and rate
            uint32_t duration = 10000;  // Default 10s
            uint32_t rate = _default_rate;

            const char* dur_ptr = strstr(data, "\"duration\":");
            if (dur_ptr) {
                duration = atoi(dur_ptr + 11);
            }

            const char* rate_ptr = strstr(data, "\"rate\":");
            if (rate_ptr) {
                rate = atoi(rate_ptr + 7);
            }

            startProfiling(duration, rate);
        }
        else if (strstr(data, "\"cmd\":\"stop\"")) {
            stopProfiling();
        }
        else if (strstr(data, "\"cmd\":\"status\"")) {
            broadcastStatus();
        }
    }

    // Stack profiling functions (adapted from PerfProf)

#ifdef HAS_PERFMON
    void IRAM_ATTR PerformanceProfiler::walkStack() {
        // NOTE: This thing should be *fast* because we work in a timer ISR.

        auto cycles       = xtensa_perfmon_value(0);
        auto instructions = xtensa_perfmon_value(1);

        esp_backtrace_frame_t start;
        memset(&start, 0, sizeof(esp_backtrace_frame_t));
        esp_backtrace_get_start(&(start.pc), &(start.sp), &(start.next_pc));

        // BaseType_t current_core_id = xPortGetCoreID(); -- not useful.

        const int STACK_MAX_DEPTH = 10;
        const int STACK_SKIP      = 2;
        const int calls           = 1;

        // Get rid of compiler warnings:
        (void)cycles;
        (void)instructions;
        (void)calls;

        // Initialize stk_frame with first frame of stack
        esp_backtrace_frame_t stk_frame;
        memcpy(&stk_frame, &start, sizeof(esp_backtrace_frame_t));

        // Check if first frame is valid
        // NOTE: On some ESP-IDF versions/configs, ISR context PCs have cache bits that cause
        // esp_ptr_executable() to fail. Since stack walking in ISR context is inherently risky,
        // we'll just validate the SP and continue - if the walk fails, esp_backtrace_get_next_frame
        // will return false and we'll exit safely.
        bool corrupted = !esp_stack_ptr_is_sane(stk_frame.sp);

        // Walk the stack, keep track of caller/callee.
        intptr_t caller = esp_cpu_process_stack_pc(stk_frame.pc);
        intptr_t callee = 0;
        for (uint_fast16_t i = 0; i < STACK_MAX_DEPTH && caller != 0 && !corrupted; ++i) {
            // Get previous stack frame
            if (i >= STACK_SKIP) {
                // Update the hash table:
                int index = (caller * 101429u + (uint32_t)callee * 67723) % HashSize;
                int i     = 0;

            again:
                for (; i < MaxCollisions && _hashTable[index].caller != 0 &&
                       (_hashTable[index].caller != caller || _hashTable[index].callee != callee);
                     ++i, ++index) {}

                auto& p = _hashTable[index];
                if (p.caller == 0) {
                    if (unlikely(index == HashSize)) {
                        index = 0;
                        goto again;
                    }
                    p.caller = caller;
                    p.callee = callee;
                    p.calls  = 1;
                    p.cycles = cycles;
                } else if (p.caller == caller && p.callee == callee) {
                    p.calls++;
                    p.cycles += cycles;
                }
                // Otherwise we're out of luck.

                // Done.
            } else {
                caller = 0;
            }

            if (!esp_backtrace_get_next_frame(&stk_frame)) {
                break;
            }

            callee = caller;
            caller = esp_cpu_process_stack_pc(stk_frame.pc);
        }
    }

    bool IRAM_ATTR PerformanceProfiler::timerISRCallback(void* args) {
        auto profiler = static_cast<PerformanceProfiler*>(args);
        
        // Sample counter for debugging (increment even when locked)
        static uint32_t isr_counter = 0;
        isr_counter++;

        if (!profiler->_locked) {
            profiler->walkStack();
        }

        // Reset counters for next sample
        xtensa_perfmon_reset(0);
        xtensa_perfmon_reset(1);
        return false;
    }

    void PerformanceProfiler::initializeProfilerTimer(void* parameters) {
        static const int TIMER_DIVIDER      = 8000;
        static const int SAMPLES_PER_SECOND = 100;

        auto current_core_id = static_cast<timer_idx_t>(xPortGetCoreID());

        // Setup timer:
        timer_config_t config;
        memset(&config, 0, sizeof(timer_config_t));
        config.divider     = TIMER_DIVIDER;
        config.counter_dir = TIMER_COUNT_UP;
        config.counter_en  = TIMER_PAUSE;
        config.alarm_en    = TIMER_ALARM_EN;
        config.auto_reload = TIMER_AUTORELOAD_EN;

        // APB clock is 80 MHz
        timer_init(TIMER_GROUP_1, current_core_id, &config);
        timer_set_counter_value(TIMER_GROUP_1, current_core_id, 0);

        // 80,000,000 [APB Clock] / 8,000 [TIMER_DIVIDER] / 100 [samples/sec]
        uint64_t alarm = 80000000 / TIMER_DIVIDER / SAMPLES_PER_SECOND;
        timer_set_alarm_value(TIMER_GROUP_1, current_core_id, alarm);
        timer_enable_intr(TIMER_GROUP_1, current_core_id);

        esp_err_t err = timer_isr_callback_add(TIMER_GROUP_1, current_core_id, timerISRCallback, parameters, 0);
        if (err != ESP_OK) {
            // Can't use log_error in this task context easily, but we can at least check
            printf("ERROR: Failed to add timer ISR callback: %d\n", err);
        }
        
        err = timer_start(TIMER_GROUP_1, current_core_id);
        if (err != ESP_OK) {
            printf("ERROR: Failed to start timer: %d\n", err);
        } else {
            printf("Timer started successfully on core %d\n", current_core_id);
        }

        vTaskDelete(NULL);
    }

    void PerformanceProfiler::monitorTaskWrapper(void* param) {
        PerformanceProfiler* profiler = static_cast<PerformanceProfiler*>(param);
        profiler->monitorTask();
        vTaskDelete(NULL);
    }

    void PerformanceProfiler::monitorTask() {
        log_info("Performance monitor task started");

        TickType_t update_delay = pdMS_TO_TICKS(_update_rate);
        TickType_t last_update = xTaskGetTickCount();

        while (true) {
            TickType_t current_time = xTaskGetTickCount();
            if (current_time - last_update >= update_delay) {
                if (hasActiveClients()) {
                    broadcastPerfCounters();
                }
                last_update = current_time;
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    void PerformanceProfiler::broadcastPerfCounters() {
        if (!_profiler_socket || !hasActiveClients()) {
            return;
        }

        // Get task stats for CPU usage
        UBaseType_t task_count = uxTaskGetNumberOfTasks();
        TaskStatus_t* task_array = (TaskStatus_t*)malloc(task_count * sizeof(TaskStatus_t));
        if (!task_array) {
            return;
        }

        uint32_t total_runtime;
        task_count = uxTaskGetSystemState(task_array, task_count, &total_runtime);

        // Calculate CPU usage properly
        // uxTaskGetSystemState returns total_runtime in ticks (not cycles!)
        // On ESP32, tick rate is configHZ (typically 100 or 1000 Hz)
        uint32_t current_timestamp = xTaskGetTickCount();
        
        float cpu_usage = 0.0f;
        if (_last_timestamp > 0) {
            // Delta time in ticks
            uint32_t delta_ticks = current_timestamp - _last_timestamp;
            
            if (delta_ticks > 0) {
                // Delta runtime in ticks (sum of all task runtimes)
                uint32_t delta_runtime = total_runtime - _last_cycles;
                
                // CPU usage = (task runtime / total time) * 100
                // On dual-core ESP32, max is 200% (both cores at 100%)
                // Divide by number of cores to get average per-core usage
                #ifdef CONFIG_FREERTOS_UNICORE
                    cpu_usage = (delta_runtime * 100.0f) / delta_ticks;
                #else
                    // Dual core: total runtime can be up to 2x wall clock time
                    cpu_usage = (delta_runtime * 100.0f) / (delta_ticks * 2);
                #endif
                
                // Clamp to 0-100%
                if (cpu_usage < 0.0f) cpu_usage = 0.0f;
                if (cpu_usage > 100.0f) cpu_usage = 100.0f;
            }
        }

        _last_cycles = total_runtime;
        _last_timestamp = current_timestamp;

        // Get heap info
        size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
        size_t min_free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);

        // Build JSON
        char buffer[512];
        snprintf(buffer, sizeof(buffer),
            "{\"type\":\"perf_counters\","
            "\"cpu_usage\":%.1f,"
            "\"tasks\":%u,"
            "\"heap_free\":%u,"
            "\"heap_total\":%u,"
            "\"heap_min_free\":%u}",
            cpu_usage,
            static_cast<unsigned int>(task_count),
            static_cast<unsigned int>(free_heap),
            static_cast<unsigned int>(total_heap),
            static_cast<unsigned int>(min_free_heap));

        _profiler_socket->textAll(buffer);
        
        free(task_array);
    }
#else
    // Stub implementations for non-Xtensa platforms
    void PerformanceProfiler::initPerfCounters() {}
    void PerformanceProfiler::readPerfCounters() {}
    void PerformanceProfiler::monitorTaskWrapper(void* param) {}
    void PerformanceProfiler::monitorTask() {}
    void PerformanceProfiler::broadcastPerfCounters() {}
#endif

    // Module registration
    ConfigurableModuleFactory::InstanceBuilder<PerformanceProfiler> performance_profiler_module __attribute__((init_priority(0x5600))) ("performance_profiler");
}

