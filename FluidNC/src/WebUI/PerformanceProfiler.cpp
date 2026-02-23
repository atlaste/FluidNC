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
        if (_update_rate < 100 || _update_rate > 10000) {
            log_error("PerformanceProfiler update_rate must be between 100 and 10000 ms");
            _error = true;
        }
    }

    void PerformanceProfiler::group(Configuration::HandlerBase& handler) {
        handler.item("enable", _enable);
        handler.item("enable_pc_sampling", _enable_pc_sampling);
        handler.item("update_rate", _update_rate, 100, 10000);  // How often to send results to browser (ms)
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
            // Note: perfmon is initialized per-core in initializeProfilerTimer()
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
            xTaskCreatePinnedToCore(initializeProfilerTimer, "prof0", 4096, this, configMAX_PRIORITIES - 1, NULL, 0);

#ifndef CONFIG_FREERTOS_UNICORE
            // Small delay to avoid race condition in perfmon initialization
            vTaskDelay(pdMS_TO_TICKS(100));
            xTaskCreatePinnedToCore(initializeProfilerTimer, "prof1", 4096, this, configMAX_PRIORITIES - 1, NULL, 1);
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


    bool PerformanceProfiler::startProfiling() {
        if (_profiling) {
            log_warn("Profiling already in progress");
            return false;
        }

        if (!_enable_pc_sampling || !_hashTable) {
            log_error("Stack profiling not enabled or not initialized");
            return false;
        }

        log_info("Starting stack profiling at 1 kHz (fixed rate)");

        // Clear hash table and total cycles
        memset(_hashTable, 0, sizeof(StackItem) * (HashSize + 1));
        _total_cycles = 0;

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

        // Send final results if any remain in hash table
        if (_enable_pc_sampling && _hashTable) {
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

        // Clean up stale clients before iterating
        _profiler_socket->cleanupClients();

        // Check again after cleanup
        if (_profiler_socket->count() == 0) {
            return;
        }
    
        // Capture total_cycles for this batch (for percentage calculation)
        uint64_t batch_total_cycles = _total_cycles;
        
        // Build complete JSON string with only non-empty entries
        // Include total_cycles so UI can calculate correct percentages
        char header[64];
        snprintf(header, sizeof(header), "{\"type\":\"results\",\"total_cycles\":%llu,\"items\":[", 
            (unsigned long long)batch_total_cycles);
        String json = header;
        
        bool first = true;
        size_t sent = 0;
        
        for (int i = 0; i < HashSize; i++) {
            if (_hashTable[i].caller != 0 && _hashTable[i].calls > 0) {
                if (!first) {
                    json += ",";
                }
                first = false;

                char item[128];
                snprintf(item, sizeof(item), 
                    "{\"caller\":\"0x%08x\",\"callee\":\"0x%08x\",\"calls\":%u,\"cycles\":%u}",
                    static_cast<unsigned int>(_hashTable[i].caller),
                    static_cast<unsigned int>(_hashTable[i].callee),
                    static_cast<unsigned int>(_hashTable[i].calls),
                    static_cast<unsigned int>(_hashTable[i].cycles));
                json += item;
                sent++;
                
                // Limit to avoid huge JSON payloads
                if (sent >= 1000) break;
            }
        }

        json += "]}";
        
        if (sent > 0) {
            log_debug("Broadcasting " << sent << " stack profiler results, " << batch_total_cycles << " total cycles (" << json.length() << " bytes)");
            _profiler_socket->textAll(json);
        }
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
        // Format: {"cmd":"start"} or {"cmd":"stop"}
        
        if (len < 10) return;

        if (strstr(data, "\"cmd\":\"start\"")) {
            startProfiling();
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

        // Track total cycles ONCE per sample (not per frame!)
        // This is the denominator for percentage calculations
        _total_cycles += cycles;

        esp_backtrace_frame_t start;
        memset(&start, 0, sizeof(esp_backtrace_frame_t));
        esp_backtrace_get_start(&(start.pc), &(start.sp), &(start.next_pc));

        // BaseType_t current_core_id = xPortGetCoreID(); -- not useful.

        const int STACK_MAX_DEPTH = 30;
        const int STACK_SKIP      = 2;
        const int calls           = 1;

        // Get rid of compiler warnings:
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
        // Note: This rate is fixed at compile time. UI rate selection is not yet implemented.
        // Higher rates give better resolution but more overhead. 1000 Hz is a good balance.
        static const int SAMPLES_PER_SECOND = 1000;

        auto current_core_id = static_cast<timer_idx_t>(xPortGetCoreID());
        
        // Initialize perfmon on THIS core (must be done per-core!)
        esp_err_t perr;
        perr = xtensa_perfmon_init(0, XTPERF_CNT_CYCLES, XTPERF_MASK_CYCLES, 0, -1);
        if (perr != ESP_OK) {
            printf("ERROR: Failed to init perfmon counter 0 on core %d: %d\n", current_core_id, perr);
        }
        perr = xtensa_perfmon_init(1, XTPERF_CNT_INSN, XTPERF_MASK_INSN_ALL, 0, -1);
        if (perr != ESP_OK) {
            printf("ERROR: Failed to init perfmon counter 1 on core %d: %d\n", current_core_id, perr);
        }
        xtensa_perfmon_start();
        printf("Perfmon initialized on core %d\n", current_core_id);

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
                    // Always broadcast performance counters
                    broadcastPerfCounters();
                    
                    // If profiling, also broadcast incremental results and clear hash table
                    if (_profiling && _enable_pc_sampling && _hashTable) {
                        broadcastHashTableResults();
                        
                        // Clear hash table and total_cycles after broadcasting to:
                        // 1. Prevent overflow on long profiling sessions
                        // 2. Show incremental data (not cumulative)
                        // Note: We don't lock here as the ISR uses atomic operations
                        memset(_hashTable, 0, sizeof(StackItem) * (HashSize + 1));
                        _total_cycles = 0;
                    }
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

        // Calculate overall CPU usage (excluding IDLE tasks)
        uint32_t current_timestamp = xTaskGetTickCount();
        
        float cpu_usage = 0.0f;
        if (_last_timestamp > 0 && current_timestamp > _last_timestamp) {
            uint32_t delta_ticks = current_timestamp - _last_timestamp;
            
            // Calculate total runtime excluding IDLE tasks
            uint32_t total_runtime_no_idle = 0;
            for (UBaseType_t i = 0; i < task_count; i++) {
                // Skip IDLE tasks (they run when CPU is idle)
                if (strncmp(task_array[i].pcTaskName, "IDLE", 4) != 0) {
                    total_runtime_no_idle += task_array[i].ulRunTimeCounter;
                }
            }
            
            uint32_t delta_runtime = total_runtime_no_idle - _last_cycles;
            
            // CPU usage = (active runtime / available time / num_cores) * 100
            #ifdef CONFIG_FREERTOS_UNICORE
                cpu_usage = (delta_runtime * 100.0f) / delta_ticks;
            #else
                // Dual core: available time = delta_ticks * 2
                cpu_usage = (delta_runtime * 100.0f) / (delta_ticks * 2.0f);
            #endif
        }

        // Store total runtime (excluding IDLE) for next delta calculation
        uint32_t total_runtime_no_idle = 0;
        for (UBaseType_t i = 0; i < task_count; i++) {
            if (strncmp(task_array[i].pcTaskName, "IDLE", 4) != 0) {
                total_runtime_no_idle += task_array[i].ulRunTimeCounter;
            }
        }
        _last_cycles = total_runtime_no_idle;
        _last_timestamp = current_timestamp;

        // Get heap info
        size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
        size_t min_free_heap = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);

        // Build JSON with per-task info
        String json = "{\"type\":\"perf_counters\",";
        json += "\"cpu_usage\":";
        json += String(cpu_usage, 1);
        json += ",\"task_count\":";
        json += String(task_count);
        json += ",\"heap_free\":";
        json += String(free_heap);
        json += ",\"heap_total\":";
        json += String(total_heap);
        json += ",\"heap_min_free\":";
        json += String(min_free_heap);
        json += ",\"tasks\":[";
        
        // Add per-task statistics
        for (UBaseType_t i = 0; i < task_count; i++) {
            if (i > 0) json += ",";
            
            // Calculate per-task CPU usage if we have previous data
            float task_cpu = 0.0f;
            // Note: ulRunTimeCounter is cumulative, so we'd need to track previous values per task
            // For now, just send the runtime counter and let the frontend calculate deltas
            
            json += "{\"name\":\"";
            json += task_array[i].pcTaskName;
            json += "\",\"priority\":";
            json += String(task_array[i].uxCurrentPriority);
            json += ",\"runtime\":";
            json += String(task_array[i].ulRunTimeCounter);
            json += ",\"stack_hwm\":";
            json += String(task_array[i].usStackHighWaterMark);
            json += "}";
        }
        
        json += "]}";
        
        _profiler_socket->textAll(json);
        
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

