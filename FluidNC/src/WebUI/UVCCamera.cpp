// Copyright (c) Stefan de Bruijn, 2025. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

// This UVC Camera implementation uses ESP32's usb_stream library for USB camera support.
// It streams MJPEG frames from USB cameras to WebSocket clients with minimal overhead.
// Based on: https://github.com/espressif/esp-iot-solution/tree/master/examples/usb/host/usb_camera_mic_spk

#include "UVCCamera.h"
#include "WebUI/WebUIServer.h"
#include "Logging.h"

#include "Settings.h"

#include <ESPAsyncWebServer.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// UVC Host includes
#include <usb_stream.h>

// Buffer size for each frame (1MB should be enough for most JPEG frames)
static constexpr size_t FRAME_BUFFER_SIZE = 1024 * 1024;

// Forward declarations for C-style callbacks
static void camera_frame_callback(uvc_frame_t* frame, void* user_ptr);
static void stream_state_callback(usb_stream_state_t event, void* user_ptr);

namespace WebUI {
    UVCCamera::UVCCamera(const char* name) : ConfigurableModule(name) {
        _buffer_mutex = xSemaphoreCreateMutex();
    }

    UVCCamera::~UVCCamera() {
        deinit();
        if (_buffer_mutex) {
            vSemaphoreDelete(_buffer_mutex);
            _buffer_mutex = nullptr;
        }
    }

    void UVCCamera::validate() {
        if (_buffer_count < 2 || _buffer_count > 4) {
            log_error("UVC Camera buffer_count must be between 2 and 4");
            _error = true;
        }
        if (_frame_rate_limit < 1 || _frame_rate_limit > 30) {
            log_error("UVC Camera frame_rate_limit must be between 1 and 30");
            _error = true;
        }
        if (_preferred_width < 320 || _preferred_width > 3840) {
            log_error("UVC Camera preferred_width must be between 320 and 3840");
            _error = true;
        }
        if (_preferred_height < 240 || _preferred_height > 2160) {
            log_error("UVC Camera preferred_height must be between 240 and 2160");
            _error = true;
        }
    }

    void UVCCamera::group(Configuration::HandlerBase& handler) {
        handler.item("enable", _enable);
        handler.item("frame_rate_limit", _frame_rate_limit, 1, 30);
        handler.item("preferred_width", _preferred_width, 320, 3840);
        handler.item("preferred_height", _preferred_height, 240, 2160);
        handler.item("buffer_count", _buffer_count, 2, 4);
    }

    void UVCCamera::init() {
        auto cdc_enable = new EnumSetting("USB CDC Enable", WEBSET, WG, NULL, "USBCDC/Enable", false, &onoffOptions);
        if (cdc_enable->get()) {
            return;  // UVC camera cannot work when USB CDC is enabled
        }

        if (_error || !_enable) {
            if (!_enable) {
                log_info("UVC Camera module disabled in configuration");
            }
            return;
        }

        log_info("Initializing UVC Camera module");
        log_info("  Frame rate limit: " << _frame_rate_limit << " fps");
        log_info("  Preferred resolution: " << _preferred_width << "x" << _preferred_height);
        log_info("  Buffer count: " << _buffer_count);

        // Allocate frame buffers in PSRAM
        if (!allocateFrameBuffers()) {
            log_error("Failed to allocate frame buffers");
            _error = true;
            return;
        }

        // Setup websocket endpoint
        if (!setupWebSocket()) {
            log_error("Failed to setup websocket");
            _error = true;
            freeFrameBuffers();
            return;
        }

        // Initialize UVC
        if (!initUVC()) {
            log_error("Failed to initialize UVC camera - no camera present or initialization failed");
            _camera_present = false;
            // Don't set _error - this is expected if camera isn't plugged in
            // We'll continue to check for camera in the capture task
        } else {
            _camera_present = true;
            log_info("UVC Camera initialized successfully");
        }

        // Create capture task
        BaseType_t result = xTaskCreatePinnedToCore(captureTaskWrapper,
                                                    "uvc_capture",
                                                    4096,
                                                    this,
                                                    5,  // Priority
                                                    &_capture_task,
                                                    0  // Core 0
        );

        if (result != pdPASS) {
            log_error("Failed to create UVC capture task");
            _error = true;
            deinitUVC();
            cleanupWebSocket();
            freeFrameBuffers();
            return;
        }

        _initialized = true;
        log_info("UVC Camera module initialized");
    }

    void UVCCamera::deinit() {
        if (!_initialized) {
            return;
        }

        log_info("Deinitializing UVC Camera module");

        // Stop capture task
        if (_capture_task) {
            vTaskDelete(_capture_task);
            _capture_task = nullptr;
        }

        // Cleanup UVC
        deinitUVC();

        // Cleanup websocket
        cleanupWebSocket();

        // Free buffers
        freeFrameBuffers();

        _initialized    = false;
        _camera_present = false;
    }

    bool UVCCamera::initUVC() {
        log_info("Initializing UVC device with usb_stream...");

        // Allocate transfer buffers for USB streaming
        // These need to be sized appropriately for MJPEG frames
        #ifdef CONFIG_IDF_TARGET_ESP32S2
        #define UVC_XFER_BUFFER_SIZE (45 * 1024)
        #else
        #define UVC_XFER_BUFFER_SIZE (55 * 1024)
        #endif

        _uvc_xfer_buffer_a = (uint8_t*)heap_caps_malloc(UVC_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        _uvc_xfer_buffer_b = (uint8_t*)heap_caps_malloc(UVC_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        _uvc_frame_buffer  = (uint8_t*)heap_caps_malloc(UVC_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        if (!_uvc_xfer_buffer_a || !_uvc_xfer_buffer_b || !_uvc_frame_buffer) {
            log_error("Failed to allocate USB transfer buffers");
            if (_uvc_xfer_buffer_a) heap_caps_free(_uvc_xfer_buffer_a);
            if (_uvc_xfer_buffer_b) heap_caps_free(_uvc_xfer_buffer_b);
            if (_uvc_frame_buffer) heap_caps_free(_uvc_frame_buffer);
            _uvc_xfer_buffer_a = nullptr;
            _uvc_xfer_buffer_b = nullptr;
            _uvc_frame_buffer = nullptr;
            return false;
        }

        // Configure UVC streaming
        // Initialize to zero first to set all fields
        uvc_config_t uvc_config = {};
        uvc_config.frame_width = (uint16_t)(_preferred_width == 0 ? FRAME_RESOLUTION_ANY : _preferred_width);
        uvc_config.frame_height = (uint16_t)(_preferred_height == 0 ? FRAME_RESOLUTION_ANY : _preferred_height);
        uvc_config.frame_interval = FPS2INTERVAL(_frame_rate_limit);
        uvc_config.xfer_buffer_size = UVC_XFER_BUFFER_SIZE;
        uvc_config.xfer_buffer_a = _uvc_xfer_buffer_a;
        uvc_config.xfer_buffer_b = _uvc_xfer_buffer_b;
        uvc_config.frame_buffer_size = UVC_XFER_BUFFER_SIZE;
        uvc_config.frame_buffer = _uvc_frame_buffer;
        uvc_config.frame_cb = &camera_frame_callback;
        uvc_config.frame_cb_arg = this;

        esp_err_t ret = uvc_streaming_config(&uvc_config);
        if (ret != ESP_OK) {
            log_error("UVC streaming config failed: " << ret);
            heap_caps_free(_uvc_xfer_buffer_a);
            heap_caps_free(_uvc_xfer_buffer_b);
            heap_caps_free(_uvc_frame_buffer);
            _uvc_xfer_buffer_a = nullptr;
            _uvc_xfer_buffer_b = nullptr;
            _uvc_frame_buffer = nullptr;
            return false;
        }

        // Register state callback for connection/disconnection events
        ret = usb_streaming_state_register(&stream_state_callback, this);
        if (ret != ESP_OK) {
            log_error("Failed to register stream state callback: " << ret);
            return false;
        }

        // Start USB streaming
        ret = usb_streaming_start();
        if (ret != ESP_OK) {
            log_error("Failed to start USB streaming: " << ret);
            return false;
        }

        log_info("UVC streaming initialized, waiting for camera device...");
        _uvc_initialized = true;
        return true;
    }

    void UVCCamera::deinitUVC() {
        if (_uvc_initialized) {
            log_info("Stopping UVC streaming...");
            usb_streaming_stop();
            _uvc_initialized = false;
        }

        // Free USB transfer buffers
        if (_uvc_xfer_buffer_a) {
            heap_caps_free(_uvc_xfer_buffer_a);
            _uvc_xfer_buffer_a = nullptr;
        }
        if (_uvc_xfer_buffer_b) {
            heap_caps_free(_uvc_xfer_buffer_b);
            _uvc_xfer_buffer_b = nullptr;
        }
        if (_uvc_frame_buffer) {
            heap_caps_free(_uvc_frame_buffer);
            _uvc_frame_buffer = nullptr;
        }

        _camera_present = false;
    }

    bool UVCCamera::setupWebSocket() {
        // Get the webserver instance from WebUI_Server
        AsyncWebServer* webserver = WebUI::WebUI_Server::getWebServer();
        if (!webserver) {
            log_error("WebUI server not available");
            return false;
        }

        // Create websocket handler for /camera endpoint
        _camera_socket = new AsyncWebSocket("/camera");
        if (!_camera_socket) {
            log_error("Failed to create websocket");
            return false;
        }

        // Set event handler - use lambda to capture 'this'
        _camera_socket->onEvent(
            [this](AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
                this->onWebSocketEvent(server, client, type, arg, data, len);
            });

        // Add handler to webserver
        webserver->addHandler(_camera_socket);

        log_info("UVC Camera websocket endpoint /camera registered");
        return true;
    }

    void UVCCamera::cleanupWebSocket() {
        if (_camera_socket) {
            _camera_socket->closeAll();
            // Note: Don't delete _camera_socket as it's managed by the webserver
            _camera_socket = nullptr;
        }
        _active_client_count = 0;
    }

    bool UVCCamera::allocateFrameBuffers() {
        _frame_buffers = new FrameBuffer[_buffer_count];
        if (!_frame_buffers) {
            log_error("Failed to allocate frame buffer array");
            return false;
        }

        for (int i = 0; i < _buffer_count; i++) {
            _frame_buffers[i].capacity = FRAME_BUFFER_SIZE;
            _frame_buffers[i].data     = static_cast<uint8_t*>(heap_caps_malloc(FRAME_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

            if (!_frame_buffers[i].data) {
                log_error("Failed to allocate frame buffer " << i << " in PSRAM");
                // Free previously allocated buffers
                for (int j = 0; j < i; j++) {
                    heap_caps_free(_frame_buffers[j].data);
                }
                delete[] _frame_buffers;
                _frame_buffers = nullptr;
                return false;
            }

            _frame_buffers[i].size  = 0;
            _frame_buffers[i].ready = false;
        }

        log_info("Allocated " << _buffer_count << " frame buffers of " << (FRAME_BUFFER_SIZE / 1024) << "KB each in PSRAM");
        return true;
    }

    void UVCCamera::freeFrameBuffers() {
        if (_frame_buffers) {
            for (int i = 0; i < _buffer_count; i++) {
                if (_frame_buffers[i].data) {
                    heap_caps_free(_frame_buffers[i].data);
                }
            }
            delete[] _frame_buffers;
            _frame_buffers = nullptr;
        }
        _write_index = 0;
        _read_index  = 0;
    }

    UVCCamera::FrameBuffer* UVCCamera::getWriteBuffer() {
        if (!_frame_buffers) {
            return nullptr;
        }
        return &_frame_buffers[_write_index];
    }

    UVCCamera::FrameBuffer* UVCCamera::getReadBuffer() {
        if (!_frame_buffers) {
            return nullptr;
        }

        // Find a ready buffer starting from read_index
        for (int i = 0; i < _buffer_count; i++) {
            uint8_t idx = (_read_index + i) % _buffer_count;
            if (_frame_buffers[idx].ready) {
                _read_index = idx;
                return &_frame_buffers[idx];
            }
        }
        return nullptr;
    }

    void UVCCamera::advanceWriteIndex() {
        _write_index = (_write_index + 1) % _buffer_count;
    }

    void UVCCamera::advanceReadIndex() {
        _read_index = (_read_index + 1) % _buffer_count;
    }

    bool UVCCamera::addClient(uint32_t client_id) {
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
        log_info("UVC Camera client connected: " << client_id << " (total: " << _active_client_count << ")");
        return true;
    }

    void UVCCamera::removeClient(uint32_t client_id) {
        for (size_t i = 0; i < _active_client_count; i++) {
            if (_active_clients[i] == client_id) {
                // Shift remaining clients down
                for (size_t j = i; j < _active_client_count - 1; j++) {
                    _active_clients[j] = _active_clients[j + 1];
                }
                _active_client_count--;
                log_info("UVC Camera client disconnected: " << client_id << " (total: " << _active_client_count << ")");
                break;
            }
        }
    }

    bool UVCCamera::hasActiveClients() {
        return _active_client_count > 0;
    }

    void UVCCamera::streamFrameToClients() {
        if (!_camera_socket || !hasActiveClients()) {
            return;
        }

        if (xSemaphoreTake(_buffer_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return;
        }

        FrameBuffer* buffer = getReadBuffer();
        if (buffer && buffer->ready && buffer->size > 0) {
            // Send frame to all connected clients
            for (size_t i = 0; i < _active_client_count; i++) {
                AsyncWebSocketClient* client = _camera_socket->client(_active_clients[i]);
                if (client && client->canSend()) {
                    // Send binary frame (JPEG data)
                    client->binary(buffer->data, buffer->size);
                }
            }

            // Mark buffer as consumed
            buffer->ready = false;
        }

        xSemaphoreGive(_buffer_mutex);
    }

    void UVCCamera::onWebSocketEvent(
        AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
        switch (type) {
            case WS_EVT_CONNECT:
                log_info("UVC Camera websocket client connected from " << client->remoteIP().toString().c_str());
                addClient(client->id());
                break;

            case WS_EVT_DISCONNECT:
                log_info("UVC Camera websocket client disconnected");
                removeClient(client->id());
                break;

            case WS_EVT_ERROR:
                log_error("UVC Camera websocket error for client " << client->id());
                removeClient(client->id());
                break;

            case WS_EVT_DATA:
                // We don't expect data from clients (streaming is one-way)
                break;

            case WS_EVT_PONG:
                // Pong received, keep-alive
                break;

            case WS_EVT_PING:
                // Ping received, will auto-respond with pong
                break;
        }
    }


    void UVCCamera::captureTaskWrapper(void* param) {
        UVCCamera* camera = static_cast<UVCCamera*>(param);
        camera->captureTask();
        vTaskDelete(NULL);
    }

    void UVCCamera::captureTask() {
        log_info("UVC Camera capture task started");

        TickType_t frame_delay     = pdMS_TO_TICKS(1000 / _frame_rate_limit);
        TickType_t last_frame_time = 0;

        // Main loop - just stream frames that arrive via callback
        while (true) {
            // Stream frames to websocket clients at the configured rate
            TickType_t current_time = xTaskGetTickCount();
            if (current_time - last_frame_time >= frame_delay) {
                if (hasActiveClients()) {
                    streamFrameToClients();
                }
                last_frame_time = current_time;
            }

            // Small delay to prevent busy-waiting
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // Module registration
    ConfigurableModuleFactory::InstanceBuilder<UVCCamera> uvc_camera_module __attribute__((init_priority(0x5200))) ("uvc_camera");
}

// Static callback implementations (C-style callbacks for usb_stream)
static void camera_frame_callback(uvc_frame_t* frame, void* user_ptr) {
    WebUI::UVCCamera* camera = static_cast<WebUI::UVCCamera*>(user_ptr);

    // Only process MJPEG frames
    if (frame->frame_format != UVC_FRAME_FORMAT_MJPEG) {
        return;
    }

    // Skip frames if no clients are connected
    if (!camera->hasActiveClients()) {
        return;
    }

    // Get write buffer and copy frame data
    SemaphoreHandle_t mutex = camera->getBufferMutex();
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        WebUI::UVCCamera::FrameBuffer* buffer = camera->getWriteBuffer();
        if (buffer && frame->data_bytes <= buffer->capacity) {
            // Copy JPEG data to buffer
            memcpy(buffer->data, frame->data, frame->data_bytes);
            buffer->size  = frame->data_bytes;
            buffer->ready = true;
            camera->advanceWriteIndex();
        }
        xSemaphoreGive(mutex);
    }
}

static void stream_state_callback(usb_stream_state_t event, void* user_ptr) {
    WebUI::UVCCamera* camera = static_cast<WebUI::UVCCamera*>(user_ptr);

    switch (event) {
        case STREAM_CONNECTED: {
            // Query and log frame size info
            size_t frame_size = 0;
            size_t frame_index = 0;
            uvc_frame_size_list_get(NULL, &frame_size, &frame_index);
            
            if (frame_size) {
                log_info("UVC: Device connected, frame list size = " << frame_size << ", current = " << frame_index);
                uvc_frame_size_t* uvc_frame_list = (uvc_frame_size_t*)malloc(frame_size * sizeof(uvc_frame_size_t));
                if (uvc_frame_list) {
                    uvc_frame_size_list_get(uvc_frame_list, NULL, NULL);
                    for (size_t i = 0; i < frame_size; i++) {
                        log_info("  frame[" << i << "] = " << uvc_frame_list[i].width << "x" << uvc_frame_list[i].height);
                    }
                    free(uvc_frame_list);
                }
            } else {
                log_warn("UVC: Device connected but no frame list available");
            }
            
            camera->setCameraPresent(true);
            log_info("UVC Camera device connected");
            break;
        }
        
        case STREAM_DISCONNECTED:
            camera->setCameraPresent(false);
            log_info("UVC Camera device disconnected");
            break;
            
        default:
            log_error("UVC: Unknown stream event: " << (int)event);
            break;
    }
}
