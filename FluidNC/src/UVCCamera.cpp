// Copyright (c) 2024. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "UVCCamera.h"
#include "WebUI/WebUIServer.h"
#include "Report.h"

#include <ESPAsyncWebServer.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// UVC Host includes
#include <usb/usb_host.h>
#include <libuvc/libuvc.h>

static const char* TAG = "UVCCamera";

// Buffer size for each frame (1MB should be enough for most JPEG frames)
static constexpr size_t FRAME_BUFFER_SIZE = 1024 * 1024;

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
    BaseType_t result = xTaskCreatePinnedToCore(
        captureTaskWrapper,
        "uvc_capture",
        4096,
        this,
        5,  // Priority
        &_capture_task,
        0   // Core 0
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

    _initialized = false;
    _camera_present = false;
}

bool UVCCamera::initUVC() {
    log_info("Initializing UVC device...");
    
    // Initialize USB Host library
    static bool usb_host_initialized = false;
    if (!usb_host_initialized) {
        usb_host_config_t host_config = {
            .skip_phy_setup = false,
            .intr_flags = ESP_INTR_FLAG_LEVEL1,
        };
        esp_err_t err = usb_host_install(&host_config);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            log_error("Failed to install USB host: " << err);
            return false;
        }
        usb_host_initialized = true;
    }

    // Initialize libuvc context
    uvc_context_t** ctx_ptr = reinterpret_cast<uvc_context_t**>(&_uvc_device_handle);
    uvc_error_t res = uvc_init(ctx_ptr, NULL);
    if (res != UVC_SUCCESS) {
        log_error("Failed to initialize UVC context: " << uvc_strerror(res));
        return false;
    }

    log_info("UVC context initialized, looking for camera device...");
    
    // Try to find and open a UVC device
    uvc_device_t* dev = nullptr;
    uvc_device_handle_t* devh = nullptr;
    
    res = uvc_find_device(*ctx_ptr, &dev, 0, 0, NULL);  // Find first device
    if (res != UVC_SUCCESS) {
        log_info("No UVC camera device found");
        uvc_exit(*ctx_ptr);
        _uvc_device_handle = nullptr;
        return false;
    }

    res = uvc_open(dev, &devh);
    uvc_unref_device(dev);
    
    if (res != UVC_SUCCESS) {
        log_error("Failed to open UVC device: " << uvc_strerror(res));
        uvc_exit(*ctx_ptr);
        _uvc_device_handle = nullptr;
        return false;
    }

    // Print device info
    uvc_device_descriptor_t* desc;
    if (uvc_get_device_descriptor(dev, &desc) == UVC_SUCCESS) {
        log_info("UVC Device: " << desc->manufacturer << " " << desc->product);
        uvc_free_device_descriptor(desc);
    }

    // Store the device handle in our opaque pointer (we'll retrieve it in the capture task)
    // For now, we just verify we can access the device
    uvc_close(devh);

    return true;
}

void UVCCamera::deinitUVC() {
    if (_uvc_device_handle) {
        uvc_context_t* ctx = reinterpret_cast<uvc_context_t*>(_uvc_device_handle);
        uvc_exit(ctx);
        _uvc_device_handle = nullptr;
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
    _camera_socket->onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client, 
                                   AwsEventType type, void* arg, uint8_t* data, size_t len) {
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
        _frame_buffers[i].data = static_cast<uint8_t*>(
            heap_caps_malloc(FRAME_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
        );
        
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
        
        _frame_buffers[i].size = 0;
        _frame_buffers[i].ready = false;
    }

    log_info("Allocated " << _buffer_count << " frame buffers of " << 
             (FRAME_BUFFER_SIZE / 1024) << "KB each in PSRAM");
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
    _read_index = 0;
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

void UVCCamera::onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                                 AwsEventType type, void* arg, uint8_t* data, size_t len) {
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
    }
}

void UVCCamera::captureTaskWrapper(void* param) {
    UVCCamera* camera = static_cast<UVCCamera*>(param);
    camera->captureTask();
    vTaskDelete(NULL);
}

void UVCCamera::captureTask() {
    log_info("UVC Camera capture task started");

    uvc_context_t* ctx = nullptr;
    uvc_device_t* dev = nullptr;
    uvc_device_handle_t* devh = nullptr;
    uvc_stream_ctrl_t ctrl;
    bool stream_started = false;
    
    TickType_t frame_delay = pdMS_TO_TICKS(1000 / _frame_rate_limit);
    TickType_t last_frame_time = 0;
    
    bool camera_error_logged = false;

    while (true) {
        // Check if we need to initialize/reinitialize the camera
        if (!_camera_present || !devh) {
            if (!camera_error_logged) {
                log_info("Waiting for UVC camera...");
                camera_error_logged = true;
            }

            // Try to initialize
            if (ctx == nullptr) {
                uvc_error_t res = uvc_init(&ctx, NULL);
                if (res != UVC_SUCCESS) {
                    vTaskDelay(pdMS_TO_TICKS(5000));  // Wait 5 seconds before retry
                    continue;
                }
            }

            // Try to find device
            uvc_error_t res = uvc_find_device(ctx, &dev, 0, 0, NULL);
            if (res != UVC_SUCCESS) {
                vTaskDelay(pdMS_TO_TICKS(5000));  // Wait 5 seconds before retry
                continue;
            }

            // Try to open device
            res = uvc_open(dev, &devh);
            if (res != UVC_SUCCESS) {
                uvc_unref_device(dev);
                dev = nullptr;
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }

            // Try to negotiate stream parameters
            // Request MJPEG format at preferred resolution
            res = uvc_get_stream_ctrl_format_size(
                devh, &ctrl,
                UVC_FRAME_FORMAT_MJPEG,
                _preferred_width, _preferred_height,
                _frame_rate_limit
            );

            if (res != UVC_SUCCESS) {
                log_warn("Could not negotiate preferred format, trying defaults...");
                // Try with default resolution
                res = uvc_get_stream_ctrl_format_size(
                    devh, &ctrl,
                    UVC_FRAME_FORMAT_MJPEG,
                    640, 480,
                    _frame_rate_limit
                );
            }

            if (res != UVC_SUCCESS) {
                log_error("Failed to negotiate stream format: " << uvc_strerror(res));
                uvc_close(devh);
                uvc_unref_device(dev);
                devh = nullptr;
                dev = nullptr;
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }

            log_info("UVC Camera connected - streaming at " << ctrl.bmHint << "x" << ctrl.bMaxVideoFrameSize << " @ " << _frame_rate_limit << " fps");
            _camera_present = true;
            camera_error_logged = false;
        }

        // Only stream if we have active clients
        if (!hasActiveClients()) {
            if (stream_started) {
                uvc_stop_streaming(devh);
                stream_started = false;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Start streaming if not already started
        if (!stream_started) {
            // Start streaming with callback
            uvc_error_t res = uvc_start_streaming(devh, &ctrl, 
                [](uvc_frame_t* frame, void* user_ptr) {
                    UVCCamera* camera = static_cast<UVCCamera*>(user_ptr);
                    
                    // Only process MJPEG frames
                    if (frame->frame_format != UVC_FRAME_FORMAT_MJPEG) {
                        return;
                    }

                    // Get write buffer
                    if (xSemaphoreTake(camera->_buffer_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        FrameBuffer* buffer = camera->getWriteBuffer();
                        if (buffer && frame->data_bytes <= buffer->capacity) {
                            // Copy JPEG data to buffer
                            memcpy(buffer->data, frame->data, frame->data_bytes);
                            buffer->size = frame->data_bytes;
                            buffer->ready = true;
                            camera->advanceWriteIndex();
                        }
                        xSemaphoreGive(camera->_buffer_mutex);
                    }
                }, 
                this, 0);

            if (res != UVC_SUCCESS) {
                log_error("Failed to start streaming: " << uvc_strerror(res));
                uvc_close(devh);
                uvc_unref_device(dev);
                devh = nullptr;
                dev = nullptr;
                _camera_present = false;
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }

            stream_started = true;
            log_info("UVC streaming started");
        }

        // Stream frames to websocket clients at the configured rate
        TickType_t current_time = xTaskGetTickCount();
        if (current_time - last_frame_time >= frame_delay) {
            streamFrameToClients();
            last_frame_time = current_time;
        }

        // Small delay to prevent busy-waiting
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Cleanup (unreachable in normal operation)
    if (stream_started && devh) {
        uvc_stop_streaming(devh);
    }
    if (devh) {
        uvc_close(devh);
    }
    if (dev) {
        uvc_unref_device(dev);
    }
    if (ctx) {
        uvc_exit(ctx);
    }
}

// Module registration
ConfigurableModuleFactory::InstanceBuilder<UVCCamera> uvc_camera_module __attribute__((init_priority(0x5200))) ("uvc_camera");

