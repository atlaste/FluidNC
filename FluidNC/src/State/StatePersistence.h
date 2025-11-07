// Copyright (c) 2025 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Module.h"
#include "Pin.h"
#include "FM25VXX.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

class StatePersistence : public ConfigurableModule {
private:
    // FRAM driver instance
    FM25VXX* _fram = nullptr;

    // Configuration pins
    Pin _csPin;
    Pin _wpPin;
    Pin _holdPin;

    // Configuration parameters
    int32_t _saveIntervalMs = 100;   // Default 100ms save interval
    int32_t _spiFreqMhz     = 20;    // Default 20MHz SPI frequency
    bool    _saveParameters = true;  // Whether to save interpreter variables

    // FreeRTOS task and queue
    TaskHandle_t         _saveTask = nullptr;
    static QueueHandle_t _forceSaveQueue;

    // Running flag
    bool _running     = false;
    bool _initialized = false;

    // Current config hash
    uint32_t _configHash = 0;

    // Private methods
    uint32_t calculateConfigHash();
    void     saveAllSections();
    void     savePositionState();
    void     saveParserState();
    void     saveParameters();
    void     saveOverrides();

    void restoreAllSections();
    void restorePositionState();
    void restoreParserState();
    void restoreParameters();
    void restoreOverrides();

    static void saveTaskFunc(void* param);

public:
    StatePersistence(const char* name) : ConfigurableModule(name) {}
    ~StatePersistence();

    // ConfigurableModule overrides
    void init() override;
    int  init_priority() override { return 1'000'000; }; // Usually last to initialize.
    void deinit() override;
    void group(Configuration::HandlerBase& handler) override;

    // Static method for forced save (called from Protocol.cpp)
    static void forceSave();
};
