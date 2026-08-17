// Copyright (c) 2025 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Module.h"
#include "Pin.h"
#include "FramDevice.h"
#include "FM25VXX.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

class StatePersistence : public ConfigurableModule {
private:
    // Where each section lives.  Derived from the actual capacity of the attached part at
    // init rather than fixed, because an I2C FRAM such as the MB85RC04V holds 512 bytes
    // where an SPI part holds 8 KB, and the same sections have to fit in both.
    struct Layout {
        uint32_t configHash   = 0;
        uint32_t motorSteps   = 0;
        uint32_t homingStatus = 0;
        uint32_t overrides    = 0;
        uint32_t parserState  = 0;
        uint32_t atc          = 0;
        uint32_t atcEnd       = 0;  // one past the last byte the ATC section may use
        uint32_t parameters   = 0;
        uint32_t end          = 0;  // one past the last usable byte
    };

    FramDevice* _fram = nullptr;
    Layout      _layout;

    // SPI FRAM pins
    Pin _csPin;
    Pin _wpPin;
    Pin _holdPin;

    // Configuration parameters
    int32_t _saveIntervalMs = 100;   // Default 100ms save interval
    int32_t _spiFreqMhz     = 20;    // Default 20MHz SPI frequency
    bool    _saveParameters = true;  // Whether to save interpreter variables

    // I2C FRAM. A bus number of -1 leaves the I2C path alone, so a configuration that only
    // names cs_pin keeps behaving exactly as it did.
    int32_t _i2cNum       = -1;
    int32_t _i2cAddress   = 0x50;
    int32_t _i2cSizeBytes = 512;

    // FreeRTOS task and queue
    TaskHandle_t         _saveTask = nullptr;
    static QueueHandle_t _forceSaveQueue;

    // Running flag
    bool _running     = false;
    bool _initialized = false;

    // Current config hash
    uint32_t _configHash = 0;

    // Private methods
    FramDevice* createDevice();
    bool        buildLayout(uint32_t deviceSize);

    uint32_t calculateConfigHash();
    void     saveAllSections();
    void     savePositionState();
    void     saveParserState();
    void     saveParameters();
    void     saveOverrides();
    void     saveSpindleState();

    void restoreAllSections();
    void restorePositionState();
    void restoreParserState();
    void restoreParameters();
    void restoreOverrides();
    void restoreSpindleState();

    bool usable() const { return _fram != nullptr && _fram->initialized(); }

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
