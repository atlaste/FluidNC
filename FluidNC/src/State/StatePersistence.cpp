// Copyright (c) 2025 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "StatePersistence.h"
#include "Logging.h"
#include "NutsBolts.h"
#include "System.h"
#include "GCode.h"
#include "Stepping.h"
#include "Machine/Homing.h"
#include "Machine/Axes.h"
#include "FileStream.h"
#include "Parameters.h"
#include "SettingsDefinitions.h"
#include "Machine/Axes.h"

#include <mbedtls/sha256.h>
#include <cstring>

// FRAM memory layout
namespace {
    // NOTE: 0x0000 is reserved for the initialization sequence.
    static const uint16_t FRAM_CONFIG_HASH_ADDR   = 0x0004;
    static const uint16_t FRAM_MOTOR_STEPS_ADDR   = 0x0008;
    static const uint16_t FRAM_HOMING_STATUS_ADDR = 0x0030;
    static const uint16_t FRAM_OVERRIDES_ADDR     = 0x0040;
    static const uint16_t FRAM_PARSER_STATE_ADDR  = 0x0050;
    static const uint16_t FRAM_PARAMETERS_ADDR    = 0x0200;
}

// Static members
QueueHandle_t StatePersistence::_forceSaveQueue = nullptr;

StatePersistence::~StatePersistence() {
    deinit();
    if (_fram) {
        delete _fram;
        _fram = nullptr;
    }
}

void StatePersistence::group(Configuration::HandlerBase& handler) {
    handler.item("cs_pin", _csPin);
    handler.item("wp_pin", _wpPin);
    handler.item("hold_pin", _holdPin);
    handler.item("save_interval_ms", _saveIntervalMs, 10, 10000);
    handler.item("spi_freq_mhz", _spiFreqMhz, 1, 40);  // 1-40MHz range
    handler.item("save_parameters", _saveParameters);
}

uint32_t StatePersistence::calculateConfigHash() {
    log_debug("Calculating config hash");

    try {
        // Read and hash config.yaml in 4KB chunks (RAM efficient)
        FileStream file(config_filename->get(), "rb", "");
        size_t     size = file.size();

        if (size == 0) {
            log_warn("Config file is empty, using default hash");
            return 0;
        }

        mbedtls_sha256_context ctx;
        unsigned char          hash[32];
        mbedtls_sha256_init(&ctx);
        mbedtls_sha256_starts(&ctx, 0);  // SHA256, not SHA224

        // Read and hash in 4KB chunks
        const size_t CHUNK_SIZE = 4096;
        uint8_t      buffer[CHUNK_SIZE];
        size_t       remaining = size;

        while (remaining > 0) {
            size_t to_read = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;
            file.read(buffer, to_read);
            mbedtls_sha256_update(&ctx, buffer, to_read);
            remaining -= to_read;
        }

        mbedtls_sha256_finish(&ctx, hash);
        mbedtls_sha256_free(&ctx);

        // Return first 32 bits
        uint32_t result = *(uint32_t*)hash;
        log_debug("Config hash: " << to_hex(result));
        return result;
    } catch (...) {
        log_error("Failed to calculate config hash");
        return 0;
    }
}

void StatePersistence::savePositionState() {
    if (!_fram || !_fram->IsInitialized()) {
        return;
    }

    // Save motor steps (all axes at once)
    steps_t steps[MAX_N_AXIS];
    auto    n_axis = Machine::Axes::_numberAxis;
    for (axis_t axis = X_AXIS; axis < n_axis; axis++) {
        steps[axis] = Machine::Stepping::getSteps(axis);
    }

    _fram->WriteBlock(FRAM_MOTOR_STEPS_ADDR, sizeof(steps), 1, (uint8_t*)steps);

    // Save homing status
    AxisMask homed = Machine::Homing::unhomed_axes();
    _fram->WriteBlock(FRAM_HOMING_STATUS_ADDR, sizeof(homed), 1, (uint8_t*)&homed);
}

void StatePersistence::saveParserState() {
    if (!_fram || !_fram->IsInitialized()) {
        return;
    }

    // Save entire gc_state structure directly
    _fram->WriteBlock(FRAM_PARSER_STATE_ADDR, sizeof(gc_state), 1, (uint8_t*)&gc_state);
}

void StatePersistence::saveParameters() {
    if (!_fram || !_fram->IsInitialized() || !_saveParameters) {
        return;
    }

    // Get all named parameters
    auto&    params = get_all_named_params();
    uint32_t count  = params.size();
    uint32_t offset = FRAM_PARAMETERS_ADDR;

    // Write count
    _fram->WriteBlock(offset, sizeof(count), 1, (uint8_t*)&count);
    offset += sizeof(count);

    // Write each parameter
    for (const auto& [parname, parvalue] : params) {
        if (offset >= FRAM_OVERRIDES_ADDR) {
            log_warn("Parameter section full, skipping remaining parameters");
            break;
        }

        uint8_t name_len = parname.length();
        _fram->WriteByte(offset++, name_len);

        for (char c : parname) {
            _fram->WriteByte(offset++, c);
        }

        _fram->WriteBlock(offset, sizeof(float), 1, (uint8_t*)&parvalue);
        offset += sizeof(float);
    }
}

void StatePersistence::saveOverrides() {
    if (!_fram || !_fram->IsInitialized()) {
        return;
    }

    // Save system overrides
    Percent feed_ovr    = sys.f_override();
    Percent rapid_ovr   = sys.r_override();
    Percent spindle_ovr = sys.spindle_speed_ovr();

    uint32_t offset = FRAM_OVERRIDES_ADDR;
    _fram->WriteBlock(offset, sizeof(feed_ovr), 1, (uint8_t*)&feed_ovr);
    offset += sizeof(feed_ovr);

    _fram->WriteBlock(offset, sizeof(rapid_ovr), 1, (uint8_t*)&rapid_ovr);
    offset += sizeof(rapid_ovr);

    _fram->WriteBlock(offset, sizeof(spindle_ovr), 1, (uint8_t*)&spindle_ovr);
}

void StatePersistence::saveAllSections() {
    if (!_fram || !_fram->IsInitialized()) {
        return;
    }

    savePositionState();
    saveParserState();
    saveParameters();
    saveOverrides();
}

void StatePersistence::restorePositionState() {
    if (!_fram || !_fram->IsInitialized()) {
        return;
    }

    // Restore motor steps
    steps_t steps[MAX_N_AXIS];
    _fram->ReadBlock(FRAM_MOTOR_STEPS_ADDR, sizeof(steps), 1, (uint8_t*)steps);

    auto n_axis = Machine::Axes::_numberAxis;
    for (axis_t axis = X_AXIS; axis < n_axis; axis++) {
        Machine::Stepping::setSteps(axis, steps[axis]);
    }

    // Restore homing status
    AxisMask homed;
    _fram->ReadBlock(FRAM_HOMING_STATUS_ADDR, sizeof(homed), 1, (uint8_t*)&homed);
    // Note: We can't directly set Homing::_unhomed_axes as it's private
    // We need to restore it through the axes
    for (axis_t axis = X_AXIS; axis < n_axis; axis++) {
        if ((homed & (1 << axis)) == 0) {
            Machine::Homing::set_axis_homed(axis);
        } else {
            Machine::Homing::set_axis_unhomed(axis);
        }
    }
}

void StatePersistence::restoreParserState() {
    if (!_fram || !_fram->IsInitialized()) {
        return;
    }

    // Restore entire gc_state structure directly
    _fram->ReadBlock(FRAM_PARSER_STATE_ADDR, sizeof(gc_state), 1, (uint8_t*)&gc_state);
}

void StatePersistence::restoreParameters() {
    if (!_fram || !_fram->IsInitialized() || !_saveParameters) {
        return;
    }

    uint32_t count;
    uint32_t offset = FRAM_PARAMETERS_ADDR;

    _fram->ReadBlock(offset, sizeof(count), 1, (uint8_t*)&count);
    offset += sizeof(count);

    // Sanity check
    if (count > 1000) {  // Reasonable limit
        log_warn("Invalid parameter count in FRAM, skipping restore");
        return;
    }

    for (uint32_t i = 0; i < count; i++) {
        if (offset >= FRAM_OVERRIDES_ADDR) {
            break;
        }

        uint8_t name_len;
        _fram->ReadByte(offset++, &name_len);

        char name[256];
        for (uint8_t j = 0; j < name_len; j++) {
            _fram->ReadByte(offset++, (uint8_t*)&name[j]);
        }
        name[name_len] = '\0';

        float value;
        _fram->ReadBlock(offset, sizeof(value), 1, (uint8_t*)&value);
        offset += sizeof(value);

        set_named_param(name, value);
    }
}

void StatePersistence::restoreOverrides() {
    if (!_fram || !_fram->IsInitialized()) {
        return;
    }

    // Restore system overrides
    Percent feed_ovr, rapid_ovr, spindle_ovr;

    uint32_t offset = FRAM_OVERRIDES_ADDR;
    _fram->ReadBlock(offset, sizeof(feed_ovr), 1, (uint8_t*)&feed_ovr);
    offset += sizeof(feed_ovr);

    _fram->ReadBlock(offset, sizeof(rapid_ovr), 1, (uint8_t*)&rapid_ovr);
    offset += sizeof(rapid_ovr);

    _fram->ReadBlock(offset, sizeof(spindle_ovr), 1, (uint8_t*)&spindle_ovr);

    sys.set_f_override(feed_ovr);
    sys.set_r_override(rapid_ovr);
    sys.set_spindle_speed_ovr(spindle_ovr);
}

void StatePersistence::restoreAllSections() {
    if (!_fram || !_fram->IsInitialized()) {
        return;
    }

    restorePositionState();
    restoreParserState();
    restoreParameters();
    restoreOverrides();
}

void StatePersistence::saveTaskFunc(void* param) {
    StatePersistence* self = static_cast<StatePersistence*>(param);

    while (self->_running) {
        // Check for force save request
        uint8_t force;
        if (xQueueReceive(_forceSaveQueue, &force, pdMS_TO_TICKS(self->_saveIntervalMs))) {
            // Immediate save requested
            log_debug("Force save requested");
            self->saveAllSections();
        } else {
            // Normal periodic save
            self->saveAllSections();
        }
    }

    // Task ending
    vTaskDelete(nullptr);
}

void StatePersistence::init() {
    log_info("StatePersistence initializing");

    // Check if CS pin is defined
    if (!_csPin.defined()) {
        log_info("StatePersistence CS pin not configured, module disabled");
        return;
    }

    // Initialize pins
    _csPin.setAttr(Pin::Attr::Output);
    _csPin.on();  // CS high (inactive)

    if (_wpPin.defined()) {
        _wpPin.setAttr(Pin::Attr::Output);
        _wpPin.on();  // WP high (write enabled)
    }

    if (_holdPin.defined()) {
        _holdPin.setAttr(Pin::Attr::Output);
        _holdPin.on();  // HOLD high (not held)
    }

    // Create FRAM driver instance
    uint32_t spi_freq_hz = _spiFreqMhz * 1000000;                                     // Convert MHz to Hz
    _fram                = new FM25VXX(_csPin, _wpPin, _holdPin, 8192, spi_freq_hz);  // 8KB FRAM with configurable frequency
    _fram->Initialize();

    if (!_fram->IsInitialized()) {
        log_error("FRAM initialization failed");
        delete _fram;
        _fram = nullptr;
        return;
    }

    _initialized = true;
    log_info("FRAM initialized successfully");

    // Calculate current config hash
    _configHash = calculateConfigHash();

    // Read stored hash
    uint32_t stored_hash;
    _fram->ReadBlock(FRAM_CONFIG_HASH_ADDR, sizeof(stored_hash), 1, (uint8_t*)&stored_hash);
    log_debug("Read stored hash from FRAM: " << to_hex(stored_hash) << " (bytes: " << to_hex(((uint8_t*)&stored_hash)[0]) << " "
                                             << to_hex(((uint8_t*)&stored_hash)[1]) << " " << to_hex(((uint8_t*)&stored_hash)[2]) << " "
                                             << to_hex(((uint8_t*)&stored_hash)[3]) << ")");

    if (stored_hash != _configHash) {
        log_info("Configuration changed (stored: " << to_hex(stored_hash) << ", current: " << to_hex(_configHash)
                                                   << "), initializing FRAM state");
        log_debug("Writing hash to FRAM: " << to_hex(_configHash) << " (bytes: " << to_hex(((uint8_t*)&_configHash)[0]) << " "
                                           << to_hex(((uint8_t*)&_configHash)[1]) << " " << to_hex(((uint8_t*)&_configHash)[2]) << " "
                                           << to_hex(((uint8_t*)&_configHash)[3]) << ")");
        _fram->WriteBlock(FRAM_CONFIG_HASH_ADDR, sizeof(_configHash), 1, (uint8_t*)&_configHash);

        // Verify the write
        uint32_t verify_hash;
        _fram->ReadBlock(FRAM_CONFIG_HASH_ADDR, sizeof(verify_hash), 1, (uint8_t*)&verify_hash);
        log_debug("Verified hash after write: " << to_hex(verify_hash));

        saveAllSections();  // Save current state
    } else {
        log_info("Restoring state from FRAM");
        restoreAllSections();
    }

    // Create force save queue
    _forceSaveQueue = xQueueCreate(1, sizeof(uint8_t));

    // Start save task on support core (core 0)
    _running = true;
    xTaskCreatePinnedToCore(saveTaskFunc, "state_save", 8192, this, tskIDLE_PRIORITY + 1, &_saveTask, 0);

    log_info("StatePersistence module started (save interval: " << _saveIntervalMs << "ms)");
}

void StatePersistence::deinit() {
    if (_running) {
        log_info("StatePersistence shutting down");
        _running = false;

        // Final save before shutdown
        if (_fram && _fram->IsInitialized()) {
            saveAllSections();
        }

        // Wait for task to finish
        if (_saveTask) {
            vTaskDelay(pdMS_TO_TICKS(100));
            _saveTask = nullptr;
        }

        // Clean up queue
        if (_forceSaveQueue) {
            vQueueDelete(_forceSaveQueue);
            _forceSaveQueue = nullptr;
        }
    }
}

void StatePersistence::forceSave() {
    if (_forceSaveQueue) {
        uint8_t dummy = 1;
        xQueueSend(_forceSaveQueue, &dummy, 0);
        // Give time for save to complete
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

// Module registration - auto-discovered by ConfigurableModule system
ConfigurableModuleFactory::InstanceBuilder<StatePersistence> state_persistence_module
    __attribute__((init_priority(105))) ("state_persistence");
