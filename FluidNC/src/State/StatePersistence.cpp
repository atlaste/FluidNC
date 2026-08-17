// Copyright (c) 2025 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "StatePersistence.h"
#include "MB85RC.h"
#include "Logging.h"
#include "NutsBolts.h"
#include "System.h"
#include "GCode.h"
#include "Stepping.h"
#include "Machine/Homing.h"
#include "Machine/Axes.h"
#include "Machine/MachineConfig.h"
#include "FileStream.h"
#include "Parameters.h"
#include "SettingsDefinitions.h"
#include "Machine/Axes.h"
#include "Spindles/Spindle.h"
#include "ToolChangers/atc.h"

#include <mbedtls/sha256.h>
#include <cstring>

namespace {
    // The ATC section is variable length, so it gets a window rather than an offset.  The
    // upper bound matches the window the fixed 8 KB layout used to give it; the lower bound
    // is just the length word that prefixes it.
    const uint32_t MinAtcBytes = 4;
    const uint32_t MaxAtcBytes = 3584;

    uint32_t align4(uint32_t value) { return (value + 3) & ~uint32_t(3); }
}

extern const char* git_info;

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
    handler.item("i2c_num", _i2cNum, -1, MAX_N_I2C - 1);
    handler.item("i2c_address", _i2cAddress, 0x08, 0x77);
    handler.item("i2c_size_bytes", _i2cSizeBytes, 256, 262144);
}

FramDevice* StatePersistence::createDevice() {
    // cs_pin wins, so an existing SPI configuration is unaffected by the I2C items below.
    if (_csPin.defined()) {
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

        return new FM25VXX(_csPin, _wpPin, _holdPin, 8192, uint32_t(_spiFreqMhz) * 1000000);
    }

#if MAX_N_I2C
    if (_i2cNum >= 0) {
        auto bus = config->_i2c[_i2cNum];
        if (bus == nullptr) {
            log_error("StatePersistence: i2c" << _i2cNum << " is not configured");
            return nullptr;
        }
        return new MB85RC(bus, uint8_t(_i2cAddress), uint32_t(_i2cSizeBytes));
    }
#endif

    return nullptr;
}

bool StatePersistence::buildLayout(uint32_t deviceSize) {
    // NOTE: 0x0000 is reserved for the initialization sequence.
    uint32_t offset = 4;

    _layout.configHash = offset;
    offset             = align4(offset + sizeof(uint32_t));

    _layout.motorSteps = offset;
    offset             = align4(offset + sizeof(steps_t) * MAX_N_AXIS);

    _layout.homingStatus = offset;
    offset               = align4(offset + sizeof(AxisMask));

    _layout.overrides = offset;
    offset            = align4(offset + 3 * sizeof(Percent));

    _layout.parserState = offset;
    offset              = align4(offset + sizeof(parser_state_t));

    _layout.atc = offset;

    if (offset + MinAtcBytes > deviceSize) {
        log_error("FRAM holds " << deviceSize << " bytes but the machine state needs at least " << (offset + MinAtcBytes));
        return false;
    }

    // Split what is left between the ATC and the named parameters, so neither can starve
    // the other on a small part.
    uint32_t spare    = deviceSize - offset;
    uint32_t atcBytes = spare / 2;
    if (atcBytes > MaxAtcBytes) {
        atcBytes = MaxAtcBytes;
    }
    if (atcBytes < MinAtcBytes) {
        atcBytes = MinAtcBytes;
    }

    _layout.atcEnd     = offset + atcBytes;
    _layout.parameters = _layout.atcEnd;
    _layout.end        = deviceSize;

    log_debug("FRAM layout: steps " << to_hex(_layout.motorSteps) << " homing " << to_hex(_layout.homingStatus) << " overrides "
                                    << to_hex(_layout.overrides) << " parser " << to_hex(_layout.parserState) << " atc "
                                    << to_hex(_layout.atc) << " params " << to_hex(_layout.parameters) << " end "
                                    << to_hex(_layout.end));

    return true;
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

        size_t git_len = strlen(git_info);
        mbedtls_sha256_update(&ctx, reinterpret_cast<const unsigned char*>(git_info), git_len);

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
    if (!usable()) {
        return;
    }

    // Save motor steps (all axes at once)
    steps_t steps[MAX_N_AXIS];
    auto    n_axis = Machine::Axes::_numberAxis;
    for (axis_t axis = X_AXIS; axis < n_axis; axis++) {
        steps[axis] = Machine::Stepping::getSteps(axis);
    }

    _fram->write(_layout.motorSteps, (uint8_t*)steps, sizeof(steps));

    // Save homing status
    AxisMask homed = Machine::Homing::unhomed_axes();
    _fram->write(_layout.homingStatus, (uint8_t*)&homed, sizeof(homed));
}

void StatePersistence::saveParserState() {
    if (!usable()) {
        return;
    }

    // Save entire gc_state structure directly
    _fram->write(_layout.parserState, (uint8_t*)&gc_state, sizeof(gc_state));
}

void StatePersistence::saveParameters() {
    if (!usable() || !_saveParameters) {
        return;
    }

    // Get all named parameters
    auto&    params = get_all_named_params();
    uint32_t count  = params.size();
    uint32_t offset = _layout.parameters;

    // Write count
    _fram->write(offset, (uint8_t*)&count, sizeof(count));
    offset += sizeof(count);

    // Write each parameter
    for (const auto& [parname, parvalue] : params) {
        // Each entry is a length byte, the name, then the value, so check for the whole
        // entry rather than discovering halfway through that it does not fit.
        if (offset + 1 + parname.length() + sizeof(float) > _layout.end) {
            static bool warned = false;
            if (!warned) {
                log_warn("Parameter section full, skipping remaining parameters (" << count << " params)");
                warned = true;
            }
            break;
        }

        uint8_t name_len = parname.length();
        _fram->writeByte(offset++, name_len);

        for (char c : parname) {
            _fram->writeByte(offset++, c);
        }

        _fram->write(offset, (const uint8_t*)&parvalue, sizeof(float));
        offset += sizeof(float);
    }
}

void StatePersistence::saveOverrides() {
    if (!usable()) {
        return;
    }

    // Save system overrides
    Percent feed_ovr    = sys.f_override();
    Percent rapid_ovr   = sys.r_override();
    Percent spindle_ovr = sys.spindle_speed_ovr();

    uint32_t offset = _layout.overrides;
    _fram->write(offset, (uint8_t*)&feed_ovr, sizeof(feed_ovr));
    offset += sizeof(feed_ovr);

    _fram->write(offset, (uint8_t*)&rapid_ovr, sizeof(rapid_ovr));
    offset += sizeof(rapid_ovr);

    _fram->write(offset, (uint8_t*)&spindle_ovr, sizeof(spindle_ovr));
}

void StatePersistence::saveSpindleState() {
    if (!usable()) {
        return;
    }

    auto                 atcs = ATCs::ATCFactory::objects();
    std::vector<uint8_t> atcData;

    atcData.resize(4);

    for (auto it : atcs) {
        it->save_atc_data(atcData);
    }

    uint32_t size = atcData.size();
    memcpy(atcData.data(), &size, 4);

    if (_layout.atc + size > _layout.atcEnd) {
        static bool warned = false;
        if (!warned) {
            log_warn("ATC state needs " << size << " bytes but only " << (_layout.atcEnd - _layout.atc) << " are available; not saved");
            warned = true;
        }
        return;
    }

    _fram->write(_layout.atc, atcData.data(), size);
}

void StatePersistence::saveAllSections() {
    if (!usable()) {
        return;
    }

    savePositionState();
    saveParserState();
    saveParameters();
    saveOverrides();
    saveSpindleState();
}

void StatePersistence::restorePositionState() {
    if (!usable()) {
        return;
    }

    // Restore motor steps
    steps_t steps[MAX_N_AXIS];
    _fram->read(_layout.motorSteps, (uint8_t*)steps, sizeof(steps));

    auto n_axis = Machine::Axes::_numberAxis;
    for (axis_t axis = X_AXIS; axis < n_axis; axis++) {
        Machine::Stepping::setSteps(axis, steps[axis]);
    }

    // Restore homing status
    AxisMask homed;
    _fram->read(_layout.homingStatus, (uint8_t*)&homed, sizeof(homed));
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
    if (!usable()) {
        return;
    }

    _fram->read(_layout.parserState, (uint8_t*)&gc_state, sizeof(gc_state));

    // Spindle and coolant hardware are not running after a restart — force off to match reality.
    // Keep spindle_speed so the user can resume with M3 at the previous speed.
    gc_state.modal.spindle = SpindleState::Disable;
    gc_state.modal.coolant = {};

    // G92 and TLO coords[] entries are RAM-only (is_saved=false), so they're
    // zeroed on boot. Push the FRAM-restored values into them.
    coords[CoordIndex::G92]->set(gc_state.coord_offset);
    coords[CoordIndex::TLO]->set(gc_state.tool_length_offset);

    // WCS offsets (G54-G59.3) are NVS-backed; reload the active one.
    coords[gc_state.modal.coord_select]->get(gc_state.coord_system);

    // Assume we're in the running state (e.g. idle or waiting).
    gc_state.modal.program_flow = ProgramFlow::Running;

    // Recompute position from the motor steps restored by restorePositionState()
    // rather than trusting the FRAM-saved position which may be slightly stale.
    gc_sync_position();

    gc_state_restored = true;

    gc_ngc_changed(CoordIndex::G92);
    gc_ngc_changed(CoordIndex::TLO);
    gc_wco_changed();
}

void StatePersistence::restoreParameters() {
    if (!usable() || !_saveParameters) {
        return;
    }

    uint32_t count;
    uint32_t offset = _layout.parameters;

    _fram->read(offset, (uint8_t*)&count, sizeof(count));
    offset += sizeof(count);

    // Sanity check
    if (count > 1000) {  // Reasonable limit
        log_warn("Invalid parameter count in FRAM, skipping restore");
        return;
    }

    for (uint32_t i = 0; i < count; i++) {
        if (offset >= _layout.end) {
            break;
        }

        uint8_t name_len;
        _fram->readByte(offset++, &name_len);

        // A truncated or garbled entry would otherwise walk off the end of name[].
        if (name_len == 0 || name_len > 200 || offset + name_len + sizeof(float) > _layout.end) {
            break;
        }

        char name[256];
        for (uint8_t j = 0; j < name_len; j++) {
            _fram->readByte(offset++, (uint8_t*)&name[j]);
        }
        name[name_len] = '\0';

        float value;
        _fram->read(offset, (uint8_t*)&value, sizeof(value));
        offset += sizeof(value);

        set_named_param(name, value);
    }
}

void StatePersistence::restoreOverrides() {
    if (!usable()) {
        return;
    }

    // Restore system overrides
    Percent feed_ovr, rapid_ovr, spindle_ovr;

    uint32_t offset = _layout.overrides;
    _fram->read(offset, (uint8_t*)&feed_ovr, sizeof(feed_ovr));
    offset += sizeof(feed_ovr);

    _fram->read(offset, (uint8_t*)&rapid_ovr, sizeof(rapid_ovr));
    offset += sizeof(rapid_ovr);

    _fram->read(offset, (uint8_t*)&spindle_ovr, sizeof(spindle_ovr));

    sys.set_f_override(feed_ovr);
    sys.set_r_override(rapid_ovr);
    sys.set_spindle_speed_ovr(spindle_ovr);
}

void StatePersistence::restoreSpindleState() {
    if (!usable()) {
        return;
    }

    // Read the size first
    uint32_t length;
    _fram->read(_layout.atc, (uint8_t*)&length, sizeof(length));

    // Sanity check
    if (length < 4 || _layout.atc + length > _layout.atcEnd) {
        log_debug("Invalid ATC data length: " << length);
        return;
    }

    // Read the entire ATC data block
    std::vector<uint8_t> atcData;
    atcData.resize(length);
    _fram->read(_layout.atc, atcData.data(), length);

    // Restore ATC data directly to ATC objects (not through spindles,
    // because spindle->_atc isn't set until spindle init runs later).
    size_t index = 4;
    auto   atcs  = ATCs::ATCFactory::objects();
    for (auto it : atcs) {
        if (index >= length) {
            break;
        }
        it->restore_atc_data(atcData, index);
    }
}

void StatePersistence::restoreAllSections() {
    if (!usable()) {
        return;
    }

    log_info("Restoring state");

    restorePositionState();
    restoreParserState();
    restoreParameters();
    restoreOverrides();
    restoreSpindleState();
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
    _fram = createDevice();
    if (_fram == nullptr) {
        log_debug("StatePersistence: no FRAM configured, module disabled");
        return;
    }

    log_info("StatePersistence initializing with " << _fram->description());

    if (!_fram->initialize()) {
        log_error("FRAM initialization failed");
        delete _fram;
        _fram = nullptr;
        return;
    }

    if (!buildLayout(_fram->size())) {
        delete _fram;
        _fram = nullptr;
        return;
    }

    _initialized = true;
    log_info("FRAM initialized successfully, " << _fram->size() << " bytes");

    // Calculate current config hash
    _configHash = calculateConfigHash();

    // Read stored hash
    uint32_t stored_hash = 0;
    _fram->read(_layout.configHash, (uint8_t*)&stored_hash, sizeof(stored_hash));

    if (stored_hash != _configHash) {
        log_info("Configuration changed (stored: " << to_hex(stored_hash) << ", current: " << to_hex(_configHash)
                                                   << "), initializing FRAM state");
        _fram->write(_layout.configHash, (uint8_t*)&_configHash, sizeof(_configHash));

        // Verify the write
        uint32_t verify_hash = 0;
        _fram->read(_layout.configHash, (uint8_t*)&verify_hash, sizeof(verify_hash));
        if (verify_hash != _configHash) {
            log_error("FRAM hash write did not stick; state will not persist");
            delete _fram;
            _fram        = nullptr;
            _initialized = false;
            return;
        }

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
        if (usable()) {
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
