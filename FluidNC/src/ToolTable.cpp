// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "ToolTable.h"
#include "FileStream.h"
#include "Logging.h"
#include "Machine/MachineConfig.h"
#include "Configuration/Parser.h"
#include "Configuration/ParserHandler.h"
#include "Configuration/Generator.h"
#include "Configuration/TokenState.h"
#include "NutsBolts.h"

#include <cstring>
#include <memory>
#include <string>

// Global tool table instance
ToolTable* toolTable = nullptr;

// ToolEntry implementation
void ToolEntry::group(Configuration::HandlerBase& handler) {
    handler.item("name", _name);
    handler.item("x", _offset[X_AXIS]);
    handler.item("y", _offset[Y_AXIS]);
    handler.item("z", _offset[Z_AXIS]);
    // Support additional axes if configured
    if (MAX_N_AXIS > A_AXIS) {
        handler.item("a", _offset[A_AXIS]);
    }
    if (MAX_N_AXIS > B_AXIS) {
        handler.item("b", _offset[B_AXIS]);
    }
    if (MAX_N_AXIS > C_AXIS) {
        handler.item("c", _offset[C_AXIS]);
    }
    // Tool radius for cutter compensation (G41/G42)
    handler.item("radius", _radius);
}

std::string ToolEntry::getDisplayName() const {
    if (!_name.empty()) {
        return _name;
    }
    return "Tool " + std::to_string(_number);
}

// TurretMapping implementation
void TurretMapping::group(Configuration::HandlerBase& handler) {
    for (int i = 0; i < MaxNumberPositions; ++i) {
        char buf[50];
        snprintf(buf, 50, "position%d", (i + 1));
        handler.item(buf, _position[i]);
    }
}

int32_t TurretMapping::getToolForPosition(int32_t position) const {
    if (position >= 1 && position <= MaxNumberPositions) {
        return _position[position - 1];
    }
    return 0;
}

void TurretMapping::setToolForPosition(int32_t position, int32_t toolNumber) {
    if (position >= 1 && position <= MaxNumberPositions) {
        _position[position - 1] = toolNumber;
    }
}

bool TurretMapping::hasAnyMapping() const {
    for (int i = 0; i < MaxNumberPositions; i++) {
        if (_position[i] != 0) {
            return true;
        }
    }
    return false;
}

// ToolTable implementation
ToolTable::ToolTable() : _filename("/localfs/tooltable.yaml") {}

ToolTable::~ToolTable() {
    clearTools();
}

void ToolTable::clearTools() {
    for (auto tool : _tools) {
        delete tool;
    }
    _tools.clear();
}

ToolEntry* ToolTable::findTool(int32_t toolNum) {
    for (auto tool : _tools) {
        if (tool && tool->_number == toolNum) {
            return tool;
        }
    }
    return nullptr;
}

const ToolEntry* ToolTable::findTool(int32_t toolNum) const {
    for (auto tool : _tools) {
        if (tool && tool->_number == toolNum) {
            return tool;
        }
    }
    return nullptr;
}

int32_t ToolTable::parseToolNumber(const char* sectionName) {
    // Check if section name starts with "tool"
    if (strncmp(sectionName, "tool", 4) == 0 && strlen(sectionName) > 4) {
        try {
            return std::stoi(sectionName + 4);
        } catch (...) { return -1; }
    }
    return -1;
}

bool ToolTable::load(const std::string& filename) {
    _filename = filename;
    clearTools();
    if (_turret == nullptr)
    {
        _turret = new TurretMapping();
    }
    memset(_turret->_position, 0, sizeof(_turret->_position));
    _loaded = false;
    _dirty  = false;

    try {
        FileStream file(filename, "rb", "");

        auto filesize = file.size();
        if (filesize <= 0) {
            log_info("Tool table file " << filename << " is empty or doesn't exist, starting with empty table");
            _loaded = true;
            return true;
        }

        auto buffer      = std::make_unique<char[]>(filesize + 1);
        buffer[filesize] = '\0';
        auto actual      = file.read(buffer.get(), filesize);
        if (actual != filesize) {
            log_error("Tool table file read error - expected " << filesize << " got " << actual);
            return false;
        }

        log_info("Loading tool table from " << filename);

        // Parse the YAML content using the Configuration parser
        Configuration::Parser        parser(std::string_view { buffer.get(), filesize });
        Configuration::ParserHandler handler(parser);

        // Parse each top-level section
        parser.Tokenize();
        while (parser._token._state != Configuration::TokenState::Eof) {
            // Convert string_view to std::string for safe comparison
            std::string key(parser.key());

            if (parser.is("turret")) {
                // Parse turret mapping section
                handler.enterSection("turret", _turret);
            } else {
                // Check if it's a tool section
                int32_t toolNum = parseToolNumber(key.c_str());
                if (toolNum > 0) {
                    // Create new tool entry and parse it
                    ToolEntry* entry = new ToolEntry(toolNum);
                    handler.enterSection(key.c_str(), entry);
                    _tools.push_back(entry);
                } else {
                    // Skip unknown section
                    log_warn("Unknown section in tool table: " << key);
                    parser.Tokenize();
                }
            }

            // Move to next token if held
            if (parser._token._state == Configuration::TokenState::Held) {
                parser._token._state = Configuration::TokenState::Matching;
            }
            if (parser._token._state == Configuration::TokenState::Matching) {
                parser.Tokenize();
            }
        }

        _loaded = true;
        log_info("Loaded " << _tools.size() << " tools from tool table");
        return true;

    } catch (const std::exception& e) {
        log_info("Tool table file " << filename << " not found or error: " << e.what() << ", starting with empty table");
        _loaded = true;
        return true;
    } catch (...) {
        log_info("Tool table file " << filename << " not found, starting with empty table");
        _loaded = true;
        return true;
    }
}

bool ToolTable::save() {
    if (!_dirty && _loaded) {
        return true;  // Nothing to save
    }

    try {
        FileStream file(_filename, "wb", "");

        // Use the Generator to write the YAML
        Configuration::Generator generator(file, 0);

        // Write tool entries
        for (auto tool : _tools) {
            if (tool) {
                std::string sectionName = "tool" + std::to_string(tool->_number);
                generator.section(sectionName.c_str(), tool);
            }
        }

        // Write turret mapping if any positions are mapped
        if (_turret->hasAnyMapping()) {
            generator.section("turret", _turret);
        }

        file.flush();
        _dirty = false;
        log_info("Saved tool table to " << _filename);
        return true;

    } catch (const std::exception& e) {
        log_error("Failed to save tool table: " << e.what());
        return false;
    } catch (...) {
        log_error("Failed to save tool table");
        return false;
    }
}

bool ToolTable::getToolOffset(int32_t toolNum, float* offset) const {
    const ToolEntry* tool = findTool(toolNum);
    if (tool == nullptr) {
        return false;
    }

    for (int i = 0; i < MAX_N_AXIS; i++) {
        offset[i] = tool->_offset[i];
    }
    return true;
}

void ToolTable::setToolOffset(int32_t toolNum, const float* offset) {
    ToolEntry* tool = findTool(toolNum);
    if (tool == nullptr) {
        // Create new entry
        tool = new ToolEntry(toolNum);
        for (int i = 0; i < MAX_N_AXIS; i++) {
            tool->_offset[i] = offset[i];
        }
        _tools.push_back(tool);
    } else {
        // Update existing entry
        for (int i = 0; i < MAX_N_AXIS; i++) {
            tool->_offset[i] = offset[i];
        }
    }
    _dirty = true;
}

float ToolTable::getToolRadius(int32_t toolNum) const {
    const ToolEntry* tool = findTool(toolNum);
    if (tool == nullptr) {
        return 0.0f;
    }
    return tool->_radius;
}

void ToolTable::setToolRadius(int32_t toolNum, float radius) {
    ToolEntry* tool = findTool(toolNum);
    if (tool == nullptr) {
        // Create new entry with just radius
        tool = new ToolEntry(toolNum);
        tool->_radius = radius;
        _tools.push_back(tool);
    } else {
        tool->_radius = radius;
    }
    _dirty = true;
}

ToolEntry* ToolTable::getTool(int32_t toolNum) {
    return findTool(toolNum);
}

const ToolEntry* ToolTable::getTool(int32_t toolNum) const {
    return findTool(toolNum);
}

ToolEntry* ToolTable::getToolByPosition(int32_t position) {
    Assert(_turret != nullptr, "Turret is not initialized.");
    int32_t toolNum = _turret->getToolForPosition(position);
    if (toolNum <= 0) {
        return nullptr;
    }
    return getTool(toolNum);
}

int32_t ToolTable::getToolNumberForPosition(int32_t position) const {
    Assert(_turret != nullptr, "Turret is not initialized.");
    return _turret->getToolForPosition(position);
}

void ToolTable::setTurretMapping(int32_t position, int32_t toolNumber) {
    Assert(_turret != nullptr, "Turret is not initialized.");
    _turret->setToolForPosition(position, toolNumber);
    _dirty = true;
}
