// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Config.h"
#include "Configuration/Configurable.h"
#include "Configuration/HandlerBase.h"

#include <vector>
#include <string>

// Individual tool entry in the tool table
class ToolEntry : public Configuration::Configurable {
public:
    int32_t     _number = 0;               // Tool number (extracted from section name)
    std::string _name;                     // Optional descriptive name
    float       _offset[MAX_N_AXIS] = {};  // X, Y, Z, etc. offsets (mm), defaults to 0

    ToolEntry() = default;
    explicit ToolEntry(int32_t number) : _number(number) {}

    void group(Configuration::HandlerBase& handler) override;
    void validate() override {}
    void afterParse() override {}

    // Get default name if none specified
    std::string getDisplayName() const;
};

// Turret/magazine position mapping
class TurretMapping : public Configuration::Configurable {
public:
    static const int MaxNumberPositions = 24;

    // Position to tool number mapping (position1 -> tool #, etc.)
    int32_t _position[MaxNumberPositions] = {};  // Support up to 20 positions, 0 = empty

    void group(Configuration::HandlerBase& handler) override;
    void validate() override {}
    void afterParse() override {}

    // Get tool number for a turret position (1-based), returns 0 if not mapped
    int32_t getToolForPosition(int32_t position) const;

    // Set tool number for a turret position (1-based)
    void setToolForPosition(int32_t position, int32_t toolNumber);

    // Check if any positions are mapped
    bool hasAnyMapping() const;
};

// Main tool table class - container that parses/generates the YAML file
class ToolTable {
private:
    std::vector<ToolEntry*> _tools;             // Tool entries (linear scan by _number)
    TurretMapping*          _turret = nullptr;  // Turret position mapping
    std::string             _filename;          // Path to tooltable.yaml
    bool                    _loaded = false;
    bool                    _dirty  = false;  // True if changes need to be saved

    // Parse a tool section name like "tool1" -> 1, "tool10" -> 10
    static int32_t parseToolNumber(const char* sectionName);

    // Clean up tool entries
    void clearTools();

    // Find tool by number (linear scan), returns nullptr if not found
    ToolEntry*       findTool(int32_t toolNum);
    const ToolEntry* findTool(int32_t toolNum) const;

public:
    ToolTable();
    ~ToolTable();

    // Load tool table from file
    bool load(const std::string& filename = "/localfs/tooltable.yaml");

    // Save tool table to file
    bool save();

    // Get tool offset by tool number
    // Returns true if tool exists, copies offsets to provided array
    bool getToolOffset(int32_t toolNum, float* offset) const;

    // Set tool offset for a tool number
    // Creates tool entry if it doesn't exist
    void setToolOffset(int32_t toolNum, const float* offset);

    // Get tool entry by number (returns nullptr if not found)
    ToolEntry*       getTool(int32_t toolNum);
    const ToolEntry* getTool(int32_t toolNum) const;

    // Get tool by turret position (uses turret mapping)
    // Returns nullptr if position not mapped or tool doesn't exist
    ToolEntry* getToolByPosition(int32_t position);

    // Get tool number for turret position
    int32_t getToolNumberForPosition(int32_t position) const;

    // Set turret position mapping
    void setTurretMapping(int32_t position, int32_t toolNumber);

    // Get number of tools in table
    size_t count() const { return _tools.size(); }

    // Check if table has been loaded
    bool isLoaded() const { return _loaded; }

    // Check if there are unsaved changes
    bool isDirty() const { return _dirty; }

    // Mark as dirty (needs save)
    void markDirty() { _dirty = true; }

    // Iterate over all tools
    const std::vector<ToolEntry*>& tools() const { return _tools; }

    // Access turret mapping
    TurretMapping*       turret() { return _turret; }
    const TurretMapping* turret() const { return _turret; }

    // Get filename
    const std::string& filename() const { return _filename; }
};

// Global tool table instance (set by MachineConfig)
extern ToolTable* toolTable;
