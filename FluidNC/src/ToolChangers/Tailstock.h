#pragma once

#include "Configuration/Configurable.h"
#include "DynamicLimits.h"
#include "Config.h"
#include "Module.h"
#include "../SoftLimits/SoftLimitsComponent.h"

namespace ATCs {

    // Tailstock component that provides dynamic soft limits based on its position
    // and loaded tool. Prevents collision between turret tools and tailstock.
    // Implemented as a ConfigurableModule so it is created when a tailstock: section exists in config.
    //
    // Configuration example:
    //   tailstock:
    //     axis: Z                    # Axis the tailstock is on
    //     tool: 10                   # Tool number for loaded tool (e.g., live center)
    //     safety_margin: 5.0         # mm of clearance required
    //
    class Tailstock : public ConfigurableModule, public DynamicLimitProvider, public SoftLimitsComponent {
    private:
        // Configuration
        int32_t _loaded_tool = 0;           // Tool number currently loaded (0 = none)
        float   _safety_margin = 5.0f;      // Required clearance in mm
        char    _axis_letter = 'Z';         // Which axis tailstock is on
        axis_t  _axis = Z_AXIS;             // Axis index (computed from letter)
        float   _position = 0.0f;           // Current position (machine coords)
        bool    _extended = false;          // Is tailstock extended (active)?

        // For motorized tailstock, the spindle index that controls it
        int32_t _spindle_index = 1;         // Default to spindle/holder 1

    public:
        explicit Tailstock(const char* name) : ConfigurableModule(name) {}

        // Configuration::Configurable interface
        void group(Configuration::HandlerBase& handler) override;
        void afterParse() override;
        void validate() override {}

        // ConfigurableModule interface
        void init() override;

        // DynamicLimitProvider interface
        void getDynamicLimits(
            const float* current_mpos,
            const float* active_tlo,
            float* axis_min,
            float* axis_max
        ) override;

        const char* limitProviderName() override { return name(); }
        bool isActive() override { return _extended && _loaded_tool > 0; }

        // SoftLimitsComponent interface
        bool TestLimit(const float* from, const float* to) override;
        const char* componentName() const override { return name(); }

        // Tailstock control
        void setExtended(bool extended) { _extended = extended; }
        bool isExtended() const { return _extended; }

        void setPosition(float pos) { _position = pos; }
        float getPosition() const { return _position; }

        void setLoadedTool(int32_t tool) { _loaded_tool = tool; }
        int32_t getLoadedTool() const { return _loaded_tool; }

        int32_t getSpindleIndex() const { return _spindle_index; }

        ~Tailstock() override;
    };

}  // namespace ATCs

// Global tailstock instance (set when a tailstock module is configured)
extern ATCs::Tailstock* tailstock;
