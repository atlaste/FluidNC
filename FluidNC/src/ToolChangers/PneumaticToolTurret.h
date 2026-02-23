#pragma once

#include <algorithm>
#include <vector>
#include <string>
#include <cstring>
#include "Pin.h"
#include "Machine/MachineConfig.h"
#include "NutsBolts.h"
#include "atc.h"
#include "PressureSensor.h"
#include "../State/StatePersistence.h"
#include "../SoftLimits/SoftLimitsComponent.h"

namespace ATCs {

    class PneumaticToolTurret : public ATC {
        // Configuration things:
        int32_t        offsetsPerRevolution = 3600;  // we assume 3600 values per revolution. That should be plenty
        char           toolChangeAxis       = 'C';   // could be a string I suppose
        const char*    pneumaticActionOn    = "M62 P1\n";
        const char*    pneumaticActionOff   = "M63 P1\n";
        PressureSensor pneumaticSensor;

        // For each tool we need:
        std::vector<float> toolOffsets;  // Position on the wheel, e.g. 0, 90, 180, 270. (in 'mm' whatever that might mean)
        std::string        toolTypes;    // 'I' = inside (boring), 'O' = outside (turning). e.g. "OOIO"

        // Safe position
        float safeX        = 0;
        float safeY        = 0;
        float safeZ        = 0;
        float safetyMargin = 5.0f;  // Extra clearance margin in mm

        Macro macro;

        // Persisted things:
        std::vector<float> toolLengthOffsets;
        int32_t            currentToolNumber = 0;
        bool               usePressureSensor = false;

        void run(const char* str);

        void setToolChangeStepperEnable(bool enabled);

    public:
        PneumaticToolTurret(const char* name) : ATC(name), pneumaticSensor() {}

        void init() override;

        void group(Configuration::HandlerBase& handler) override;

        void validate() override;

        bool changeTool(int toolNumber, bool probe = false);

        void setTool(int32_t toolNumber);

        // void probeToolLengthOffset();

        // // ATC API:
        void probe_notification() override;
        bool tool_change(tool_t value, bool pre_select, bool set_tool) override;

        // This ATC handles TLO internally when useTLO is enabled
        bool handles_tlo() override { return true; }
    };
}
