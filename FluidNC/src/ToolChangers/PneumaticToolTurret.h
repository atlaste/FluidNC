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

namespace ATCs {

    class PneumaticToolTurret : public ATC {
        // Configuration things:
        int32_t        offsetsPerRevolution = 3600;  // we assume 3600 values per revolution. That should be plenty
        char           toolChangeAxis       = 'C';   // could be a string I suppose
        const char*    pneumaticActionOn    = "M62 P1\n";
        const char*    pneumaticActionOff   = "M63 P1\n";
        bool           useTLO               = false;
        int32_t        probeSeekRate        = 200;
        int32_t        probeFeedRate        = 80;
        PressureSensor pneumaticSensor;

        // For each tool we need:
        std::vector<float> toolOffsets;          // Position on the wheel, e.g. 0, 90, 180, 270.
        std::string        toolProbeDirections;  // Direction to probe this tool, e.g. XXXZ
        std::vector<float> probeMaxTravel;       // Max travel for probing this tool, e.g. 100, 100, 80, 20
        std::vector<float> probePosition;        // Position to start probing this tool from, e.g. 0, 0, 0, 0

        Macro macro;

        // Persisted things:
        std::vector<float> toolLengthOffsets;
        int32_t            currentToolNumber = 0;

        void run(const char* str);

        void setToolChangeStepperEnable(bool enabled);

    public:
        void init() override;

        void group(Configuration::HandlerBase& handler) override;

        void validate() override;

        bool changeTool(int toolNumber, bool probe = false);

        void setTool(int32_t toolNumber);

        void probeToolLengthOffset();

        // // ATC API:
        void probe_notification() override;
        bool tool_change(uint8_t value, bool pre_select, bool set_tool) override;
    };
}
