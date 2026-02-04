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

    class PneumaticToolTurret : public ATC, public SoftLimitsComponent {
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
        std::string        toolTypes;            // 'I' = inside (boring), 'O' = outside (turning). e.g. "OOIO"

        // std::string        toolProbeDirections;  // Direction to probe this tool, e.g. XXXZ
        // std::vector<float> probeMaxTravel;       // Max travel for probing this tool, e.g. 100, 100, 80, 20
        // std::vector<float> probePosition;        // Position to start probing this tool from, e.g. 0, 0, 0, 0

        // Safe retract configuration:
        float              safeX = 0;            // Safe X position in machine coordinates
        float              safeZ = 0;            // Safe Z position in machine coordinates
        float              safetyMargin = 5.0f;  // Extra clearance margin in mm
        float              maxRetractZ = 0;       // Maximum Z position to retract to (due to tool post)

        // ============== ATC Collision Geometry Configuration ==============
        // Bed geometry (for slanted bed lathes)
        float _bedAngleDeg = 30.0f;              // Bed slant angle in degrees
        float _carriageOffsetX = 0.0f;           // Offset from carriage to chuck centerline (X)
        float _carriageOffsetY = 0.0f;           // Offset from carriage to chuck centerline (Y)

        // ATC body box (relative to carriage mounting point)
        float _boxOriginX = 0.0f;
        float _boxOriginY = 0.0f;
        float _boxOriginZ = 0.0f;
        float _boxSizeX = 168.0f;                // Box width in X
        float _boxSizeY = 126.0f;                // Box height in Y
        float _boxSizeZ = 165.5f;                // Box depth in Z

        // Tool cylinder body (Z-aligned, relative to carriage)
        float _cylinderCenterX = 55.0f;          // Cylinder center X from carriage
        float _cylinderCenterY = 71.0f;          // Cylinder center Y from carriage
        float _cylinderRadius = 70.0f;           // Cylinder radius (140mm diameter / 2)
        float _cylinderZMin = 0.0f;              // Cylinder Z start
        float _cylinderZMax = 28.0f;             // Cylinder Z end (depth)

        // Square tool (OD/turning) geometry
        float _squareToolSize = 12.0f;           // 12x12mm cross-section
        float _squareToolDefaultStickout = 50.0f;// Default stickout if no TLO
        float _squareToolRadiusFromCenter = 70.0f;// Distance from cylinder center

        // Boring tool (ER11) geometry
        float _boringToolRadiusFromCenter = 55.0f;// Distance from cylinder center
        float _boringToolDiameter = 14.0f;       // Tool holder diameter

        // Tool angle arrays (degrees) - 8 square tools and 8 boring positions
        static constexpr float SQUARE_TOOL_ANGLES[8] = {0, 45, 90, 135, 180, 225, 270, 315};
        static constexpr float BORING_TOOL_ANGLES[8] = {24, 69, 114, 159, 204, 249, 294, 339};

        // External collision objects (chuck)
        float _chuckCenterX = 0.0f;              // Chuck center X (usually 0 = centerline)
        float _chuckCenterY = 0.0f;              // Chuck center Y
        float _chuckRadius = 75.0f;              // Chuck radius
        float _chuckZMin = 0.0f;                 // Chuck Z start (face)
        float _chuckZMax = 80.0f;                // Chuck Z end (depth)
        bool _chuckCollisionEnabled = false;     // Enable chuck collision checking

        // ============== Helper methods for collision detection ==============
        void transformToChuckSpace(const float* mpos, float* chuckSpace) const;
        bool checkATCBodyCollision(const float* fromChuck, const float* toChuck) const;
        bool checkToolCylinderCollision(const float* fromChuck, const float* toChuck) const;
        bool checkSquareToolCollision(int toolIndex, const float* fromChuck, const float* toChuck) const;
        bool checkBoringToolCollision(int toolIndex, const float* fromChuck, const float* toChuck) const;
        bool checkChuckCollision(const float* fromChuck, const float* toChuck) const;
        float getToolStickout(int toolNumber) const;

        Macro macro;

        // Persisted things:
        std::vector<float> toolLengthOffsets;
        int32_t            currentToolNumber = 0;

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
        bool handles_tlo() override { return useTLO; }

        // SoftLimitsComponent implementation
        bool TestLimit(const float* from, const float* to) override;
        const char* componentName() const override { return "ToolTurret"; }
    };
}
