#include "PneumaticToolTurret.h"

#include <algorithm>
#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include "Pin.h"
#include "Protocol.h"
#include "System.h"
#include "Machine/MachineConfig.h"
#include "NutsBolts.h"
#include "atc.h"
#include "../State/StatePersistence.h"
#include "../Logging.h"
#include "../ToolTable.h"
#include "../SoftLimits/LimitsChecker.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ATCs {

    // Static constexpr definitions
    constexpr float PneumaticToolTurret::SQUARE_TOOL_ANGLES[8];
    constexpr float PneumaticToolTurret::BORING_TOOL_ANGLES[8];
    void PneumaticToolTurret::run(const char* str)  // execute g-code, wait until it's done. Should be "macro.addf"
    {
        macro.erase();
        macro.addf("%s", str);
        macro.run(nullptr);
    }

    void PneumaticToolTurret::setToolChangeStepperEnable(bool enabled) {
        const char* axis = "XYZABCUVW";
        auto        idx  = strchr(axis, std::toupper(toolChangeAxis));
        Assert(idx != NULL, "Incorrect tool change axis");

        axis_t id = axis_t(int(idx - axis));
        config->_axes->set_disable(id, !enabled);
    }

    void PneumaticToolTurret::init() {
        log_info("Initializing Pneumatic Tool Turret. Disabling tool change stepper.");
        // disable the tool change stepper:
        setToolChangeStepperEnable(false);

        log_info("Current tool number: " << int(currentToolNumber));

        // Set the correct tool length offset from tool table:
        if (useTLO && currentToolNumber > 0) {
            // Use G43 H# to load TLO from tool table
            char setTLO[32];
            snprintf(setTLO, sizeof(setTLO), "G43 H%d\n", int(currentToolNumber));
            run(setTLO);
        }

        // Register with soft limits checker
        LimitsChecker::instance().Register(this);
    }

    void PneumaticToolTurret::group(Configuration::HandlerBase& handler) {
        handler.item("offsetsPerRevolution", offsetsPerRevolution);
        handler.item("toolOffsets", toolOffsets);

        // Probing:
        handler.item("useTLO", useTLO);
        // handler.item("toolProbeDirections", toolProbeDirections);
        // handler.item("probeMaxTravel", probeMaxTravel);
        // handler.item("probePosition", probePosition);

        // Tool types and safe retract:
        handler.item("toolTypes", toolTypes);          // 'I' = inside (boring), 'O' = outside (turning)
        handler.item("safeX", safeX);                  // Safe X position (machine coords)
        handler.item("safeZ", safeZ);                  // Safe Z position (machine coords)
        handler.item("safetyMargin", safetyMargin);    // Extra clearance in mm
        handler.item("maxRetractZ", maxRetractZ);      // Maximum Z position to retract to (due to tool post)

        // Bed geometry (for slanted bed lathes)
        handler.item("bed_angle_deg", _bedAngleDeg);
        handler.item("carriage_offset_x", _carriageOffsetX);
        handler.item("carriage_offset_y", _carriageOffsetY);

        // ATC body box
        handler.item("box_origin_x", _boxOriginX);
        handler.item("box_origin_y", _boxOriginY);
        handler.item("box_origin_z", _boxOriginZ);
        handler.item("box_size_x", _boxSizeX);
        handler.item("box_size_y", _boxSizeY);
        handler.item("box_size_z", _boxSizeZ);

        // Tool cylinder
        handler.item("cylinder_center_x", _cylinderCenterX);
        handler.item("cylinder_center_y", _cylinderCenterY);
        handler.item("cylinder_radius", _cylinderRadius);
        handler.item("cylinder_z_min", _cylinderZMin);
        handler.item("cylinder_z_max", _cylinderZMax);

        // Tool geometry
        handler.item("square_tool_size", _squareToolSize);
        handler.item("square_tool_default_stickout", _squareToolDefaultStickout);
        handler.item("square_tool_radius", _squareToolRadiusFromCenter);
        handler.item("boring_tool_radius", _boringToolRadiusFromCenter);
        handler.item("boring_tool_diameter", _boringToolDiameter);

        // Chuck collision
        handler.item("chuck_collision_enabled", _chuckCollisionEnabled);
        handler.item("chuck_center_x", _chuckCenterX);
        handler.item("chuck_center_y", _chuckCenterY);
        handler.item("chuck_radius", _chuckRadius);
        handler.item("chuck_z_min", _chuckZMin);
        handler.item("chuck_z_max", _chuckZMax);
    }

    void PneumaticToolTurret::validate() {
        Assert(toolOffsets.size() > 0, "No tool offsets are configured");
        // if (useTLO) {
        //     Assert(toolProbeDirections.size() == toolOffsets.size(), "Probe directions length should match the tool offsets vector length");
        //     Assert(probeMaxTravel.size() == toolOffsets.size(), "Probe max travel vector should match the tool offsets vector length");
        //     Assert(probePosition.size() == toolOffsets.size(), "Probe positions vector should match the tool offsets vector length");
        // }
        // If toolTypes is specified, it should match the number of tools
        if (toolTypes.size() > 0) {
            Assert(toolTypes.size() == toolOffsets.size(), "Tool types length should match the tool offsets vector length");
            // Validate that all characters are 'I' or 'O'
            for (char c : toolTypes) {
                Assert(c == 'I' || c == 'O' || c == 'i' || c == 'o', "Tool types must be 'I' (inside) or 'O' (outside)");
            }
        }
        Assert(maxRetractZ >= safeZ, "maxRetractZ must be greater than or equal to safe Z position.");
    }

    bool PneumaticToolTurret::changeTool(int toolNumber, bool probe) {
        // NOTE: See manual_atc for details on how to set this up.
        //
        // Sequence:
        // - Get current position
        // - Move to safe position
        // - Do the pneumatic stuff
        // - Do a tool change
        // - Pneumatic stuff again
        // - Update tool number and tool offset
        // - Move to old position, keep tool offset in mind

        protocol_buffer_synchronize();  // wait for all motion to complete

        // First thing we're going to do here is enable the stepper motor for the tool changer:
        setToolChangeStepperEnable(true);

        //Assert(pneumaticEndstop.read(), "Pneumatic endstop is not active. Tool is not engaged.");

        bool was_inch_mode = (gc_state.modal.units == Units::Inches);
        if (was_inch_mode) {
            run("G21\n");
        }

        run("#<start_x >= #<_x>\n");
        run("#<start_y >= #<_y>\n");
        run("#<start_z >= #<_z>\n");

        // Determine if current tool is inside (boring/drilling) or outside (turning/facing)
        bool isInsideTool = false;
        if (toolTypes.size() > currentToolNumber) {
            char toolType = std::toupper(toolTypes[currentToolNumber]);
            isInsideTool = (toolType == 'I');
        }
        else {
            Assert(false, "Tool type not found for tool number %d. Cannot retract safely.", currentToolNumber);
        }

        // Safe retract sequence depends on tool type:
        // - Inside tools (boring): Z first (out of hole, don't crash into tailstock!), then X, then more Z.
        // - Outside tools (turning): X first (away from OD), then Z.
        char safeRetract[100];
        if (isInsideTool) {
            // Inside tool: First retract Z (out of the bore), then X
            // Use TLO + safety margin if available, otherwise use configured safeZ
            float tlo[MAX_N_AXIS] = {};
            bool hasTLO = useTLO && toolTable != nullptr && toolTable->getToolOffset(currentToolNumber, tlo);
            if (hasTLO) {
                // Calculate safe Z position based on TLO + margin
                // This ensures we clear the bore before moving X
                float safeRetractLength = tlo[Z_AXIS] + safetyMargin;
                if (safeRetractLength > maxRetractZ) {
                    safeRetractLength = maxRetractZ;
                    log_info("Safe retract Z is above maxRetractZ. Aborting.");
                    mc_critical(ExecAlarm::HardLimit);

                    return false;
                }
                snprintf(safeRetract, 100, "G53 G0 Z%0.4f\n", safeRetractLength);
                run(safeRetract);
            } else {
                log_error("TLO not found for tool number " << currentToolNumber << ". Cannot retract safely.");
                return false;
            }
        }
        // fallthrough:
        {
            // Inside & outside tool: First retract X (away from workpiece OD), then Z
            snprintf(safeRetract, 100, "G53 G0 X%0.4f\n", safeX);
            run(safeRetract);
            snprintf(safeRetract, 100, "G53 G0 Z%0.4f\n", safeZ);
            run(safeRetract);
        }

        // Before doing the tool change, we need to check if the pressure is on:
        while (pneumaticSensor.readBar() < 2.0f) {
            log_info("Cannot do pneumatic action; pressure is not enough. We need 2.0 bar, read: " << pneumaticSensor.readBar() << " bar.");
            for (int i = 0; i < 20 && pneumaticSensor.readBar() < 2.0f && sys.state() != State::Alarm; ++i) {
                delay_ms(50);
                protocol_buffer_synchronize();
            }
        }
        if (sys.state() == State::Alarm)
        {
            return false;
        }

        // Start tool change
        run(pneumaticActionOn);

        // wait for pneumatic actuator
        protocol_buffer_synchronize();  // Wait for all motion to complete
        delay_ms(300);                  // Wait for pneumatic action to complete

        // Assert(pneumaticEndstop.read(), "Pneumatic endstop is active. Cannot change tool.");
        // for (int i = 0; i < 50 && pneumaticEndstop.read(); ++i) {
        //     Timer::delayMillis(10);
        // }
        // Assert(!pneumaticEndstop.read(), "Pneumatic endstop is active. Cannot change tool.");

        // Should we move forward or backward?
        auto offset1 = toolOffsets[currentToolNumber];
        auto offset2 = toolOffsets[toolNumber];

        // Calculate how much we have to move
        auto diff = offset2 - offset1;
        if (std::abs(diff) > offsetsPerRevolution / 2) {
            // we need to go the other way
            diff = diff > 0 ? diff - offsetsPerRevolution : diff + offsetsPerRevolution;
        }

        char toolChange[100];
        snprintf(toolChange, 100, "G92 %c0\nG0 %c%0.3f\n", toolChangeAxis, toolChangeAxis, diff);
        run(toolChange);

        protocol_buffer_synchronize();  // wait for all motion to complete
        run(pneumaticActionOff);
        protocol_buffer_synchronize();  // wait for all motion to complete
        delay_ms(300);                  // Wait for pneumatic action to complete

        // Wait till the endstop is active again
        // for (int i = 0; i < 50 && !pneumaticEndstop.read(); ++i) {
        //     delay_ms(10);
        // }
        // Assert(pneumaticEndstop.read(), "Pneumatic endstop is still active. Tool change failed!");

        // And disable the stepper again. Otherwise it's just going to fight the coupling.
        setToolChangeStepperEnable(false);

        if (useTLO) {
            // Load TLO from tool table using G43 H#
            snprintf(toolChange, sizeof(toolChange), "G43 H%d\n", toolNumber);
            run(toolChange);
        }

        // DO NOT return to location before the tool change. Because you don't know the tool geometry, it
        // might crash the machine!!!
        // 
        // CAM needs to handle the approach after a tool change in lathes!

        // run("G0Z#<start_z>\n");
        // run("G0X#<start_x>\n");

        // restore inch mode
        if (was_inch_mode) {
            run("G20\n");
        }

        // Save the tool number.
        currentToolNumber = toolNumber;

        return true;
    }

    void PneumaticToolTurret::setTool(int32_t toolNumber) {
        currentToolNumber = toolNumber;
    }

    /*
    void PneumaticToolTurret::probeToolLengthOffset() {
        if (!useTLO) {
            return;
        }

        // NOTE: *ONLY* to be used from the toolchange method!

        Assert(toolProbeDirections.size() > currentToolNumber, "Tool probe directions are missing.");

        char direction = toolProbeDirections[currentToolNumber];

        const char* axis = "XYZABCUVW";
        auto        idx  = strchr(axis, std::toupper(direction));
        Assert(idx != NULL, "Incorrect tool change axis");
        auto maxTravel = probeMaxTravel[idx - axis];

        // TODO: Perhaps move to some position before probing...
        char buf[100];

        // do a fast probe if there is a seek that is faster than feed
        if (probeSeekRate > probeFeedRate) {
            snprintf(buf, 100, "G53 G38.2 %c%0.3f F%0.3f\n", direction, double(maxTravel), double(probeSeekRate));
            run(buf);
            run("G0Z[#<_z> + 5]\n");  // retract befor next probe

            maxTravel = 7;  // 7mm is plenty now.
        }

        // do the feed rate probe
        snprintf(buf, 100, "G53 G38.2 %c%0.3f F%0.3f\n", direction, double(maxTravel), double(probeFeedRate));

        // get the position:
        float probe_position[MAX_N_AXIS];
        steps_to_mpos(probe_position, probe_steps);
        toolLengthOffsets[currentToolNumber] = probe_position[idx - axis] + probePosition[idx - axis];
    }
    */

    // ATC API:
    void PneumaticToolTurret::probe_notification() {}
    bool PneumaticToolTurret::tool_change(tool_t value, bool pre_select, bool set_tool) {
        if (pre_select) {
            // TODO FIXME: Is this ever used?
            return true;
        }

        if (set_tool) {
            // M61 set-tool. This has nothing to do with TLO, it just sets the tool as 'current'.
            currentToolNumber = value;
            return true;
        } else {
            // M6 tool change
            if (!changeTool(value)) {
                return false;
            } else {
                return true;
            }
        }
    }

    // ============== Collision Detection Helper Methods ==============

    void PneumaticToolTurret::transformToChuckSpace(const float* mpos, float* chuckSpace) const {
        // Transform machine position to chuck centerline-relative coordinates
        // Accounts for slanted bed angle
        float angleRad = _bedAngleDeg * float(M_PI) / 180.0f;
        float cosAngle = std::cos(angleRad);
        float sinAngle = std::sin(angleRad);

        // X axis is on slant - project to horizontal and vertical components
        chuckSpace[X_AXIS] = mpos[X_AXIS] * cosAngle + _carriageOffsetX;
        chuckSpace[Y_AXIS] = mpos[X_AXIS] * sinAngle + _carriageOffsetY;
        chuckSpace[Z_AXIS] = mpos[Z_AXIS];
    }

    float PneumaticToolTurret::getToolStickout(int toolNumber) const {
        if (toolTable == nullptr) {
            return _squareToolDefaultStickout;
        }
        float tlo[MAX_N_AXIS] = {};
        if (toolTable->getToolOffset(toolNumber, tlo)) {
            // TLO X component is the stickout for square tools
            // TLO Z component is the stickout for boring tools
            // Return the larger absolute value
            float stickout = std::max(std::abs(tlo[X_AXIS]), std::abs(tlo[Z_AXIS]));
            return stickout > 0 ? stickout : _squareToolDefaultStickout;
        }
        return _squareToolDefaultStickout;
    }

    bool PneumaticToolTurret::checkATCBodyCollision(const float* fromChuck, const float* toChuck) const {
        // Build AABB for ATC body box at current position
        float boxMin[3] = {
            fromChuck[X_AXIS] + _boxOriginX,
            fromChuck[Y_AXIS] + _boxOriginY,
            fromChuck[Z_AXIS] + _boxOriginZ
        };
        float boxMax[3] = {
            fromChuck[X_AXIS] + _boxOriginX + _boxSizeX,
            fromChuck[Y_AXIS] + _boxOriginY + _boxSizeY,
            fromChuck[Z_AXIS] + _boxOriginZ + _boxSizeZ
        };

        // Check if line segment (which represents motion of a point, like cutting tip)
        // would enter the ATC body box
        // Note: We need to check if external objects (chuck, tailstock) would collide with ATC body
        // For now, we check if the motion path intersects the body
        return lineIntersectsAABB(fromChuck, toChuck, boxMin, boxMax);
    }

    bool PneumaticToolTurret::checkToolCylinderCollision(const float* fromChuck, const float* toChuck) const {
        // Tool cylinder center moves with the carriage
        float cylCenterX = fromChuck[X_AXIS] + _cylinderCenterX;
        float cylCenterY = fromChuck[Y_AXIS] + _cylinderCenterY;

        // Check line intersection with circle in XY plane
        if (lineIntersectsCircle2D(fromChuck[X_AXIS], fromChuck[Y_AXIS],
                                   toChuck[X_AXIS], toChuck[Y_AXIS],
                                   cylCenterX, cylCenterY, _cylinderRadius)) {
            // Check Z range overlap
            float segZMin = std::min(fromChuck[Z_AXIS], toChuck[Z_AXIS]);
            float segZMax = std::max(fromChuck[Z_AXIS], toChuck[Z_AXIS]);
            float cylZMin = fromChuck[Z_AXIS] + _cylinderZMin;
            float cylZMax = fromChuck[Z_AXIS] + _cylinderZMax;
            if (zRangesOverlap(segZMin, segZMax, cylZMin, cylZMax)) {
                return true;
            }
        }
        return false;
    }

    bool PneumaticToolTurret::checkSquareToolCollision(int toolIndex, const float* fromChuck, const float* toChuck) const {
        if (toolIndex < 0 || toolIndex >= 8) return false;

        // Get tool angle and calculate position
        float angleDeg = SQUARE_TOOL_ANGLES[toolIndex];
        float angleRad = angleDeg * float(M_PI) / 180.0f;

        // Tool position relative to cylinder center
        float toolOffsetX = _squareToolRadiusFromCenter * std::cos(angleRad);
        float toolOffsetY = _squareToolRadiusFromCenter * std::sin(angleRad);

        // Tool center in chuck space (moves with carriage)
        float toolCenterX = fromChuck[X_AXIS] + _cylinderCenterX + toolOffsetX;
        float toolCenterY = fromChuck[Y_AXIS] + _cylinderCenterY + toolOffsetY;

        // Get tool stickout from TLO
        // Tool numbers: square tools are positions 0-7, so tool number = toolIndex + 1
        // But this depends on your mapping - adjust as needed
        float stickout = getToolStickout(toolIndex);

        // Build AABB for the square tool (12x12mm cross-section, stickout length)
        // The tool sticks out radially from the cylinder
        float halfSize = _squareToolSize / 2.0f;
        float toolMin[3] = {
            toolCenterX - halfSize,
            toolCenterY - halfSize,
            fromChuck[Z_AXIS] + _cylinderZMin
        };
        float toolMax[3] = {
            toolCenterX + halfSize + stickout * std::cos(angleRad),
            toolCenterY + halfSize + stickout * std::sin(angleRad),
            fromChuck[Z_AXIS] + _cylinderZMax + _squareToolSize  // Z depth of tool
        };

        // Ensure min < max
        if (toolMin[0] > toolMax[0]) std::swap(toolMin[0], toolMax[0]);
        if (toolMin[1] > toolMax[1]) std::swap(toolMin[1], toolMax[1]);
        if (toolMin[2] > toolMax[2]) std::swap(toolMin[2], toolMax[2]);

        return lineIntersectsAABB(fromChuck, toChuck, toolMin, toolMax);
    }

    bool PneumaticToolTurret::checkBoringToolCollision(int toolIndex, const float* fromChuck, const float* toChuck) const {
        if (toolIndex < 0 || toolIndex >= 8) return false;

        // Check if this boring position has a tool loaded
        // Boring tool numbers are typically offset from square tools
        // Tool number for boring position i would be 8 + i + 1 = 9 to 16
        int toolNumber = 8 + toolIndex + 1;

        float tlo[MAX_N_AXIS] = {};
        bool hasToolLoaded = (toolTable != nullptr && toolTable->getToolOffset(toolNumber, tlo));

        // If no tool loaded in this position, no collision possible
        if (!hasToolLoaded) {
            return false;
        }

        // Get tool angle and calculate position
        float angleDeg = BORING_TOOL_ANGLES[toolIndex];
        float angleRad = angleDeg * float(M_PI) / 180.0f;

        // Tool holder position relative to cylinder center
        float holderOffsetX = _boringToolRadiusFromCenter * std::cos(angleRad);
        float holderOffsetY = _boringToolRadiusFromCenter * std::sin(angleRad);

        // Tool center in chuck space
        float toolCenterX = fromChuck[X_AXIS] + _cylinderCenterX + holderOffsetX;
        float toolCenterY = fromChuck[Y_AXIS] + _cylinderCenterY + holderOffsetY;
        float toolRadius = _boringToolDiameter / 2.0f;

        // Tool stickout (Z direction for boring tools)
        float stickout = std::abs(tlo[Z_AXIS]);
        if (stickout <= 0) stickout = 50.0f;  // Default if not set

        // Z range of the boring tool (sticks out in Z direction)
        float toolZMin = fromChuck[Z_AXIS] + _cylinderZMin;
        float toolZMax = fromChuck[Z_AXIS] + _cylinderZMax + stickout;

        // Check circle intersection in XY plane
        if (lineIntersectsCircle2D(fromChuck[X_AXIS], fromChuck[Y_AXIS],
                                   toChuck[X_AXIS], toChuck[Y_AXIS],
                                   toolCenterX, toolCenterY, toolRadius)) {
            float segZMin = std::min(fromChuck[Z_AXIS], toChuck[Z_AXIS]);
            float segZMax = std::max(fromChuck[Z_AXIS], toChuck[Z_AXIS]);
            if (zRangesOverlap(segZMin, segZMax, toolZMin, toolZMax)) {
                return true;
            }
        }
        return false;
    }

    bool PneumaticToolTurret::checkChuckCollision(const float* fromChuck, const float* toChuck) const {
        if (!_chuckCollisionEnabled) return false;

        // Chuck is fixed at the centerline, check if ATC volumes would collide with it
        // Check circle intersection in XY plane
        if (lineIntersectsCircle2D(fromChuck[X_AXIS], fromChuck[Y_AXIS],
                                   toChuck[X_AXIS], toChuck[Y_AXIS],
                                   _chuckCenterX, _chuckCenterY, _chuckRadius)) {
            float segZMin = std::min(fromChuck[Z_AXIS], toChuck[Z_AXIS]);
            float segZMax = std::max(fromChuck[Z_AXIS], toChuck[Z_AXIS]);
            if (zRangesOverlap(segZMin, segZMax, _chuckZMin, _chuckZMax)) {
                return true;
            }
        }
        return false;
    }

    // ============== Main TestLimit Implementation ==============

    bool PneumaticToolTurret::TestLimit(const float* from, const float* to) {
        // Legacy check: maximum Z retract position
        if (maxRetractZ != 0) {
            if (to[Z_AXIS] > maxRetractZ || from[Z_AXIS] > maxRetractZ) {
                log_debug("ToolTurret: Z position " << to[Z_AXIS] << " exceeds maxRetractZ " << maxRetractZ);
                return true;
            }
        }

        // Transform to chuck centerline space
        float fromChuck[3], toChuck[3];
        transformToChuckSpace(from, fromChuck);
        transformToChuckSpace(to, toChuck);

        // Check ATC body box collision with chuck
        // This checks if the ATC body would crash into the chuck
        if (_chuckCollisionEnabled) {
            // For ATC body vs chuck, we check if the motion brings any ATC part
            // into the chuck's space. We approximate by checking if the line
            // (representing the cutting tool path) enters the chuck volume.
            if (checkChuckCollision(fromChuck, toChuck)) {
                log_debug("ToolTurret: Motion would collide with chuck");
                return true;
            }
        }

        // Check all 8 square tool positions for collision with chuck
        for (int i = 0; i < 8; ++i) {
            // Build the tool's bounding volume and check against chuck
            if (_chuckCollisionEnabled) {
                // Get tool position
                float angleDeg = SQUARE_TOOL_ANGLES[i];
                float angleRad = angleDeg * float(M_PI) / 180.0f;
                float stickout = getToolStickout(i);

                // Tool tip position (furthest point from cylinder center)
                float toolTipX = fromChuck[X_AXIS] + _cylinderCenterX +
                                (_squareToolRadiusFromCenter + stickout) * std::cos(angleRad);
                float toolTipY = fromChuck[Y_AXIS] + _cylinderCenterY +
                                (_squareToolRadiusFromCenter + stickout) * std::sin(angleRad);

                // Check if tool tip circle overlaps with chuck circle
                float toolEffectiveRadius = _squareToolSize / 2.0f;
                if (circlesOverlapXY(toolTipX, toolTipY, toolEffectiveRadius,
                                     _chuckCenterX, _chuckCenterY, _chuckRadius)) {
                    // Check Z overlap
                    float toolZMin = fromChuck[Z_AXIS] + _cylinderZMin;
                    float toolZMax = fromChuck[Z_AXIS] + _cylinderZMax + _squareToolSize;
                    if (zRangesOverlap(toolZMin, toolZMax, _chuckZMin, _chuckZMax)) {
                        log_debug("ToolTurret: Square tool " << i << " would collide with chuck");
                        return true;
                    }
                }
            }
        }

        // Check all 8 boring tool positions for collision with chuck
        for (int i = 0; i < 8; ++i) {
            if (!_chuckCollisionEnabled) continue;

            // Check if this boring position has a tool loaded
            int toolNumber = 8 + i + 1;  // Boring tools are numbered 9-16
            float tlo[MAX_N_AXIS] = {};
            bool hasToolLoaded = (toolTable != nullptr && toolTable->getToolOffset(toolNumber, tlo));
            if (!hasToolLoaded) continue;

            // Get tool position
            float angleDeg = BORING_TOOL_ANGLES[i];
            float angleRad = angleDeg * float(M_PI) / 180.0f;
            float stickout = std::abs(tlo[Z_AXIS]);
            if (stickout <= 0) stickout = 50.0f;

            // Boring tool holder position
            float holderX = fromChuck[X_AXIS] + _cylinderCenterX +
                           _boringToolRadiusFromCenter * std::cos(angleRad);
            float holderY = fromChuck[Y_AXIS] + _cylinderCenterY +
                           _boringToolRadiusFromCenter * std::sin(angleRad);
            float toolRadius = _boringToolDiameter / 2.0f;

            // Check if tool cylinder overlaps with chuck cylinder
            if (circlesOverlapXY(holderX, holderY, toolRadius,
                                 _chuckCenterX, _chuckCenterY, _chuckRadius)) {
                float toolZMin = fromChuck[Z_AXIS] + _cylinderZMin;
                float toolZMax = fromChuck[Z_AXIS] + _cylinderZMax + stickout;
                if (zRangesOverlap(toolZMin, toolZMax, _chuckZMin, _chuckZMax)) {
                    log_debug("ToolTurret: Boring tool " << i << " would collide with chuck");
                    return true;
                }
            }
        }

        // Check tool cylinder body collision with chuck
        if (_chuckCollisionEnabled) {
            float cylCenterX = fromChuck[X_AXIS] + _cylinderCenterX;
            float cylCenterY = fromChuck[Y_AXIS] + _cylinderCenterY;
            if (circlesOverlapXY(cylCenterX, cylCenterY, _cylinderRadius,
                                 _chuckCenterX, _chuckCenterY, _chuckRadius)) {
                float cylZMin = fromChuck[Z_AXIS] + _cylinderZMin;
                float cylZMax = fromChuck[Z_AXIS] + _cylinderZMax;
                if (zRangesOverlap(cylZMin, cylZMax, _chuckZMin, _chuckZMax)) {
                    log_debug("ToolTurret: Tool cylinder would collide with chuck");
                    return true;
                }
            }
        }

        // TODO: Check against tailstock (if configured)
        // This would be similar to chuck checking but using tailstock geometry

        return false;  // No collision detected
    }

    namespace {
        ATCFactory::InstanceBuilder<PneumaticToolTurret> registration("pneumatic_tool_turret");
    }
}
