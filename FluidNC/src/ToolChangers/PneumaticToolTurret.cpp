#include "PneumaticToolTurret.h"

#include <algorithm>
#include <vector>
#include <string>
#include <cstring>
#include "Pin.h"
#include "Protocol.h"
#include "System.h"
#include "Machine/MachineConfig.h"
#include "NutsBolts.h"
#include "atc.h"
#include "../State/StatePersistence.h"
#include "../Logging.h"

namespace ATCs {
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

        // TODO FIXME: I'm honstely not sure if the currentToolNumber is setup at this point. Let's log it for now.
        log_info("Current tool number: " << int(currentToolNumber));

        // Set the correct tool length offset:
        if (useTLO) {
            bool isInsideTool = false;
            if (toolTypes.size() > currentToolNumber) {
                char toolType = std::toupper(toolTypes[currentToolNumber]);
                isInsideTool = (toolType == 'I');
            }
            else {
                Assert(false, "Tool type not found for tool number %d. Cannot retract safely.", currentToolNumber);
            }
            char directionAxis = isInsideTool ? 'Z' : 'X';
    
            // Set TLO
            char setTLO[50];
            // TODO FIXME: Check syntax! I'm not sure if this %c is correct, or if we should K/I offsets.
            snprintf(setTLO, 100, "G43.1 %c%0.4f\n", directionAxis, toolLengthOffsets[currentToolNumber]);
            run(setTLO);
        }
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
            if (useTLO && toolLengthOffsets.size() > currentToolNumber) {
                // Calculate safe Z position based on current Z + TLO + margin
                // This ensures we clear the bore before moving X
                auto safeRetractLength = toolLengthOffsets[currentToolNumber] + safetyMargin; // e.g. -50. Max is -say- -40. 
                if (safeRetractLength > maxRetractZ) {
                    safeRetractLength = maxRetractZ;
                    log_info("Safe retract Z is above maxRetractZ. Aborting.");
                    mc_critical(ExecAlarm::HardLimit);

                    return false;
                }
                snprintf(safeRetract, 100, "G53 G0 Z%0.4f\n", safeRetractLength);
                run(safeRetract);
            } else {
                Assert(false, "TLO not found for tool number %d. Cannot retract safely.", currentToolNumber);
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
            // Quick check tool length offset that's persisted. If it's there -> use it.
            // Otherwise run a probe sequence.
            // if (toolLengthOffsets[currentToolNumber] <= 0.0) {
            //     probeToolLengthOffset();
            // }

            // Set TLO:
            snprintf(toolChange, 100, "G43.1 X%0.4f\n", toolLengthOffsets[toolNumber]);
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

    namespace {
        ATCFactory::InstanceBuilder<PneumaticToolTurret> registration("pneumatic_tool_turret");
    }
}
