#include "PneumaticToolTurret.h"

#include <algorithm>
#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include "Pin.h"
#include "Protocol.h"
#include "System.h"
#include "GCode.h"
#include "Machine/MachineConfig.h"
#include "NutsBolts.h"
#include "atc.h"
#include "../State/StatePersistence.h"
#include "../Logging.h"
#include "../ToolTable.h"
#include "../SoftLimits/LimitsChecker.h"

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

extern parser_block_t gc_block;

namespace ATCs {
    void PneumaticToolTurret::run(const char* str) {
        log_info("ATC command: " << str);

        if (sys.abort() || sys.state() == State::Alarm) {
            throw std::runtime_error("System in alarm/abort before ATC command");
        }

        // gc_execute_line overwrites the global gc_block, so save/restore it
        // since we're called re-entrantly from within M6 processing.
        parser_block_t saved_block = gc_block;
        auto           err         = gc_execute_line(str);
        gc_block                   = saved_block;

        if (err != Error::Ok) {
            auto msg = std::string("ATC command failed (error ") + std::to_string(int(err)) + "): " + str;
            log_error(msg);
            throw std::runtime_error(msg);
        }

        protocol_buffer_synchronize();

        if (sys.abort() || sys.state() == State::Alarm) {
            throw std::runtime_error(std::string("Alarm/abort during ATC command: ") + str);
        }
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
        // setToolChangeStepperEnable(false);

        log_info("Current tool number: " << int(currentToolNumber));

        // Set the correct tool length offset from tool table:
        if (currentToolNumber > 0) {
            // Use G43 H# to load TLO from tool table
            char setTLO[32];
            snprintf(setTLO, sizeof(setTLO), "G43 H%d", int(currentToolNumber));
            run(setTLO);
        }
    }

    void PneumaticToolTurret::group(Configuration::HandlerBase& handler) {
        handler.item("offsets_per_revolution", offsetsPerRevolution);
        handler.item("tool_offsets", toolOffsets);
        handler.item("use_pressure_sensor", usePressureSensor);

        // Tool types and safe retract:
        handler.item("tool_types", toolTypes);        // 'I' = inside (boring), 'O' = outside (turning)
        handler.item("safe_x", safeX);                // Safe X position (machine coords)
        handler.item("safe_y", safeY);                // Safe Y position (machine coords)
        handler.item("safe_z", safeZ);                // Safe Z position (machine coords)
        handler.item("safety_margin", safetyMargin);  // Extra clearance in mm
    }

    void PneumaticToolTurret::validate() {
        Assert(toolOffsets.size() > 0, "No tool offsets are configured");

        // If toolTypes is specified, it should match the number of tools
        if (toolTypes.size() > 0) {
            Assert(toolTypes.size() == toolOffsets.size(), "Tool types length should match the tool offsets vector length");
            // Validate that all characters are 'I' or 'O'
            for (char c : toolTypes) {
                Assert(c == 'I' || c == 'O' || c == 'i' || c == 'o', "Tool types must be 'I' (inside) or 'O' (outside)");
            }
        }
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

        // There's a bit of an odd issue here. Tool number #0 is reserved for 'no tool'. So we need to
        // take this into account when accessing the tool offsets and types.

        try {
            protocol_buffer_synchronize();  // wait for all motion to complete

            if (toolNumber <= 0 || toolNumber > int(toolOffsets.size())) {
                log_info("Attempting to select an invalid tool.");
                return false;
            }

            if (currentToolNumber == toolNumber) {
                return true;
            }

            log_info("Starting pneumatic tool change");

            // First thing we're going to do here is enable the stepper motor for the tool changer:
            // setToolChangeStepperEnable(true);

            //Assert(pneumaticEndstop.read(), "Pneumatic endstop is not active. Tool is not engaged.");

            // Save modal states that we might change during the tool change sequence.
            // gc_execute_line() modifies gc_state, so we need to restore these after.
            bool was_inch_mode        = (gc_state.modal.units == Units::Inches);
            bool was_incremental_mode = (gc_state.modal.distance == Distance::Incremental);
            bool was_css_mode         = (gc_state.modal.spindle_speed_mode == SpindleSpeedMode::ConstantSurfaceSpeed);
            bool was_feed_per_rev     = (gc_state.modal.feed_rate == FeedRate::UnitsPerRev);
            bool was_diameter_mode    = (gc_state.modal.lathe_diameter_mode == LatheDiameterMode::Diameter);

            // Switch to safe modal states for tool change operations:
            // - G21 (mm mode) for consistent units
            // - G90 (absolute mode) so our coordinates are interpreted correctly
            // - G97 (constant RPM) to avoid CSS complications
            // - G94 (feed per minute) for consistent feed rates
            // - G8 (radius mode) so X coordinates are not doubled
            if (was_inch_mode) {
                run("G21");
            }
            if (was_incremental_mode) {
                run("G90");
            }
            if (was_css_mode) {
                run("G97");
            }
            if (was_feed_per_rev) {
                run("G94");
            }
            if (was_diameter_mode) {
                run("G8");
            }

            // run("#<start_x >= #<_x>");
            // run("#<start_y >= #<_y>");
            // run("#<start_z >= #<_z>");

            // Determine if current tool is inside (boring/drilling) or outside (turning/facing)
            bool isInsideTool = false;
            if (toolTypes.size() + 1 > currentToolNumber) {
                char toolType = std::toupper(toolTypes[currentToolNumber - 1]);
                isInsideTool  = (toolType == 'I' || toolType == 'i');
            } else {
                Assert(false, "Tool type not found for tool number %d. Cannot retract safely.", currentToolNumber);
            }

            log_info("Current tool is " << (isInsideTool ? "" : "not ") << "an inside tool.");

            // Safe retract sequence depends on tool type:
            // - Inside tools (boring): Z first (out of hole, don't crash into tailstock!), then X, then more Z.
            // - Outside tools (turning): X first (away from OD), then Z.
            char safeRetract[100];
            if (isInsideTool) {
                // Get MPOS.
                // If 0 <= Mpos_Z <= -72, we're not going to do any Z retraction.
                // If 0 <= Mpos_X <= safeX, we're safe to proceed. If not, let's error out like we do here.

                // TODO FIXME: Make all the constants (-72.0f) configurable.
                float* mpos   = get_mpos();
                float  mpos_z = mpos[Z_AXIS];
                float  mpos_x = mpos[X_AXIS];

                bool z_is_safe = (mpos_z >= -72.0f);  // already cleared because a possible toolstock collision is impossible.
                bool x_is_safe =
                    (mpos_x >= safeX);  // already retracted in the x direction. Only possible if we're at the safe position already.

                if (!z_is_safe || !x_is_safe) {
                    // Inside tool: First retract Z (out of the bore), then X
                    // Use TLO + safety margin if available, otherwise use configured safeZ
                    float tlo[MAX_N_AXIS] = {};
                    bool  hasTLO          = toolTable != nullptr && toolTable->getToolOffset(currentToolNumber, tlo);
                    if (hasTLO) {
                        log_info("Retracting boring tool (Z)");

                        // Calculate safe Z position based on TLO + margin
                        // This ensures we clear the bore before moving X
                        float safeRetractLength = tlo[Z_AXIS] + safetyMargin;
                        if (safeRetractLength > -72.0f) {
                            snprintf(safeRetract, 100, "G53 G0 Z%0.4f", safeRetractLength);
                            run(safeRetract);
                        } else {
                            log_error("Z retraction would be too low. Cannot retract safely.");
                            return false;
                        }
                    } else {
                        log_error("TLO not found for tool number " << currentToolNumber << ". Cannot retract safely.");
                        return false;
                    }
                }
            }

            // fallthrough:
            //
            // It's fine to always do this; if we're already at the safe location it's an implicit no-op.
            {
                log_info("Retracting turret (X)");

                // Inside & outside tool: First retract X (away from workpiece OD), then Z
                snprintf(safeRetract, 100, "G53 G0 X%0.4f", safeX);
                run(safeRetract);

                log_info("Going to change position (Z)");

                snprintf(safeRetract, 100, "G53 G0 Z%0.4f", safeZ);
                run(safeRetract);
            }

            // Before doing the tool change, we need to check if the pressure is on:
            if (usePressureSensor) {
                bool waited = false;
                auto bar = pneumaticSensor.readBar();
                log_info("Pressure of tool changer: " << bar << " bar.");
                bool isWaiting = false;

                while (pneumaticSensor.readBar() < 2.7f) {
                    log_info("Cannot do pneumatic action; pressure is not enough. We need 2.7 bar, read: " << pneumaticSensor.readBar()
                                                                                                           << " bar.");
                    waited = true;
                    for (int i = 0; i < 40 && pneumaticSensor.readBar() < 2.7f && sys.state() != State::Alarm; ++i) {
                        delay_ms(50);
                        protocol_buffer_synchronize();
                    }
                }

                if (isWaiting) {
                    // If we had to wait, we'll just wait an additional 2 seconds just to be sure everything is fine.
                    delay_ms(2000);
                }

                if (waited) {
                    // Wait a bit just to be sure we have enough pressure. Let's just take ~2 seconds to be sure.
                    for (int i = 0; i < 20; ++i) {
                        protocol_buffer_synchronize();  // Wait for all motion to complete
                        delay_ms(100);                  // Wait for pneumatic action to complete
                    }
                }
            }

            if (sys.state() == State::Alarm) {
                return false;
            }

            // Start tool change
            run(pneumaticActionOn);

            // wait for pneumatic actuator, this takes ~2 seconds before the motion completely stops.
            for (int i = 0; i < 20; ++i) {
                protocol_buffer_synchronize();  // Wait for all motion to complete
                delay_ms(100);                  // Wait for pneumatic action to complete
            }

            // Assert(pneumaticEndstop.read(), "Pneumatic endstop is active. Cannot change tool.");
            // for (int i = 0; i < 50 && pneumaticEndstop.read(); ++i) {
            //     Timer::delayMillis(10);
            // }
            // Assert(!pneumaticEndstop.read(), "Pneumatic endstop is active. Cannot change tool.");

            log_info("Changing tool");

            // Should we move forward or backward?
            auto offset1 = currentToolNumber == 0 ? 0 : toolOffsets[currentToolNumber - 1];
            auto offset2 = toolNumber == 0 ? 0 : toolOffsets[toolNumber - 1];

            // Calculate how much we have to move.
            // Always move in the same direction to ensure we don't have to deal with backlash.
            auto diff = offset2 - offset1;
            if (diff < 0) {
                diff = offsetsPerRevolution + diff;
            }

            // Do the tool change.
            char toolChange[100];
            snprintf(toolChange, 100, "G92 %c0", toolChangeAxis);
            run(toolChange);

            // Tool changers like these have massive backlash. We just assume that backlash here, 
            // instead of being stupid about it.
            snprintf(toolChange, 100, "G0 %c%0.3f", toolChangeAxis, (diff + 0.5f));
            run(toolChange);
            snprintf(toolChange, 100, "G0 %c%0.3f", toolChangeAxis, (diff - 0.2f));
            run(toolChange);

            protocol_buffer_synchronize();  // wait for all motion to complete
            delay_ms(500);                  // Give everything a bit of time to settle.

            run(pneumaticActionOff);
            protocol_buffer_synchronize();  // wait for all motion to complete
            delay_ms(700);                  // Wait for pneumatic action to complete

            snprintf(toolChange, 100, "G0 %c%0.3f", toolChangeAxis, diff);
            run(toolChange);

            // Wait till the endstop is active again
            // for (int i = 0; i < 50 && !pneumaticEndstop.read(); ++i) {
            //     delay_ms(10);
            // }
            // Assert(pneumaticEndstop.read(), "Pneumatic endstop is still active. Tool change failed!");

            // And disable the stepper again. Otherwise it's just going to fight the coupling.
            // setToolChangeStepperEnable(false);

            log_info("Setting TLO");

            // Load TLO from tool table using G43 H#
            snprintf(toolChange, sizeof(toolChange), "G43 H%d", toolNumber);
            run(toolChange);

            // DO NOT return to location before the tool change. Because you don't know the tool geometry, it
            // might crash the machine!!!
            //
            // CAM needs to handle the approach after a tool change in lathes!

            // run("G0Z#<start_z>");
            // run("G0X#<start_x>");

            // restore inch mode
            if (was_inch_mode) {
                run("G20");
            }

            // Restore other modal states that were changed
            if (was_incremental_mode) {
                run("G91");
            }
            if (was_css_mode) {
                run("G96");
            }
            if (was_feed_per_rev) {
                run("G95");
            }
            if (was_diameter_mode) {
                run("G7");
            }

            // Save the tool number.
            currentToolNumber = toolNumber;

            return true;
        } catch (const std::exception& e) {
            log_error("Tool change aborted: " << e.what());
            setToolChangeStepperEnable(false);
            return false;
        }
    }

    void PneumaticToolTurret::setTool(int32_t toolNumber) {
        currentToolNumber = toolNumber;
    }

    // ATC API:
    void PneumaticToolTurret::probe_notification() {}
    bool PneumaticToolTurret::tool_change(tool_t value, bool pre_select, bool set_tool) {
        if (int(value) <= 0 || int(value) > int(toolOffsets.size())) {
            log_info("Attempting to select an invalid tool.");
            return false;
        }

        if (pre_select) {
            // TODO FIXME: Is this ever used?
            return true;
        }

        if (currentToolNumber == value) {
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

    void PneumaticToolTurret::save_atc_data(std::vector<uint8_t>& buffer) {
        ATC::save_atc_data(buffer);
        auto offset = buffer.size();
        buffer.resize(offset + sizeof(currentToolNumber));
        memcpy(buffer.data() + offset, &currentToolNumber, sizeof(currentToolNumber));
    }

    void PneumaticToolTurret::restore_atc_data(const std::vector<uint8_t>& buffer, size_t& index) {
        ATC::restore_atc_data(buffer, index);
        if (index + sizeof(currentToolNumber) <= buffer.size()) {
            memcpy(&currentToolNumber, buffer.data() + index, sizeof(currentToolNumber));
            index += sizeof(currentToolNumber);
            log_info("Restored ATC tool number: " << currentToolNumber);
        }
    }

    namespace {
        ATCFactory::InstanceBuilder<PneumaticToolTurret> registration("pneumatic_tool_turret");
    }
}
