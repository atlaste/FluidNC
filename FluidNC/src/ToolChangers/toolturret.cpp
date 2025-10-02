#pragma once

#ifdef TOOL_TURRET
#    include <algorithm>

class PneumaticToolTurret {
    // Configuration things:
    std::vector<float> toolOffsets;
    int32_t            offsetsPerRevolution = 360;  // we assume 360 values per revolution. That should be plenty
    char               toolChangeAxis       = 'C';  // could be a string I suppose
    Pin                pneumaticAction;
    Pin                pneumaticEndstop;
    std::string        toolProbeDirections;
    std::vector<float> probeMaxTravel;
    std::vector<float> probePosition;
    int32_t            probeSeekRate = 200;
    int32_t            probeFeedRate = 80;

    // Persisted things:
    std::vector<float> toolLengthOffsets;
    int32_t            currentToolNumber = 0;

    void loadToolNVS();  // load tool length offsets and current tool number from NVS or Flash or whatever
    void saveToolNVS();  // save tool length offsets and current tool number in NVS or Flash or whatever

    void run(const char* str);  // execute g-code, wait until it's done. Should be "macro.addf"

    void setToolChangeStepperEnable(bool enabled) {
        const char* axis = "XYZABC";
        auto        idx  = strchr(axis, std::toupper(toolChangeAxis));
        Assert(idx != NULL, "Incorrect tool change axis");

        machine->axes[idx - axis].enable(enabled);
    }

public:
    void init() {
        // load current tool number from NVS:
        loadToolNVS();

        // disable the tool change stepper:
        setToolChangeStepperEnable(false);

        // Set the correct tool length offset:
        char setTLO[50];
        snprintf(setTLO, 100, "G43.1 X%0.4f\n", toolLengthOffsets[currentToolNumber]);
        run(setTLO);
    }

    bool changeTool(int toolNumber) {
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

        Assert(pneumaticEndstop.read(), "Pneumatic endstop is not active. Tool is not engaged.");

        bool was_inch_mode = (gc_state.modal.units == Units::Inches);
        if (was_inch_mode) {
            run("G21\n");
        }

        run("#<start_x >= #<_x>\n");
        run("#<start_y >= #<_y>\n");
        run("#<start_z >= #<_z>\n");

        run("G53 G0 X0\n");  // TODO: Safe_x and Safe_z ?
        run("G53 G0 Z0\n");

        // Start tool change
        pneumaticAction.on();

        // wait for pneumatic actuator
        for (int i = 0; i < 50 && pneumaticEndstop.read(); ++i) {
            Timer::delayMillis(10);
        }
        Assert(!pneumaticEndstop.read(), "Pneumatic endstop is active. Cannot change tool.");

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
        pneumaticAction.off();

        // Wait till the endstop is active again
        for (int i = 0; i < 50 && !pneumaticEndstop.read(); ++i) {
            Timer::delayMillis(10);
        }
        Assert(pneumaticEndstop.read(), "Pneumatic endstop is still active. Tool change failed!");

        // And disable the stepper again. Otherwise it's just going to fight things.
        setToolChangeStepperEnable(false);

        // Set TLO:
        snprintf(toolChange, 100, "G43.1 X%0.4f\n", toolLengthOffsets[toolNumber]);
        run(toolChange);

        // return to location before the tool change
        run("G0Z#<start_z>");
        run("G0X#<start_x>");

        // restore inch mode
        if (was_inch_mode) {
            run("G20\n");
        }

        // Save the tool number.
        currentToolNumber = toolNumber;
        saveToolNVS();

        return true;
    }

    void setTool(int32_t toolNumber) {
        currentToolNumber = toolNumber;
        saveToolNVS();
    }

    void probeToolLengthOffset() {
        bool was_inch_mode = (gc_state.modal.units == Units::Inches);
        if (was_inch_mode) {
            run("G21\n");
        }

        run("#<start_x >= #<_x>\n");
        run("#<start_y >= #<_y>\n");
        run("#<start_z >= #<_z>\n");

        run("G53 G0 X0\n");  // TODO: Safe_x and Safe_z ?
        run("G53 G0 Z0\n");

        char direction = toolProbeDirections[currentToolNumber];

        const char* axis = "XYZABC";
        auto        idx  = strchr(axis, std::toupper(direction));
        Assert(idx != NULL, "Incorrect tool change axis");
        auto maxTravel = probeMaxTravel[idx - axis];

        // TODO: Perhaps move to some position before probing...
        char buf[100];
        snprintf(buf, 100, "G53 G38.2 %c%0.2f\n", direction, maxTravel);  // rapid down
        run(buf);

        // do a fast probe if there is a seek that is faster than feed
        if (probeSeekRate > probeFeedRate) {
            snprintf(buf, 100, "G53 G38.2 %c%0.3f F%0.3f\n", direction, maxTravel, probeSeekRate);
            run(buf);
            run("G0Z[#<_z> + 5]");  // retract befor next probe

            maxTravel = 7;  // 7mm is plenty now.
        }

        // do the feed rate probe
        snprintf(buf, 100, "G53 G38.2 %c%0.3f F%0.3f\n", direction, maxTravel, probeFeedRate);

        // get the position:
        float probe_position[MAX_N_AXIS];
        motor_steps_to_mpos(probe_position, probe_steps);
        toolLengthOffsets[currentToolNumber] = probe_position[idx - axis] + probePosition[idx - axis];

        // Set TLO:
        snprintf(toolChange, 100, "G43.1 X%0.4f\n", toolLengthOffsets[toolNumber]);
        run(toolChange);

        // return to location before the tool change
        run("G0Z#<start_z>");
        run("G0X#<start_x>");

        // restore inch mode
        if (was_inch_mode) {
            run("G20\n");
        }
    }

    // // ATC API:
    // void probe_notification() override {}
    // bool tool_change(uint8_t value, bool pre_select, bool set_tool) override
    // {
    //     if (pre_select)
    //     {
    //         // TODO FIXME: Is this ever used?
    //
    //         setTool(value);
    //         return true;
    //     }
    //
    //     if (!changeTool(value)) // TODO FIXME: If set_tool, don't return to start!
    //     {
    //         return false;
    //     }
    //
    //     if (set_tool)
    //     {
    //         if (toolLengthOffsets[currentToolNumber] <= 0.0)
    //         {
    //             // Quick check tool length offset that's persisted. If it's there -> use it.
    //             probeToolLengthOffset();
    //         }
    //     }
    //
    //     return true;
    // }
};
#endif
