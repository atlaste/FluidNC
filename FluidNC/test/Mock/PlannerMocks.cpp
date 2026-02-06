// Mock implementations for BasePlanner and TrapezoidPlanner dependencies
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Types.h"
#include "Mock/TestLogging.h"
#include "EnumItem.h"
#include "System.h"
#include "Channel.h"
#include "Protocol.h"
#include "Stepper.h"
#include "NutsBolts.h"

#include <cmath>
#include <string>
#include <string_view>

// Include MachineConfig and related headers
#include "Machine/MachineConfig.h"

// Static MachineConfig for testing - initialize with test values
namespace {
    Machine::MachineConfig testConfig;
    
    struct ConfigInitializer {
        ConfigInitializer() {
            testConfig._planner_blocks = 16;  // Match TEST_PLANNER_BLOCKS
            testConfig._junctionDeviation = 0.01f;
            testConfig._arcTolerance = 0.002f;
        }
    };
    static ConfigInitializer configInit;
}

// Global config pointer
Machine::MachineConfig* config = &testConfig;

// MachineConfig virtual methods (needed for static initialization)
Machine::MachineConfig::~MachineConfig() {}
void Machine::MachineConfig::group(Configuration::HandlerBase& handler) {}
void Machine::MachineConfig::afterParse() {}

// Global system state
system_t sys;

// Initialize sys with proper defaults for testing
namespace {
    struct SysInitializer {
        SysInitializer() {
            sys.set_f_override(100);  // 100% feed override
            sys.set_r_override(100);  // 100% rapids override
            sys.set_spindle_speed_ovr(100);  // 100% spindle override
        }
    };
    static SysInitializer sysInit;
}

// Logging mocks
static MsgLevel currentMsgLevel = MsgLevelInfo;

bool atMsgLevel(MsgLevel level) {
    return level <= currentMsgLevel;
}

void set_state(State s) {
    sys.set_state(s);
}

bool state_is(State s) {
    return sys.state() == s;
}

// No-op LogStream stub for test build (src/ code still expands macros to LogStream).
// Destructor intentionally does not call _channel.sendLine(); _channel is never used.
LogStream::LogStream(Channel& channel, MsgLevel level) : _channel(channel), _level(level), _line(nullptr) {}
LogStream::LogStream(Channel& channel, const char* name) : _channel(channel), _level(MsgLevelNone), _line(nullptr) {}
LogStream::LogStream(Channel& channel, MsgLevel level, const char* name) : _channel(channel), _level(level), _line(nullptr) {}
LogStream::LogStream(MsgLevel level, const char* name) : _channel(*static_cast<Channel*>(nullptr)), _level(level), _line(nullptr) {}
size_t LogStream::write(uint8_t) { return 1; }
LogStream::~LogStream() {}

const EnumItem messageLevels2[] = {
    { MsgLevelNone, "None" },
    { MsgLevelError, "Error" },
    { MsgLevelWarning, "Warning" },
    { MsgLevelInfo, "Info" },
    { MsgLevelDebug, "Debug" },
    { MsgLevelVerbose, "Verbose" },
    EnumItem(0)  // Terminator
};

// NutsBolts mocks
float convert_delta_vector_to_unit_vector(float* vector) {
    // Calculate magnitude and normalize
    float magnitude = 0.0f;
    for (int i = 0; i < static_cast<int>(axis_t::MAX_N_AXIS); i++) {
        magnitude += vector[i] * vector[i];
    }
    magnitude = sqrtf(magnitude);
    if (magnitude > 0.0f) {
        float inv_mag = 1.0f / magnitude;
        for (int i = 0; i < static_cast<int>(axis_t::MAX_N_AXIS); i++) {
            vector[i] *= inv_mag;
        }
    }
    return magnitude;
}

float limit_acceleration_by_axis_maximum(float* unit_vec) {
    // Return a default value for testing
    return 60000.0f;  // 60 mm/s^2 typical
}

float limit_rate_by_axis_maximum(float* unit_vec) {
    // Return a default value for testing
    return 6000.0f;  // 6000 mm/min typical
}

// Protocol mocks
void send_alarm(ExecAlarm alarm) {
    sys.set_state(State::Alarm);
}

// Stepper mocks
static int32_t mock_steps[static_cast<int>(axis_t::MAX_N_AXIS)] = {0};

float steps_to_motor_pos(steps_t steps, size_t motor) {
    return static_cast<float>(steps);
}

steps_t motor_pos_to_steps(float pos, size_t motor) {
    return static_cast<steps_t>(pos);
}

void get_steps(int* steps) {
    for (int i = 0; i < static_cast<int>(axis_t::MAX_N_AXIS); i++) {
        steps[i] = mock_steps[i];
    }
}

namespace Stepper {
    bool update_plan_block_parameters() {
        // Mock - just return success
        return true;
    }
}

// Machine::Homing mocks
namespace Machine {
    uint32_t Homing::unhomed_axes() {
        // Mock - return 0 (all axes homed)
        return 0;
    }
}
