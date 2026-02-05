// Mock implementations for BasePlanner and TrapezoidPlanner dependencies
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Types.h"
#include "Logging.h"
#include "EnumItem.h"
#include "System.h"
#include "Channel.h"
#include "Protocol.h"
#include "Stepper.h"
#include "NutsBolts.h"

#include <cmath>
#include <string>

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

// Axes static
axis_t Machine::Axes::_numberAxis = axis_t::MAX_N_AXIS;

// axis_t increment operator
axis_t operator++(axis_t& a, int) {
    axis_t orig = a;
    a = static_cast<axis_t>(static_cast<int>(a) + 1);
    return orig;
}

axis_t& operator++(axis_t& a) {
    a = static_cast<axis_t>(static_cast<int>(a) + 1);
    return a;
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

LogStream::LogStream(Channel& channel, MsgLevel level) : _channel(channel), _level(level), _line(new std::string()) {}
LogStream::LogStream(Channel& channel, const char* name) : _channel(channel), _level(MsgLevelNone), _line(new std::string()) {}
LogStream::LogStream(Channel& channel, MsgLevel level, const char* name) : _channel(channel), _level(level), _line(new std::string()) {}
LogStream::LogStream(MsgLevel level, const char* name) : _channel(*static_cast<Channel*>(nullptr)), _level(level), _line(new std::string()) {}
size_t LogStream::write(uint8_t c) {
    if (_line) {
        _line->push_back(static_cast<char>(c));
    }
    return 1;
}
LogStream::~LogStream() {
    delete _line;
}

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

float steps_to_motor_pos(int axis, uint64_t steps_per_unit) {
    if (steps_per_unit == 0) return 0.0f;
    return static_cast<float>(mock_steps[axis]) / static_cast<float>(steps_per_unit);
}

int motor_pos_to_steps(float pos, uint64_t steps_per_unit) {
    return static_cast<int>(pos * static_cast<float>(steps_per_unit));
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
    
    std::string Axes::maskToNames(uint32_t mask) {
        std::string result;
        if (mask & 0x01) result += "X";
        if (mask & 0x02) result += "Y";
        if (mask & 0x04) result += "Z";
        if (mask & 0x08) result += "A";
        if (mask & 0x10) result += "B";
        if (mask & 0x20) result += "C";
        return result;
    }
}
