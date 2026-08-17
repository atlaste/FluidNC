// Copyright (c) 2020 Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Settings.h"

#include "Machine/MachineConfig.h"
#include "Configuration/RuntimeSetting.h"
#include "Configuration/AfterParse.h"
#include "Configuration/Validator.h"
#include "Machine/Axes.h"
#include "Regexpr.h"
#include "WebUI/Authentication.h"
#include "Report.h"
#include "MotionControl.h"
#include "System.h"
#include "Limit.h"                // homingAxes
#include "SettingsDefinitions.h"  // build_info
#include "Protocol.h"             // LINE_BUFFER_SIZE
#include "UartChannel.h"          // UartChannel
#include "FileStream.h"           // FileStream()
#include "StartupLog.h"           // startupLog
#include "Driver/gpio_dump.h"     // gpio_dump()
#include "Driver/fluidnc_gpio.h"  // gpio_glitch_count()
#include "FileCommands.h"         // make_file_commands()
#include "Job.h"                  // Job::active()
#include "ToolTable.h"            // toolTable
#include "Kinematics/Compensated1D.h"  // compensated1D
#include "Kinematics/Compensated2D.h"  // compensated2D
#include "CAN/CanActuator.h"           // CanActuator
#include "CAN/CanNode.h"               // CanNodes
#include "string_util.h"               // split_prefix
#include "Logging.h"              // LogStream

#include "FluidPath.h"
#include "HashFS.h"
#include "wdt.h"

#include <cstring>
#include <string_view>
#include <map>
#include <filesystem>
#include <cmath>

// WG Readable and writable as guest
// WU Readable and writable as user and admin
// WA Readable as user and admin, writable as admin

static Error switchInchMM(const char* value, AuthenticationLevel auth_level, Channel& out);
static Error report_init_message_cmd(const char* value, AuthenticationLevel auth_level, Channel& out);

#ifdef ENABLE_AUTHENTICATION
// If authentication is disabled, auth_level will be LEVEL_ADMIN
static bool auth_failed(Word* w, std::string_view value, AuthenticationLevel auth_level) {
    permissions_t permissions = w->getPermissions();
    switch (auth_level) {
        case AuthenticationLevel::LEVEL_ADMIN:  // Admin can do anything
            return false;                       // Nothing is an Admin auth fail
        case AuthenticationLevel::LEVEL_GUEST:  // Guest can only access open settings
            return permissions != WG;           // Anything other than RG is Guest auth fail
        case AuthenticationLevel::LEVEL_USER:   // User is complicated...
            if (value.empty()) {                // User can read anything
                return false;                   // No read is a User auth fail
            }
            return permissions == WA;  // User cannot write WA
        default:
            return true;
    }
}
#else
static bool auth_failed(Word* w, std::string_view value, AuthenticationLevel auth_level) {
    return false;
}
#endif

// Replace GRBL realtime characters with the corresponding URI-style
// escape sequence.
static std::string uriEncodeGrblCharacters(const char* clear) {
    std::string escaped;
    char        c;
    while ((c = *clear++) != '\0') {
        switch (c) {
            case '%':  // The escape character itself
                escaped += "%25";
                break;
            case '!':  // Cmd::FeedHold
                escaped += "%21";
                break;
            case '?':  // Cmd::StatusReport
                escaped += "%3F";
                break;
            case '~':  // Cmd::CycleStart
                escaped += "%7E";
                break;
            default:
                escaped += c;
                break;
        }
    }
    return escaped;
}

// Replace URI-style escape sequences like %HH with the character
// corresponding to the hex number HH.  This works with any escaped
// characters, not only those that are special to Grbl
static std::string uriDecode(std::string_view s) {
    static std::string decoded;
    decoded.clear();
    char c;
    while (!s.empty()) {
        c = s.front();
        s.remove_prefix(1);
        if (c == '%') {
            if (s.length() < 2) {
                log_error("Bad % encoding - too short");
                goto done;
            }
            uint8_t esc;
            if (!string_util::from_hex(s.substr(0, 2), esc)) {
                log_error("Bad % encoding - not hex");
                goto done;
            }
            s.remove_prefix(2);
            c = (char)esc;
        }
        decoded += c;
    }
done:
    return decoded;
}

static void show_setting(const char* name, const char* value, const char* description, Channel& out) {
    LogStream s(out, "$");
    s << name << "=" << uriEncodeGrblCharacters(value);
    if (description) {
        s << "    ";
        s << description;
    }
}

void settings_restore(uint8_t restore_flag) {
    if (restore_flag & SettingsRestore::Wifi) {
        for (Setting* s : Setting::List) {
            if (s->getType() != WEBSET) {
                s->setDefault();
            }
        }
    }

    if (restore_flag & SettingsRestore::Defaults) {
        bool restore_startup = restore_flag & SettingsRestore::StartupLines;
        for (Setting* s : Setting::List) {
            if (!s->getDescription()) {
                const char* name = s->getName();
                if (restore_startup) {  // all settings get restored
                    s->setDefault();
                } else if ((strcmp(name, "Line0") != 0) && (strcmp(name, "Line1") != 0)) {  // non startup settings get restored
                    s->setDefault();
                }
            }
        }
        log_info("Settings reset done");
    }
    if (restore_flag & SettingsRestore::Parameters) {
        for (auto idx = CoordIndex::Begin; idx < CoordIndex::End; ++idx) {
            coords[idx]->setDefault();
        }
        coords[gc_state.modal.coord_select]->get(gc_state.coord_system);
        report_wco_counter = 0;  // force next report to include WCO
    }
    log_info("Position offsets reset done");
}

extern void make_settings();
extern void make_user_commands();

void settings_init() {
    make_settings();
    make_file_commands();
}

static Error show_help(const char* value, AuthenticationLevel auth_level, Channel& out) {
    log_string(out, "HLP:$$ $+ $# $S $L $G $I $N $x=val $Nx=line $J=line $SLP $C $X $H $F $E=err ~ ! ? ctrl-x");
    return Error::Ok;
}

static Error report_gcode(const char* value, AuthenticationLevel auth_level, Channel& out) {
    report_gcode_modes(out);
    return Error::Ok;
}

static void show_settings(Channel& out, type_t type) {
    switchInchMM(NULL, AuthenticationLevel::LEVEL_ADMIN, out);  // Print Report/Inches

    for (Setting* s : Setting::List) {
        if (s->getType() == type && s->getGrblName()) {
            show_setting(s->getGrblName(), s->getCompatibleValue(), NULL, out);
        }
    }
}

static Error report_normal_settings(const char* value, AuthenticationLevel auth_level, Channel& out) {
    show_settings(out, GRBL);  // GRBL non-axis settings
    return Error::Ok;
}
static Error list_grbl_names(const char* value, AuthenticationLevel auth_level, Channel& out) {
    log_stream(out, "$13 => $Report/Inches");

    for (Setting* setting : Setting::List) {
        const char* gn = setting->getGrblName();
        if (gn) {
            log_stream(out, "$" << gn << " => $" << setting->getName());
        }
    }
    return Error::Ok;
}
static Error list_settings(const char* value, AuthenticationLevel auth_level, Channel& out) {
    for (Setting* s : Setting::List) {
        const char* displayValue = auth_failed(s, value, auth_level) ? "<Authentication required>" : s->getStringValue();
        if (s->getType() != PIN) {
            show_setting(s->getName(), displayValue, NULL, out);
        }
    }
    return Error::Ok;
}
static Error list_changed_settings(const char* value, AuthenticationLevel auth_level, Channel& out) {
    for (Setting* s : Setting::List) {
        const char* value = s->getStringValue();
        if (!auth_failed(s, value, auth_level) && strcmp(value, s->getDefaultString())) {
            if (s->getType() != PIN) {
                show_setting(s->getName(), value, NULL, out);
            }
        }
    }
    log_string(out, "(Passwords not shown)");
    return Error::Ok;
}
static Error list_commands(const char* value, AuthenticationLevel auth_level, Channel& out) {
    for (Command* cp : Command::List) {
        const char* name    = cp->getName();
        const char* oldName = cp->getGrblName();
        LogStream   s(out, "$");
        s << name;
        if (oldName) {
            s << " or $" << oldName;
        }
        const char* description = cp->getDescription();
        if (description) {
            s << " =" << description;
        }
    }
    return Error::Ok;
}
static Error toggle_check_mode(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (state_is(State::ConfigAlarm)) {
        return Error::ConfigurationInvalid;
    }

    // Perform reset when toggling off. Check g-code mode should only work when
    // idle and ready, regardless of alarm locks. This is mainly to keep things
    // simple and consistent.
    if (state_is(State::CheckMode)) {
        report_feedback_message(Message::Disabled);
        sys.set_abort(true);
    } else {
        if (!state_is(State::Idle)) {
            return Error::IdleError;  // Requires no alarm mode.
        }
        set_state(State::CheckMode);
        report_feedback_message(Message::Enabled);
    }
    return Error::Ok;
}
static Error isStuck() {
    // Block if a control pin is stuck on
    if (config->_control->safety_door_ajar()) {
        send_alarm(ExecAlarm::ControlPin);
        return Error::CheckDoor;
    }
    if (config->_control->stuck()) {
        log_info("Control pins:" << config->_control->report_status());
        send_alarm(ExecAlarm::ControlPin);
        return Error::CheckControlPins;
    }
    return Error::Ok;
}
static Error disable_alarm_lock(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (state_is(State::ConfigAlarm)) {
        return Error::ConfigurationInvalid;
    }
    if (state_is(State::Alarm)) {
        Error err = isStuck();
        if (err != Error::Ok) {
            return err;
        }
        Homing::set_all_axes_homed();
        config->_kinematics->releaseMotors(Axes::motorMask, Axes::hardLimitMask());
        report_feedback_message(Message::AlarmUnlock);
        set_state(State::Idle);
    }
    // Run the after_unlock macro even if no unlock was necessary
    config->_macros->_after_unlock.run(&out);
    return Error::Ok;
}
static Error report_ngc(const char* value, AuthenticationLevel auth_level, Channel& out) {
    report_ngc_parameters(out);
    return Error::Ok;
}
static Error msg_to_uart0(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (value) {
        Channel* dest = allChannels.find("uart_channel0");
        if (dest) {
            log_msg_to(*dest, value);
        }
    }
    return Error::Ok;
}
static Error msg_to_uart1(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (value && config->_uart_channels[1]) {
        log_msg_to(*(config->_uart_channels[1]), value);
    }
    return Error::Ok;
}
static Error msg_to_channel(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (value) {
        std::string_view rest(value);
        std::string_view first;
        if (string_util::split_prefix(rest, first, ',')) {
            auto channel = allChannels.find(first);
            if (channel) {
                log_msg_to(*channel, rest);
                return Error::Ok;
            } else {
                log_error("Invalid channel name " << first);
            }
        }
    } else {
        log_error("Missing channel name");
    }

    return Error::InvalidValue;
}
static Error cmd_log_msg(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (value) {
        if (*value == '*') {
            log_msg(value + 1);
        } else {
            log_msg_to(out, value);
        }
    }
    return Error::Ok;
}
static Error cmd_log_error(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (value) {
        if (*value == '*') {
            log_error(value + 1);
        } else {
            log_error_to(out, value);
        }
    }
    return Error::Ok;
}
static Error cmd_log_warn(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (value) {
        if (*value == '*') {
            log_warn(value + 1);
        } else {
            log_warn_to(out, value);
        }
    }
    return Error::Ok;
}
static Error cmd_log_info(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (value) {
        if (*value == '*') {
            log_info(value + 1);
        } else {
            log_info_to(out, value);
        }
    }
    return Error::Ok;
}
static Error cmd_log_debug(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (value) {
        if (*value == '*') {
            log_debug(value + 1);
        } else {
            log_debug_to(out, value);
        }
    }
    return Error::Ok;
}
static Error cmd_log_verbose(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (value) {
        if (*value == '*') {
            log_verbose(value + 1);
        } else {
            log_verbose_to(out, value);
        }
    }
    return Error::Ok;
}
static Error home(AxisMask axisMask, Channel& out) {
    // see if blocking control switches are active
    if (config->_control->pins_block_unlock()) {
        return Error::CheckControlPins;
    }
    if (axisMask != Machine::Homing::AllCycles) {  // if not AllCycles we need to make sure the cycle is not prohibited
        // if there is a cycle it is the axis from $H<axis>
        auto n_axis = Axes::_numberAxis;
        for (axis_t axis = X_AXIS; axis < n_axis; axis++) {
            if (bitnum_is_true(axisMask, axis)) {
                auto axisConfig     = Axes::_axis[axis];
                auto homing         = axisConfig->_homing;
                auto homing_allowed = homing && homing->_allow_single_axis;
                if (!homing_allowed)
                    return Error::SingleAxisHoming;
            }
        }
    }

    if (state_is(State::ConfigAlarm)) {
        return Error::ConfigurationInvalid;
    }
    if (!Machine::Axes::homingMask) {
        return Error::SettingDisabled;
    }

    if (config->_control->safety_door_ajar()) {
        return Error::CheckDoor;  // Block if safety door is ajar.
    }

    Machine::Homing::run_cycles(axisMask);

    do {
        feed_WDT();
        protocol_execute_realtime();
    } while (state_is(State::Homing));

    return Error::Ok;
}
static Error home_all(const char* value, AuthenticationLevel auth_level, Channel& out) {
    AxisMask requestedAxes = Machine::Homing::AllCycles;
    auto     retval        = Error::Ok;

    // value can be a list of cycle numbers like "21", which will run homing cycle 2 then cycle 1,
    // or a list of axis names like "XZ", which will home the X and Z axes simultaneously
    if (value) {
        uint8_t    ndigits  = 0;
        const auto lenValue = strlen(value);
        for (int i = 0; i < lenValue; i++) {
            char cycleName = value[i];
            if (isdigit(cycleName)) {
                if (!Machine::Homing::axis_mask_from_cycle(cycleName - '0')) {
                    log_error("No axes for homing cycle " << cycleName);
                    return Error::InvalidValue;
                }
                ++ndigits;
            }
        }
        if (ndigits) {
            if (ndigits != lenValue) {
                log_error("Invalid homing cycle list");
                return Error::InvalidValue;
            } else {
                for (int i = 0; i < lenValue; i++) {
                    char cycleName = value[i];
                    requestedAxes  = Machine::Homing::axis_mask_from_cycle(cycleName - '0');
                    retval         = home(requestedAxes, out);
                    if (retval != Error::Ok) {
                        return retval;
                    }
                }
                return retval;
            }
        }
        if (!Axes::namesToMask(value, requestedAxes)) {
            return Error::InvalidValue;
        }
    }

    return home(requestedAxes, out);
}

static Error home_x(const char* value, AuthenticationLevel auth_level, Channel& out) {
    return home(bitnum_to_mask(X_AXIS), out);
}
static Error home_y(const char* value, AuthenticationLevel auth_level, Channel& out) {
    return home(bitnum_to_mask(Y_AXIS), out);
}
static Error home_z(const char* value, AuthenticationLevel auth_level, Channel& out) {
    return home(bitnum_to_mask(Z_AXIS), out);
}
static Error home_a(const char* value, AuthenticationLevel auth_level, Channel& out) {
    return home(bitnum_to_mask(A_AXIS), out);
}
static Error home_b(const char* value, AuthenticationLevel auth_level, Channel& out) {
    return home(bitnum_to_mask(B_AXIS), out);
}
static Error home_c(const char* value, AuthenticationLevel auth_level, Channel& out) {
    return home(bitnum_to_mask(C_AXIS), out);
}
static Error home_u(const char* value, AuthenticationLevel auth_level, Channel& out) {
    return home(bitnum_to_mask(U_AXIS), out);
}
static Error home_v(const char* value, AuthenticationLevel auth_level, Channel& out) {
    return home(bitnum_to_mask(V_AXIS), out);
}
static Error home_w(const char* value, AuthenticationLevel auth_level, Channel& out) {
    return home(bitnum_to_mask(W_AXIS), out);
}
static std::string limit_set(uint32_t mask) {
    std::string s;
    for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; axis++) {
        s += bitnum_is_true(mask, Machine::Axes::motor_bit(axis, 0)) ? tolower(Machine::Axes::axisName(axis)[0]) : ' ';
    }
    for (axis_t axis = X_AXIS; axis < MAX_N_AXIS; axis++) {
        s += bitnum_is_true(mask, Machine::Axes::motor_bit(axis, 1)) ? toupper(Machine::Axes::axisName(axis)[0]) : ' ';
    }
    return s;
}
static Error show_limits(const char* value, AuthenticationLevel auth_level, Channel& out) {
    log_string(out, "Send ! to exit");
    log_stream(out, "Homing Axes : " << limit_set(Machine::Axes::homingMask));
    log_stream(out, "Limit Axes : " << limit_set(Machine::Axes::limitMask));
    log_string(out, "  PosLimitPins NegLimitPins Probe Toolsetter");

    const TickType_t interval = 500;
    TickType_t       limit    = xTaskGetTickCount();
    runLimitLoop              = true;
    do {
        TickType_t thisTime = xTaskGetTickCount();
        if (((long)(thisTime - limit)) > 0) {
            log_stream(out,
                       ": " << limit_set(Machine::Axes::posLimitMask) << " " << limit_set(Machine::Axes::negLimitMask)
                            << (config->_probe->probePin().get() ? " P" : "") << (config->_probe->toolsetterPin().get() ? " T" : ""));
            limit = thisTime + interval;
        }
        delay_ms(1);
        protocol_handle_events();
    } while (runLimitLoop);
    log_string(out, "");
    return Error::Ok;
}
static Error go_to_sleep(const char* value, AuthenticationLevel auth_level, Channel& out) {
    protocol_send_event(&sleepEvent);
    return Error::Ok;
}
static Error get_report_build_info(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (!value) {
        report_build_info(build_info->get(), out);
        return Error::Ok;
    }
    return Error::InvalidStatement;
}

const std::map<const char*, uint8_t, cmp_str> restoreCommands = {
    { "$", SettingsRestore::Defaults },   { "settings", SettingsRestore::Defaults },
    { "#", SettingsRestore::Parameters }, { "gcode", SettingsRestore::Parameters },
    { "*", SettingsRestore::All },        { "all", SettingsRestore::All },
    { "@", SettingsRestore::Wifi },       { "wifi", SettingsRestore::Wifi },
};
static Error restore_settings(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (!value) {
        return Error::InvalidStatement;
    }
    auto it = restoreCommands.find(value);
    if (it == restoreCommands.end()) {
        return Error::InvalidStatement;
    }
    settings_restore(it->second);
    return Error::Ok;
}

static Error showState(const char* value, AuthenticationLevel auth_level, Channel& out) {
    const char* name;
    const State state = sys.state();
    auto        it    = StateName.find(state);
    name              = it == StateName.end() ? "<invalid>" : it->second;

    log_stream(out, "State " << int(state) << " (" << name << ")");
    return Error::Ok;
}

static Error doJog(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (state_is(State::ConfigAlarm)) {
        return Error::ConfigurationInvalid;
    }

    // For jogging, you must give gc_execute_line() a line that
    // begins with $J=.  There are several ways we can get here,
    // including  $J, $J=xxx, [J]xxx.  For any form other than
    // $J without =, we reconstruct a $J= line for gc_execute_line().
    if (!value) {
        return Error::InvalidStatement;
    }
    char jogLine[LINE_BUFFER_SIZE];
    strcpy(jogLine, "$J=");
    strcat(jogLine, value);
    return gc_execute_line(jogLine);
}

// $CA=<label>,<command>[,<argument>[,<timeout_ms>]]
//
// Drives one of the auxiliary actuators on a CAN node.  This is the interface a tool
// changer macro uses: the actuator runs its own motion profile on the node and, unless
// the timeout is given as 0, this blocks until the node reports that it finished.
static Error runCanActuator(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (!value) {
        log_string(out, "Usage: $CA=<label>,<move_abs|move_rel|home|output|stop>[,<arg>[,<timeout_ms>]]");
        return Error::InvalidStatement;
    }

    std::string_view rest(value);
    std::string_view label;
    if (!string_util::split_prefix(rest, label, ',')) {
        return Error::InvalidStatement;
    }

    auto actuator = CAN::CanActuator::byLabel(std::string(label));
    if (actuator == nullptr) {
        log_error_to(out, "No CAN actuator labelled '" << label << "'");
        return Error::InvalidValue;
    }

    std::string_view verb;
    string_util::split_prefix(rest, verb, ',');

    CAN::CanActuator::Command command;
    if (verb == "move_abs") {
        command = CAN::CanActuator::Command::MoveAbsolute;
    } else if (verb == "move_rel") {
        command = CAN::CanActuator::Command::MoveRelative;
    } else if (verb == "home") {
        command = CAN::CanActuator::Command::Home;
    } else if (verb == "output") {
        command = CAN::CanActuator::Command::SetOutput;
    } else if (verb == "stop") {
        command = CAN::CanActuator::Command::Stop;
    } else {
        log_error_to(out, "Unknown actuator command '" << verb << "'");
        return Error::InvalidValue;
    }

    std::string_view argText;
    int32_t          argument = 0;
    if (string_util::split_prefix(rest, argText, ',')) {
        argument = int32_t(strtol(std::string(argText).c_str(), nullptr, 10));
    } else if (!rest.empty()) {
        argument = int32_t(strtol(std::string(rest).c_str(), nullptr, 10));
        rest     = {};
    }

    uint32_t timeout = uint32_t(actuator->defaultTimeoutMs());
    if (!rest.empty()) {
        timeout = uint32_t(strtoul(std::string(rest).c_str(), nullptr, 10));
    }

    if (!actuator->run(command, argument, timeout)) {
        return Error::AnotherInterfaceBusy;
    }
    return Error::Ok;
}

static Error listCanActuators(const char* value, AuthenticationLevel auth_level, Channel& out) {
    for (auto actuator : CAN::CanActuator::all()) {
        log_stream(out, "Actuator " << actuator->label());
    }
    return Error::Ok;
}

static Error showCanStatus(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (config->_can == nullptr) {
        log_string(out, "No CAN bus configured");
        return Error::Ok;
    }
    log_stream(out, config->_can->statusString());
    log_stream(out, CAN::CanNodes::instance().statusString());
    return Error::Ok;
}

static Error listAlarms(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (state_is(State::ConfigAlarm)) {
        log_string(out, "Configuration alarm is active. Check the boot messages for 'ERR'.");
    } else if (state_is(State::Alarm)) {
        log_stream(out, "Active alarm: " << int(lastAlarm) << " (" << alarmString(lastAlarm) << ")");
    }
    if (value) {
        uint32_t alarmNumber;
        if (!string_util::from_decimal(value, alarmNumber)) {
            log_stream(out, "Malformed alarm number: " << value);
            return Error::InvalidValue;
        }
        const char* alarmName = alarmString(static_cast<ExecAlarm>(alarmNumber));
        if (alarmName) {
            log_stream(out, alarmNumber << ": " << alarmName);
            return Error::Ok;
        } else {
            log_stream(out, "Unknown alarm number: " << alarmNumber);
            return Error::InvalidValue;
        }
    }

    for (auto it = AlarmNames.begin(); it != AlarmNames.end(); it++) {
        log_stream(out, static_cast<int>(it->first) << ": " << it->second);
    }
    return Error::Ok;
}

const char* errorString(Error errorNumber) {
    auto it = ErrorNames.find(errorNumber);
    return it == ErrorNames.end() ? NULL : it->second;
}

static Error listErrors(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (value) {
        uint32_t errorNumber;
        if (!string_util::from_decimal(value, errorNumber)) {
            log_stream(out, "Malformed error number: " << value);
            return Error::InvalidValue;
        }
        const char* errorName = errorString(static_cast<Error>(errorNumber));
        if (errorName) {
            log_stream(out, errorNumber << ": " << errorName);
            return Error::Ok;
        } else {
            log_stream(out, "Unknown error number: " << errorNumber);
            return Error::InvalidValue;
        }
    }

    for (auto it = ErrorNames.begin(); it != ErrorNames.end(); it++) {
        log_stream(out, static_cast<int>(it->first) << ": " << it->second);
    }
    return Error::Ok;
}

static Error motor_control(const char* value, bool disable) {
    if (state_is(State::ConfigAlarm)) {
        return Error::ConfigurationInvalid;
    }

    while (value && isspace(*value)) {
        ++value;
    }
    if (!value || *value == '\0') {
        log_info((disable ? "Dis" : "En") << "abling all motors");
        Axes::set_disable(disable);
        return Error::Ok;
    }

    auto axes = config->_axes;

    if (axes->_sharedStepperDisable.defined()) {
        log_error("Cannot " << (disable ? "dis" : "en") << "able individual axes with a shared disable pin");
        return Error::InvalidStatement;
    }

    axis_t axis = Machine::Axes::axisNum(value);
    if (axis == INVALID_AXIS) {
        return Error::InvalidValue;
    }
    log_info((disable ? "Dis" : "En") << "abling " << value << " motors");
    axes->set_disable(axis, disable);
    return Error::Ok;
}
static Error motor_disable(const char* value, AuthenticationLevel auth_level, Channel& out) {
    return motor_control(value, true);
}

static Error motor_enable(const char* value, AuthenticationLevel auth_level, Channel& out) {
    return motor_control(value, false);
}

static Error motors_init(const char* value, AuthenticationLevel auth_level, Channel& out) {
    Axes::config_motors();
    return Error::Ok;
}

static Error macros_run(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (value) {
        size_t macro_num = (*value) - '0';

        auto ok = config->_macros->_macro[macro_num].run(&out);
        return ok ? Error::Ok : Error::NumberRange;
    }
    log_error("$Macros/Run requires a macro number argument");
    return Error::InvalidStatement;
}

static Error dump_config(const char* value, AuthenticationLevel auth_level, Channel& out) {
    Channel* ss;
    if (value) {
        // Use a file on the local file system unless there is an explicit prefix like /sd/
        std::error_code ec;

        try {
            //            ss = new FileStream(std::string(value), "", "w");
            ss = new FileStream(value, "w", "");
        } catch (Error err) { return err; }
    } else {
        ss = &out;
    }
    try {
        Configuration::Generator generator(*ss);
        config->group(generator);
    } catch (std::exception& ex) { log_info("Config dump error: " << ex.what()); }
    if (value) {
        drain_messages();
        delete ss;
    }
    return Error::Ok;
}

static Error report_init_message_cmd(const char* value, AuthenticationLevel auth_level, Channel& out) {
    report_init_message(out);

    return Error::Ok;
}

static Error switchInchMM(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (!value) {
        log_stream(out, "$13=" << (config->_reportInches ? "1" : "0"));
    } else {
        config->_reportInches = ((value[0] == '1') ? true : false);
    }

    return Error::Ok;
}

static Error showChannelInfo(const char* value, AuthenticationLevel auth_level, Channel& out) {
    allChannels.listChannels(out);
    return Error::Ok;
}

static Error showStartupLog(const char* value, AuthenticationLevel auth_level, Channel& out) {
    StartupLog::dump(out);
    return Error::Ok;
}

static Error showGPIOs(const char* value, AuthenticationLevel auth_level, Channel& out) {
    gpio_dump(out);
    return Error::Ok;
}

// Rejected glitches per input.  A count that climbs while a VFD or other noisy
// load is running says the debounce filter is earning its keep on that pin; a
// count that climbs fast enough to let false triggers through means the
// sampling interval is colliding with the noise and should be changed.
static Error showGlitches(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (value) {
        gpio_glitch_reset();
        log_string(out, "Glitch counters cleared");
        return Error::Ok;
    }
    uint32_t total = gpio_glitch_total();
    log_stream(out, "Rejected input glitches: " << total);
    if (total) {
        for (int32_t gpio_num = 0; gpio_num <= MAX_N_GPIO; gpio_num++) {
            uint32_t count = gpio_glitch_count(gpio_num);
            if (count) {
                log_stream(out, "  gpio." << gpio_num << " " << count);
            }
        }
    }
    return Error::Ok;
}

#include "UartTypes.h"

static Error uartPassthrough(const char* value, AuthenticationLevel auth_level, Channel& out) {
    uint32_t    timeout = 2000;
    std::string uart_name("auto");
    objnum_t    uart_num;

    if (value) {
        std::string_view rest(value);
        std::string_view first;
        while (string_util::split_prefix(rest, first, ',')) {
            if (string_util::equal_ignore_case(first, "auto")) {
                uart_name = "auto";
            } else if (!first.empty() && string_util::tolower(first.back()) == 's') {
                first.remove_suffix(1);
                if (!string_util::from_decimal(first, timeout)) {
                    log_error_to(out, "Invalid timeout number");
                    return Error::InvalidValue;
                }
                timeout *= 1000;
            } else {
                uart_name = first;
            }
        }
    }
    Uart* downstream_uart = nullptr;
    if (uart_name == "auto") {
        // Find a UART device with a non-empty passthrough_baud config item
        for (uart_num = 1; uart_num < MAX_N_UARTS; ++uart_num) {
            downstream_uart = config->_uarts[uart_num];
            if (downstream_uart) {
                if (downstream_uart->_passthrough_baud != 0) {
                    break;
                }
            }
        }
        if (uart_num == MAX_N_UARTS) {
            log_error_to(out, "No uart has passthrough_baud configured");
            return Error::InvalidValue;
        }
    } else {
        // Find a UART device that matches the name
        for (uart_num = 1; uart_num < MAX_N_UARTS; ++uart_num) {
            downstream_uart = config->_uarts[uart_num];
            if (downstream_uart) {
                if (downstream_uart->name() == uart_name) {
                    if (downstream_uart->_passthrough_baud == 0) {
                        log_error_to(out, uart_name << " does not have passthrough_baud configured");
                        return Error::InvalidValue;
                    } else {
                        break;
                    }
                }
            }
        }
        if (uart_num == MAX_N_UARTS) {
            log_error_to(out, uart_name << " does not exist");
            return Error::InvalidValue;
        }
    }

    out.pause();  // Stop input polling on the upstream channel

    UartChannel* channel = nullptr;
    for (size_t n = 0; (channel = config->_uart_channels[n]) != nullptr; ++n) {
        if (channel->uart_num() == uart_num) {
            break;
        }
        channel = nullptr;  // Leave channel null if not found
    }

    if (channel) {
        channel->pause();
    }
    downstream_uart->enterPassthrough();

    const int buflen = 256;
    uint8_t   buffer[buflen];

    TickType_t last_ticks = xTaskGetTickCount();

    while (xTaskGetTickCount() - last_ticks < timeout) {
        size_t len;
        len = out.timedReadBytes((char*)buffer, buflen, 10);
        if (len > 0) {
            last_ticks = xTaskGetTickCount();
            downstream_uart->write(buffer, len);
        }
        len = downstream_uart->timedReadBytes((char*)buffer, buflen, 10);
        if (len > 0) {
            last_ticks = xTaskGetTickCount();
            out.write(buffer, len);
        }
    }

    downstream_uart->exitPassthrough();
    if (channel) {
        channel->resume();
    }
    out.resume();
    return Error::Ok;
}

std::map<std::string, Pin*> pins;

static Error setGPIOInput(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (pins.find(value) == pins.end()) {
        Pin* thePin = new Pin(Pin::create(value));
        pins[value] = thePin;
    }

    pins[value]->setAttr(Pin::Attr::Input);
    log_info("Pin " << value << " set to input");

    return Error::Ok;
}

static Error setGPIOOutput(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (pins.find(value) == pins.end()) {
        Pin* thePin = new Pin(Pin::create(value));
        pins[value] = thePin;
    }

    pins[value]->setAttr(Pin::Attr::Output);
    log_info("Pin " << value << " set to output");

    return Error::Ok;
}

static Error readGPIO(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (pins.find(value) == pins.end()) {
        Pin* thePin = new Pin(Pin::create(value));
        pins[value] = thePin;
        thePin->setAttr(Pin::Attr::Input);
    }

    const auto v = pins[value]->read() ? "on" : "off";
    log_info("Pin " << value << " reads " << v);

    return Error::Ok;
}

static Error writeGPIOOn(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (pins.find(value) == pins.end()) {
        Pin* thePin = new Pin(Pin::create(value));
        pins[value] = thePin;
        thePin->setAttr(Pin::Attr::Output);
    }

    pins[value]->synchronousWrite(true);
    log_info("Pin " << value << " is on");

    return Error::Ok;
}

static Error writeGPIOOff(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (pins.find(value) == pins.end()) {
        Pin* thePin = new Pin(Pin::create(value));
        pins[value] = thePin;
        thePin->setAttr(Pin::Attr::Output);
    }

    pins[value]->synchronousWrite(0);
    log_info("Pin " << value << " is off");

    return Error::Ok;
}

static Error setReportInterval(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (!value) {
        uint32_t actual = out.getReportInterval();
        if (actual) {
            log_info_to(out, out.name() << " auto report interval is " << actual << " ms");
        } else {
            log_info_to(out, out.name() << " auto reporting is off");
        }
        return Error::Ok;
    }
    uint32_t intValue;

    if (!string_util::from_decimal(value, intValue)) {
        return Error::BadNumberFormat;
    }

    uint32_t actual = out.setReportInterval(intValue);
    if (actual) {
        log_info(out.name() << " auto report interval set to " << actual << " ms");
    } else {
        log_info(out.name() << " auto reporting turned off");
    }

    // Send a full status report immediately so the client has all the data
    out.notifyWco();
    out.notifyOvr();

    return Error::Ok;
}

static Error sendAlarm(const char* value, AuthenticationLevel auth_level, Channel& out) {
    int32_t   intValue = value ? atoi(value) : 0;
    ExecAlarm alarm    = static_cast<ExecAlarm>(intValue);
    log_debug("Sending alarm " << intValue << " " << alarmString(alarm));
    send_alarm(alarm);
    return Error::Ok;
}

static Error showHeap(const char* value, AuthenticationLevel auth_level, Channel& out) {
    log_info("Heap free: " << xPortGetFreeHeapSize() << " min: " << heapLowWater);
    return Error::Ok;
}

static Error crash(const char* value, AuthenticationLevel auth_level, Channel& out) {
    log_error("Deliberate crash via $Crash command");
    delay_ms(100);
    *((volatile int*)0) = 0xdeadbeef;
    return Error::Ok;  // unreachable
}

// Tool table commands
static Error showToolTable(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (toolTable == nullptr) {
        log_error_to(out, "Tool table not loaded");
        return Error::InvalidStatement;
    }
    
    log_info_to(out, "Tool Table: " << toolTable->filename());
    log_info_to(out, "Tools: " << toolTable->count());
    
    for (auto tool : toolTable->tools()) {
        if (tool == nullptr) continue;
        LogStream ss(out, MsgLevelInfo, "[MSG:INFO: ");
        ss << "  T" << tool->_number << ": ";
        if (!tool->_name.empty()) {
            ss << "\"" << tool->_name << "\" ";
        }
        ss << "X:" << tool->_offset[X_AXIS] 
           << " Y:" << tool->_offset[Y_AXIS] 
           << " Z:" << tool->_offset[Z_AXIS]
           << " R:" << tool->_radius;
    }
    
    // Show turret mapping if any
    const auto turret = toolTable->turret();
    if (turret != nullptr && turret->hasAnyMapping()) {
        log_info_to(out, "Turret mapping:");
        for (int pos = 1; pos <= 20; pos++) {
            int32_t toolNum = turret->getToolForPosition(pos);
            if (toolNum > 0) {
                log_info_to(out, "  Position " << pos << " -> Tool " << toolNum);
            }
        }
    }
    
    return Error::Ok;
}

static Error reloadToolTable(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (toolTable == nullptr) {
        toolTable = new ToolTable();
    }
    
    std::string filename = "/localfs/tooltable.yaml";
    if (value && *value) {
        filename = value;
    }
    
    if (toolTable->load(filename)) {
        log_info_to(out, "Loaded tool table from " << filename);
        return Error::Ok;
    } else {
        log_error_to(out, "Failed to load tool table from " << filename);
        return Error::FsFailedOpenFile;
    }
}

static Error saveToolTable(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (toolTable == nullptr) {
        log_error_to(out, "Tool table not loaded");
        return Error::InvalidStatement;
    }
    
    if (toolTable->save()) {
        log_info_to(out, "Saved tool table to " << toolTable->filename());
        return Error::Ok;
    } else {
        log_error_to(out, "Failed to save tool table");
        return Error::FsFailedCreateFile;
    }
}

// 1D Compensation commands (Lathe X-from-Z)
static Error showComp1D(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (compensated1D == nullptr) {
        log_error_to(out, "Compensated1D kinematic not active");
        return Error::InvalidStatement;
    }
    
    log_info_to(out, "1D Compensation (X from Z):");
    log_info_to(out, "  File: " << compensated1D->getFilename());
    log_info_to(out, "  Z range: " << compensated1D->getZMin() << " to " << compensated1D->getZMax() << " mm");
    log_info_to(out, "  Granularity: " << compensated1D->getGranularity() << " mm");
    log_info_to(out, "  Table size: " << compensated1D->getOffsets().size() << " points");
    
    // Show non-zero offsets (sparse view)
    const auto& offsets = compensated1D->getOffsets();
    float       gran    = compensated1D->getGranularity();
    float       z_min   = compensated1D->getZMin();
    int         count   = 0;
    
    for (size_t i = 0; i < offsets.size(); i++) {
        if (std::abs(offsets[i]) > 0.0001f) {
            float z = z_min + i * gran;
            log_info_to(out, "  Z=" << z << " X_offset=" << offsets[i]);
            count++;
            if (count >= 50) {
                log_info_to(out, "  ... (more points not shown)");
                break;
            }
        }
    }
    
    if (count == 0) {
        log_info_to(out, "  (all offsets are zero)");
    }
    
    return Error::Ok;
}

static Error setComp1DPoint(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (compensated1D == nullptr) {
        log_error_to(out, "Compensated1D kinematic not active");
        return Error::InvalidStatement;
    }
    
    if (value == nullptr || *value == '\0') {
        log_error_to(out, "Usage: $C1P=z,x_offset");
        return Error::InvalidStatement;
    }
    
    // Parse z,x_offset
    float z, x_offset;
    if (sscanf(value, "%f,%f", &z, &x_offset) != 2) {
        log_error_to(out, "Invalid format. Usage: $C1P=z,x_offset");
        return Error::InvalidStatement;
    }
    
    compensated1D->setOffset(z, x_offset);
    log_info_to(out, "Set compensation at Z=" << z << " to X_offset=" << x_offset << " (auto-saved)");
    return Error::Ok;
}

static Error clearComp1D(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (compensated1D == nullptr) {
        log_error_to(out, "Compensated1D kinematic not active");
        return Error::InvalidStatement;
    }
    
    compensated1D->clearOffsets();
    log_info_to(out, "Cleared all 1D compensation offsets (auto-saved)");
    return Error::Ok;
}

// 2D Compensation commands (Mill Z-from-XY)
static Error showComp2D(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (compensated2D == nullptr) {
        log_error_to(out, "Compensated2D kinematic not active");
        return Error::InvalidStatement;
    }
    
    log_info_to(out, "2D Compensation (Z from X,Y):");
    log_info_to(out, "  File: " << compensated2D->getFilename());
    log_info_to(out, "  X range: " << compensated2D->getXMin() << " to " << compensated2D->getXMax() << " mm");
    log_info_to(out, "  Y range: " << compensated2D->getYMin() << " to " << compensated2D->getYMax() << " mm");
    log_info_to(out, "  Grid: " << compensated2D->getXCount() << " x " << compensated2D->getYCount() << " points");
    
    // Show grid
    int x_count = compensated2D->getXCount();
    int y_count = compensated2D->getYCount();
    
    // Header with X indices
    {
        LogStream ss(out, MsgLevelInfo, "[MSG:INFO: ");
        ss << "     ";
        for (int x = 0; x < x_count && x < 10; x++) {
            char buf[16];
            snprintf(buf, sizeof(buf), " %6d", x);
            ss << buf;
        }
        if (x_count > 10) {
            ss << " ...";
        }
    }
    
    // Grid rows
    for (int y = 0; y < y_count && y < 20; y++) {
        LogStream ss(out, MsgLevelInfo, "[MSG:INFO: ");
        char buf[16];
        snprintf(buf, sizeof(buf), "Y%2d:", y);
        ss << buf;
        for (int x = 0; x < x_count && x < 10; x++) {
            float val = compensated2D->getOffset(x, y);
            snprintf(buf, sizeof(buf), " %6.3f", val);
            ss << buf;
        }
        if (x_count > 10) {
            ss << " ...";
        }
    }
    
    if (y_count > 20) {
        log_info_to(out, "  ... (more rows not shown)");
    }
    
    return Error::Ok;
}

static Error setComp2DPoint(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (compensated2D == nullptr) {
        log_error_to(out, "Compensated2D kinematic not active");
        return Error::InvalidStatement;
    }
    
    if (value == nullptr || *value == '\0') {
        log_error_to(out, "Usage: $C2P=ix,iy,z_offset");
        return Error::InvalidStatement;
    }
    
    // Parse ix,iy,z_offset
    int   ix, iy;
    float z_offset;
    if (sscanf(value, "%d,%d,%f", &ix, &iy, &z_offset) != 3) {
        log_error_to(out, "Invalid format. Usage: $C2P=ix,iy,z_offset");
        return Error::InvalidStatement;
    }
    
    if (ix < 0 || ix >= compensated2D->getXCount() || iy < 0 || iy >= compensated2D->getYCount()) {
        log_error_to(out, "Index out of range. X: 0-" << (compensated2D->getXCount() - 1) 
                          << ", Y: 0-" << (compensated2D->getYCount() - 1));
        return Error::InvalidStatement;
    }
    
    compensated2D->setOffset(ix, iy, z_offset);
    log_info_to(out, "Set compensation at [" << ix << "," << iy << "] to Z_offset=" << z_offset << " (auto-saved)");
    return Error::Ok;
}

static Error clearComp2D(const char* value, AuthenticationLevel auth_level, Channel& out) {
    if (compensated2D == nullptr) {
        log_error_to(out, "Compensated2D kinematic not active");
        return Error::InvalidStatement;
    }
    
    compensated2D->clearOffsets();
    log_info_to(out, "Cleared all 2D compensation offsets (auto-saved)");
    return Error::Ok;
}

// Commands use the same syntax as Settings, but instead of setting or
// displaying a persistent value, a command causes some action to occur.
// That action could be anything, from displaying a run-time parameter
// to performing some system state change.  Each command is responsible
// for decoding its own value string, if it needs one.
void make_user_commands() {
    new UserCommand("GD", "GPIO/Dump", showGPIOs, anyState);
    new UserCommand("GI", "GPIO/Input", setGPIOInput, anyState);
    new UserCommand("GO", "GPIO/Output", setGPIOOutput, anyState);
    new UserCommand("G+", "GPIO/On", writeGPIOOn, anyState);
    new UserCommand("G-", "GPIO/Off", writeGPIOOff, anyState);
    new UserCommand("GR", "GPIO/Read", readGPIO, anyState);
    new UserCommand("GG", "GPIO/Glitches", showGlitches, anyState);

    new UserCommand("CI", "Channel/Info", showChannelInfo, anyState);
    new UserCommand("CD", "Config/Dump", dump_config, anyState);
    new UserCommand("", "Help", show_help, anyState);
    new UserCommand("T", "State", showState, anyState);

    new UserCommand("$", "GrblSettings/List", report_normal_settings, cycleOrHold);
    new UserCommand("L", "GrblNames/List", list_grbl_names, cycleOrHold);
    new UserCommand("Limits", "Limits/Show", show_limits, cycleOrHold);
    new UserCommand("S", "Settings/List", list_settings, cycleOrHold);
    new UserCommand("SC", "Settings/ListChanged", list_changed_settings, cycleOrHold);
    new UserCommand("CMD", "Commands/List", list_commands, cycleOrHold);
    new UserCommand("A", "Alarms/List", listAlarms, anyState);
    new UserCommand("E", "Errors/List", listErrors, anyState);
    new UserCommand("C", "GCode/Check", toggle_check_mode, anyState);
    new UserCommand("X", "Alarm/Disable", disable_alarm_lock, anyState);
    new UserCommand("NVX", "Settings/Erase", Setting::eraseNVS, notIdleOrAlarm, WA);
    new UserCommand("V", "Settings/Stats", Setting::report_nvs_stats, notIdleOrAlarm);
    new UserCommand("#", "GCode/Offsets", report_ngc, notIdleOrAlarm);
    new UserCommand("MD", "Motor/Disable", motor_disable, notIdleOrAlarm);
    new UserCommand("ME", "Motor/Enable", motor_enable, notIdleOrAlarm);
    new UserCommand("MI", "Motors/Init", motors_init, notIdleOrAlarm);

    new UserCommand("RM", "Macros/Run", macros_run, nullptr);

    new UserCommand("H", "Home", home_all, allowConfigStates);
    new UserCommand("HX", "Home/X", home_x, allowConfigStates);
    new UserCommand("HY", "Home/Y", home_y, allowConfigStates);
    new UserCommand("HZ", "Home/Z", home_z, allowConfigStates);
    new UserCommand("HA", "Home/A", home_a, allowConfigStates);
    new UserCommand("HB", "Home/B", home_b, allowConfigStates);
    new UserCommand("HC", "Home/C", home_c, allowConfigStates);
    new UserCommand("HU", "Home/U", home_u, allowConfigStates);
    new UserCommand("HV", "Home/V", home_v, allowConfigStates);
    new UserCommand("HW", "Home/W", home_w, allowConfigStates);

    new UserCommand("MU0", "Msg/Uart0", msg_to_uart0, anyState);
    new UserCommand("MU1", "Msg/Uart1", msg_to_uart1, anyState);
    new UserCommand("MC", "Msg/Channel", msg_to_channel, anyState);
    new UserCommand("LM", "Log/Msg", cmd_log_msg, anyState);
    new UserCommand("LE", "Log/Error", cmd_log_error, anyState);
    new UserCommand("LW", "Log/Warn", cmd_log_warn, anyState);
    new UserCommand("LI", "Log/Info", cmd_log_info, anyState);
    new UserCommand("LD", "Log/Debug", cmd_log_debug, anyState);
    new UserCommand("LV  ", "Log/Verbose", cmd_log_verbose, anyState);

    new UserCommand("SLP", "System/Sleep", go_to_sleep, notIdleOrAlarm);
    new UserCommand("I", "Build/Info", get_report_build_info, notIdleOrAlarm);
    new UserCommand("RST", "Settings/Restore", restore_settings, notIdleOrAlarm, WA);

    new UserCommand("SA", "Alarm/Send", sendAlarm, anyState);
    new UserCommand("Heap", "Heap/Show", showHeap, anyState);
    new UserCommand("", "Crash", crash, anyState);
    new UserCommand("SS", "Startup/Show", showStartupLog, anyState);
    new UserCommand("UP", "Uart/Passthrough", uartPassthrough, notIdleOrAlarm);

    new UserCommand("RI", "Report/Interval", setReportInterval, anyState);

    new UserCommand("13", "Report/Inches", switchInchMM, notIdleOrAlarm);

    new UserCommand("GS", "GRBL/Show", report_init_message_cmd, notIdleOrAlarm);

    // Tool table commands
    new UserCommand("TT", "ToolTable/Show", showToolTable, anyState);
    new UserCommand("TTL", "ToolTable/Load", reloadToolTable, anyState);
    new UserCommand("TTS", "ToolTable/Save", saveToolTable, anyState);

    // 1D Compensation commands (Lathe X-from-Z)
    new UserCommand("C1", "Comp1D/Show", showComp1D, anyState);
    new UserCommand("C1P", "Comp1D/Set", setComp1DPoint, anyState);
    new UserCommand("C1C", "Comp1D/Clear", clearComp1D, anyState);

    // 2D Compensation commands (Mill Z-from-XY)
    new UserCommand("C2", "Comp2D/Show", showComp2D, anyState);
    new UserCommand("C2P", "Comp2D/Set", setComp2DPoint, anyState);
    new UserCommand("C2C", "Comp2D/Clear", clearComp2D, anyState);

    new UserCommand("CA", "CanActuator/Run", runCanActuator, anyState);
    new UserCommand("CAL", "CanActuator/List", listCanActuators, anyState);
    new UserCommand("CAN", "Can/Status", showCanStatus, anyState);

    new AsyncUserCommand("J", "Jog", doJog, notIdleOrJog);
    new AsyncUserCommand("G", "GCode/Modes", report_gcode, anyState);
};

// This is the handler for all forms of settings commands,
// $..= and [..], with and without a value.
Error do_command_or_setting(std::string_view key, std::string_view value, AuthenticationLevel auth_level, Channel& out) {
    // If value is empty, it means that there was no value string, i.e.
    // $key without =, or [key] with nothing following.
    // If value is not NULL, but the string is empty, that is the form
    // $key= with nothing following the = .

    // Try to execute a command.  Commands handle values internally;
    // you cannot determine whether to set or display solely based on
    // the presence of a value.
    for (Command* cp : Command::List) {
        if (string_util::equal_ignore_case(cp->getName(), key) ||
            (cp->getGrblName() && string_util::equal_ignore_case(cp->getGrblName(), key))) {
            if (auth_failed(cp, value, auth_level)) {
                return Error::AuthenticationFailed;
            }
            if (cp->synchronous()) {
                protocol_buffer_synchronize();
            }
            if (value.empty()) {
                return cp->action(nullptr, auth_level, out);
            }
            std::string s(value);
            return cp->action(s.c_str(), auth_level, out);
        }
    }

    // First search the yaml settings by name. If found, set a new
    // value if one is given, otherwise display the current value
    try {
        Configuration::RuntimeSetting rts(key, value, out);
        config->group(rts);

        if (rts.isHandled_) {
            if (!value.empty()) {
                // Validate only if something changed, not for display
                try {
                    Configuration::Validator validator;
                    config->validate();
                    config->group(validator);
                } catch (std::exception& ex) {
                    log_error("Validation error: " << ex.what());
                    return Error::ConfigurationInvalid;
                }

                Configuration::AfterParse afterParseHandler;
                config->afterParse();
                config->group(afterParseHandler);
            }
            return Error::Ok;
        }
    } catch (std::exception& ex) {
        log_error("Configuration change failed: " << ex.what());
        return Error::ConfigurationInvalid;
    }

    // Next search the settings list by text name. If found, set a new
    // value if one is given, otherwise display the current value
    for (Setting* s : Setting::List) {
        if (string_util::equal_ignore_case(s->getName(), key)) {
#if 0
            if (auth_failed(s, value, auth_level)) {
                return Error::AuthenticationFailed;
            }
#endif
            if (value.empty()) {
                show_setting(s->getName(), s->getStringValue(), NULL, out);
                return Error::Ok;
            }
            return s->setStringValue(uriDecode(value));
        }
    }

    // Then search the setting list by compatible name.  If found, set a new
    // value if one is given, otherwise display the current value in compatible mode
    for (Setting* s : Setting::List) {
        if (s->getGrblName() && string_util::equal_ignore_case(s->getGrblName(), key)) {
#if 0
            if (auth_failed(s, value, auth_level)) {
                return Error::AuthenticationFailed;
            }
#endif
            if (value.empty()) {
                show_setting(s->getGrblName(), s->getCompatibleValue(), NULL, out);
                return Error::Ok;
            }
            return s->setStringValue(uriDecode(value));
        }
    }

    // If we did not find an exact match and there is no value,
    // indicating a display operation, we allow partial matches
    // and display every possibility.  This only applies to the
    // text form of the name, not to the nnn and ESPnnn forms.
    if (value.empty()) {
        bool found = false;
        for (Setting* s : Setting::List) {
            auto test = s->getName();
            // The C++ standard regular expression library supports many more
            // regular expression forms than the simple one in Regex.cpp, but
            // consumes a lot of FLASH.  The extra capability is rarely useful
            // especially now that there are only a few NVS settings.
            if (regexMatch(key, test, false)) {
                const char* displayValue = s->getStringValue();
#if 0
                if (auth_failed(s, value, auth_level)) {
                    displayValue = "<Authentication required>";
                }
#endif
                show_setting(test, displayValue, NULL, out);
                found = true;
            }
        }
        if (found) {
            return Error::Ok;
        }
    }
    return Error::InvalidStatement;
}

Error settings_execute_line(const char* line, Channel& out, AuthenticationLevel auth_level) {
    std::string_view key(line + 1);
    std::string_view value;

    string_util::split(key, value, *line == '[' ? ']' : '=');
    key = string_util::trim(key);

    // At this point there are three possibilities for value
    // NULL - $xxx without =
    // NULL - [ESPxxx] with nothing after ]
    // empty string - $xxx= with nothing after
    // non-empty string - [ESPxxx]yyy or $xxx=yyy
    return do_command_or_setting(key, value, auth_level, out);
}

Error execute_line(const char* line, Channel& channel, AuthenticationLevel auth_level) {
    // Empty or comment line. For syncing purposes.
    if (line[0] == 0) {
        return Error::Ok;
    }
    // Skip leading whitespace
    while (isspace(*line)) {
        ++line;
    }
    // User '$' or WebUI '[ESPxxx]' command
    if (line[0] == '$' || line[0] == '[') {
        if (gc_state.skip_blocks) {
            return Error::Ok;
        }

        return settings_execute_line(line, channel, auth_level);
    }
    // Everything else is gcode. Block if in alarm or jog mode.
    if (state_is(State::Alarm) || state_is(State::ConfigAlarm) || state_is(State::Jog)) {
        return Error::SystemGcLock;
    }
    Error result = gc_execute_line(line);
    if (result != Error::Ok && result != Error::Reset) {
        log_error_to(channel, "Bad GCode: " << line);
        if (Job::active()) {
            send_alarm(ExecAlarm::GCodeError);
        }
    }
    return result;
}
