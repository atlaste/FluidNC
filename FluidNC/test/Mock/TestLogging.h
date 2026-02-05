// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
// Unit tests: include this instead of Logging.h to use NullLogStream (no-op) for all log macros.

#pragma once

#include "Logging.h"

#undef log_msg
#undef log_verbose
#undef log_debug
#undef log_info
#undef log_warn
#undef log_error
#undef log_config_error
#undef log_fatal
#undef log_msg_to
#undef log_verbose_to
#undef log_debug_to
#undef log_info_to
#undef log_warn_to
#undef log_error_to
#undef log_fatal_to
#undef log_stream
#undef log_string

#define log_msg(x) { NullLogStream ss(MsgLevelNone, "[MSG:"); ss << x; }
#define log_verbose(x) if (atMsgLevel(MsgLevelVerbose)) { NullLogStream ss(MsgLevelVerbose, "[MSG:VRB: "); ss << x; }
#define log_debug(x) if (atMsgLevel(MsgLevelDebug)) { NullLogStream ss(MsgLevelDebug, "[MSG:DBG: "); ss << x; }
#define log_info(x) if (atMsgLevel(MsgLevelInfo)) { NullLogStream ss(MsgLevelInfo, "[MSG:INFO: "); ss << x; }
#define log_warn(x) if (atMsgLevel(MsgLevelWarning)) { NullLogStream ss(MsgLevelWarning, "[MSG:WARN: "); ss << x; }
#define log_error(x) if (atMsgLevel(MsgLevelError)) { NullLogStream ss(MsgLevelError, "[MSG:ERR: "); ss << x; }
#define log_config_error(x) if (atMsgLevel(MsgLevelError)) { NullLogStream ss(MsgLevelError, "[MSG:ERR: "); ss << x; set_state(State::ConfigAlarm); }
#define log_fatal(x) { NullLogStream ss(MsgLevelNone, "[MSG:FATAL: "); ss << x; Assert(false, "A fatal error occurred"); }

#define log_msg_to(out, x) { NullLogStream ss(out, MsgLevelNone, "[MSG:"); ss << x; }
#define log_verbose_to(out, x) if (atMsgLevel(MsgLevelVerbose)) { NullLogStream ss(out, MsgLevelVerbose, "[MSG:VRB: "); ss << x; }
#define log_debug_to(out, x) if (atMsgLevel(MsgLevelDebug)) { NullLogStream ss(out, MsgLevelDebug, "[MSG:DBG: "); ss << x; }
#define log_info_to(out, x) if (atMsgLevel(MsgLevelInfo)) { NullLogStream ss(out, MsgLevelInfo, "[MSG:INFO: "); ss << x; }
#define log_warn_to(out, x) if (atMsgLevel(MsgLevelWarning)) { NullLogStream ss(out, MsgLevelWarning, "[MSG:WARN: "); ss << x; }
#define log_error_to(out, x) if (atMsgLevel(MsgLevelError)) { NullLogStream ss(out, MsgLevelError, "[MSG:ERR: "); ss << x; }
#define log_fatal_to(out, x) { NullLogStream ss(out, MsgLevelNone, "[MSG:FATAL: "); ss << x; Assert(false, "A fatal error occurred"); }

#define log_stream(out, x) { NullLogStream ss(out, MsgLevelNone); ss << x; }
#define log_string(out, x) ((void)(out), (void)(x))
