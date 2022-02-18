// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "AssertionFailed.h"

#include <cstdarg>
#include <cstring>

#ifdef ESP32

#    include "debug_helpers.h"
#    include "WString.h"
#    include "stdio.h"

AssertionFailed AssertionFailed::create(const char* condition, const char* msg, ...) {
    String st = condition;
    st += ": ";

    char    tmp[255];
    va_list arg;
    va_start(arg, msg);
    size_t len = vsnprintf(tmp, 255, msg, arg);
    va_end(arg);
    tmp[254] = 0;

    st += tmp;

    st += " at: ";
    st += esp_backtrace_print(10);

    return AssertionFailed(st, tmp);
}

#else

#    include <iostream>
#    include <string>
#    include <sstream>
#    include "WString.h"

extern void DumpStackTrace(std::ostringstream& builder);

String stackTrace;

AssertionFailed AssertionFailed::create(const char* condition, const char* msg, ...) {
    std::ostringstream oss;
    oss << std::endl;
    oss << "Error: " << std::endl;

    char    tmp[255];
    va_list arg;
    va_start(arg, msg);
    size_t len = vsnprintf(tmp, 255, msg, arg);
    tmp[254]   = 0;
    msg        = tmp;
    oss << tmp;

    oss << " at ";
    DumpStackTrace(oss);

    auto   str = oss.str();
    String str2(str.c_str());

    return AssertionFailed(str2, tmp);
}

#endif
