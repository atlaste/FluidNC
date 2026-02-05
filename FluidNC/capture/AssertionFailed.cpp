// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "AssertionFailed.h"
#include "ExceptionHelper.h"

#include <cstdarg>
#include <cstring>
#include <sstream>

std::runtime_error AssertionFailed::create() {
    std::ostringstream oss;
    oss << "Assertion failed\n" << GetStackTrace();
    return std::runtime_error(oss.str());
}

std::runtime_error AssertionFailed::create(const char* msg, ...) {
    char    buffer[512];
    va_list args;
    va_start(args, msg);
    vsnprintf(buffer, sizeof(buffer), msg, args);
    va_end(args);

    std::ostringstream oss;
    oss << buffer << "\n" << GetStackTrace();
    return std::runtime_error(oss.str());
}
