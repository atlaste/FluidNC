// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "AssertionFailed.h"

#include <cstdarg>
#include <cstring>

std::runtime_error AssertionFailed::create(const char* msg, ...) {
    char    buffer[512];
    va_list args;
    va_start(args, msg);
    vsnprintf(buffer, sizeof(buffer), msg, args);
    va_end(args);
    return std::runtime_error(buffer);
}
