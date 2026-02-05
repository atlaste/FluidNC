// Copyright (c) 2021 - Stefan de Bruijn
// Mock AssertionFailed.h for Windows unit testing
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <stdexcept>
#include <string>
#include <cstdarg>
#include <cstdio>

class AssertionFailed : public std::runtime_error {
public:
    std::string stackTrace;
    
    AssertionFailed(const std::string& msg) : std::runtime_error(msg), stackTrace(msg) {}
    
    // Overload for no-argument Assert(expr) calls
    static std::runtime_error create();
    static std::runtime_error create(const char* msg, ...);
};
