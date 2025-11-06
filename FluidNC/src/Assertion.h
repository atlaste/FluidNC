// Copyright (c) 2021 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "AssertionFailed.h"

class AssertionFailed;

#undef Assert

#define Assert(condition, ...)                                                                                                             \
    {                                                                                                                                      \
        if (!(condition)) {                                                                                                                \
            throw AssertionFailed::create(__VA_ARGS__);                                                                                    \
        }                                                                                                                                  \
    }

// Helper macros for AssertOK overloading
#define AssertOK_1(variable) Assert(variable == ESP_OK, "Assertion failed")
#define AssertOK_N(variable, ...) Assert(variable == ESP_OK, __VA_ARGS__)

// Macro overloading: selects AssertOK_1 for 1 arg, AssertOK_N for 2+ args
#define GET_ASSERTOK_MACRO(_1, _2, NAME, ...) NAME
#define AssertOK(...) GET_ASSERTOK_MACRO(__VA_ARGS__, AssertOK_N, AssertOK_1)(__VA_ARGS__)
