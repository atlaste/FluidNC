// Copyright (c) 2024 - FluidNC Authors
// Windows Google Test main entry point
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#if defined(_WIN32) || defined(_WIN64)

#include "gtest/gtest.h"

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#endif
