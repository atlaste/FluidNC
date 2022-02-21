// Copyright (c) 2020 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "LimitedResource.h"

// This file holds containers for peripherals and hard limits for the hardware.

const int MAX_N_AXIS = 6;  // TODO FIXME: Find a better place for this

class Peripherals {
public:
    LimitedResource<8> RMT = "RMT";

    static Peripherals& instance() {
        static Peripherals instance;
        return instance;
    }
};
