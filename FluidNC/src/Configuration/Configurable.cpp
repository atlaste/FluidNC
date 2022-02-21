// Copyright (c) 2021 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configurable.h"
#include "Validator.h"

namespace Configuration {
    // Default behavior: propagate to child.
    void Configurable::validate(Validator& handler) { group(handler); }
    void Configurable::afterParse(HandlerBase& handler) { group(handler); }

    Configurable::~Configurable() {}
}
