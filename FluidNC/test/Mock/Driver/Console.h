// Mock Console.h for Windows unit testing
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Channel.h"

// Mock: Console is declared but not defined - link errors will occur if it's actually used
extern Channel& Console;
