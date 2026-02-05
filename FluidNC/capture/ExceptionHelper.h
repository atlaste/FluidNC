// Copyright (c) 2021 -  Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <string>
#include <sstream>

// Dumps the current stack trace to the given stream.
// On Windows, this provides full symbol resolution with file/line info.
// On other platforms, this is a no-op.
void DumpStackTrace(std::ostringstream& builder);

// Returns a formatted stack trace string
std::string GetStackTrace();
