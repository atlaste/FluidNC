#pragma once

#include "Error.h"

namespace GCode {
    class GCodeParser {
    public:
        Error gc_execute_line(const char* input_line);
    };
}
