#pragma once

#include "Timepoint.h"

#include <cstdint>

namespace Scheduler {
    class Event {
    public:
        Timepoint deadline_;

        // NOTE: *IF* you use the fast event scheduler, you must ensure that all vtable calls appear in DRAM/IRAM with a compile script.
        // This is important, because flash access can crash the ESP32 if you don't!

        virtual void invoke() = 0;

        virtual ~Event() {}
    };
}
