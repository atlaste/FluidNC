#pragma once

#include "Platform.h"
#include "Timer.h"
#include "Timespan.h"

#include <cstdint>

namespace Scheduler {
    class Timepoint {
        int64_t timepoint_;

    public:
        Timepoint() : timepoint_(0) {}

        INLINE Timepoint(int64_t when) : timepoint_(when) {}

        INLINE int64_t IRAM operator()() const { return timepoint_; }

        INLINE Timepoint IRAM  operator+(const Timespan& ts) const { return Timepoint(timepoint_ + ts()); }
        INLINE Timepoint IRAM  operator-(const Timespan& ts) const { return Timepoint(timepoint_ - ts()); }
        INLINE Timepoint& IRAM operator+=(const Timespan& ts) {
            timepoint_ += ts();
            return *this;
        }
        INLINE Timepoint& IRAM operator-=(const Timespan& ts) {
            timepoint_ -= ts();
            return *this;
        }

        INLINE bool IRAM operator<(const Timepoint& o) const { return timepoint_ < o.timepoint_; }
        INLINE bool IRAM operator<=(const Timepoint& o) const { return timepoint_ <= o.timepoint_; }
        INLINE bool IRAM operator>(const Timepoint& o) const { return timepoint_ > o.timepoint_; }
        INLINE bool IRAM operator>=(const Timepoint& o) const { return timepoint_ >= o.timepoint_; }
        INLINE bool IRAM operator==(const Timepoint& o) const { return timepoint_ == o.timepoint_; }
        INLINE bool IRAM operator!=(const Timepoint& o) const { return timepoint_ != o.timepoint_; }

        static INLINE Timepoint IRAM now() { return Timepoint(Timer::currentTime()); }
    };
}
