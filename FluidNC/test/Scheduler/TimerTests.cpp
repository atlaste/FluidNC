#include "TestFramework.h"

#include <Scheduler/Timer.h>

namespace Scheduler {

    Test(Timer, CurrentTimeNonNegative) {
        int64_t t = Timer::currentTime();
        Assert(t >= 0, "currentTime >= 0");
    }

    Test(Timer, UpdateDeltaAdvancesTime) {
        Timer::setDeltaForTest(0);
        int64_t t0 = Timer::currentTime();
        Timer::updateDelta(1000);
        int64_t t1 = Timer::currentTime();
        Assert(t1 == t0 + 1000, "currentTime advances by delta");
        Timer::setDeltaForTest(0);
    }

    Test(Timer, SetDeltaForTest) {
        Timer::setDeltaForTest(100);
        int64_t t = Timer::currentTime();
        Timer::setDeltaForTest(200);
        int64_t t2 = Timer::currentTime();
        Assert(t2 == t - 100 + 200, "setDeltaForTest sets offset");
        Timer::setDeltaForTest(0);
    }
}
