#include "TestFramework.h"

#include <Scheduler/SlowEventScheduler.h>
#include <Scheduler/ISchedulable.h>
#include <Scheduler/Timer.h>
#include <Scheduler/Timepoint.h>

#include <functional>

namespace Scheduler {

    struct CoroutineFixture {
        SlowEventScheduler sched;

        CoroutineFixture() {
            Timer::setDeltaForTest(0);
        }

        ~CoroutineFixture() {
            slowScheduler = nullptr;
        }

        void runSchedulerUntil(int maxSteps, std::function<bool()> done) {
            for (int i = 0; i < maxSteps && !done(); ++i) {
                int32_t d = sched.tick();
                if (d <= 0) {
                    d = 1000;
                }
                if (d > 100000) {
                    d = 100000;
                }
                Timer::updateDelta(d);
            }
        }
    };

    static Schedulable<int> simpleDelayAndReturn() {
        co_yield 1_msec;
        co_return 42;
    }

    struct IntResultPayload {
        int  value  = -1;
        bool done   = false;
    };

    static void onIntResult(int&& r, void* p) {
        IntResultPayload* pl = static_cast<IntResultPayload*>(p);
        pl->value = r;
        pl->done  = true;
    }

    Test(Coroutine, DelayThenReturnValue) {
        CoroutineFixture   f;
        IntResultPayload  payload;
        schedule(simpleDelayAndReturn(), onIntResult, &payload);
        f.runSchedulerUntil(10000, [&]() { return payload.done; });
        Assert(payload.done, "coroutine completed");
        Assert(payload.value == 42, "return value 42");
    }

    static Schedulable<void> voidCoroutineWithDelay() {
        co_yield 1_msec;
        co_return;
    }

    Test(Coroutine, VoidCoroutineWithCompletionHandler) {
        CoroutineFixture f;
        bool             completed = false;
        schedule(voidCoroutineWithDelay(), [](void* p) {
            *static_cast<bool*>(p) = true;
        }, &completed);
        f.runSchedulerUntil(10000, [&]() { return completed; });
        Assert(completed, "void coroutine completed");
    }
}
