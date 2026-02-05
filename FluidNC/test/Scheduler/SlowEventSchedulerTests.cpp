#include "TestFramework.h"

#include <Scheduler/SlowEventScheduler.h>
#include <Scheduler/Event.h>
#include <Scheduler/Timepoint.h>
#include <Scheduler/Timespan.h>
#include <Scheduler/Timer.h>

namespace Scheduler {

    // Fixture: creates a SlowEventScheduler instance and clears the global in teardown
    // so the scheduler can be safely destructed. We never start SchedulerTask::start()
    // (no thread), so no thread to kill; clearing slowScheduler prevents dangling refs.
    struct SlowSchedulerFixture {
        SlowEventScheduler sched;

        SlowSchedulerFixture() {
            Timer::setDeltaForTest(0);
        }

        ~SlowSchedulerFixture() {
            slowScheduler = nullptr;
        }
    };

    class InvokeFlagEvent : public Event {
    public:
        bool invoked = false;
        void invoke() override { invoked = true; }
    };

    Test(SlowEventScheduler, ConstructionSetsGlobalAndHasSentinel) {
        SlowSchedulerFixture f;
        Assert(slowScheduler == &f.sched, "global set to instance");
        Assert(f.sched.NumberEvents() >= 1, "sentinel present");
        // Teardown: ~Fixture sets slowScheduler = nullptr, then f.sched destructs
    }

    Test(SlowEventScheduler, ScheduleAndRunEvent) {
        SlowSchedulerFixture f;
        InvokeFlagEvent     evt;
        evt.deadline_ = Timepoint::now() + Timespan(0);
        f.sched.schedule(&evt);
        int32_t delay = f.sched.tick();
        Assert(evt.invoked, "event was invoked");
        (void)delay;
    }

    Test(SlowEventScheduler, Unschedule) {
        SlowSchedulerFixture f;
        InvokeFlagEvent     evt;
        evt.deadline_ = Timepoint::now() + 1000_msec;
        f.sched.schedule(&evt);
        f.sched.unschedule(&evt);
        Timer::updateDelta(2000000);
        int32_t d = f.sched.tick();
        Assert(!evt.invoked, "unscheduled event not run");
        (void)d;
    }

    Test(SlowEventScheduler, TimeAdvanceRunsWhenDue) {
        SlowSchedulerFixture f;
        InvokeFlagEvent     evt;
        int64_t            now = Timer::currentTime();
        evt.deadline_          = Timepoint(now + 50000);
        f.sched.schedule(&evt);
        int32_t d = f.sched.tick();
        Assert(d > 0 && !evt.invoked, "not yet due");
        Timer::updateDelta(50000);
        f.sched.tick();
        Assert(evt.invoked, "run after time advance");
    }
}
