#include "TestFramework.h"

#include <Scheduler/EventScheduler.h>
#include <Scheduler/Event.h>
#include <Scheduler/Timepoint.h>

namespace Scheduler {

    class TestEvent : public Event {
    public:
        bool invoked = false;
        void invoke() override { invoked = true; }
    };

    Test(EventScheduler, AddOne) {
        EventScheduler    sched;
        TestEvent         evt;
        evt.deadline_ = Timepoint(100);
        bool isFirst  = sched.add(&evt);
        Assert(sched.NumberEvents() == 1, "count 1");
        Assert(sched.first() == &evt, "first is evt");
        Assert(isFirst, "single event is first");
    }

    Test(EventScheduler, AddMultipleOrder) {
        EventScheduler sched;
        TestEvent      e1, e2, e3;
        e1.deadline_ = Timepoint(300);
        e2.deadline_ = Timepoint(100);
        e3.deadline_ = Timepoint(200);
        sched.add(&e1);
        sched.add(&e2);
        sched.add(&e3);
        Assert(sched.NumberEvents() == 3, "count 3");
        Assert(sched.first() == &e2, "earliest is e2");
        sched.removeFirst();
        Assert(sched.first() == &e3, "then e3");
        sched.removeFirst();
        Assert(sched.first() == &e1, "then e1");
    }

    Test(EventScheduler, AddDuplicateReturnsFalse) {
        EventScheduler sched;
        TestEvent      evt;
        evt.deadline_ = Timepoint(100);
        Assert(sched.add(&evt) == true, "first add ok");
        Assert(sched.add(&evt) == false, "duplicate add false");
        Assert(sched.NumberEvents() == 1, "still 1");
    }

    Test(EventScheduler, Remove) {
        EventScheduler sched;
        TestEvent      evt;
        evt.deadline_ = Timepoint(100);
        sched.add(&evt);
        Assert(sched.remove(&evt) == true, "remove returns true");
        Assert(sched.NumberEvents() == 0, "count 0");
        Assert(sched.remove(&evt) == false, "remove again false");
    }

    Test(EventScheduler, GrabEvent) {
        EventScheduler sched;
        TestEvent      e1, e2;
        e1.deadline_ = Timepoint(200);
        e2.deadline_ = Timepoint(100);
        sched.add(&e1);
        sched.add(&e2);
        Event* grabbed = sched.grabEvent();
        Assert(grabbed == &e2, "grabbed earliest");
        Assert(sched.NumberEvents() == 1, "count 1 after grab");
        Assert(sched.first() == &e1, "first is e1");
    }

    Test(EventScheduler, UpdateTimeDeltaPositive) {
        EventScheduler sched;
        TestEvent      e1, e2;
        e1.deadline_ = Timepoint(100);
        e2.deadline_ = Timepoint(200);
        sched.add(&e1);
        sched.add(&e2);
        sched.updateTimeDelta(50);
        Assert(e1.deadline_() == 150, "e1 shifted");
        Assert(e2.deadline_() == 250, "e2 shifted");
        Assert(sched.first() == &e1, "order unchanged");
    }

    Test(EventScheduler, UpdateTimeDeltaNegative) {
        EventScheduler sched;
        TestEvent      e1, e2;
        e1.deadline_ = Timepoint(100);
        e2.deadline_ = Timepoint(200);
        sched.add(&e1);
        sched.add(&e2);
        sched.updateTimeDelta(-30);
        Assert(e1.deadline_() == 70, "e1 shifted");
        Assert(e2.deadline_() == 170, "e2 shifted");
        Assert(sched.first() == &e1, "order unchanged");
    }

    Test(EventScheduler, HeapFull) {
        EventScheduler sched;
        TestEvent      events[130];
        for (int i = 0; i < 128; ++i) {
            events[i].deadline_ = Timepoint(1000 + i);
            Assert(sched.add(&events[i]) == true, "add until full");
        }
        events[128].deadline_ = Timepoint(1);
        Assert(sched.add(&events[128]) == false, "add when full returns false");
        Assert(sched.NumberEvents() == 128, "still 128");
    }
}
