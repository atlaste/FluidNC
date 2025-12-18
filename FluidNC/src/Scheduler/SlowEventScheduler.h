#pragma once

#include "Platform.h"
#include "Timepoint.h"
#include "EventScheduler.h"
#include "ScopedSpinlock.h"

namespace Scheduler {
    class SlowEventScheduler : public EventScheduler {
        static const int32_t ThresholdUs = 20u;      // If it's within 2 us, just execute it.
        static const int32_t DeadlineUs  = 100000u;  // If it's not within 100 ms, shit is wrong.
        static const int32_t IdleTimeUs = 50000u;  // If we're idle, this is how long we wait. Must be >= portTICK_PERIOD_MS (10 x 1000 = 10k)

        int64_t lastEventTick_;

        class SentinelEvent : public Event {
        public:
            void invoke() override;

            ~SentinelEvent() {}
        };

        SentinelEvent sentinel_;
        int32_t       noSleepLock_ = 0;
        Spinlock      slowEventSchedulerLock_;

    public:
        SlowEventScheduler();

        SlowEventScheduler(const SlowEventScheduler&) = delete;
        SlowEventScheduler(SlowEventScheduler&&)      = delete;

        SlowEventScheduler& operator=(const SlowEventScheduler&) = delete;
        SlowEventScheduler& operator=(SlowEventScheduler&&)      = delete;

        int64_t ticksSleeping_ = 0;
        int64_t ticksSpinning_ = 0;

        void lockSleeping() { ++noSleepLock_; }
        void unlockSleeping() { --noSleepLock_; }

        int32_t tick();
        int32_t iterate();

        static void init();
        static void kill();

        void unschedule(Event* evt);
        void schedule(Event* evt);

        INLINE void IRAM schedule(Event* evt, Timepoint when) {
            evt->deadline_ = when;
            schedule(evt);
        }
        INLINE void IRAM schedule(Event* evt, Timespan delt) {
            evt->deadline_ = Timepoint::now() + delt;
            schedule(evt);
        }
        INLINE void IRAM schedule(Event* evt, int64_t when) {
            evt->deadline_ = Timepoint(when);
            schedule(evt);
        }

        void updateTimeDelta(int64_t delta) override {
            EventScheduler::updateTimeDelta(delta);
            lastEventTick_ += delta;
        }

        void wait(int64_t delay);
    };

    extern SlowEventScheduler* slowScheduler;
}
