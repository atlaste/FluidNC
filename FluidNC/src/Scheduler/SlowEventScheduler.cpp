#include "SlowEventScheduler.h"

#include "Platform.h"

#define ENABLE_LIGHT_SLEEP 0

#if IS_PLATFORM(HW_ESP32 | HW_ESP32_S2 | HW_ESP32_S3)
#    include <freertos/FreeRTOS.h>  // vTaskDelay

#    include <esp_sleep.h>
#endif


namespace Scheduler {
    // Public globals:
    SlowEventScheduler* slowScheduler = nullptr;

    void SlowEventScheduler::SentinelEvent::invoke() {
        slowScheduler->schedule(this, 500_msec);
    }

    int32_t SlowEventScheduler::tick() {
        lastEventTick_ = Timer::currentTime();

        // We don't want to spinlock within the ISR. Try to grab the lock, if we fail, just wait a few us to see if it
        // gets sorted out.
        if (slowEventSchedulerLock_.try_lock()) {
            // Locked. Quickly decide what to do:
            auto time = Timer::currentTime();  // Or update the ticks directly?
            auto evt  = first();
            auto w    = evt->deadline_() - time;
            auto wait = int32_t(w < 0 ? 0 : w);  // in us.

            // if (wait <= Timer::TicksPerMilliSecond && w > 0)
            // {
            //     log_info("Short event triggered by " << typeid(*evt).name());
            // }

            // NOTE: Wait will never be >= 100 ms due to the sentinel. Awesome!

            // log_debug("Waiting " << wait << " ticks"); // takes too long for the lock!

            if (wait <= ThresholdUs) {
                // We need to execute, regardless of anything else.
                // Grab the event, removing it from the heap.
                removeFirst();
                slowEventSchedulerLock_.unlock();

                // log_debug("Running scheduled item, scheduled at " << evt->deadline_());  // takes too long for the lock!

                // At this point, our heap might be empty, because the sentinel might be removed.
                // The event handler itself might add it back.
                //
                // Execute.
                evt->invoke();

                // Wait for first event. No need to do this locked:
                wait = 0;
            } else {
                slowEventSchedulerLock_.unlock();
            }

            return wait;
        } else {
            return IdleTimeUs;
        }
    }

    SlowEventScheduler::SlowEventScheduler() : EventScheduler() {
        lastEventTick_ = Timer::currentTime();

        slowScheduler = this;

        sentinel_.deadline_ = Timepoint::now() + Timespan(DeadlineUs);
        add(&sentinel_);  // Cannot run the handler yet!
    }

    void SlowEventScheduler::init() {
        slowScheduler = new SlowEventScheduler();
#if IS_PLATFORM(HW_ESP32 | HW_ESP32_S2 | HW_ESP32_S3) && ENABLE_LIGHT_SLEEP == 1
        esp_sleep_enable_timer_wakeup(IdleTimeUs);  // 2 second
#endif
    }

    void SlowEventScheduler::kill() {
        delete slowScheduler;
        slowScheduler = nullptr;
    }

    // Must be IRAM, because it might be called from the fast scheduler ISR
    void IRAM SlowEventScheduler::schedule(Event* evt) {
        slowEventSchedulerLock_.lock();
        add(evt);
        slowEventSchedulerLock_.unlock();

        // log_info("Scheduling event at " << evt->time_ << ". Queue is: [";
        //          for (size_t i = 0; i < heapCount_; ++i) { ss << heap_[i]->time_ << " "; } ss << "]");
    }

    void IRAM SlowEventScheduler::unschedule(Event* evt) {
        slowEventSchedulerLock_.lock();
        remove(evt);
        slowEventSchedulerLock_.unlock();
    }

    int32_t SlowEventScheduler::iterate() {
        auto delay = tick();
        if (delay > IdleTimeUs) {
            delay = IdleTimeUs;
        }

        return delay;
    }

    void SlowEventScheduler::wait(int64_t delay) {
        if (delay == 0) {
            return;
        }

#if IS_PLATFORM(HW_ESP32 | HW_ESP32_S2 | HW_ESP32_S3) && ENABLE_LIGHT_SLEEP == 1

        if (noSleepLock_ == 0) {
            // TODO FIXME: We should use esp_sleep_enable_touchpad_wakeup() for capacitive touch !
            //
            // No other peripherals needed. Use modem sleep:

            ticksSleeping_ += delay + 1;

            esp_sleep_enable_timer_wakeup(delay + 1);
            esp_light_sleep_start();
        } else {
            // NOTE: Do not use Timer::delayMicros. This would do 'nop' in a loop, which doesn't run a context switch.
            // In this specific, unique case we need context switches.

            ticksSpinning_ += delay + 1;

            TickType_t xDelay = (delay / 1000) / portTICK_PERIOD_MS;
            if (xDelay > 0) {
                vTaskDelay(xDelay + 1);  // We wait +1 to ensure the dealine has passed.
            }
        }
#else
        Timer::delayMillis(delay / Timer::TicksPerMilliSecond);
        Timer::delayMicros(delay % Timer::TicksPerMilliSecond);
#endif
    }
}
