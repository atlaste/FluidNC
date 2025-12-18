#include "SchedulerTask.h"

#include "SlowEventScheduler.h"
#include "Config.h"

#include <esp_task_wdt.h>

namespace Scheduler {
    namespace {
        // Dedicated scheduler task for coroutine-based non-real-time operations
        // This replaces the module->poll() pattern with a proper event-driven scheduler
        void scheduler_loop(void* unused) {
            esp_task_wdt_add(NULL);  // Add to watchdog

            while (true) {
                // Run scheduler and get time until next event
                int32_t delayUs = Scheduler::slowScheduler->iterate();

                // Convert to FreeRTOS ticks (minimum 1 tick = 10ms)
                // We use a minimum delay to avoid busy-waiting
                if (delayUs > 1000) {
                    TickType_t ticks = (delayUs / 1000) / portTICK_PERIOD_MS;
                    if (ticks > 100) { // cap at 100 ms.
                        ticks = 100;
                    }
                    vTaskDelay(ticks);
                } else {
                    Timer::delayMicros(delayUs);
                }

                esp_task_wdt_reset();
            }
        }
    }

    TaskHandle_t SchedulerTask::schedulerTask = nullptr;

    void SchedulerTask::start() {
        if (schedulerTask == nullptr) {
            // Initialize the event scheduler for coroutine-based tasks
            Scheduler::SlowEventScheduler::init();

            // Create the event scheduler task for coroutine-based scheduling
            // Priority 1 (same as polling) - low enough not to interfere with motion
            xTaskCreatePinnedToCore(scheduler_loop,    // task
                                    "scheduler",       // name for task
                                    8192,              // size of task stack
                                    0,                 // parameters
                                    1,                 // priority (non-real-time)
                                    &schedulerTask,    // task handle
                                    SUPPORT_TASK_CORE  // core
            );
        }
    }

}
