#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Scheduler {
    class SchedulerTask {
        static TaskHandle_t schedulerTask;

        SchedulerTask()                     = delete;
        SchedulerTask(const SchedulerTask&) = delete;
        SchedulerTask(SchedulerTask&&)      = delete;

    public:
        static void start();
    };
}
