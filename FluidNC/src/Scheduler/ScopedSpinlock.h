#pragma once

#include "Platform.h"

#if IS_PLATFORM(HW_ESP32 | HW_ESP32_S2 | HW_ESP32_S3)

#    include <freertos/FreeRTOS.h>

namespace Scheduler {
    class Spinlock {
        portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

    public:
        INLINE void IRAM lock() noexcept {
            if (xPortInIsrContext()) {
                portENTER_CRITICAL_ISR(&spinlock);
            } else {
                portENTER_CRITICAL(&spinlock);
            }
        }

        INLINE bool IRAM try_lock() noexcept {
            if (xPortInIsrContext()) {
                return portTRY_ENTER_CRITICAL_ISR(&spinlock, 80) == pdPASS;
            } else {
                return portTRY_ENTER_CRITICAL(&spinlock, 80) == pdPASS;
            }
        }

        INLINE void IRAM unlock() noexcept {
            if (xPortInIsrContext()) {
                portEXIT_CRITICAL_ISR(&spinlock);
            } else {
                portEXIT_CRITICAL(&spinlock);
            }
        }
    };
}
#else

#    include <algorithm>
#    include <atomic>
#    include <thread>

namespace Scheduler {
    class Spinlock {
        std::atomic<bool> lock_ = { 0 };

        Spinlock(const Spinlock&)        = delete;
        Spinlock& operator=(Spinlock& o) = delete;
        Spinlock(Spinlock&& o)           = delete;
        Spinlock& operator=(Spinlock&&)  = delete;

    public:
        Spinlock() = default;

        INLINE void IRAM lock() noexcept {
            for (;;) {
                // Optimistically assume the lock is free on the first try
                if (likely(!lock_.exchange(true, std::memory_order_acquire))) {
                    return;
                }
                // Wait for lock to be released without generating cache misses
                while (lock_.load(std::memory_order_relaxed)) {
#    ifdef _WIN32
                    // Issue X86 PAUSE or ARM YIELD instruction to reduce contention between
                    // hyper-threads
                    std::this_thread::yield();
#    else
                    asm volatile("nop");
#    endif
                }
            }
        }

        INLINE bool IRAM try_lock() noexcept {
            // First do a relaxed load to check if lock is free in order to prevent
            // unnecessary cache misses if someone does while(!try_lock())
            return !lock_.load(std::memory_order_relaxed) && !lock_.exchange(true, std::memory_order_acquire);
        }

        INLINE void IRAM unlock() noexcept { lock_.store(false, std::memory_order_release); }

        ~Spinlock() = default;
    };
}

#endif
