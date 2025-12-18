#pragma once

#include "Platform.h"

#if IS_PLATFORM(HW_ESP32)
#    include <soc/frc_timer_reg.h>  // Timer mess

#    include <esp_timer.h>

#elif IS_PLATFORM(HW_ESP32_S2 | HW_ESP32_S3)

#    include <esp_timer.h>

#elif IS_PLATFORM(HW_RPI)

#    include "ext/bcm2835/bcm2835.h"

#    define NOP() asm("nop")  // assume __GNUC__ inline asm
#    include <chrono>
#    include <thread>
#    include <time.h>

#elif IS_PLATFORM(HW_Linux)

#    define NOP() asm("nop")  // assume __GNUC__ inline asm
#    include <chrono>
#    include <thread>
#    include <time.h>

#elif IS_PLATFORM(HW_Win32 | HW_Win64)

#    ifdef _MSC_VER
#        include <intrin.h>
#        define NOP() __nop()  // _emit 0x90
#    else
#        define NOP() asm("nop")  // assume __GNUC__ inline asm
#    endif
#    include <chrono>
#    include <thread>

#endif

#include <cstdint>

namespace Scheduler {
    class Timer {
        Timer() = delete;

        Timer(const Timer&) = delete;
        Timer(Timer&&)      = delete;

        Timer& operator=(const Timer&) = delete;
        Timer& operator=(Timer&&)      = delete;

        static int64_t delta_;

    public:
        static void        timingInit() {}
        static INLINE void updateDelta(int64_t delta) { delta_ += delta; }

#if IS_PLATFORM(HW_ESP32)

        static INLINE void    nop() { __asm__ __volatile__("nop"); }
        static INLINE int64_t currentTime() {
            return esp_timer_get_time() + delta_;  // TG0_LAC
        }
        static const int64_t TicksPerMicroSecond = 1;

#elif IS_PLATFORM(HW_ESP32_S2 | HW_ESP32_S3)

        static INLINE void    nop() { __asm__ __volatile__("nop"); }
        static INLINE int64_t currentTime() { return esp_timer_get_time() + delta_; }
        static const int64_t  TicksPerMicroSecond = 1;

#elif IS_PLATFORM(HW_RPI)

        static INLINE void nop() { NOP(); }

        static INLINE int64_t currentTime() { return bcm2835_st_read(); }

        static const int64_t TicksPerMicroSecond = 1;
#else
        static int64_t currentTime();

        static const int64_t TicksPerMicroSecond = 1;

        static INLINE void nop() {}
#endif

        static const int64_t TicksPerMilliSecond = TicksPerMicroSecond * 1000;
        static const int64_t TicksPerSecond      = TicksPerMilliSecond * 1000;

        static void delayMicros(int64_t micros);
        static void delayMillis(int64_t millis);
    };
}
